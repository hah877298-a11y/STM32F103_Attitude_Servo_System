/**
 * @file    usart.c
 * @brief   USART1 + DMA driver with VOFA+ FireWater (CSV) support.
 *          PA9=TX / PA10=RX, 115200-8-N-1; TX via DMA1_Channel4.
 */
#include "usart.h"
#include <stddef.h>
#include <stdio.h>

/* Busy flag: drop new frames while the previous DMA transfer runs */
static volatile uint8_t dmaTxBusy = 0;

/**
 * @brief  Initialize USART1: PA9=TX, PA10=RX, 115200-8-N-1.
 * @note   Register-level access; BRR=0x0271 for 72MHz PCLK2
 *         (0.16% baud error, reliable for 115200).
 */
void UART1_Configuration(void)
{
    /* Enable GPIOA and USART1 clocks (APB2) */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9 = AF push-pull output, 50MHz (CRH bits[7:4], CNF=10, MODE=11) */
    GPIOA->CRH &= ~(0xF << 4);
    GPIOA->CRH |=  (0xB << 4);

    /* PA10 = floating input (RX, CRH bits[15:8]) */
    GPIOA->CRH &= ~(0xF << 8);
    GPIOA->CRH |=  (0x4 << 8);

    /* Baud rate 115200: BRR = (39 << 4) | 1 */
    USART1->BRR = (39 << 4) | 1;

    /* Enable USART: TX + RX + module */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

/**
 * @brief  Configure DMA1_Channel4 for USART1 TX (memory-to-peripheral).
 * @note   Channel mapping is fixed by hardware: DMA1_CH4 = USART1_TX.
 */
void UART1_DMA_Init(void)
{
    /* Enable DMA1 clock (AHB) */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitTypeDef dma;

    DMA_DeInit(DMA1_Channel4);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr     = 0;                      /* set per transfer in UART1_DMA_Send() */
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize         = 0;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_Medium;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &dma);

    /* Enable USART1 DMA TX request (USART_CR3_DMAT) */
    USART1->CR3 |= USART_CR3_DMAT;
}

/**
 * @brief  Start a non-blocking DMA transfer of a data buffer.
 * @param  buf: Source buffer; must stay valid until the transfer ends
 * @param  len: Number of bytes to send
 * @note   Frames are dropped while a previous transfer is still busy.
 */
void UART1_DMA_Send(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return;

    /* Drop frame while previous transfer is in progress */
    if (dmaTxBusy)
        return;

    dmaTxBusy = 1;

    /* Reconfigure channel: disable, set source/length, restart */
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CMAR  = (uint32_t)buf;       /* Memory source address */
    DMA1_Channel4->CNDTR = len;                 /* Transfer length (bytes) */
    DMA_ClearFlag(DMA1_FLAG_TC4);               /* Clear TC flag of previous run */
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/**
 * @brief  Clear the busy flag once the pending DMA transfer completes.
 * @note   Must be polled periodically from the main loop, otherwise
 *         all subsequent DMA frames are dropped.
 */
void UART1_DMA_Update(void)
{
    if (dmaTxBusy && DMA_GetFlagStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);
        dmaTxBusy = 0;
    }
}

/**
 * @brief  Send one frame of float channels in VOFA+ FireWater CSV format.
 * @param  data:     channel values
 * @param  channels: channel count (1..VOFA_MAX_CHANNELS)
 * @note   ASCII CSV line, e.g. "1.23,4.56,90.00\r\n".
 *         Static buffer keeps the pointer valid while DMA is reading
 *         it (avoids use-after-stack-return).
 */
void VOFA_SendFrame(const float *data, uint8_t channels)
{
    static char buf[128];  /* static: stays valid during DMA transfer */
    uint8_t i;
    int pos = 0;

    if (channels == 0 || channels > VOFA_MAX_CHANNELS)
        return;

    for (i = 0; i < channels; i++)
    {
        /* "%.2f" per channel into the remaining space (leave room for \r\n\0) */
        int remain = (int)sizeof(buf) - pos;
        if (remain < 10) break;  /* guard against overflow */

        pos += snprintf(buf + pos, remain, "%.2f", (double)data[i]);

        if (i < channels - 1 && pos < (int)sizeof(buf) - 2)
            buf[pos++] = ',';
    }

    if (pos < (int)sizeof(buf) - 2)
    {
        buf[pos++] = '\r';
        buf[pos++] = '\n';
    }

    UART1_DMA_Send((const uint8_t *)buf, (uint16_t)pos);
}

/**
 * @brief  Blocking UART send of a single byte (waits for TXE).
 * @param  byte: Byte to transmit
 * @note   Debug output only; use VOFA_SendFrame() for sensor data.
 */
void UART_SendByte(uint8_t byte)
{
    while (!(USART1->SR & USART_SR_TXE));   /* wait for TXE */
    USART1->DR = byte;
}

/**
 * @brief  Blocking UART send of a NUL-terminated string.
 * @param  str: String to transmit
 */
void UART_SendStr(const char *str)
{
    while (*str)
    {
        UART_SendByte((uint8_t)(*str++));
    }
}

/**
 * @brief  Blocking UART send of an unsigned decimal value.
 * @param  num: Value to transmit
 */
void UART_SendDec(uint32_t num)
{
    char buf[12];
    uint8_t i = 0;

    if (num == 0)
    {
        UART_SendByte('0');
        return;
    }

    while (num > 0)
    {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }

    while (i > 0)
        UART_SendByte(buf[--i]);
}

/**
 * @brief  Blocking UART send of a byte as two hex digits.
 * @param  num: Value to transmit
 */
void UART_SendHex(uint8_t num)
{
    static const char hex[] = "0123456789ABCDEF";
    UART_SendByte(hex[(num >> 4) & 0x0F]);
    UART_SendByte(hex[num & 0x0F]);
}

/**
 * @brief  Print the system reset source over UART, then clear flags.
 * @note   IWDG/WWDG resets indicate a faulted program; this output
 *         helps diagnose unexpected reboots.
 */
void CheckAndReportResetSource(void)
{
    UART_SendStr("\r\n========== Reset Source ==========\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)
        UART_SendStr("[!] WWDG Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
        UART_SendStr("[!] IWDG Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        UART_SendStr("[!] NRST Pin Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        UART_SendStr("[!] Power-On Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)
        UART_SendStr("[!] Software Reset Detected\r\n");

    RCC_ClearFlag();

    UART_SendStr("==================================\r\n\r\n");
}
