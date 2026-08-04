#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"

/**
 * @file    usart.h
 * @brief   USART1 + DMA driver interface with VOFA+ JustFloat support.
 *          PA9 = USART1_TX, PA10 = USART1_RX (no RX DMA);
 *          USART1_TX uses DMA1_Channel4.
 */

/* VOFA+ frame configuration */
#define VOFA_MAX_CHANNELS   8       /* Max float channels per frame */
#define VOFA_TAIL           0x7F800000  /* IEEE754 +Inf frame delimiter */

void UART1_Configuration(void);     /* USART1 init, 115200-8-N-1, register-based */
void UART1_DMA_Init(void);          /* DMA1_Channel4 for USART1_TX */

void UART_SendByte(uint8_t byte);   /* Blocking byte send (debug) */
void UART_SendStr(const char *str); /* Blocking string send (debug) */
void UART_SendDec(uint32_t num);    /* Blocking decimal send (debug) */
void UART_SendHex(uint8_t num);     /* Blocking hex send (debug) */

void UART1_DMA_Send(const uint8_t *buf, uint16_t len);  /* Non-blocking DMA send */
void UART1_DMA_Update(void);        /* Poll DMA completion, clear busy flag */

void VOFA_SendFrame(const float *data, uint8_t channels); /* Send float frame to VOFA+ */

void CheckAndReportResetSource(void);

#endif /* __USART_H */
