#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"

/**
 * @file    usart.h
 * @brief   USART1 + DMA driver interface with VOFA+ FireWater (CSV) support.
 *          PA9 = USART1_TX, PA10 = USART1_RX (no RX DMA);
 *          USART1_TX uses DMA1_Channel4.
 */

#include "system_config.h"  /* VOFA_MAX_CHANNELS */

void UART1_Configuration(void);     /* 115200-8-N-1, register-level */
void UART1_DMA_Init(void);          /* DMA1_Channel4 for USART1_TX */

void UART_SendByte(uint8_t byte);   /* blocking */
void UART_SendStr(const char *str); /* blocking */
void UART_SendDec(uint32_t num);    /* blocking */
void UART_SendHex(uint8_t num);     /* blocking */

void UART1_DMA_Send(const uint8_t *buf, uint16_t len);  /* non-blocking */
void UART1_DMA_Update(void);        /* poll DMA completion, clear busy flag */

void VOFA_SendFrame(const float *data, uint8_t channels);

void CheckAndReportResetSource(void);

#endif /* __USART_H */
