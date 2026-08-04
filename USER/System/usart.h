#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"

/* ============================================================
 *            USART1 + DMA 串 口 高 级 驱 动 头 文 件
 * ============================================================
 *
 *  引脚:  PA9  = USART1_TX  (发送)
 *         PA10 = USART1_RX  (接收, 暂不做接收 DMA)
 *
 *  两种发送模式:
 *    ┌───────────────┬──────────────────┬─────────────────────┐
 *    │     模式      │      原理        │      适用场景        │
 *    ├───────────────┼──────────────────┼─────────────────────┤
 *    │ 阻塞发送      │ CPU 循环等 TXE   │ debug 字符串, 少量  │
 *    │ DMA 非阻塞    │ DMA 后台搬运     │ 大批量/高频传感器数据│
 *    └───────────────┴──────────────────┴─────────────────────┘
 *
 *  VOFA+ JustFloat 协议:
 *    用于 VOFA+ 上位机波形显示, 数据格式:
 *      [float ch1][float ch2]...[float chN][tail: 0x00 0x00 0x80 0x7F]
 *    尾部 4 字节是 float +Inf (正无穷), VOFA+ 用它判断帧边界.
 *
 *  DMA 通道资源 (STM32F103):
 *    USART1_TX → DMA1_Channel4
 *    USART1_RX → DMA1_Channel5
 * ============================================================
 */

/* ========== VOFA+ 帧 配 置 ========== */
#define VOFA_MAX_CHANNELS   8       /* 最多发送 8 通道浮点数据 */
#define VOFA_TAIL           0x7F800000  /* float +Inf 的 IEEE 754 表示 */

/* ========== 初 始 化 ========== */
void UART1_Configuration(void);     /* 基础 USART1 初始化 (115200-8-N-1, 寄存器方式) */
void UART1_DMA_Init(void);          /* 配置 DMA1_Channel4 用于 USART1_TX */

/* ========== 阻 塞 发 送 (调试用) ========== */
void UART_SendByte(uint8_t byte);
void UART_SendStr(const char *str);
void UART_SendDec(uint32_t num);
void UART_SendHex(uint8_t num);

/* ========== DMA 非 阻 塞 发 送 ========== */
void UART1_DMA_Send(const uint8_t *buf, uint16_t len);  /* DMA 发送任意数据 */
void UART1_DMA_Update(void);                               /* 周期性调用, 检查 DMA 完成 */

/* ========== VOFA+ 协 议 ========== */
void VOFA_SendFrame(const float *data, uint8_t channels); /* 发送一帧浮点数据到 VOFA+ */

/* ========== 复 位 源 检 查 ========== */
void CheckAndReportResetSource(void);

#endif /* __USART_H */
