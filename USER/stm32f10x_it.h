#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#include "stm32f10x.h"

// 系统异常处理
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

// 外设中断
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);    /* 旋转编码器 PB12/PB13 */
void DMA1_Channel1_IRQHandler(void);
void WWDG_IRQHandler(void);
void ADC1_2_IRQHandler(void);       /* ADC 转换完成中断 */

#endif /* __STM32F10x_IT_H */
