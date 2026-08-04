/**
 * @file    stm32f10x_it.h
 * @brief   Interrupt service routine declarations.
 */
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#include "stm32f10x.h"

/* System exception handlers */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Peripheral interrupts */
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);    /* Encoder PB12/PB13 */
void DMA1_Channel1_IRQHandler(void);
void WWDG_IRQHandler(void);
void ADC1_2_IRQHandler(void);       /* ADC1 EOC */

#endif /* __STM32F10x_IT_H */
