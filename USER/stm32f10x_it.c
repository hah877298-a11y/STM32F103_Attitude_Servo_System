/**
 * @file    stm32f10x_it.c
 * @brief   Interrupt service routines.
 *          ISRs stay minimal (set flags, clear pending bits);
 *          heavy processing is deferred to the main loop.
 */
#include "stm32f10x.h"
#include "SysTick.h"
#include "Encoder.h"
#include "adc.h"

/** WWDG early-wakeup flag set from WWDG_IRQHandler; only ~910 us remain before reset. */
volatile uint8_t wwdg_ewi_triggered = 0;

/* System exception handlers */
void NMI_Handler(void)           {}
void HardFault_Handler(void)     { while (1); }
void MemManage_Handler(void)     { while (1); }
void BusFault_Handler(void)      { while (1); }
void UsageFault_Handler(void)    { while (1); }
void SVC_Handler(void)           {}
void DebugMon_Handler(void)      {}
void PendSV_Handler(void)        {}
/**
 * @brief  SysTick ISR, triggered every 1ms.
 * @note   Kept minimal: only increments the tick counter.
 */
void SysTick_Handler(void)
{
    SysTick_IncTick();
}

/**
 * @brief  WWDG early-wakeup interrupt (counter == 0x40).
 * @note   Clears the interrupt flag and notifies the main loop via
 *         wwdg_ewi_triggered; ~910us remain before reset.
 */
void WWDG_IRQHandler(void)
{
    if (WWDG_GetFlagStatus() != RESET)
    {
        WWDG_ClearFlag();
        wwdg_ewi_triggered = 1;
    }
}

/**
 * @brief  Encoder interrupt on PB12/PB13 falling edges.
 * @note   PB12 (A phase) drives Encoder_OnInterrupt(); PB13 (B phase)
 *         pending flag is cleared only (single-edge scheme).
 */
void EXTI15_10_IRQHandler(void)
{
    /* A phase (PB12) falling edge */
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line12);
        Encoder_OnInterrupt();
    }

    /* B phase (PB13): clear flag only (single-edge scheme) */
    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

/**
 * @brief  ADC1 end-of-conversion interrupt (single conversion mode).
 * @note   Stores the 12-bit result and sets adc_conversion_done;
 *         display/printing is deferred to the main loop.
 */
void ADC1_2_IRQHandler(void)
{
    /* EOC check: ADC1 and ADC2 share this interrupt vector */
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

        adc_value = ADC_GetConversionValue(ADC1);  /* 12-bit conversion result */
        adc_conversion_done = 1;                   /* Notify main loop */
    }
}

/* Stub handlers: clear pending flags only; reserved for future use */

void EXTI4_IRQHandler(void)
{
    /* Reserved: external trigger/button */
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    /* Reserved: DMA1_CH1 transfer complete */
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}
