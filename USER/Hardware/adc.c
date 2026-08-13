#include "adc.h"

/**
 * @file    adc.c
 * @brief   ADC1 single-channel driver (PA1 / CH1) with EOC interrupt.
 */

/** Latest conversion result (updated in ISR). */
volatile uint16_t adc_value = 0;
/** Conversion-ready flag: set by ISR, cleared by main loop. */
volatile uint8_t  adc_conversion_done = 0;

/**
 * @brief  Configure PA1 as analog input and enable ADC1 clock.
 * @note   ADC clock must be <= 14 MHz: PCLK2 72 MHz / 6 = 12 MHz.
 *         The default prescaler left by SystemInit is too fast and
 *         causes calibration failure / invalid conversions.
 */
void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PA1: analog input */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  Configure ADC1: independent, single conversion, software
 *         trigger, channel 1 (55.5 cycles sample time), EOC interrupt,
 *         calibration and NVIC (ADC1_2_IRQn).
 */
void ADC1_Mode_Config(void)
{
    ADC_InitTypeDef   ADC_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* rank 1: first in the (single-channel) sequence */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    /* EOC interrupt: notified when a conversion completes */
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    /* calibration is mandatory; clock must be <= 14 MHz (set above) */
    ADC_Cmd(ADC1, ENABLE);

    /* wait for ADC power-up (tSTAB ~1 us) */
    {
        volatile uint32_t t = 10000;
        while (t--);
    }

    /* reset calibration (timeout-guarded) */
    ADC_ResetCalibration(ADC1);
    {
        volatile uint32_t cal_timeout = 100000;
        while (ADC_GetResetCalibrationStatus(ADC1) && --cal_timeout);
    }

    /* run calibration (timeout-guarded) */
    ADC_StartCalibration(ADC1);
    {
        volatile uint32_t cal_timeout = 100000;
        while (ADC_GetCalibrationStatus(ADC1) && --cal_timeout);
    }

    /* ADC1/ADC2 share the ADC1_2_IRQn vector */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
