#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

/**
 * @file    adc.h
 * @brief   ADC1 single-channel driver (PA1 / CH1) with EOC interrupt.
 */

/** Latest conversion result, updated in ADC1_2_IRQHandler. */
extern volatile uint16_t adc_value;

/** Conversion-ready flag: set by ISR, cleared by main loop. */
extern volatile uint8_t  adc_conversion_done;

/** @brief Configure PA1 as analog input (and ADC clock <= 14 MHz). */
void ADC_GPIO_Config(void);
/** @brief Configure ADC1 mode, EOC interrupt, calibration and NVIC. */
void ADC1_Mode_Config(void);

#endif /* __ADC_H */
