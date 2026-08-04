#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

/*
 * adc_value - 全局 ADC 转换结果
 *
 * 由 ADC1_2_IRQHandler 在中断中更新，
 * 主循环读取这个变量获取最新转换值。
 */
extern volatile uint16_t adc_value;

/*
 * adc_conversion_done - 通知主循环 "数据已就绪"
 *
 * ISR 中置 1，主循环检测到后清零。
 */
extern volatile uint8_t adc_conversion_done;

void ADC_GPIO_Config(void);
void ADC1_Mode_Config(void);

#endif /* __ADC_H */
