/**
  ******************************************************************************
  * @file    stm32f10x_conf.h
  * @brief   STM32 标准外设库 — 全局配置文件
  *
  *  作用:
  *    1. 决定项目中包含哪些外设库的头文件
  *    2. 配置 assert_param 宏 (调试断言)
  *
  *  本实验实际只需要 gpio, rcc, misc 三个外设,
  *  但保留全部 include 以避免编译其他库文件时缺少头文件.
  ******************************************************************************
  */

#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* ========== 外设头文件 ========== */
#include "stm32f10x_adc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_can.h"
#include "stm32f10x_cec.h"
#include "stm32f10x_crc.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_wwdg.h"
#include "misc.h"

/* ========== 断言配置 ========== */
/* #define USE_FULL_ASSERT    1 */  /* 取消注释以启用参数断言检查 (调试用) */

#ifdef  USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */
