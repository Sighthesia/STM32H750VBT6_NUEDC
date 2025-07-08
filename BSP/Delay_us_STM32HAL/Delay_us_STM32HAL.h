/** ***************************************************************************
 * @File Name: Delay_us_STM32HAL.h
 * @brief 利用 STM32HAL 库 SysTick实现微秒级延时
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-08
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __DELAY_US_STM32HAL_H
#define __DELAY_US_STM32HAL_H

/* ------------------------------- 此处更改所用平台头文件 ------------------------------ */
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"

/* ------------------------------- 此处更改所用单片机主频 ------------------------------ */
#define SYSTEM_CLOCK_MHZ_FREQUENCY 480 // 480MHz / 1000 = 480000 ticks per ms

void delay_us(uint32_t udelay);

#endif /* __DELAY_US_STM32HAL_H */