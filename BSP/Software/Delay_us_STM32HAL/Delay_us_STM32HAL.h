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
#include "main.h"
#include "tim.h"

/* -------------------------------- 此处选择延时方式 -------------------------------- */
#define USE_SYSTICK_TIME_SOURCE // 使用 SysTick 定时器作为延时源
// #define USE_TIM_TIME_SOURCE // 使用 TIM 定时器作为延时源
// 定时器设置为1us中断一次

/* -------------------------- 此处更改SYSTICK方式所用单片机主频 -------------------------- */
#ifdef USE_SYSTICK_TIME_SOURCE
#define SYSTEM_CLOCK_MHZ_FREQUENCY 480 // 480MHz / 1000 = 480000 ticks per ms
#endif                                 /* USE_SYSTICK_TIME_SOURCE */

/* ----------------------------- 此处更改定时器方式所用定时器 ----------------------------- */
#ifdef USE_TIM_TIME_SOURCE
#define TIM_INSTANCE TIM13 // 定时器实例
#define TIM_HANDLE htim13  // 定时器句柄
#endif                     /* USE_TIM_TIME_SOURCE */

/* --------------------------------- 更改部分结束 --------------------------------- */

void Delay_us(uint32_t us);
void Delay_us_Timer_Start(void);
void Delay_us_Timer_Stop(void);
uint8_t Delay_us_Timer_IsRunning(void);
void Delay_us_Timer_Reset(void);

#endif /* __DELAY_US_STM32HAL_H */