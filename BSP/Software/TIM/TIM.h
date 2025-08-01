/** ***************************************************************************
 * @File Name: TIM.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-10
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __TIM_STM32HAL_H
#define __TIM_STM32HAL_H

#include "main.h"

/* -------------------------------- 包含所用的宏定义 -------------------------------- */
#include "tim.h"
#include "ebtn_APP.h"

#include "usart.h"
#include "stdio.h"

#include "USART.h"
#include "Stepper_PID.h"
/* ------------------------------- 定义所使用的定时器句柄 ------------------------------ */

extern _Bool pid_cal_out_debug_flag;

#ifdef USE_SOFTWARE_PWM_TIM
#define SERVO_TIM_HANDLE htim6  /* 舵机使用的定时器句柄 */
#define SERVO_TIM_INSTANCE TIM6 /* 舵机使用的定时器 */
void Servo_Software_PWM_TIM_Init(void);
#endif /* USE_SOFTWARE_PWM_TIM */

void PID_TIM_Init(void);

#endif /* USE_PID_TIM */