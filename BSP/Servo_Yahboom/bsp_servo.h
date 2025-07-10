/**
 * @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
 * @file         bsp_servo.h
 * @author       liusen
 * @version      V1.0
 * @date         2017.07.17
 * @brief        6个舵机控制驱动头文件
 * @details
 * @par History  见如下说明
 *
 * version:		liusen_20170717
 */
#ifndef __BSP_SERVO_H__
#define __BSP_SERVO_H__

#include "Servo_Yahboom_HAL.h"
#include "Delay_us_STM32HAL.h"
#include "tim.h"

/* ------------------------------- 定义所使用的定时器句柄 ------------------------------ */
// htim6.Instance = TIM6;  // 句柄指向外设实例

#define SERVO_TIM_HANDLE htim6  /* 舵机使用的定时器句柄 */
#define SERVO_TIM_INSTANCE TIM6 /* 舵机使用的定时器 */

/* ----------------------------- 定义需要初始化的舵机宏定义开关 ---------------------------- */

#define USE_SERVO_J1
#define USE_SERVO_J2
// #define USE_SERVO_J3
// #define USE_SERVO_J4
// #define USE_SERVO_J5
// #define USE_SERVO_J6

/* ------------------------------ 定义舵机PWM引脚和端口 ------------------------------ */

#ifdef USE_SERVO_J1
#define Servo_J1_PIN GPIO_PIN_4
#endif
#ifdef USE_SERVO_J2
#define Servo_J2_PIN GPIO_PIN_5
#endif
#ifdef USE_SERVO_J3
#define Servo_J3_PIN GPIO_PIN_14
#endif
#ifdef USE_SERVO_J4
#define Servo_J4_PIN GPIO_PIN_15
#endif
#ifdef USE_SERVO_J5
#define Servo_J5_PIN GPIO_PIN_8
#endif
#ifdef USE_SERVO_J6
#define Servo_J6_PIN GPIO_PIN_1
#endif

#ifdef USE_SERVO_J1
#define Servo_J1_PORT GPIOC
#endif
#ifdef USE_SERVO_J2
#define Servo_J2_PORT GPIOC
#endif
#ifdef USE_SERVO_J3
#define Servo_J3_PORT GPIOB
#endif
#ifdef USE_SERVO_J4
#define Servo_J4_PORT GPIOB
#endif
#ifdef USE_SERVO_J5
#define Servo_J5_PORT GPIOA
#endif
#ifdef USE_SERVO_J6
#define Servo_J6_PORT GPIOA
#endif

/* --------------------------------- 自定义部分结束 -------------------------------- */

#ifdef USE_SERVO_J1
extern volatile float Angle_J1;
#endif
#ifdef USE_SERVO_J2
extern volatile float Angle_J2;
#endif
#ifdef USE_SERVO_J3
extern volatile float Angle_J3;
#endif
#ifdef USE_SERVO_J4
extern volatile float Angle_J4;
#endif
#ifdef USE_SERVO_J5
extern volatile float Angle_J5;
#endif
#ifdef USE_SERVO_J6
extern volatile float Angle_J6;
#endif

#ifdef USE_SERVO_J1
void Servo_J1(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif
#ifdef USE_SERVO_J2
void Servo_J2(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif
#ifdef USE_SERVO_J3
void Servo_J3(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif
#ifdef USE_SERVO_J4
void Servo_J4(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif
#ifdef USE_SERVO_J5
void Servo_J5(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif
#ifdef USE_SERVO_J6
void Servo_J6(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif

void front_detection(void);
void left_detection(void);
void right_detection(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); /* 舵机定时器中断回调函数 */

#endif
