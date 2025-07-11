/**
 * @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
 * @file         bsp_servo.h
 * @author
 * @version      V1.0
 * @date
 * @brief        24个舵机控制驱动头文件
 * @details
 * @par History  见如下说明
 *
 *
 */

#ifndef __BSP_SERVO_H__
#define __BSP_SERVO_H__

#include "main.h"
#include "Servo_Yahboom_HAL.h"
#include "Delay_us_STM32HAL.h"

/*定义需要初始化的舵机宏定义开关*/
#define USE_SERVO_J1
#define USE_SERVO_J2
// #define USE_SERVO_J3
// #define USE_SERVO_J4
// #define USE_SERVO_J5
// #define USE_SERVO_J6
// #define USE_SERVO_J7
// #define USE_SERVO_J8
// #define USE_SERVO_J9
// #define USE_SERVO_J10
// #define USE_SERVO_J11
// #define USE_SERVO_J12
// #define USE_SERVO_J13
// #define USE_SERVO_J14
// #define USE_SERVO_J15
// #define USE_SERVO_J16
// #define USE_SERVO_J17
// #define USE_SERVO_J18
// #define USE_SERVO_J19
// #define USE_SERVO_J20
// #define USE_SERVO_J21
// #define USE_SERVO_J22
// #define USE_SERVO_J23
// #define USE_SERVO_J24

/*舵机数量配置*/
#define GROUP_NUM 1 // 舵机组数量
#define SERVO_NUM 2 // 每组舵机数量

/*舵机引脚定义 - 条件编译*/

#ifdef USE_SERVO_J1
#define Servo_J1_PIN GPIO_PIN_4
#define Servo_J1_PORT GPIOC
#endif

#ifdef USE_SERVO_J2
#define Servo_J2_PIN GPIO_PIN_5
#define Servo_J2_PORT GPIOC
#endif

#ifdef USE_SERVO_J3
#define Servo_J3_PIN GPIO_PIN_2
#define Servo_J3_PORT GPIOA
#endif

#ifdef USE_SERVO_J4
#define Servo_J4_PIN GPIO_PIN_3
#define Servo_J4_PORT GPIOA
#endif

#ifdef USE_SERVO_J5
#define Servo_J5_PIN GPIO_PIN_4
#define Servo_J5_PORT GPIOB
#endif

#ifdef USE_SERVO_J6
#define Servo_J6_PIN GPIO_PIN_5
#define Servo_J6_PORT GPIOB
#endif

#ifdef USE_SERVO_J7
#define Servo_J7_PIN GPIO_PIN_0
#define Servo_J7_PORT GPIOB
#endif

#ifdef USE_SERVO_J8
#define Servo_J8_PIN GPIO_PIN_1
#define Servo_J8_PORT GPIOB
#endif

#ifdef USE_SERVO_J9
#define Servo_J9_PIN GPIO_PIN_15
#define Servo_J9_PORT GPIOA
#endif

#ifdef USE_SERVO_J10
#define Servo_J10_PIN GPIO_PIN_3
#define Servo_J10_PORT GPIOB
#endif

#ifdef USE_SERVO_J11
#define Servo_J11_PIN GPIO_PIN_10
#define Servo_J11_PORT GPIOB
#endif

#ifdef USE_SERVO_J12
#define Servo_J12_PIN GPIO_PIN_11
#define Servo_J12_PORT GPIOB
#endif

#ifdef USE_SERVO_J13
#define Servo_J13_PIN GPIO_PIN_6
#define Servo_J13_PORT GPIOB
#endif

#ifdef USE_SERVO_J14
#define Servo_J14_PIN GPIO_PIN_7
#define Servo_J14_PORT GPIOB
#endif

#ifdef USE_SERVO_J15
#define Servo_J15_PIN GPIO_PIN_8
#define Servo_J15_PORT GPIOB
#endif

#ifdef USE_SERVO_J16
#define Servo_J16_PIN GPIO_PIN_9
#define Servo_J16_PORT GPIOB
#endif

#ifdef USE_SERVO_J17
#define Servo_J17_PIN GPIO_PIN_6
#define Servo_J17_PORT GPIOC
#endif

#ifdef USE_SERVO_J18
#define Servo_J18_PIN GPIO_PIN_7
#define Servo_J18_PORT GPIOC
#endif

#ifdef USE_SERVO_J19
#define Servo_J19_PIN GPIO_PIN_8
#define Servo_J19_PORT GPIOC
#endif

#ifdef USE_SERVO_J20
#define Servo_J20_PIN GPIO_PIN_9
#define Servo_J20_PORT GPIOC
#endif

#ifdef USE_SERVO_J21
#define Servo_J21_PIN GPIO_PIN_8
#define Servo_J21_PORT GPIOA
#endif

#ifdef USE_SERVO_J22
#define Servo_J22_PIN GPIO_PIN_14
#define Servo_J22_PORT GPIOB
#endif

#ifdef USE_SERVO_J23
#define Servo_J23_PIN GPIO_PIN_15
#define Servo_J23_PORT GPIOB
#endif

#ifdef USE_SERVO_J24
#define Servo_J24_PIN GPIO_PIN_11
#define Servo_J24_PORT GPIOA
#endif

typedef struct
{
	uint16_t pwm;
	uint16_t MAXPWM;
	uint16_t MINPWM;
	uint16_t MIDPWM;
} ServTypdef;

/*角度数组声明 - 支持多组舵机，volatile修饰保证多线程/中断安全*/
extern volatile int Angle_J[GROUP_NUM][SERVO_NUM];

/*传统单独角度变量（向后兼容）*/
#define Angle_J1 Angle_J[0][0]
#define Angle_J2 Angle_J[0][1]
#define Angle_J3 Angle_J[0][2]
#define Angle_J4 Angle_J[0][3]
#define Angle_J5 Angle_J[0][4]
#define Angle_J6 Angle_J[0][5]
#define Angle_J7 Angle_J[0][6]
#define Angle_J8 Angle_J[0][7]

extern ServTypdef Xserv, Yserv;
void servo_ctr(ServTypdef *servo, int PidInput);

/*舵机控制函数声明 - 条件编译*/
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

#ifdef USE_SERVO_J7
void Servo_J7(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif

#ifdef USE_SERVO_J8
void Servo_J8(int v_iAngle); /*定义一个脉冲函数，用来模拟方式产生PWM值*/
#endif

// 9-24路舵机自行增加即可，本工程不采用以上的方法，而是使用定时器的方式进行。

void front_detection(void);
void left_detection(void);
void right_detection(void);

#endif
