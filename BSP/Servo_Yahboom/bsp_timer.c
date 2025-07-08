/**
 * @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
 * @file         bsp_timer.h
 * @author       liusen
 * @version      V1.0
 * @date         2017.07.17
 * @brief        定时器
 * @details
 * @par History  见如下说明
 *
 * version:		liusen_20170717
 */

#include "bsp_timer.h"

/**
 * Function       TIM1_UP_IRQHandler
 * @author        liusen (Modified for STM32H743)
 * @date          2025.07.08
 * @brief         定时器1中断服务程序: 主要控制6路舵机运行 - HAL库版本
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   适配STM32H743IIT6，每10us中断一次，2000次为20ms周期
 */
int num = 0;

void TIM1_UP_IRQHandler(void) // TIM1中断
{
	if (__HAL_TIM_GET_FLAG(&TIMx, TIM_FLAG_UPDATE) != RESET) // 检查TIM1更新中断发生与否
	{
		__HAL_TIM_CLEAR_IT(&TIMx, TIM_IT_UPDATE); // 清除TIM1更新中断标志
		num++;

#ifdef USE_SERVO_J1
		if (num <= (Angle_J1 * 11 + 500) / 10)
		{
			HAL_GPIO_WritePin(Servo_J1_PORT, Servo_J1_PIN, GPIO_PIN_SET); // 将舵机接口电平置高
		}
		else
		{
			HAL_GPIO_WritePin(Servo_J1_PORT, Servo_J1_PIN, GPIO_PIN_RESET); // 将舵机接口电平置低
		}
#endif

#ifdef USE_SERVO_J2
		if (num <= (Angle_J2 * 11 + 500) / 10)
		{
			HAL_GPIO_WritePin(Servo_J2_PORT, Servo_J2_PIN, GPIO_PIN_SET); // 将舵机接口电平置高
		}
		else
		{
			HAL_GPIO_WritePin(Servo_J2_PORT, Servo_J2_PIN, GPIO_PIN_RESET); // 将舵机接口电平置低
		}
#endif

#ifdef USE_SERVO_J3
		if (num <= (Angle_J3 * 11 + 500) / 10)
		{
			HAL_GPIO_WritePin(Servo_J3_PORT, Servo_J3_PIN, GPIO_PIN_SET); // 将舵机接口电平置高
		}
		else
		{
			HAL_GPIO_WritePin(Servo_J3_PORT, Servo_J3_PIN, GPIO_PIN_RESET); // 将舵机接口电平置低
		}
#endif

#ifdef USE_SERVO_J4
		if (num <= (Angle_J4 * 11 + 500) / 10)
		{
			HAL_GPIO_WritePin(Servo_J4_PORT, Servo_J4_PIN, GPIO_PIN_SET); // 将舵机接口电平置高
		}
		else
		{
			HAL_GPIO_WritePin(Servo_J4_PORT, Servo_J4_PIN, GPIO_PIN_RESET); // 将舵机接口电平置低
		}
#endif

#ifdef USE_SERVO_J5
		if (num <= (Angle_J5 * 11 + 500) / 10)
		{
			HAL_GPIO_WritePin(Servo_J5_PORT, Servo_J5_PIN, GPIO_PIN_SET); // 将舵机接口电平置高
		}
		else
		{
			HAL_GPIO_WritePin(Servo_J5_PORT, Servo_J5_PIN, GPIO_PIN_RESET); // 将舵机接口电平置低
		}
#endif

#ifdef USE_SERVO_J6
		if (num <= (Angle_J6 * 11 + 500) / 10)
		{
			HAL_GPIO_WritePin(Servo_J6_PORT, Servo_J6_PIN, GPIO_PIN_SET); // 将舵机接口电平置高
		}
		else
		{
			HAL_GPIO_WritePin(Servo_J6_PORT, Servo_J6_PIN, GPIO_PIN_RESET); // 将舵机接口电平置低
		}
#endif

		if (num == 2000) // 2000*10us=20ms  20ms一个周期
		{
			num = 0;
		}
	}
}
