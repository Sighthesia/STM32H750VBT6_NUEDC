/** ***************************************************************************
 * @File Name: TIM_STM32HAL.c
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
#include "TIM_STM32HAL.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == SERVO_TIM_INSTANCE)
    {
#ifdef USE_SERVO_J1
        if (num <= (int)(Angle_J1 * 11.0f + 500.0f) / 10)
        {
            servo_yahboom_hal.GPIO_SetBits(Servo_J1_PORT, Servo_J1_PIN); // 将舵机接口电平置高
        }
        else
        {
            servo_yahboom_hal.GPIO_ResetBits(Servo_J1_PORT, Servo_J1_PIN); // 将舵机接口电平置低
        }
#endif

#ifdef USE_SERVO_J2
        if (num <= (int)(Angle_J2 * 11.0f + 500.0f) / 10)
        {
            servo_yahboom_hal.GPIO_SetBits(Servo_J2_PORT, Servo_J2_PIN); // 将舵机接口电平置高
        }
        else
        {
            servo_yahboom_hal.GPIO_ResetBits(Servo_J2_PORT, Servo_J2_PIN); // 将舵机接口电平置低
        }
#endif

#ifdef USE_SERVO_J3
        if (num <= (int)(Angle_J3 * 11.0f + 500.0f) / 10)
        {
            servo_yahboom_hal.GPIO_SetBits(Servo_J3_PORT, Servo_J3_PIN); // 将舵机接口电平置高
        }
        else
        {
            servo_yahboom_hal.GPIO_ResetBits(Servo_J3_PORT, Servo_J3_PIN); // 将舵机接口电平置低
        }
#endif

#ifdef USE_SERVO_J4
        if (num <= (int)(Angle_J4 * 11.0f + 500.0f) / 10)
        {
            servo_yahboom_hal.GPIO_SetBits(Servo_J4_PORT, Servo_J4_PIN); // 将舵机接口电平置高
        }
        else
        {
            servo_yahboom_hal.GPIO_ResetBits(Servo_J4_PORT, Servo_J4_PIN); // 将舵机接口电平置低
        }
#endif

#ifdef USE_SERVO_J5
        if (num <= (int)(Angle_J5 * 11.0f + 500.0f) / 10)
        {
            servo_yahboom_hal.GPIO_SetBits(Servo_J5_PORT, Servo_J5_PIN); // 将舵机接口电平置高
        }
        else
        {
            servo_yahboom_hal.GPIO_ResetBits(Servo_J5_PORT, Servo_J5_PIN); // 将舵机接口电平置低
        }
#endif

#ifdef USE_SERVO_J6
        if (num <= (int)(Angle_J6 * 11.0f + 500.0f) / 10)
        {
            servo_yahboom_hal.GPIO_SetBits(Servo_J6_PORT, Servo_J6_PIN); // 将舵机接口电平置高
        }
        else
        {
            servo_yahboom_hal.GPIO_ResetBits(Servo_J6_PORT, Servo_J6_PIN); // 将舵机接口电平置低
        }
#endif

        if (num == 2000) // 2000*10us=20ms  20ms一个周期
        {
            num = 0;
        }
        else
        {
            num++; // 每次中断递增计数器
        }
    }
    if (htim->Instance == PID_TIM_INSTANCE)
    {
        // PID控制相关代码
        pid_calculate(&x_pid, kfp_x.out);
        pid_calculate(&y_pid, kfp_y.out);
    }
}

/** ***************************************************************************
 * @brief 	  舵机PWM定时器初始化函数
 */
void Servo_PWM_TIM_Init()
{
    // 清除定时器初始化过程中的更新中断标志，避免定时器一启动就中断
    __HAL_TIM_CLEAR_IT(&SERVO_TIM_HANDLE, TIM_IT_UPDATE);
    // 使能定时器更新中断并启动定时器
    HAL_TIM_Base_Start_IT(&SERVO_TIM_HANDLE);
}

/** ***************************************************************************
 * @brief 	  PID定时器初始化函数
 */
void PID_TIM_Init()
{
    // 清除定时器初始化过程中的更新中断标志，避免定时器一启动就中断
    __HAL_TIM_CLEAR_IT(&PID_TIM_HANDLE, TIM_IT_UPDATE);
    // 使能定时器更新中断并启动定时器
    HAL_TIM_Base_Start_IT(&PID_TIM_HANDLE);
}
