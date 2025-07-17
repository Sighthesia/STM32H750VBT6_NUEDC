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
#include "usart.h"
#include <stdio.h>

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == PID_TIM_INSTANCE)
    {
        /* --------------------------------- 此处修改暂时取消整个PID计算 -------------------------------- */
        // PID控制相关代码 - 暂时完全注释，直接用坐标控制舵机
        // pid_calculate(&x_pid, kfp_x.out);  // 原来使用滤波后的值
        // pid_calculate(&y_pid, kfp_y.out);  // 原来使用滤波后的值

        // 直接使用原始值进行PID计算，跳过卡尔曼滤波
        // pid_calculate(&x_pid, kfp_x.source); // 使用原始输入值
        // pid_calculate(&y_pid, kfp_y.source); // 使用原始输入值
        /* 注释部分结束 */

        ebtn_APP_Key_Process();
        /* --------------------------------- 此处修改中断串口打印 -------------------------------- */
        {
            char uart_buf[64];
            int uart_len = snprintf(uart_buf, sizeof(uart_buf),
                                    "Direct Control Mode - No PID\r\n");
            HAL_UART_Transmit_IT(&huart1, (uint8_t *)uart_buf, uart_len);
        }
        /* 自定义部分结束 */
    }
}

#ifdef USE_SOFTWARE_PWM_TIM
/** ***************************************************************************
 * @brief 	  舵机PWM定时器初始化函数
 */
void Servo_Software_PWM_TIM_Init(void)
{
    // 清除定时器初始化过程中的更新中断标志，避免定时器一启动就中断
    __HAL_TIM_CLEAR_IT(&SERVO_TIM_HANDLE, TIM_IT_UPDATE);
    // 使能定时器更新中断并启动定时器
    HAL_TIM_Base_Start_IT(&SERVO_TIM_HANDLE);
}
#endif /* USE_SOFTWARE_PWM_TIM */

/** ***************************************************************************
 * @brief 	  PID定时器初始化函数
 */
void PID_TIM_Init(void)
{
    // 清除定时器初始化过程中的更新中断标志，避免定时器一启动就中断
    __HAL_TIM_CLEAR_IT(&PID_TIM_HANDLE, TIM_IT_UPDATE);
    // 使能定时器更新中断并启动定时器
    HAL_TIM_Base_Start_IT(&PID_TIM_HANDLE);
}
