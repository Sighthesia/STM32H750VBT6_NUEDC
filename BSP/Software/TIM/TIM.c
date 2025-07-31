/** ***************************************************************************
 * @File Name: TIM.c
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
#include "TIM.h"
#include "usart.h"
#include <stdio.h>

#include "Stepper_PID.h"
#include "USART.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        ebtn_APP_Key_Process(); // 处理按键事件

        // 只有接收完成标志为1时才更新PID
        if (uart_rx_done_flag)
        {
            yaw_pid_output = PIDController_Update(&yaw_pid, yaw_pid_target, rectangle_coordinate_arr[0]);
            pitch_pid_output = PIDController_Update(&pitch_pid, pitch_pid_target, rectangle_coordinate_arr[1]);
            uart_rx_done_flag = 0; // 清零标志，等待下次接收
        }
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
