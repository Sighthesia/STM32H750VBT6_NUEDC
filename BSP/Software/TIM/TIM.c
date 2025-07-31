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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        ebtn_APP_Key_Process(); // 处理按键事件

        if (uart_rx_enable_flag) // 允许解包
        {
            uint8_t data;
            while (lwrb_read(&uart3_rx_buffer, &data, 1) > 0)
            {
                UART_Parse_Packet(data); // 这里调用解包函数
            }
        }

        // 只有接收完成标志为1时才更新PID
        if (uart_rx_done_flag)
        {
            yaw_pid_output = PIDController_Update(&yaw_pid, yaw_pid_target, (float)rectangle_coordinate_arr[0]);
            pitch_pid_output = PIDController_Update(&pitch_pid, pitch_pid_target, (float)rectangle_coordinate_arr[1]);
            uart_rx_done_flag = 0; // 清零标志，等待下次接收
        }

        if (pid_update_done_flag)
        {
            // 更新步进电机位置
            Yaw_Stepper_Position_Update(yaw_pid_output);
            Pitch_Stepper_Position_Update(pitch_pid_output);
            pid_update_done_flag = 0; // 清零标志，等待下次更新
        }
        sync_pid_slider_to_float();
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
