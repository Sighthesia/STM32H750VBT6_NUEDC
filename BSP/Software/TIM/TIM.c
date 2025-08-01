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
        if (pid_debug_flag == 1)
        {
            // 同步PID滑块到浮点数
            sync_pid_slider_to_float();
            Stepper_PID_Parameters_Init(&yaw_pid);   // 同步航向轴PID参数
            Stepper_PID_Parameters_Init(&pitch_pid); // 同步俯仰轴PID参数
        }
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
        if (pid_update_flag)
        {
            yaw_pid_output = PIDController_Update(&yaw_pid, yaw_pid_target, (float)rectangle_coordinate_arr[0]);
            pitch_pid_output = PIDController_Update(&pitch_pid, pitch_pid_target, (float)rectangle_coordinate_arr[1]);
#ifdef TIM_DEBUG
            printf("Target: Yaw=%f, Pitch=%f | Current: Yaw=%f, Pitch=%f\n",
                   yaw_pid_target, pitch_pid_target,
                   (float)rectangle_coordinate_arr[0], (float)rectangle_coordinate_arr[1]);
#endif // TIM_DEBUG

            pid_update_flag = 0;     // 清零标志，等待下次接收
            stepper_update_flag = 1; // 设置PID更新标志位
        }

        if (stepper_enable_flag == 1) // 如果步进电机使能标志位为1
        {
            if (stepper_update_flag)
            {
                // 更新步进电机位置
                Yaw_Stepper_Position_Update(yaw_pid_output);
                Pitch_Stepper_Position_Update(pitch_pid_output);
            }

#ifdef TIM_DEBUG
            printf("PID Update: Yaw: %d, Pitch: %d\n", yaw_pid_output, pitch_pid_output);
#endif // TIM_DEBUG

            stepper_update_flag = 0; // 清零标志，等待下次更新
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
