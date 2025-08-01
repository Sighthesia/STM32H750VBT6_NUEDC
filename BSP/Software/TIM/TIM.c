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

/* --------------------------------- 系统状态标志位（内部使用） -------------------------------- */
_Bool tim_pid_calculation_debug = 0; // TIM模块：PID计算调试输出标志位（系统设置）
/* --------------------------------- 自定义部分结束 -------------------------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        /* -------------------------------- 菜单更新编码器值 -------------------------------- */
        if (menu_pid_params_update_enable == 1)
        {
            // 同步PID滑块到浮点数
            sync_pid_slider_to_float();
            Stepper_PID_Parameters_Init(&yaw_pid);   // 同步航向轴PID参数
            Stepper_PID_Parameters_Init(&pitch_pid); // 同步俯仰轴PID参数
        }

        /* ----------------------------------- 解包 ----------------------------------- */
        if (menu_uart_data_receive_enable) // 允许解包
        {
            uint8_t data;
            while (lwrb_read(&uart3_rx_buffer, &data, 1) > 0)
            {
                UART_Parse_Packet(data); // 调用解包函数
            }
        }

        /* ---------------------------------- 计算PID --------------------------------- */
        if (uart_packet_received)
        {
            yaw_pid_output = PIDController_Update(&yaw_pid, yaw_pid_target, (float)rectangle_coordinate_arr[0]);
            pitch_pid_output = PIDController_Update(&pitch_pid, pitch_pid_target, (float)rectangle_coordinate_arr[1]);
            if (menu_pid_calculation_debug)
            {
                printf("[PID Calc] Target: Yaw=%.2f, Pitch=%.2f | Current: Yaw=%.2f, Pitch=%.2f\r\n",
                       yaw_pid_target, pitch_pid_target,
                       (float)rectangle_coordinate_arr[0], (float)rectangle_coordinate_arr[1]);
            }

            uart_packet_received = 0;   // 清零标志，等待下次接收
            stepper_update_pending = 1; // 设置PID更新标志位
        }

        /* --------------------------------- 步进电机跟踪 --------------------------------- */
        if (menu_stepper_tracking_enable == 1) // 如果步进电机使能标志位为1
        {
            if (stepper_update_pending)
            {
                // 更新步进电机位置
                Yaw_Stepper_Position_Update(yaw_pid_output);
                Pitch_Stepper_Position_Update(pitch_pid_output);
            }

            if (menu_pid_calculation_debug)
            {
                printf("[PID Output] Yaw: %.2f, Pitch: %.2f\r\n", yaw_pid_output, pitch_pid_output);
            }

            stepper_update_pending = 0; // 清零标志，等待下次更新
        }

        ebtn_APP_Key_Process(); // 处理按键事件
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
