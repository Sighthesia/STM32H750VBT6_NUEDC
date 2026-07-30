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
#include <math.h> // 用于 fabsf 函数

/* --------------------------------- 系统状态标志位（内部使用） -------------------------------- */
_Bool tim_pid_calculation_debug = 0; // TIM模块：PID计算调试输出标志位（系统设置）

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

        /* --------------------------------- 此处修改: 添加超时自动回零检测 -------------------------------- */
        // 检测目标丢失超时，触发自动回零
        if (target_lost && auto_zero_return_enable && !auto_zero_return_triggered)
        {
            uint32_t current_time = HAL_GetTick();
            if ((current_time - target_lost_start_time) >= auto_zero_return_timeout_ms)
            {
                printf("[Auto Zero] Target lost for %lums, triggering auto zero return\r\n",
                       current_time - target_lost_start_time);

                // 触发唯一PITCH轴回零
                ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[0], 0, 0);

                auto_zero_return_triggered = 1; // 设置已触发标志，避免重复触发
                printf("[Auto Zero] Zero return command sent to pitch motor\r\n");
            }
        }

        /* ---------------------------------- 计算PID --------------------------------- */
        if (uart_packet_received)
        {
            // 计算当前误差
            yaw_pid_error = yaw_pid_target - (float)rectangle_coordinate_arr[0];
            pitch_pid_error = pitch_pid_target - (float)rectangle_coordinate_arr[1];

            if (target_lost)
            {
                // 目标丢失时的处理策略
                if (menu_pid_calculation_debug)
                {
                    printf("[PID] Target lost, using last valid coordinates\r\n");
                }

                // 策略1：继续使用上一次有效坐标进行控制（当前选择）
                // 添加误差死区，减少小幅振荡
                if (fabsf(yaw_pid_error) > error_deadzone_threshold)
                {
                    yaw_pid_output = PIDController_Update(&yaw_pid, yaw_pid_target, (float)rectangle_coordinate_arr[0]);
                }
                else
                {
                    yaw_pid_output = 0.0f; // 在误差死区内，输出为0
                }

                if (fabsf(pitch_pid_error) > error_deadzone_threshold)
                {
                    pitch_pid_output = PIDController_Update(&pitch_pid, pitch_pid_target, (float)rectangle_coordinate_arr[1]);
                }
                else
                {
                    pitch_pid_output = 0.0f; // 在误差死区内，输出为0
                }

                // 可选策略2：停止PID控制，保持当前位置
                // yaw_pid_output = 0.0f;
                // pitch_pid_output = 0.0f;

                // 可选策略3：逐渐回到中心位置
                // yaw_pid_output = PIDController_Update(&yaw_pid, 0.0f, (float)rectangle_coordinate_arr[0]);
                // pitch_pid_output = PIDController_Update(&pitch_pid, 0.0f, (float)rectangle_coordinate_arr[1]);
            }
            else
            {
                // 目标正常，执行正常PID控制
                // 添加误差死区，减少小幅振荡
                if (fabsf(yaw_pid_error) > error_deadzone_threshold)
                {
                    yaw_pid_output = PIDController_Update(&yaw_pid, yaw_pid_target, (float)rectangle_coordinate_arr[0]);
                }
                else
                {
                    yaw_pid_output = 0.0f; // 在误差死区内，输出为0
                }

                if (fabsf(pitch_pid_error) > error_deadzone_threshold)
                {
                    pitch_pid_output = PIDController_Update(&pitch_pid, pitch_pid_target, (float)rectangle_coordinate_arr[1]);
                }
                else
                {
                    pitch_pid_output = 0.0f; // 在误差死区内，输出为0
                }
            }

            if (menu_pid_calculation_debug)
            {
                printf("[PID Calc] Target: Yaw=%.2f, Pitch=%.2f | Current: Yaw=%.2f, Pitch=%.2f | Status: %s\r\n",
                       yaw_pid_target, pitch_pid_target,
                       (float)rectangle_coordinate_arr[0], (float)rectangle_coordinate_arr[1],
                       target_lost ? "LOST" : "OK");
                printf("[PID Calc] Error: Yaw=%.2f, Pitch=%.2f | Output: Yaw=%.3f, Pitch=%.3f\r\n",
                       yaw_pid_error, pitch_pid_error, yaw_pid_output, pitch_pid_output);
            }

            // 调试信息：坐标差值详细信息
            if (debug_coordinate_info)
            {
                static uint32_t last_coord_debug_time = 0;
                uint32_t current_time = HAL_GetTick();

                // 每200ms输出一次坐标信息
                if ((current_time - last_coord_debug_time) >= 200)
                {
                    float total_error = sqrtf(yaw_pid_error * yaw_pid_error + pitch_pid_error * pitch_pid_error);
                    printf("[坐标差值] X轴偏差: %.2f | Y轴偏差: %.2f | 总偏差: %.2f\r\n",
                           yaw_pid_error, pitch_pid_error, total_error);
                    printf("[坐标信息] 目标坐标: (%.1f, %.1f) | 当前坐标: (%d, %d)\r\n",
                           yaw_pid_target, pitch_pid_target,
                           rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
                    last_coord_debug_time = current_time;
                }
            }

            // PID激光检查与发射
            PID_Laser_Check_And_Fire();

            uart_packet_received = 0;   // 清零标志，等待下次接收
            stepper_update_pending = 1; // 设置PID更新标志位
        }

        /* --------------------------------- 步进电机跟踪 --------------------------------- */
        if (menu_stepper_tracking_enable == 1) // 如果步进电机使能标志位为1
        {
            if (stepper_update_pending)
            {
                float yaw_feedforward;
                float pitch_feedforward;
                float yaw_total;
                float pitch_total;

                Stepper_Feedforward_Get_Output(&yaw_feedforward, &pitch_feedforward);
                yaw_total = fminf(fmaxf(yaw_pid_output + yaw_feedforward, yaw_pid.limMin), yaw_pid.limMax);
                pitch_total = fminf(fmaxf(pitch_pid_output + pitch_feedforward, pitch_pid.limMin), pitch_pid.limMax);
                Stepper_Feedforward_Set_Total_Output(yaw_total, pitch_total);
                // 更新步进电机位置
                Yaw_Stepper_Position_Update(yaw_total);
                Pitch_Stepper_Position_Update(pitch_total);

                if (menu_pid_calculation_debug)
                {
                    printf("[PID Output] Yaw: %.2f, Pitch: %.2f\r\n", yaw_pid_output, pitch_pid_output);
                }

                stepper_update_pending = 0; // 清零标志，等待下次更新
            }
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
