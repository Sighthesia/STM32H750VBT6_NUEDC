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
    if (htim->Instance == PID_TIM_INSTANCE)
    {
        // PID控制相关代码
        pid_calculate(&x_pid, kfp_x.out);
        pid_calculate(&y_pid, kfp_y.out);
    }
}

#ifdef USE_TIM_PWM
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
#endif /* USE_TIM_PWM */

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
