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
#include "Servo_APP.h"

uint8_t button_count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        ebtn_APP_Key_Process();

        Servo_Update_All_PWM();
        printf("Servo PWM Updated\n");
        printf("X Servo: %d\n", servo_list[SERVO_0X].pwm);
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
