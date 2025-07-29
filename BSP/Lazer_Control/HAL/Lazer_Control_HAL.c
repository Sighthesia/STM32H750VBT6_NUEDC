/** ***************************************************************************
 * @File Name: Lazer_Control_HAL.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-29
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "Lazer_Control_HAL.h"

void Move_Lazer_x(uint16_t x)
{
    Servo_Set_Angle(SERVO_0X, (float)x);
}

void Move_Lazer_y(uint16_t y)
{
    Servo_Set_Angle(SERVO_0Y, (float)y);
}

lazer_control_hal_t lazer_control_hal = {
    .Move_Lazer_x = Move_Lazer_x,
    .Move_Lazer_y = Move_Lazer_y};