/** ***************************************************************************
 * @File Name: Stepper_PID.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-31
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "Stepper_PID.h"

float yaw_speed = 1.0f;
float pitch_speed = 1.0f;

float pid_yaw_angle = 0.0f;
float pid_pitch_angle = 0.0f;

PIDController yaw_pid = {}; // 航向轴PID

PIDController pitch_pid = {}; // 俯仰轴PID

// PID目标值
uint16_t yaw_pid_target = 0;
uint16_t pitch_pid_target = 0;

// PID误差值
uint16_t yaw_pid_error = 0;
uint16_t pitch_pid_error = 0;

void Yaw_Stepper_Position_Update(float pid_output)
{
    pid_yaw_angle = pid_output;

    ZDT_Stepper_Set_position(zdt_stepper_motors[STEPPER_YAW], 00, yaw_speed, pid_yaw_angle, ZDT_STEPPER_REL_POS_MODE, ZDT_STEPPER_SYNC_DISABLE);
}

void Pitch_Stepper_Position_Update(float pid_output)
{
    pid_pitch_angle = pid_output;

    ZDT_Stepper_Set_position(zdt_stepper_motors[STEPPER_PITCH], 00, pitch_speed, pid_pitch_angle, ZDT_STEPPER_REL_POS_MODE, ZDT_STEPPER_SYNC_DISABLE);
}
