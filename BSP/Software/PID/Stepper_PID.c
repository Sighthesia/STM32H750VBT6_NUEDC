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

/* --------------------------------- 此处修改: PID参数变量定义与初始化 -------------------------------- */
float pid_kp = 2.0f;
float pid_ki = 0.0f;
float pid_kd = 0.0f;
float pid_tau = 0.02f;
float pid_lim_min = -10.0f;
float pid_lim_max = 10.0f;
float pid_lim_min_int = -5.0f;
float pid_lim_max_int = 5.0f;
float sample_time_s = 0.01f;
/* --------------------------------- 自定义部分结束 -------------------------------- */

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

void Stepper_PID_Parameters_Init(PIDController *pid)
{
    // 初始化PID参数
    pid->Kp = pid_kp;
    pid->Ki = pid_ki;
    pid->Kd = pid_kd;
    pid->tau = pid_tau;
    pid->limMin = pid_lim_min;
    pid->limMax = pid_lim_max;
    pid->limMinInt = pid_lim_min_int;
    pid->limMaxInt = pid_lim_max_int;
    pid->T = sample_time_s;
}

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
