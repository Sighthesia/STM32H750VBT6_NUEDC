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
float pid_kp = 0.0f;
float pid_ki = 0.0f;
float pid_kd = 0.0f;
float pid_tau = 0.0f;
float pid_lim_min = 0.0f;
float pid_lim_max = 0.0f;
float pid_lim_min_int = 0.0f;
float pid_lim_max_int = 0.0f;
float sample_time_s = 0.0f;
/* --------------------------------- 自定义部分结束 -------------------------------- */

float yaw_speed = 1.0f;
float pitch_speed = 1.0f;

float pid_yaw_angle = 0.0f;
float pid_pitch_angle = 0.0f;

PIDController yaw_pid = {0};   // 航向轴PID
PIDController pitch_pid = {0}; // 俯仰轴PID

float yaw_pid_output = 0.0f;   // 航向轴PID输出
float pitch_pid_output = 0.0f; // 俯仰轴PID输出

// PID目标值
float yaw_pid_target = 0.0f;
float pitch_pid_target = 0.0f;

// PID误差值
float yaw_pid_error = 0.0f;
float pitch_pid_error = 0.0f;

_Bool stepper_update_flag = 0; // PID更新标志位
_Bool pid_debug_flag = 0;      // PID调试标志位

// 中间变量
int16_t pid_kp_slider_val = 20;           // 2.0f -> 20*0.1
int16_t pid_ki_slider_val = 0;            // 0.0f -> 0*0.1
int16_t pid_kd_slider_val = 0;            // 0.0f -> 0*0.1
int16_t pid_tau_slider_val = 2;           // 0.02f -> 2*0.01
int16_t pid_lim_min_slider_val = -100;    // -10.0f -> -100*0.1
int16_t pid_lim_max_slider_val = 100;     // 10.0f -> 100*0.1
int16_t pid_lim_min_int_slider_val = -50; // -5.0f -> -50*0.1
int16_t pid_lim_max_int_slider_val = 50;  // 5.0f -> 50*0.1
int16_t pid_sample_time_slider_val = 1;   // 0.01f -> 1*0.01

/** ***************************************************************************
 * @brief  初始化步进电机PID参数
 * @param  pid: 指向PID控制器的指针
 */
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

/** ***************************************************************************
 * @brief   同步PID滑块值到浮点数
 */
void sync_pid_slider_to_float(void)
{
    pid_kp = pid_kp_slider_val * 0.1f;
    pid_ki = pid_ki_slider_val * 0.1f;
    pid_kd = pid_kd_slider_val * 0.1f;
    pid_tau = pid_tau_slider_val * 0.01f;
    pid_lim_min = pid_lim_min_slider_val * 0.1f;
    pid_lim_max = pid_lim_max_slider_val * 0.1f;
    pid_lim_min_int = pid_lim_min_int_slider_val * 0.1f;
    pid_lim_max_int = pid_lim_max_int_slider_val * 0.1f;
    sample_time_s = pid_sample_time_slider_val * 0.01f;
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