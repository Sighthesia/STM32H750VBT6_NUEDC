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
#include <math.h> // 用于 fabsf 函数

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

float yaw_speed = 0.0f;   // 航向轴步进电机速度（单位：RPM）
float pitch_speed = 0.0f; // 俯仰轴步进电机速度（单位：RPM）

float pid_yaw_angle = 0.0f;   // 航向轴PID输出角度（单位：度）
float pid_pitch_angle = 0.0f; // 俯仰轴PID输出角度（单位：度）

PIDController yaw_pid = {0};   // 航向轴PID控制器结构体
PIDController pitch_pid = {0}; // 俯仰轴PID控制器结构体

float yaw_pid_output = 0.0f;   // 航向轴PID输出值
float pitch_pid_output = 0.0f; // 俯仰轴PID输出值

// PID目标值
float yaw_pid_target = 0.0f;
float pitch_pid_target = 0.0f;

// PID误差值
float yaw_pid_error = 0.0f;
float pitch_pid_error = 0.0f;

/* --------------------------------- 系统状态标志位（内部使用） -------------------------------- */
_Bool stepper_update_pending = 0; // 步进电机更新待处理标志位（系统设置）
/* --------------------------------- 自定义部分结束 -------------------------------- */

// 中间变量
int16_t pid_kp_slider_val = 20;           // 比例系数滑块值（2.0f -> 20*0.1）
int16_t pid_ki_slider_val = 0;            // 积分系数滑块值（0.0f -> 0*0.1）
int16_t pid_kd_slider_val = 0;            // 微分系数滑块值（0.0f -> 0*0.1）
int16_t pid_tau_slider_val = 2;           // 滤波时间常数滑块值（0.02f -> 2*0.01）
int16_t pid_lim_min_slider_val = -1;      // 输出下限滑块值（-10.0f -> -100*0.1）
int16_t pid_lim_max_slider_val = 1;       // 输出上限滑块值（10.0f -> 100*0.1）
int16_t pid_lim_min_int_slider_val = -50; // 积分下限滑块值（-5.0f -> -50*0.1）
int16_t pid_lim_max_int_slider_val = 50;  // 积分上限滑块值（5.0f -> 50*0.1）
int16_t pid_sample_time_slider_val = 1;   // 采样时间滑块值（0.01f -> 1*0.01）

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

    /* --------------------------------- 此处修改: 条件调试输出 -------------------------------- */
    if (menu_pid_params_debug)
    {
        printf("[PID Params] Kp=%.2f, Ki=%.2f, Kd=%.2f, Tau=%.3f, Min=%.2f, Max=%.2f, MinInt=%.2f, MaxInt=%.2f, T=%.3f\r\n",
               pid_kp, pid_ki, pid_kd, pid_tau, pid_lim_min, pid_lim_max, pid_lim_min_int, pid_lim_max_int, sample_time_s);
    }
    /* --------------------------------- 自定义部分结束 -------------------------------- */
}

/** ***************************************************************************
 * @brief   更新航向轴步进电机位置，负值顺时针，正值逆时针
 * @param   pid_output: PID输出值
 */
void Yaw_Stepper_Position_Update(float pid_output)
{
    uint8_t dir = (pid_output >= 0) ? 0x00 : 0x01; // 0x01: CCW(逆时针), 0x00: CW(顺时针)
    float speed = fabsf(pid_output);
    pid_yaw_angle = pid_output;

    /* --------------------------------- 此处修改: 条件调试输出 -------------------------------- */
    if (menu_stepper_command_debug)
    {
        printf("[Yaw] Dir: %s, Speed: %.2f, Angle: %.2f, Output: %.2f\r\n",
               dir ? "CCW" : "CW", speed, pid_yaw_angle, pid_output);
    }

    // 内存屏障，防止编译器过度优化
    __asm volatile("" ::: "memory");

    /* --------------------------------- 自定义部分结束 -------------------------------- */
    ZDT_Stepper_Set_Speed(zdt_stepper_motors[0], dir, speed, ZDT_STEPPER_REL_POS_MODE, ZDT_STEPPER_SYNC_DISABLE);
}

/** ***************************************************************************
 * @brief   更新俯仰轴步进电机位置，负值逆时针，正值顺时针
 * @param   pid_output: PID输出值
 */
void Pitch_Stepper_Position_Update(float pid_output)
{
    uint8_t dir = (pid_output >= 0) ? 0x01 : 0x00; // 0x00: CW(顺时针), 0x01: CCW(逆时针)
    float speed = fabsf(pid_output);
    pid_pitch_angle = pid_output;

    /* --------------------------------- 此处修改: 条件调试输出 -------------------------------- */
    if (menu_stepper_command_debug)
    {
        printf("[Pitch] Dir: %s, Speed: %.2f, Angle: %.2f, Output: %.2f\r\n",
               dir ? "CCW" : "CW", speed, pid_pitch_angle, pid_output);
    }

    // 内存屏障，防止编译器过度优化
    __asm volatile("" ::: "memory");

    /* --------------------------------- 自定义部分结束 -------------------------------- */
    ZDT_Stepper_Set_Speed(zdt_stepper_motors[1], dir, speed, ZDT_STEPPER_REL_POS_MODE, ZDT_STEPPER_SYNC_DISABLE);
}