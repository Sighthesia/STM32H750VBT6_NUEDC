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

_Bool menu_stepper_tracking_enable;
_Bool menu_uart_data_receive_enable;
_Bool menu_pid_params_update_enable;
_Bool menu_pid_calculation_debug;
_Bool menu_stepper_command_debug;
_Bool menu_pid_params_debug;
static float pid_laser_threshold;
static uint32_t pid_laser_duration_ms;

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
float stepper_deadzone_threshold = 0.0f;  // 步进电机死区阈值（单位：RPM）
float error_deadzone_threshold = 5.0f;    // 误差死区阈值（单位：像素）
uint32_t min_update_interval_ms = 50;     // 最小更新间隔（单位：ms）
float min_output_change_threshold = 0.1f; // 最小输出变化阈值
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
_Bool stepper_update_pending = 0;    // 步进电机更新待处理标志位（系统设置）
uint32_t last_yaw_update_time = 0;   // 航向轴上次更新时间（ms）
uint32_t last_pitch_update_time = 0; // 俯仰轴上次更新时间（ms）
float last_yaw_output = 0.0f;        // 航向轴上次输出值
float last_pitch_output = 0.0f;      // 俯仰轴上次输出值
static _Bool yaw_stepper_running = 0;
static _Bool pitch_stepper_running = 0;
static uint8_t last_yaw_direction = ZDT_STEPPER_CCW;
static uint8_t last_pitch_direction = ZDT_STEPPER_CCW;
/* --------------------------------- 自定义部分结束 -------------------------------- */

// 中间变量
int16_t pid_kp_slider_val = 3;            // 比例系数滑块值（2.0f -> 20*0.1）
int16_t pid_ki_slider_val = 1;            // 积分系数滑块值（0.0f -> 0*0.1）
int16_t pid_kd_slider_val = 0;            // 微分系数滑块值（0.0f -> 0*0.1）
int16_t pid_tau_slider_val = 5;           // 滤波时间常数滑块值（0.02f -> 2*0.01）
int16_t pid_lim_min_slider_val = -4;      // 输出下限滑块值（-10.0f -> -100*0.1）
int16_t pid_lim_max_slider_val = 4;       // 输出上限滑块值（10.0f -> 100*0.1）
int16_t pid_lim_min_int_slider_val = -50; // 积分下限滑块值（-5.0f -> -50*0.1）
int16_t pid_lim_max_int_slider_val = 50;  // 积分上限滑块值（5.0f -> 50*0.1）
int16_t pid_sample_time_slider_val = 1;   // 采样时间滑块值（0.01f -> 1*0.01）
int16_t stepper_deadzone_slider_val = 20; // 死区阈值滑块值（1.0f -> 10*0.1）
int16_t error_deadzone_slider_val = 50;   // 误差死区滑块值（5.0f -> 50*0.1）
int16_t update_interval_slider_val = 50;  // 更新间隔滑块值（50ms -> 50*1）
int16_t output_change_slider_val = 10;    // 输出变化阈值滑块值（0.1f -> 10*0.01）
int16_t pitch_pid_target_slider_val = 20; // 俯仰轴目标值滑块值（0.0f -> 0*1）
int16_t yaw_pid_target_slider_val = 10;   // 航向轴目标值滑块值（0.0f -> 0*1）

// PID激光控制滑块变量定义
int16_t pid_laser_threshold_slider_val = 50; // PID激光阈值滑块值（5.0f -> 50*0.1）
int16_t pid_laser_duration_slider_val = 50;  // PID激光持续时间滑块值（50ms -> 50*1）

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
    pid_kp = pid_kp_slider_val;
    pid_ki = pid_ki_slider_val;
    pid_kd = pid_kd_slider_val;
    pid_tau = pid_tau_slider_val * 0.01f;
    pid_lim_min = pid_lim_min_slider_val * 0.1f;
    pid_lim_max = pid_lim_max_slider_val * 0.1f;
    pid_lim_min_int = pid_lim_min_int_slider_val * 0.1f;
    pid_lim_max_int = pid_lim_max_int_slider_val * 0.1f;
    sample_time_s = pid_sample_time_slider_val * 0.01f;
    stepper_deadzone_threshold = stepper_deadzone_slider_val * 0.01f;
    error_deadzone_threshold = error_deadzone_slider_val * 0.1f;
    min_update_interval_ms = update_interval_slider_val;
    min_output_change_threshold = output_change_slider_val * 0.01f;
    pitch_pid_target = pitch_pid_target_slider_val * 1.0f;
    yaw_pid_target = yaw_pid_target_slider_val * 1.0f;

    // PID激光控制参数同步
    pid_laser_threshold = pid_laser_threshold_slider_val * 0.1f;
    pid_laser_duration_ms = pid_laser_duration_slider_val;

    /* --------------------------------- 此处修改: 条件调试输出 -------------------------------- */
    if (menu_pid_params_debug)
    {
        printf("[PID Params] Kp=%.2f, Ki=%.2f, Kd=%.2f, Tau=%.3f, Min=%.2f, Max=%.2f, MinInt=%.2f, MaxInt=%.2f, T=%.3f\r\n",
               pid_kp, pid_ki, pid_kd, pid_tau, pid_lim_min, pid_lim_max, pid_lim_min_int, pid_lim_max_int, sample_time_s);
        printf("[Control Params] Deadzone=%.2f, ErrorDeadzone=%.1f, UpdateInterval=%lums, OutputChange=%.3f\r\n",
               stepper_deadzone_threshold, error_deadzone_threshold, min_update_interval_ms, min_output_change_threshold);
    }
    /* --------------------------------- 自定义部分结束 -------------------------------- */
}

/** ***************************************************************************
 * @brief   更新航向轴步进电机位置，负值顺时针，正值逆时针
 * @param   pid_output: PID输出值
 */
void Yaw_Stepper_Position_Update(float pid_output)
{
    uint32_t current_time = HAL_GetTick(); // 获取当前时间戳
    float speed = fabsf(pid_output);
    uint8_t dir = (pid_output >= 0) ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW;
    _Bool in_deadzone = speed <= stepper_deadzone_threshold;

    // 进入死区必须立即停止，不受正常速度更新节流限制。
    if (in_deadzone)
    {
        if (yaw_stepper_running)
        {
            ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
            yaw_stepper_running = 0;
            if (menu_stepper_command_debug)
            {
                printf("[Yaw] Entered deadzone, motor stopped\r\n");
            }
        }
        last_yaw_update_time = current_time;
        last_yaw_output = pid_output;
        return;
    }

    // 频率和幅值节流只适用于已在运行的速度更新。
    if (yaw_stepper_running && dir == last_yaw_direction &&
        ((current_time - last_yaw_update_time) < min_update_interval_ms ||
         fabsf(pid_output - last_yaw_output) < min_output_change_threshold))
    {
        if (menu_stepper_command_debug)
        {
            printf("[Yaw] Speed update skipped\r\n");
        }
        return;
    }

    pid_yaw_angle = pid_output;
    ZDT_Stepper_Set_Speed(zdt_stepper_motors[0], dir, speed,
                          ZDT_STEPPER_SPEED_ACCEL, ZDT_STEPPER_SYNC_DISABLE);
    yaw_stepper_running = 1;
    last_yaw_direction = dir;
    last_yaw_update_time = current_time;
    last_yaw_output = pid_output;

    if (menu_stepper_command_debug)
    {
        printf("[Yaw] Dir: %s, Speed: %.2f, Angle: %.2f, Output: %.2f\r\n",
               dir == ZDT_STEPPER_CCW ? "CCW" : "CW", speed, pid_yaw_angle, pid_output);
    }
}

/** ***************************************************************************
 * @brief   更新俯仰轴步进电机位置，负值逆时针，正值顺时针
 * @param   pid_output: PID输出值
 */
void Pitch_Stepper_Position_Update(float pid_output)
{
    uint32_t current_time = HAL_GetTick(); // 获取当前时间戳
    float speed = fabsf(pid_output);
    uint8_t dir = (pid_output >= 0) ? ZDT_STEPPER_CW : ZDT_STEPPER_CCW;
    _Bool in_deadzone = speed <= stepper_deadzone_threshold;

    // 进入死区必须立即停止，不受正常速度更新节流限制。
    if (in_deadzone)
    {
        if (pitch_stepper_running)
        {
            ZDT_Stepper_stop(zdt_stepper_motors[1], ZDT_STEPPER_SYNC_DISABLE);
            pitch_stepper_running = 0;
            if (menu_stepper_command_debug)
            {
                printf("[Pitch] Entered deadzone, motor stopped\r\n");
            }
        }
        last_pitch_update_time = current_time;
        last_pitch_output = pid_output;
        return;
    }

    // 频率和幅值节流只适用于已在运行的速度更新。
    if (pitch_stepper_running && dir == last_pitch_direction &&
        ((current_time - last_pitch_update_time) < min_update_interval_ms ||
         fabsf(pid_output - last_pitch_output) < min_output_change_threshold))
    {
        if (menu_stepper_command_debug)
        {
            printf("[Pitch] Speed update skipped\r\n");
        }
        return;
    }

    pid_pitch_angle = pid_output;
    ZDT_Stepper_Set_Speed(zdt_stepper_motors[1], dir, speed,
                          ZDT_STEPPER_SPEED_ACCEL, ZDT_STEPPER_SYNC_DISABLE);
    pitch_stepper_running = 1;
    last_pitch_direction = dir;
    last_pitch_update_time = current_time;
    last_pitch_output = pid_output;

    if (menu_stepper_command_debug)
    {
        printf("[Pitch] Dir: %s, Speed: %.2f, Angle: %.2f, Output: %.2f\r\n",
               dir == ZDT_STEPPER_CCW ? "CCW" : "CW", speed, pid_pitch_angle, pid_output);
    }
}
