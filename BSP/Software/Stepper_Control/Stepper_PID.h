/** ***************************************************************************
 * @File Name: Stepper_PID.h
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
#ifndef __STEPPER_PID_H
#define __STEPPER_PID_H

#include "stdint.h"
#include "stm32h7xx_hal.h" // 用于 HAL_GetTick()

#include "PID.h"
#include "ZDT_Stepper.h"

#define STEPPER_PID_DEBUG

/* --------------------------------- 此处修改: 宏定义改为全局变量 -------------------------------- */
/**
 * @brief 比例增益系数
 * @note 用于计算比例项输出，值越大系统响应越快，但过大会导致系统不稳定
 */
extern float pid_kp;

/**
 * @brief 积分增益系数
 * @note 用于消除稳态误差，值越大消除误差越快，但过大会导致系统振荡
 */
extern float pid_ki;

/**
 * @brief 微分增益系数
 * @note 用于预测误差变化趋势，提高系统响应速度和稳定性
 */
extern float pid_kd;

/**
 * @brief 微分项低通滤波时间常数
 * @note 用于对微分项进行滤波，减少噪声影响
 */
extern float pid_tau;

/**
 * @brief PID输出最小值限制
 * @note 限制控制器输出的最小值，防止执行器过度响应
 */
extern float pid_lim_min;

/**
 * @brief PID输出最大值限制
 * @note 限制控制器输出的最大值，防止执行器过度响应
 */
extern float pid_lim_max;

/**
 * @brief 积分项最小值限制
 * @note 用于抗积分饱和，限制积分项的最小值
 */
extern float pid_lim_min_int;

/**
 * @brief 积分项最大值限制
 * @note 用于抗积分饱和，限制积分项的最大值
 */
extern float pid_lim_max_int;

/**
 * @brief 控制器采样周期(秒)
 * @note 控制算法的执行周期，影响积分和微分项的计算
 */
extern float sample_time_s;

/**
 * @brief 步进电机死区阈值
 * @note 小于此值的PID输出将被忽略，消除微小震荡，单位：RPM
 */
extern float stepper_deadzone_threshold;
/* --------------------------------- 自定义部分结束 -------------------------------- */

extern float yaw_pid_output;   // 航向轴PID输出
extern float pitch_pid_output; // 俯仰轴PID输出

// PID目标值
extern float yaw_pid_target;
extern float pitch_pid_target;

// PID误差值
extern float yaw_pid_error;
extern float pitch_pid_error;

extern PIDController yaw_pid;   // 航向轴PID控制器
extern PIDController pitch_pid; // 俯仰轴PID控制器

/* --------------------------------- 系统状态标志位（内部使用） -------------------------------- */
extern _Bool stepper_update_pending; // 步进电机更新待处理标志位（系统设置）
/* --------------------------------- 自定义部分结束 -------------------------------- */

/* --------------------------------- 用户控制标志位（在main.c中定义） -------------------------------- */
extern _Bool menu_stepper_tracking_enable;  // 菜单控制：步进电机跟踪使能
extern _Bool menu_uart_data_receive_enable; // 菜单控制：UART数据接收使能
extern _Bool menu_pid_params_update_enable; // 菜单控制：PID参数实时更新使能
extern _Bool menu_pid_calculation_debug;    // 菜单控制：PID计算过程调试输出
extern _Bool menu_stepper_command_debug;    // 菜单控制：步进电机指令调试输出
extern _Bool menu_pid_params_debug;         // 菜单控制：PID参数更新调试输出
/* --------------------------------- 自定义部分结束 -------------------------------- */

extern int16_t pid_kp_slider_val;
extern int16_t pid_ki_slider_val;
extern int16_t pid_kd_slider_val;
extern int16_t pid_tau_slider_val;
extern int16_t pid_lim_min_slider_val;
extern int16_t pid_lim_max_slider_val;
extern int16_t pid_lim_min_int_slider_val;
extern int16_t pid_lim_max_int_slider_val;
extern int16_t pid_sample_time_slider_val;
extern int16_t stepper_deadzone_slider_val; // 步进电机死区阈值滑块值
extern int16_t error_deadzone_slider_val;   // 误差死区滑块值
extern int16_t update_interval_slider_val;  // 更新间隔滑块值
extern int16_t output_change_slider_val;    // 输出变化阈值滑块值

extern int16_t pitch_pid_target_slider_val; // 俯仰轴目标值滑块值（0.0f -> 0*1）
extern int16_t yaw_pid_target_slider_val;   // 航向轴目标值滑块值（0.0f -> 0*1）

// PID激光控制滑块变量
extern int16_t pid_laser_threshold_slider_val; // PID激光阈值滑块值（5.0f -> 50*0.1）
extern int16_t pid_laser_duration_slider_val;  // PID激光持续时间滑块值（50ms -> 50*1）

// 控制参数（浮点数）
extern float error_deadzone_threshold;    // 误差死区阈值（单位：像素）
extern uint32_t min_update_interval_ms;   // 最小更新间隔（单位：ms）
extern float min_output_change_threshold; // 最小输出变化阈值

void Yaw_Stepper_Position_Update(float pid_output);

void Pitch_Stepper_Position_Update(float pid_output);

void Stepper_PID_Parameters_Init(PIDController *pid);

void sync_pid_slider_to_float(void);

#endif /* __STEPPER_PID_H */