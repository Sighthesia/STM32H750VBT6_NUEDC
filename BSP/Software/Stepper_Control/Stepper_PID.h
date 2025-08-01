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

extern _Bool stepper_update_flag;        // PID更新标志位
extern _Bool stepper_update_enable_flag; // 步进电机使能标志位
extern _Bool stepper_pid_debug_flag;     // PID调试标志位
extern _Bool pid_debug_enable_flag;      // PID调试使能标志位

extern int16_t pid_kp_slider_val;
extern int16_t pid_ki_slider_val;
extern int16_t pid_kd_slider_val;
extern int16_t pid_tau_slider_val;
extern int16_t pid_lim_min_slider_val;
extern int16_t pid_lim_max_slider_val;
extern int16_t pid_lim_min_int_slider_val;
extern int16_t pid_lim_max_int_slider_val;
extern int16_t pid_sample_time_slider_val;

void Yaw_Stepper_Position_Update(float pid_output);

void Pitch_Stepper_Position_Update(float pid_output);

void Stepper_PID_Parameters_Init(PIDController *pid);

void sync_pid_slider_to_float(void);

#endif /* __STEPPER_PID_H */