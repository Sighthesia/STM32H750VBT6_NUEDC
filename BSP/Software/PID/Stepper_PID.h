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

/* Controller parameters */
/**
 * @brief 比例增益系数
 * @note 用于计算比例项输出，值越大系统响应越快，但过大会导致系统不稳定
 */
#define PID_KP 2.0f

/**
 * @brief 积分增益系数
 * @note 用于消除稳态误差，值越大消除误差越快，但过大会导致系统振荡
 */
// #define PID_KI 0.5f
#define PID_KI 0.0f

/**
 * @brief 微分增益系数
 * @note 用于预测误差变化趋势，提高系统响应速度和稳定性
 */
// #define PID_KD 0.25f
#define PID_KD 0.0f

/**
 * @brief 微分项低通滤波时间常数
 * @note 用于对微分项进行滤波，减少噪声影响
 */
#define PID_TAU 0.02f

/**
 * @brief PID输出最小值限制
 * @note 限制控制器输出的最小值，防止执行器过度响应
 */
#define PID_LIM_MIN -10.0f

/**
 * @brief PID输出最大值限制
 * @note 限制控制器输出的最大值，防止执行器过度响应
 */
#define PID_LIM_MAX 10.0f

/**
 * @brief 积分项最小值限制
 * @note 用于抗积分饱和，限制积分项的最小值
 */
#define PID_LIM_MIN_INT -5.0f

/**
 * @brief 积分项最大值限制
 * @note 用于抗积分饱和，限制积分项的最大值
 */
#define PID_LIM_MAX_INT 5.0f

/**
 * @brief 控制器采样周期(秒)
 * @note 控制算法的执行周期，影响积分和微分项的计算
 */

#define SAMPLE_TIME_S 0.01f
// PID目标值
extern uint16_t yaw_pid_target;
extern uint16_t pitch_pid_target;

// PID误差值
extern uint16_t yaw_pid_error;
extern uint16_t pitch_pid_error;

extern PIDController yaw_pid;   // 航向轴PID控制器
extern PIDController pitch_pid; // 俯仰轴PID控制器

void Yaw_Stepper_Position_Update(float pid_output);

void Pitch_Stepper_Position_Update(float pid_output);

#endif /* __STEPPER_PID_H */