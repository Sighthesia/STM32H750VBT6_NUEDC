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

#include "PID.h"
#include "stdint.h"

// PID目标值
extern uint16_t yaw_pid_target;
extern uint16_t pitch_pid_target;

// PID误差值
extern uint16_t yaw_pid_error;
extern uint16_t pitch_pid_error;

extern PIDController yaw_pid;   // 航向轴PID控制器
extern PIDController pitch_pid; // 俯仰轴PID控制器

#endif /* __STEPPER_PID_H */