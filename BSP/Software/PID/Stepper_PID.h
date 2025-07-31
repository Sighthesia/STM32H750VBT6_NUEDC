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

extern PIDController yaw_pid;   // 航向轴PID控制器
extern PIDController pitch_pid; // 俯仰轴PID控制器

#endif /* __STEPPER_PID_H */