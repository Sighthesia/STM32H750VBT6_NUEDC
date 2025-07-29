#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#define ANGLE_PWM_K 2.7777777777f

extern float Voltage;
// extern float PTZ_NowAngle,PTZ_TargetAngle;
// extern uint8_t PTZ_ControlStep,select_mode,target_reach_flag;

extern float real_arm_angle;
extern float real_base_angle;

extern float step_base;
extern float step_arm;

void Draw(void);

extern float ts_dis; // 激光笔在UV纸板的投射距离，单位是米

#endif
