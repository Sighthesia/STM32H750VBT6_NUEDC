/** ***************************************************************************
 * @File Name: Lazer_Control_HAL.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-29
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __LAZER_CONTROL_HAL_H
#define __LAZER_CONTROL_HAL_H

#include "stdint.h"

#include "Servo_APP.h"

typedef struct
{
    void (*Move_Lazer_x)(uint16_t x); // 移动激光到指定坐标
    void (*Move_Lazer_y)(uint16_t y);
} lazer_control_hal_t;

extern lazer_control_hal_t lazer_control_hal;

#endif // __LAZER_CONTROL_HAL_H
