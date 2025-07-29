/** ***************************************************************************
 * @File Name: Servo_APP.h
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

#ifndef __SERVO_APP_H
#define __SERVO_APP_H

#include "Servo_HAL.h"

// 舵机数量定义
#define MAX_SERVO 2

// 舵机ID枚举
typedef enum
{
    SERVO_0X = 0, // 舵机1
    SERVO_0Y,     // 舵机2
} servo_id_t;

// 舵机角度类型枚举
typedef enum
{
    SERVO_ANGLE_TYPE_180 = 0,
    SERVO_ANGLE_TYPE_270 = 1
} servo_angle_type_t;

// 舵机参数结构体
typedef struct
{
    TIM_HandleTypeDef *tim_handle; // 定时器句柄指针
    uint32_t tim_channel;          // 定时器通道
    uint8_t servo_id;              // 舵机ID
    const char *name;              // 舵机名称
    servo_angle_type_t angle_type; // 舵机角度类型
    uint8_t enabled;               // 舵机是否启用
    uint16_t pwm;                  // 当前PWM值 (微秒)
    uint16_t mid_pwm;              // 中间PWM值 (微秒)
    uint16_t min_pwm;              // 最小PWM值 (微秒)
    uint16_t max_pwm;              // 最大PWM值 (微秒)
} servo_config_t;

/** ***************************************************************************
 * @brief 初始化舵机模块
 */
void Servo_Init(void);

/** ***************************************************************************
 * @brief 按ID设置舵机角度 (支持180/270度)
 * @param servo_id 舵机ID
 * @param angle 角度值 (float, 支持0-180/270度)
 */
void Servo_Set_Angle(uint8_t servo_id, float angle);

/** ***************************************************************************
 * @brief 按ID设置舵机PWM值
 * @param servo_id 舵机ID
 * @param pwm PWM值(微秒)
 */
void Servo_Set_PWM(uint8_t servo_id, uint16_t pwm);

/** ***************************************************************************
 * @brief 启动所有已启用的舵机
 */
void Servo_Start_All(void);

/** ***************************************************************************
 * @brief 停止所有舵机
 */
void Servo_Stop_All(void);

/** ***************************************************************************
 * @brief PWM值转换为角度 (支持180/270度舵机)
 * @param  servo_id: 舵机ID
 * @param  pwm: PWM值 (微秒)
 */
float PWM_To_Angle(uint8_t servo_id, uint16_t pwm);

#endif /*  __SERVO_APP_H */