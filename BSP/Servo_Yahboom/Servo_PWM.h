/** ***************************************************************************
 * @File_Name: Servo_PWM.h
 * @brief
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-13
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author: Sighthesia
 * @Changes:
 * @Modifi_Date:
 */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include "main.h"
#include "tim.h"
#include "stdint.h"
#include "Servo_HAL.h"
#include "Servo_Config.h"

/* --------------------------------- 舵机参数结构体 --------------------------------- */
typedef struct
{
    uint16_t pwm;                  // 当前PWM值 (微秒)
    uint16_t max_pwm;              // 最大PWM值 (微秒)
    uint16_t min_pwm;              // 最小PWM值 (微秒)
    uint16_t mid_pwm;              // 中间PWM值 (微秒)
    uint32_t tim_channel;          // 定时器通道
    TIM_HandleTypeDef *tim_handle; // 定时器句柄指针
    uint8_t servo_id;              // 舵机ID (用于识别)
    uint8_t group_id;              // 舵机组别ID
    uint8_t enabled;               // 舵机是否启用 (0=禁用, 1=启用)
    servo_angle_type_t angle_type; // 舵机角度类型 (180°/270°/360°)
    const char *name;              // 舵机名称 (用于调试)
} servo_typedef_t;

/* --------------------------------- 舵机配置表结构体 --------------------------------- */
typedef struct
{
    TIM_HandleTypeDef *tim_handle; // 定时器句柄指针
    uint32_t tim_channel;          // 定时器通道
    uint8_t servo_id;              // 舵机ID
    uint8_t group_id;              // 舵机组别ID
    uint8_t enabled;               // 是否启用
    servo_angle_type_t angle_type; // 舵机角度类型 (180°/270°/360°)
    const char *name;              // 舵机名称
    uint16_t default_pwm;          // 默认PWM值
    uint16_t max_pwm;              // 最大PWM值
    uint16_t min_pwm;              // 最小PWM值
    uint16_t mid_pwm;              // 中位PWM值
} servo_config_t;

/* --------------------------------- 外部变量声明 --------------------------------- */
extern servo_typedef_t servo_array[MAX_SERVO];           // 舵机控制数组
extern volatile float servo_angle[MAX_GROUP][MAX_SERVO]; // 角度数组支持小数

/* ---------------------------------- 函数声明 ---------------------------------- */
// 初始化函数
void Servo_PWM_TIM_Init(void);

// 按ID控制函数
void Servo_Set_Angle_By_ID(uint8_t servo_id, float angle);
void Servo_Set_PWM_By_ID(uint8_t servo_id, uint16_t pwm);

// 按组别控制函数
void Servo_Set_Angle_By_Group(uint8_t group_id, uint8_t servo_index, float angle);
void Servo_Set_PWM_By_Group(uint8_t group_id, uint8_t servo_index, uint16_t pwm);
void Servo_Set_Group_Angles(uint8_t group_id, float *angles, uint8_t count);
void Servo_Set_Group_PWMs(uint8_t group_id, uint16_t *pwms, uint8_t count);

// 批量控制函数
void Servo_Set_Multiple_Angles(uint8_t *servo_ids, float *angles, uint8_t count);
void Servo_Set_Multiple_PWMs(uint8_t *servo_ids, uint16_t *pwms, uint8_t count);

// 启用/禁用控制
void Servo_Enable_By_ID(uint8_t servo_id, uint8_t enable);
void Servo_Enable_By_Group(uint8_t group_id, uint8_t enable);
void Servo_Set_Config_By_ID(uint8_t servo_id, uint16_t min_pwm, uint16_t max_pwm, uint16_t mid_pwm);

// 启动/停止控制
void Servo_Stop_All(void);
void Servo_Start_All(void);
void Servo_Stop_By_ID(uint8_t servo_id);
void Servo_Start_By_ID(uint8_t servo_id);
void Servo_Stop_By_Group(uint8_t group_id);
void Servo_Start_By_Group(uint8_t group_id);

// 辅助函数
uint16_t Servo_Angle_To_PWM(float angle);
float Servo_PWM_To_Angle(uint16_t pwm_value);

/* --------------------------------- 此处修改角度类型适配函数 --------------------------------- */
// 角度类型适配函数
uint16_t Servo_Angle_To_PWM_By_Type(float angle, servo_angle_type_t angle_type);
float Servo_PWM_To_Angle_By_Type(uint16_t pwm_value, servo_angle_type_t angle_type);

// 角度类型设置和获取函数
void Servo_Set_Angle_Type_By_ID(uint8_t servo_id, servo_angle_type_t angle_type);
servo_angle_type_t Servo_Get_Angle_Type_By_ID(uint8_t servo_id);
float Servo_Get_Max_Angle_By_ID(uint8_t servo_id);

// 高级功能函数
void Servo_Smooth_Move_By_ID(uint8_t servo_id, float target_angle, uint16_t step_delay);
void Servo_Calibrate_By_ID(uint8_t servo_id);
void Servo_Calibrate_By_Group(uint8_t group_id);
void Servo_Calibrate_All(void);
void Servo_Test(void);
void Servo_Print_Status(void);

/* --------------------------------- 此处修改小数角度辅助函数 --------------------------------- */
// 小数角度辅助函数
float Servo_Get_Current_Angle_By_ID(uint8_t servo_id);
void Servo_Set_Angle_With_Precision(uint8_t servo_id, float angle, float precision);
void Servo_Adjust_Angle_By_ID(uint8_t servo_id, float delta_angle);
float Servo_Clamp_Angle(float angle, float min_angle, float max_angle);

// 获取舵机数量信息
uint8_t Servo_Get_Max_Configured_Count(void);
uint8_t Servo_Get_Active_Count(void);
uint8_t Servo_Get_Group_Count(uint8_t group_id);
uint8_t Servo_Get_Max_Group_Count(void);
servo_typedef_t *Servo_Get_Group_Array(uint8_t group_id, uint8_t *count);

// 多定时器支持函数
TIM_HandleTypeDef *Servo_Get_TIM_Handle_By_ID(uint8_t servo_id);
uint8_t Servo_Get_Servo_Count_By_TIM(TIM_HandleTypeDef *tim_handle);
void Servo_Set_TIM_Handle_By_ID(uint8_t servo_id, TIM_HandleTypeDef *tim_handle);
void Servo_Stop_By_TIM_Handle(TIM_HandleTypeDef *tim_handle);
void Servo_Start_By_TIM_Handle(TIM_HandleTypeDef *tim_handle);

#endif /* SERVO_PWM_H */
