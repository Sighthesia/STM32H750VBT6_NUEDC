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

#ifndef SERVO_YAHBOOM_PWM_H
#define SERVO_YAHBOOM_PWM_H

#include "main.h"
#include "tim.h"
#include "stdint.h"

/* --------------------------------- 此处修改PWM模式选择 --------------------------------- */
#define USE_HARDWARE_PWM // 使用硬件PWM
// #define USE_SOFTWARE_PWM // 使用软件模拟PWM (兼容原版)

/* --------------------------------- 舵机组别枚举 --------------------------------- */
typedef enum
{
    SERVO_GROUP_0 = 0, // 云台组
    MAX_GROUP          // 自动获取最大组别数量，不要手动赋值
} servo_group_t;

/* --------------------------------- 舵机ID枚举 --------------------------------- */
typedef enum
{
    SERVO_ID_J0 = 0, // 舵机1
    SERVO_ID_J1,     // 舵机2
    MAX_SERVO        // 自动获取最大舵机数量，不要手动赋值
} servo_id_t;

/* --------------------------------- 此处修改硬件PWM配置 --------------------------------- */
#ifdef USE_HARDWARE_PWM

#define SERVO_PWM_FREQUENCY 50        // PWM频率 50Hz
#define SERVO_PWM_PERIOD 20000        // 周期 20ms (以μs为单位)
#define SERVO_PWM_PRESCALER (480 - 1) // 预分频器 (480MHz/480=1MHz, 1μs分辨率)
#define SERVO_PWM_ARR (20000 - 1)     // 自动重装值 (20ms周期)

#define SERVO_PWM_MIN 500     // 最小脉宽 500μs (0°)
#define SERVO_PWM_MAX 2500    // 最大脉宽 2500μs (180°)
#define SERVO_PWM_CENTER 1500 // 中心脉宽 1500μs (90°)

#endif /* USE_HARDWARE_PWM */

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
    const char *name;              // 舵机名称
    uint16_t default_pwm;          // 默认PWM值
    uint16_t max_pwm;              // 最大PWM值
    uint16_t min_pwm;              // 最小PWM值
    uint16_t mid_pwm;              // 中位PWM值
} servo_config_t;

/* --------------------------------- 外部变量声明 --------------------------------- */
extern servo_typedef_t servo_array[MAX_SERVO]; // 舵机控制数组
extern volatile int servo_angle[MAX_GROUP][MAX_SERVO];

/* ---------------------------------- 函数声明 ---------------------------------- */
// 初始化函数
void Servo_TIM_PWM_Init(void);

// 按ID控制函数
void Servo_Set_Angle_By_ID(uint8_t servo_id, uint8_t angle);
void Servo_Set_PWM_By_ID(uint8_t servo_id, uint16_t pwm);

// 按组别控制函数
void Servo_Set_Angle_By_Group(uint8_t group_id, uint8_t servo_index, uint8_t angle);
void Servo_Set_PWM_By_Group(uint8_t group_id, uint8_t servo_index, uint16_t pwm);
void Servo_Set_Group_Angles(uint8_t group_id, uint8_t *angles, uint8_t count);
void Servo_Set_Group_PWMs(uint8_t group_id, uint16_t *pwms, uint8_t count);

// 批量控制函数
void Servo_Set_Multiple_Angles(uint8_t *servo_ids, uint8_t *angles, uint8_t count);
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
uint16_t Servo_Angle_To_PWM(uint8_t angle);
uint8_t Servo_PWM_To_Angle(uint16_t pwm_value);

// 高级功能函数
void Servo_Smooth_Move_By_ID(uint8_t servo_id, uint8_t target_angle, uint16_t step_delay);
void Servo_Calibrate_By_ID(uint8_t servo_id);
void Servo_Calibrate_By_Group(uint8_t group_id);
void Servo_Calibrate_All(void);
void Servo_Test(void);
void Servo_Print_Status(void);

// 获取舵机数量信息
uint8_t Servo_Get_Active_Count(void);
uint8_t Servo_Get_Max_Configured_Count(void);
uint8_t Servo_Get_Group_Count(uint8_t group_id);
uint8_t Servo_Get_Max_Group_Count(void);
servo_typedef_t *Servo_Get_Group_Array(uint8_t group_id, uint8_t *count);

// 多定时器支持相关函数
TIM_HandleTypeDef *Servo_Get_TIM_Handle_By_ID(uint8_t servo_id);
uint8_t Servo_Get_Servo_Count_By_TIM(TIM_HandleTypeDef *tim_handle);
void Servo_Set_TIM_Handle_By_ID(uint8_t servo_id, TIM_HandleTypeDef *tim_handle);
void Servo_Stop_By_TIM_Handle(TIM_HandleTypeDef *tim_handle);
void Servo_Start_By_TIM_Handle(TIM_HandleTypeDef *tim_handle);

#endif /* SERVO_YAHBOOM_PWM_H */
