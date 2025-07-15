/** ***************************************************************************
 * @File_Name: Servo_Config.h
 * @brief 舵机配置文件 - 所有可配置项集中管理
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-13
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author: Sighthesia
 * @Changes:
 * @Modifi_Date:
 */

#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

/* --------------------------------- 此处修改PWM模式选择 --------------------------------- */
#define USE_HARDWARE_PWM // 使用硬件PWM
// #define USE_SOFTWARE_PWM // 使用软件模拟PWM (兼容原版)

/* --------------------------------- 此处修改硬件PWM配置 --------------------------------- */
#ifdef USE_HARDWARE_PWM

#define SERVO_PWM_FREQUENCY 50        // PWM频率 50Hz
#define SERVO_PWM_PERIOD 20000        // 周期 20ms (以μs为单位)
#define SERVO_PWM_PRESCALER (480 - 1) // 预分频器 (480MHz/480=1MHz, 1μs分辨率)
#define SERVO_PWM_ARR (20000 - 1)     // 自动重装值 (20ms周期)

#define SERVO_PWM_MIN 500     // 最小脉宽 500μs (0°)
#define SERVO_PWM_MAX 2500    // 最大脉宽 2500μs (180°/270°)
#define SERVO_PWM_CENTER 1500 // 中心脉宽 1500μs (90°/135°)

#endif /* USE_HARDWARE_PWM */

/* --------------------------------- 此处修改舵机角度类型枚举 --------------------------------- */
typedef enum
{
    SERVO_ANGLE_TYPE_180 = 0, // 180度舵机 (标准舵机)
    SERVO_ANGLE_TYPE_270,     // 270度舵机 (大角度舵机)
    SERVO_ANGLE_TYPE_360,     // 360度舵机 (连续旋转舵机，保留)
    MAX_ANGLE_TYPE            // 角度类型总数
} servo_angle_type_t;

/* --------------------------------- 此处修改舵机组别枚举 --------------------------------- */
typedef enum
{
    SERVO_GROUP_0 = 0, // 云台组
    MAX_GROUP          // 自动获取最大组别数量，不要手动赋值
} servo_group_t;

/* --------------------------------- 此处修改舵机ID枚举 --------------------------------- */
typedef enum
{
    SERVO_ID_J0 = 0, // 舵机1
    SERVO_ID_J1,     // 舵机2
    MAX_SERVO        // 自动获取最大舵机数量，不要手动赋值
} servo_id_t;

/* --------------------------------- 此处修改调试配置 --------------------------------- */
#define SERVO_ENABLE_DEBUG 0        // 启用调试输出
#define SERVO_ENABLE_STATUS_PRINT 0 // 启用状态打印功能

/* --------------------------------- 此处修改延时配置 --------------------------------- */
#define SERVO_DEFAULT_SMOOTH_DELAY 10 // 平滑移动默认延时(ms)
#define SERVO_CALIBRATE_DELAY 1000    // 校准延时(ms)

#endif /* SERVO_CONFIG_H */
