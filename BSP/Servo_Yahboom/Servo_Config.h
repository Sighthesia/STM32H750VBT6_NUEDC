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
#define SERVO_PWM_MAX 2500    // 最大脉宽 2500μs (180°)
#define SERVO_PWM_CENTER 1500 // 中心脉宽 1500μs (90°)

#endif /* USE_HARDWARE_PWM */

/* --------------------------------- 此处修改舵机数量配置 --------------------------------- */
#define MAX_SERVO_COUNT 8 // 最大支持舵机数量
#define MAX_GROUP_COUNT 4 // 最大支持组别数量

/* --------------------------------- 此处修改舵机组别枚举 --------------------------------- */
typedef enum
{
    SERVO_GROUP_0 = 0, // 云台组
    SERVO_GROUP_1,     // 机械臂组
    SERVO_GROUP_2,     // 附加组
    SERVO_GROUP_3,     // 预留组
    MAX_GROUP = MAX_GROUP_COUNT
} servo_group_t;

/* --------------------------------- 此处修改舵机ID枚举 --------------------------------- */
typedef enum
{
    SERVO_ID_J0 = 0, // 舵机1
    SERVO_ID_J1,     // 舵机2
    SERVO_ID_J2,     // 舵机3
    SERVO_ID_J3,     // 舵机4
    SERVO_ID_J4,     // 舵机5
    SERVO_ID_J5,     // 舵机6
    SERVO_ID_J6,     // 舵机7
    SERVO_ID_J7,     // 舵机8
    MAX_SERVO = MAX_SERVO_COUNT
} servo_id_t;

/* --------------------------------- 此处修改调试配置 --------------------------------- */
#define SERVO_ENABLE_DEBUG 1        // 启用调试输出
#define SERVO_ENABLE_STATUS_PRINT 0 // 启用状态打印功能

/* --------------------------------- 此处修改延时配置 --------------------------------- */
#define SERVO_DEFAULT_SMOOTH_DELAY 10 // 平滑移动默认延时(ms)
#define SERVO_CALIBRATE_DELAY 1000    // 校准延时(ms)

#endif /* SERVO_CONFIG_H */
