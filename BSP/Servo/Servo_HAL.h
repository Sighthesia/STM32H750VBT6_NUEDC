/** ***************************************************************************
 * @File_Name: Servo_HAL.h
 * @brief 舵机硬件抽象层头文件 - 定义HAL接口
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-13
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author: Sighthesia
 * @Changes:
 * @Modifi_Date:
 */

#ifndef SERVO_HAL_H
#define SERVO_HAL_H

#include "main.h"
#include "tim.h"
#include "stdint.h"

/* --------------------------------- 此处修改HAL状态定义 --------------------------------- */
typedef enum
{
    SERVO_HAL_OK = 0,       // 操作成功
    SERVO_HAL_ERROR,        // 操作失败
    SERVO_HAL_INVALID_PARAM // 参数无效
} Servo_HAL_Status_t;

/* --------------------------------- 此处修改HAL接口结构体定义 --------------------------------- */
typedef struct
{
    /** ***************************************************************************
     * @brief PWM启动函数指针
     * @param tim_handle: 定时器句柄
     * @param tim_channel: 定时器通道
     * @return Servo_HAL_Status_t: 操作状态
     */
    Servo_HAL_Status_t (*PWM_Start)(void *tim_handle, uint32_t tim_channel);

    /** ***************************************************************************
     * @brief PWM停止函数指针
     * @param tim_handle: 定时器句柄
     * @param tim_channel: 定时器通道
     * @return Servo_HAL_Status_t: 操作状态
     */
    Servo_HAL_Status_t (*PWM_Stop)(void *tim_handle, uint32_t tim_channel);

    /** ***************************************************************************
     * @brief 设置PWM比较值函数指针
     * @param tim_handle: 定时器句柄
     * @param tim_channel: 定时器通道
     * @param ccr_value: 比较值
     * @return Servo_HAL_Status_t: 操作状态
     */
    Servo_HAL_Status_t (*Set_PWM_CCR)(void *tim_handle, uint32_t tim_channel, uint16_t ccr_value);

    /** ***************************************************************************
     * @brief 延时函数指针
     * @param delay_ms: 延时时间(毫秒)
     * @return Servo_HAL_Status_t: 操作状态
     */
    Servo_HAL_Status_t (*Delay_Ms)(uint32_t delay_ms);

} servo_hal_t;

/* --------------------------------- 此处修改外部HAL接口声明 --------------------------------- */
extern const servo_hal_t servo_hal; // 全局HAL接口实例

#endif /* SERVO_HAL_H */
