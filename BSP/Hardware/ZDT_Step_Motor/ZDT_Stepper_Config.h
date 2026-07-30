#ifndef ZDT_STEPPER_CONFIG_H
#define ZDT_STEPPER_CONFIG_H

#include "main.h"

// 使能
#define ZDT_STEPPER_ENABLE 1
#define ZDT_STEPPER_DISABLE 0
// 舵机同步
#define ZDT_STEPPER_SYNC_ENABLE 1  // 多机同步
#define ZDT_STEPPER_SYNC_DISABLE 0 // 不使能多机同步
// 方向
#define ZDT_STEPPER_CW 1  // 负转速 顺时针，正对转轴看
#define ZDT_STEPPER_CCW 0 // 正转速 逆时针
// 位置模式
#define ZDT_STEPPER_REL_POS_MODE 0 // 相对位置模式
#define ZDT_STEPPER_ABS_POS_MODE 1 // 绝对位置模式
// 速度控制加速度档位；0 表示不使用曲线加减速。
#define ZDT_STEPPER_SPEED_ACCEL 0
// 是否存储
#define ZDT_STEPPER_STORE 1    // 存储标志
#define ZDT_STEPPER_NO_STORE 0 // 不存储标

// 最大支持的串口数量
#define ZDT_STEPPER_MAX_UART 1
// 每个串口最大支持的电机数量
#define ZDT_STEPPER_PER_UART_MAX_MOTORS 1

// 电机配置结构体
typedef struct
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma_rx;
    uint8_t motor_id;
} zdt_stepper_motor_config_t;

typedef enum
{
    STEPPER_YAW = 1,
    STEPPER_PITCH
} stepper_id_t;

// 可选：如需LED指示等功能，可定义如下
// #define ZDT_STEPPER_LED_GPIO_PORT GPIOA
// #define ZDT_STEPPER_LED_PIN GPIO_PIN_5

#endif /* ZDT_STEPPER_CONFIG_H */
/* 自定义部分结束 */
