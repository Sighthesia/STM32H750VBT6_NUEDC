#ifndef ZDT_STEPPER_CONFIG_H
#define ZDT_STEPPER_CONFIG_H

#include "main.h"

// 使能
#define Enable 1
#define Disable 0
// 舵机同步
#define SYNC_ENABLE 1  // 多机同步
#define SYNC_DISABLE 0 // 不使能多机同步
// 方向
#define CW 1  // 负转速 顺时针，正对转轴看
#define CCW 0 // 正转速 逆时针
// 位置模式
#define REL_POS_MODE 0 // 相对位置模式
#define ABS_POS_MODE 1 // 绝对位置模式
// 是否存储
#define STORE 1    // 存储标志
#define NO_STORE 0 // 不存储标

// 最大支持的串口数量
#define ZDT_STEPPER_MAX_UART 2
// 每个串口最大支持的电机数量
#define ZDT_STEPPER_PER_UART_MAX_MOTORS 1

// 电机配置结构体
typedef struct
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma_rx;
    uint8_t motor_id;
} zdt_stepper_motor_config_t;

// 可选：如需LED指示等功能，可定义如下
// #define ZDT_STEPPER_LED_GPIO_PORT GPIOA
// #define ZDT_STEPPER_LED_PIN GPIO_PIN_5

#endif /* ZDT_STEPPER_CONFIG_H */
/* 自定义部分结束 */
