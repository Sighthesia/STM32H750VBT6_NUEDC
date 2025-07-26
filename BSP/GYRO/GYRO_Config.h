/** ***************************************************************************
 * @File Name: GYRO_Config.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-25
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef GYRO_CONFIG_H
#define GYRO_CONFIG_H

#include "main.h"
#include "usart.h"

/* --------------------------------- 此处修改陀螺仪硬件配置 -------------------------------- */
// UART外设句柄（需在main.c中定义）
#define GYRO_UART_HANDLE huart3
#define GYRO_UART_INSTANCE USART3

// 帧格式定义
#define GYRO_FRAME_HEAD 0xFC // 帧头
#define GYRO_FRAME_END 0xFD  // 帧尾
#define GYRO_TYPE_IMU 0x40   // IMU数据类型
#define GYRO_TYPE_AHRS 0x41  // AHRS数据类型
#define GYRO_IMU_LEN 0x38    // IMU数据长度（56+8字节）
#define GYRO_AHRS_LEN 0x30   // AHRS数据长度（48+8字节）

// 数据缓冲区大小
#define GYRO_MAX_FRAME_SIZE 80 // 最大帧长度

// 默认配置参数
#define GYRO_DEFAULT_UART_CHANNEL 3
#define GYRO_DEFAULT_BAUD_RATE 921600
/* 自定义部分结束 */

#endif /* GYRO_CONFIG_H */
