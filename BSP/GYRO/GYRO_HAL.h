/** ***************************************************************************
 * @File Name: GYRO_HAL.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-26
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __GYRO_HAL_H
#define __GYRO_HAL_H

#include "GYRO_Config.h"
#include "stdint.h"

typedef struct gyro_hal_t
{
    void (*UART_Receive)(void);       // 启动UART接收
    void (*UART_DEBUG)(uint8_t data); // 发送陀螺仪数据(调试用)
} gyro_hal;

extern gyro_hal_t gyro_hal;

#endif /* __GYRO_HAL_H */
