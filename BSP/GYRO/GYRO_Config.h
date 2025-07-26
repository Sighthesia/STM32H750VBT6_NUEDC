/** ***************************************************************************
 * @File Name: GYRO_Config.h
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
#ifndef __GYRO_CONFIG_H
#define __GYRO_CONFIG_H

/* -------------------------------- 此处自定义功能开关 ------------------------------- */

#define USE_SEND_UART 0 // 1:使用串口发送陀螺仪数据;0:不使用串口发送陀螺仪数据(调试用)
#define USE_RS485 0     // 1:使用RS485通信;0:不使用RS485通信

/* ------------------------------- 此处定义串口相关配置 ------------------------------- */
#if USE_SEND_UART
#define GYRO_SEND_UART_INSTANCE USART3 // 选择发送陀螺仪数据的串口实例(调试用)
#define GYRO_SEND_UART_HANDLE huart3   // 选择发送陀螺仪数据的串口实例(调试用)
#endif                                 // USE_SEND_UART

#define GYRO_RECEIVE_UART_INSTANCE USART3 // 选择接收陀螺仪数据的串口实例
#define GYRO_RECEIVE_UART_HANDLE huart3   // 选择接收陀螺仪数据的串口实例

/* ------------------------------ 此处定义RS485相关配置 ----------------------------- */

#if USE_RS485
// 使用HAL库的GPIO操作
#define RS485_DE_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_11
#define RS485_RE_GPIO_Port GPIOA
#define RS485_RE_Pin GPIO_PIN_12

#define RS485_RX_DE HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET) // 485模式控制.0,接收;1,发送.
#define RS485_RX_RE HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET) // 485模式控制.0,接收;1,发送.
#define RS485_TX_DE HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)   // 485模式控制.0,接收;1,发送.
#define RS485_TX_RE HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET)   // 485模式控制.0,接收;1,发送.
#endif                                                                                  /* USE_RS485 */

#endif // __GYRO_CONFIG_H