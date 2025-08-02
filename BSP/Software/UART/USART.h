/** ***************************************************************************
 * @File Name: USART.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-31
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __USART_H
#define __USART_H

#include "main.h"
#include "usart.h"

#include "stdio.h"
#include "stdint.h"
#include "lwrb.h"
#include "ZDT_Stepper.h"

// 定义UART数据包的起始标志字节
#define UART_PACKET_HEADER 0x55
// 定义UART数据包的结束标志字节
#define UART_PACKET_TAIL 0xFA
// 定义UART数据包的总长度（字节数）
#define UART_PACKET_LEN 7

// 定义用于存储解析后坐标数据的结构体
typedef struct
{
    uint16_t cx;     // X轴坐标值
    uint16_t cy;     // Y轴坐标值
    uint8_t updated; // 标志位，收到新数据时置1
} uart_rect_coord_t;

// 全局坐标数据存储变量，初始化为(0, 0)
extern int16_t rectangle_coordinate_arr[2]; // [0]: x, [1]: y

/* --------------------------------- 系统状态标志位（内部使用） -------------------------------- */
extern volatile _Bool uart_packet_received; // UART数据包接收完成标志位（系统设置）
/* --------------------------------- 自定义部分结束 -------------------------------- */

extern uint8_t packet_buf[UART_PACKET_LEN]; // 存储当前正在接收的数据包
extern uint8_t packet_index;                // 当前数据包接收的字节索引
extern uint8_t packet_receiving;            // 数据包接收状态标志：0-未接收，1-正在接收

extern volatile lwrb_t uart3_rx_buffer;           // UART3接收环形缓冲区结构体
extern volatile uint8_t uart3_rx_buffer_data[16]; // UART3接收缓冲区数据数组
extern volatile uint8_t uart3_rx_byte;            // UART3单字节接收缓存

extern volatile _Bool target_lost;                // 目标丢失标志位（0=目标正常，1=目标丢失）
extern volatile _Bool auto_zero_return_enable;    // 自动回零使能标志位
extern volatile _Bool auto_zero_return_triggered; // 自动回零已触发标志位
extern uint32_t target_lost_start_time;           // 目标丢失开始时间戳
extern uint32_t auto_zero_return_timeout_ms;      // 自动回零超时时间（1秒）

uint8_t UART_Parse_Packet(uint8_t data);

void UART3_Start_Receive_IT(void);

#endif /* __USART_H */