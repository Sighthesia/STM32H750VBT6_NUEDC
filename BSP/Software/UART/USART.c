/** ***************************************************************************
 * @File Name: UASRT.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-24
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "main.h"
#include "usart.h"

#include "stdio.h"
#include "lwrb.h"
#include "ZDT_Stepper.h"

// === 坐标包解析相关 ===
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

// 全局坐标数据存储变量，初始化为(0, 0, 0)
static uart_rect_coord_t g_rectangle_coordinate = {0, 0, 0};

// 状态机变量
static uint8_t packet_buf[UART_PACKET_LEN]; // 存储当前正在接收的数据包
static uint8_t packet_index = 0;            // 当前数据包接收的字节索引
static uint8_t packet_receiving = 0;        // 数据包接收状态标志：0-未接收，1-正在接收

// 解包函数：解析单个字节并组装成完整数据包
static void UART_Parse_Packet(uint8_t byte)
{
    /* --------------------------------- 此处修改: 按照Python端协议解析带符号三字节差值 -------------------------------- */
    if (!packet_receiving)
    {
        if (byte == UART_PACKET_HEADER)
        {
            packet_receiving = 1;
            packet_index = 0;
            packet_buf[packet_index++] = byte;
        }
    }
    else
    {
        packet_buf[packet_index++] = byte;
        if (packet_index == UART_PACKET_LEN)
        {
            packet_receiving = 0;
            if (packet_buf[UART_PACKET_LEN - 1] == UART_PACKET_TAIL)
            {
                // 包格式: [0]=0x55, [1]=0x00/0xFF, [2]=符号, [3]=h, [4]=l, [5]=z, [6]=0xFA
                // 但C端定义长度为6, 实际应为7, 需与发送端保持一致, 这里假定长度为6, [5]为0xFA
                // Python端: [0x55, 0x00/0xFF, sign, h, l, z, 0xFA]
                // 若协议实际为7字节, 请调整UART_PACKET_LEN为7
                if (packet_buf[1] == 0x00 || packet_buf[1] == 0xFF)
                {
                    uint8_t sign = packet_buf[2];
                    uint8_t h = packet_buf[3];
                    uint8_t l = packet_buf[4];
                    uint8_t z = 0;
// 若协议为7字节, z = packet_buf[5];
// 若协议为6字节, z = 0;
// 兼容处理: 若长度为7, 取z; 否则z=0
#if UART_PACKET_LEN >= 7
                    z = packet_buf[5];
#endif

                    uint16_t val = 0;
                    if (h == 255 && l == 255)
                    {
                        val = 510 + z;
                    }
                    else if (h == 255)
                    {
                        val = 255 + l;
                    }
                    else
                    {
                        val = h;
                    }

                    // 恢复符号
                    int16_t diff = (sign ? 1 : -1) * (int16_t)val;

                    if (packet_buf[1] == 0x00)
                    {
                        g_rectangle_coordinate.cx = diff;
                        g_rectangle_coordinate.updated = 1;
                    }
                    else if (packet_buf[1] == 0xFF)
                    {
                        g_rectangle_coordinate.cy = diff;
                        g_rectangle_coordinate.updated = 1;
                    }
                }
            }
        }
        else if (packet_index > UART_PACKET_LEN)
        {
            packet_receiving = 0;
            packet_index = 0;
        }
    }
}

// 环形缓冲区相关变量
lwrb_t uart3_rx_buffer;           // UART3接收环形缓冲区结构体
uint8_t uart3_rx_buffer_data[16]; // UART3接收缓冲区数据数组
uint8_t uart3_rx_byte;            // UART3单字节接收缓存

/** ***************************************************************************
 * @brief  启动UART3中断接收
 */
void UART3_Start_Receive_IT(void)
{
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
}

/** ***************************************************************************
 * @brief UART3接收回调函数
 * @param  huart: UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // 将接收到的字节写入环形缓冲区
        lwrb_write(&uart3_rx_buffer, &uart3_rx_byte, 1);

        // 调试输出接收到的字节
        printf("[UART RX] 收到: 0x%02X\r\n", uart3_rx_byte);

        // 继续启动下一次中断接收
        UART3_Start_Receive_IT();
    }
}

/** ***************************************************************************
 * @brief  UART接收回调函数
 * @param  huart: UART句柄
 * @param  Size: 接收到的数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    ZDT_Stepper_USRT_RX_callback(huart, Size);
}