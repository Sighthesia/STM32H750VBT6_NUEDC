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
#define UART_PACKET_LEN 6

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
    // 如果当前未在接收数据包
    if (!packet_receiving)
    {
        // 检查是否为数据包起始标志
        if (byte == UART_PACKET_HEADER)
        {
            // 开始接收数据包
            packet_receiving = 1;
            packet_index = 0;
            // 将起始标志存入缓冲区
            packet_buf[packet_index++] = byte;
        }
    }
    // 如果正在接收数据包
    else
    {
        // 将当前字节存入缓冲区
        packet_buf[packet_index++] = byte;
        // 如果已接收完整数据包
        if (packet_index == UART_PACKET_LEN)
        {
            // 重置接收状态，准备接收下一个数据包
            packet_receiving = 0;
            // 校验数据包结束标志
            if (packet_buf[UART_PACKET_LEN - 1] == UART_PACKET_TAIL)
            {
                // 解析数据包类型：0x00为X坐标，0xFF为Y坐标
                if (packet_buf[1] == 0x00 || packet_buf[1] == 0xFF)
                {
                    uint16_t val = 0;
                    uint8_t h = packet_buf[2]; // 高字节
                    uint8_t l = packet_buf[3]; // 低字节
                    uint8_t z = packet_buf[4]; // 扩展字节

                    // 解析3字节值到16位数值的算法：
                    // 如果前两个字节都是255，则值为510 + 第三字节
                    if (h == 255 && l == 255)
                    {
                        val = 510 + z;
                    }
                    // 如果高字节是255，则值为255 + 低字节
                    else if (h == 255)
                    {
                        val = 255 + l;
                    }
                    // 否则值就是高字节
                    else
                    {
                        val = h;
                    }

                    // 根据包类型更新对应坐标值
                    if (packet_buf[1] == 0x00)
                    {
                        // 更新X坐标
                        g_rectangle_coordinate.cx = val;
                        g_rectangle_coordinate.updated = 1;
                    }
                    else if (packet_buf[1] == 0xFF)
                    {
                        // 更新Y坐标
                        g_rectangle_coordinate.cy = val;
                        g_rectangle_coordinate.updated = 1;
                    }
                }
            }
        }
        // 如果接收的字节数超过了数据包长度，出现异常
        else if (packet_index > UART_PACKET_LEN)
        {
            // 异常处理，重置状态机
            packet_receiving = 0;
            packet_index = 0;
        }
    }
}

// 获取最新坐标，返回1表示有新数据，0表示无新数据
uint8_t UART_Get_Rect_Coord(uint16_t *cx, uint16_t *cy)
{
    // 检查是否有新数据
    if (g_rectangle_coordinate.updated)
    {
        // 如果传入了cx指针，则将X坐标值写入
        if (cx)
            *cx = g_rectangle_coordinate.cx;
        // 如果传入了cy指针，则将Y坐标值写入
        if (cy)
            *cy = g_rectangle_coordinate.cy;
        // 清除更新标志
        g_rectangle_coordinate.updated = 0;
        // 返回有新数据标志
        return 1;
    }
    // 无新数据
    return 0;
}

// 环形缓冲区相关变量
lwrb_t uart3_rx_buffer;           // UART3接收环形缓冲区结构体
uint8_t uart3_rx_buffer_data[16]; // UART3接收缓冲区数据数组
uint8_t uart3_rx_byte;            // UART3单字节接收缓存

// 启动UART3中断接收
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