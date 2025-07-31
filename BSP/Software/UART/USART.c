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
#include "USART.h"

// 全局坐标数据存储变量，初始化为(0, 0)
int16_t rectangle_coordinate_arr[2] = {0, 0}; // [0]: x, [1]: y
// 接收控制标志位
volatile _Bool uart_rx_enable_flag = 0; // 1:允许接收，0:禁止接收
volatile _Bool pid_update_flag = 0;     // 1:接收完成，0:未完成

// 状态机变量
uint8_t packet_buf[UART_PACKET_LEN]; // 存储当前正在接收的数据包
uint8_t packet_index = 0;            // 当前数据包接收的字节索引
uint8_t packet_receiving = 0;        // 数据包接收状态标志：0-未接收，1-正在接收

// 解包函数：解析单个字节并组装成完整数据包
/** ***************************************************************************
 * @brief  解析单个字节并组装成完整数据包，解包成功返回1，否则返回0
 */
uint8_t UART_Parse_Packet(uint8_t data)
{
    if (!packet_receiving)
    {
        if (data == UART_PACKET_HEADER)
        {
            packet_receiving = 1;
            packet_index = 0;
            packet_buf[packet_index++] = data;
        }
    }
    else
    {
        packet_buf[packet_index++] = data;
        if (packet_index == UART_PACKET_LEN)
        {
            packet_receiving = 0;
            if (packet_buf[UART_PACKET_LEN - 1] == UART_PACKET_TAIL)
            {
                if (packet_buf[1] == 0x00 || packet_buf[1] == 0xFF)
                {
                    uint8_t sign = packet_buf[2];
                    uint8_t h = packet_buf[3];
                    uint8_t l = packet_buf[4];
                    uint8_t z = packet_buf[5];
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
                    int16_t diff = (sign ? 1 : -1) * (int16_t)val;
                    if (packet_buf[1] == 0x00)
                    {
                        rectangle_coordinate_arr[0] = diff; // x
                    }
                    else if (packet_buf[1] == 0xFF)
                    {
                        rectangle_coordinate_arr[1] = diff; // y
                    }
                    pid_update_flag = 1; // 解包成功，置接收完成标志
                    printf("UART Packet Received: %d, %d\n", rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
                    return 1;
                }
            }
        }
        else if (packet_index > UART_PACKET_LEN)
        {
            packet_receiving = 0;
            packet_index = 0;
        }
    }
    return 0;
}

// 环形缓冲区相关变量
volatile lwrb_t uart3_rx_buffer;           // UART3接收环形缓冲区结构体
volatile uint8_t uart3_rx_buffer_data[16]; // UART3接收缓冲区数据数组
volatile uint8_t uart3_rx_byte;            // UART3单字节接收缓存

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