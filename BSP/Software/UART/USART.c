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

// 环形缓冲区相关变量
volatile lwrb_t uart3_rx_buffer;           // UART3接收环形缓冲区结构体
volatile uint8_t uart3_rx_buffer_data[16]; // UART3接收缓冲区数据数组
volatile uint8_t uart3_rx_byte;            // UART3单字节接收缓存

// UART数据包解析相关变量
uint8_t packet_buf[UART_PACKET_LEN]; // 存储当前正在接收的数据包
uint8_t packet_index = 0;            // 当前数据包接收的字节索引
uint8_t packet_receiving = 0;        // 数据包接收状态标志：0-未接收，1-正在接收

// 全局坐标数据存储变量，初始化为(0, 0)
int16_t rectangle_coordinate_arr[2] = {0, 0}; // [0]: x, [1]: y

volatile _Bool uart_packet_received = 0; // UART数据包接收完成标志位（系统设置）

volatile _Bool target_lost = 0; // 目标丢失标志位（0=目标正常，1=目标丢失）
int16_t last_valid_x = 0;       // 上一次有效的X坐标
int16_t last_valid_y = 0;       // 上一次有效的Y坐标

volatile _Bool auto_zero_return_enable = 1;    // 自动回零使能标志位
volatile _Bool auto_zero_return_triggered = 0; // 自动回零已触发标志位
uint32_t target_lost_start_time = 0;           // 目标丢失开始时间戳
uint32_t auto_zero_return_timeout_ms = 1000;   // 自动回零超时时间（1秒）

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
                // 检查数据类型标志位
                uint8_t data_type = packet_buf[1];

                if (data_type == 0x00 || data_type == 0xFF || data_type == 0x01 || data_type == 0xF1)
                {
                    uint8_t sign = packet_buf[2];
                    uint8_t h = packet_buf[3];
                    uint8_t l = packet_buf[4];
                    uint8_t z = packet_buf[5];
                    uint16_t val = 0;

                    // 解码坐标值
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

                    // 根据数据类型处理坐标
                    if (data_type == 0x00)
                    {
                        // 正常X坐标
                        rectangle_coordinate_arr[0] = diff;
                        last_valid_x = diff; // 更新上一次有效X坐标

                        // 如果之前目标丢失，现在检测到正常坐标，记录恢复
                        if (target_lost)
                        {
                            printf("[Target] X coordinate recovered: %d\r\n", diff);
                        }
                        // 注意：这里不立即清除target_lost，需要Y坐标也正常才完全恢复
                    }
                    else if (data_type == 0xFF)
                    {
                        // 正常Y坐标
                        rectangle_coordinate_arr[1] = diff;
                        last_valid_y = diff; // 更新上一次有效Y坐标

                        // 如果之前目标丢失，现在检测到正常坐标，清除丢失标志
                        if (target_lost)
                        {
                            printf("[Target] Y coordinate recovered: %d\r\n", diff);
                            target_lost = 0;                // 恢复正常，清除目标丢失标志
                            auto_zero_return_triggered = 0; // 重置自动回零触发标志
                            printf("[Target] Target tracking resumed\r\n");
                        }
                    }
                    else if (data_type == 0x01)
                    {
                        // 目标丢失 - X坐标为上一次有效值
                        rectangle_coordinate_arr[0] = last_valid_x;
                        if (!target_lost) // 只在第一次丢失时输出和记录时间
                        {
                            printf("[Target] X coordinate lost, using last valid: %d\r\n", last_valid_x);
                            target_lost_start_time = HAL_GetTick(); // 记录目标丢失开始时间
                        }
                        target_lost = 1; // 设置目标丢失标志
                    }
                    else if (data_type == 0xF1)
                    {
                        // 目标丢失 - Y坐标为上一次有效值
                        rectangle_coordinate_arr[1] = last_valid_y;
                        if (!target_lost) // 只在第一次丢失时记录时间
                        {
                            target_lost_start_time = HAL_GetTick(); // 记录目标丢失开始时间
                        }
                        target_lost = 1; // 设置目标丢失标志
                        printf("[Target] Y lost, using last valid: %d\r\n", last_valid_y);
                    }

                    uart_packet_received = 1; // 解包成功，置接收完成标志

                    // 调试输出
                    if (target_lost)
                    {
                        printf("[UART] Target LOST - X: %d, Y: %d (using last valid)\r\n",
                               rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
                    }
                    else
                    {
                        printf("[UART] Target OK - X: %d, Y: %d\r\n",
                               rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
                    }

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

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART3)
//     {
//         // 发送完成，可以进行下一步操作
//         printf("UART3 Transmission Complete\n");
//     }
// }