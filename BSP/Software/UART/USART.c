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
int16_t rectangle_coordinate_arr[2] = {0, 0}; // [0]: x差值, [1]: y差值 (相对于屏幕中心的偏移)

volatile _Bool uart_packet_received = 0; // UART数据包接收完成标志位（系统设置）

volatile _Bool target_lost = 0; // 目标丢失标志位（0=目标正常，1=目标丢失）
int16_t last_valid_x = 0;       // 上一次有效的X差值
int16_t last_valid_y = 0;       // 上一次有效的Y差值

// 目标丢失检测相关变量
uint32_t last_packet_time = 0;     // 上一次接收到数据包的时间
uint32_t packet_timeout_ms = 500;  // 数据包超时时间（500ms）
uint32_t same_data_count = 0;      // 连续接收相同数据的次数
uint32_t same_data_threshold = 10; // 判定为目标丢失的相同数据阈值
int16_t prev_x = 0, prev_y = 0;    // 上一次的差值数据

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

                // 根据K230发送格式：[0x55, data_type, sign, value_h, value_l, value_z, 0xFA]
                // data_type: 0x00=X差值, 0xFF=Y差值
                if (data_type == 0x00 || data_type == 0xFF)
                {
                    uint8_t sign = packet_buf[2]; // 符号位：1=正数，0=负数
                    uint8_t h = packet_buf[3];    // 高字节
                    uint8_t l = packet_buf[4];    // 低字节
                    uint8_t z = packet_buf[5];    // 扩展字节
                    uint16_t val = 0;

                    // 解码差值绝对值（根据K230的conversion_position函数逻辑）
                    if (h == 255 && l == 255)
                    {
                        val = 510 + z; // 大于510的值
                    }
                    else if (h == 255)
                    {
                        val = 255 + l; // 255-510之间的值
                    }
                    else
                    {
                        val = h; // 0-254之间的值
                    }

                    // 应用符号位得到真实差值
                    int16_t diff = (sign ? 1 : -1) * (int16_t)val;

                    // 检测是否为连续相同数据（可能表示目标丢失）
                    uint32_t current_time = HAL_GetTick();
                    _Bool is_same_data = 0;

                    if (data_type == 0x00)
                    {
                        is_same_data = (diff == prev_x);
                        prev_x = diff;
                    }
                    else
                    {
                        is_same_data = (diff == prev_y);
                        prev_y = diff;
                    }

                    // 检测目标丢失：连续相同数据或数据包超时
                    if (is_same_data)
                    {
                        same_data_count++;
                    }
                    else
                    {
                        same_data_count = 0;
                    }

                    // 根据数据类型更新对应坐标差值
                    if (data_type == 0x00)
                    {
                        // X差值更新
                        rectangle_coordinate_arr[0] = diff;
                        last_valid_x = diff;
                    }
                    else if (data_type == 0xFF)
                    {
                        // Y差值更新
                        rectangle_coordinate_arr[1] = diff;
                        last_valid_y = diff;
                    }

                    // 判断目标状态
                    _Bool target_lost_prev = target_lost;

                    if (same_data_count >= same_data_threshold ||
                        (current_time - last_packet_time) > packet_timeout_ms)
                    {
                        if (!target_lost)
                        {
                            target_lost = 1;
                            target_lost_start_time = current_time;
                            printf("[Target] 目标可能丢失：连续相同数据 %lu 次或超时\r\n", same_data_count);
                        }
                    }
                    else
                    {
                        if (target_lost)
                        {
                            target_lost = 0;
                            auto_zero_return_triggered = 0;
                            printf("[Target] 目标跟踪恢复正常\r\n");
                        }
                    }

                    last_packet_time = current_time;
                    uart_packet_received = 1; // 解包成功，置接收完成标志

                    // 调试输出 - 显示差值信息
                    if (target_lost)
                    {
                        printf("[UART] 目标丢失 - X差值: %d, Y差值: %d (可能为历史数据)\r\n",
                               rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
                    }
                    else
                    {
                        printf("[UART] 目标正常 - X差值: %d, Y差值: %d (相对屏幕中心偏移)\r\n",
                               rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
                    }

                    return 1;
                }
                else
                {
                    // 未知数据类型，丢弃数据包
                    printf("[UART] 未知数据类型: 0x%02X\r\n", data_type);
                }
            }
            else
            {
                // 包尾校验失败
                printf("[UART] 包尾校验失败: 0x%02X\r\n", packet_buf[UART_PACKET_LEN - 1]);
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