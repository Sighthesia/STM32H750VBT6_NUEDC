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

#include "lwrb.h"
#include "ZDT_Stepper.h"
#include "stdio.h"

lwrb_t uart3_rx_buffer;
uint8_t uart3_rx_buffer_data[16];
uint8_t uart3_rx_byte;

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
        // 写入环形缓冲区
        lwrb_write(&uart3_rx_buffer, &uart3_rx_byte, 1);

        // 调试输出
        printf("[UART RX] 收到: 0x%02X\r\n", uart3_rx_byte);

        // 继续接收下一个字节
        UART3_Start_Receive_IT();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    ZDT_Stepper_USRT_RX_callback(huart, Size);
}