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
#include "bsp_usart.h"
#include "Gyro_APP.h"

/**
 * @brief  UART接收完成中断回调
 * @param  huart UART句柄指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 将接收到的数据写入环形缓冲区
        Write_RingBuff(&Uart1_RingBuff, uart1_rx_buffer);
        // 重新启动接收中断
        HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer, 1);
    }
    else if (huart->Instance == USART3)
    {
        // 将接收到的数据写入环形缓冲区
        Write_RingBuff(&Uart3_RingBuff, uart3_rx_buffer);

        // 转发USART3收到的数据到USART1（调试输出）
        HAL_UART_Transmit_IT(&huart1, &uart3_rx_buffer, 1);

        // 重新启动接收中断
        HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);
    }

    if (huart->Instance == g_huart.Instance)
    {
        // 处理接收到的字节
        UART_RX_Callback(g_uart_channel, g_rx_byte);

        // 重新启动接收
        HAL_UART_Receive_IT(&g_huart, &g_rx_byte, 1);
    }
}