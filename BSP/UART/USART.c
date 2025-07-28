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

#include "GYRO_APP.h"

/**
 * @brief  UART接收完成中断回调
 * @param  huart UART句柄指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // if (huart == &GYRO_RECEIVE_UART_HANDLE)
    // {
    //     printf("[UART RX] 接收到数据: %s\r\n", uart_rx_buffer);
    //     GYRO_UART_Receive_Process();
    // }
}