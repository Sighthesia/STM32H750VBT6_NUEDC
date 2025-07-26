/** ***************************************************************************
 * @File Name: GYRO_HAL_STM32HAL.c
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
#include "GYRO_HAL.h"

static uint8_t uart_rx_buffer[1]; // 单字节接收缓冲区

/** ***************************************************************************
 * @brief 启动UART接收
 */
void UART_Receive(void)
{
    HAL_UART_Receive_IT(&GYRO_RECEIVE_UART_HANDLE, uart_rx_buffer, 1);
}

/** ***************************************************************************
 * @brief 发送陀螺仪数据(调试用)
 * @param  data: 要发送的数据
 */
#if USE_DEBUG
void UART_Transmit(uint8_t data)
{
    HAL_UART_Transmit_IT(&GYRO_DEBUG_UART_HANDLE, &data, 1);
}
#endif // USE_DEBUG

gyro_hal_t gyro_hal = {
    .UART_Receive = UART_Receive,
#if USE_DEBUG
    .UART_DEBUG = UART_Transmit
#endif // USE_DEBUG
};