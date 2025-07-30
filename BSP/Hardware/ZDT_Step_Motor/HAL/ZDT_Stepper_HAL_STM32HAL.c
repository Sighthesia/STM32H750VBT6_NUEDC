#include "ZDT_Stepper_HAL.h"
#include "ZDT_Stepper_Config.h"
#include "main.h"

/* --------------------------------- 此处修改... -------------------------------- */

/**
 *****************************************************************************
 * @brief   STM32 HAL平台下ZDT步进电机串口发送实现
 * @param   huart 串口句柄
 * @param   data  数据指针
 * @param   size  数据长度
 *****************************************************************************
 */
static void Uart_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY);
}

/**
 *****************************************************************************
 * @brief   毫秒级延时（HAL库）
 * @param   ms 延时时间
 *****************************************************************************
 */
static void Delay_Ms(uint32_t ms)
{
    HAL_Delay(ms);
}

// 全局HAL实例，所有指针均为有效实现
zdt_stepper_hal_t zdt_stepper_hal = {
    .Uart_Transmit = Uart_Transmit,
    .Delay_Ms = Delay_Ms};

/* 自定义部分结束 */
