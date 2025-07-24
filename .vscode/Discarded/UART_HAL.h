#ifndef __UART_HAL_H
#define __UART_HAL_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* --------------------------------- 此处修改UART HAL配置 -------------------------------- */
#define UART_HAL_MAX_CHANNELS 6    // 最大UART通道数
#define UART_HAL_RX_BUFFER_SIZE 80 // 接收缓冲区大小

// UART通道枚举
typedef enum
{
    UART_HAL_CHANNEL_1 = 0,
    UART_HAL_CHANNEL_2,
    UART_HAL_CHANNEL_3,
    UART_HAL_CHANNEL_4,
    UART_HAL_CHANNEL_5,
    UART_HAL_CHANNEL_6
} uart_hal_channel_t;

// UART配置结构体
typedef struct
{
    uint32_t baud_rate; // 波特率
    uint8_t data_bits;  // 数据位（8位）
    uint8_t stop_bits;  // 停止位（1位）
    uint8_t parity;     // 校验位（无校验）
} uart_hal_config_t;

// 数据接收回调函数类型
typedef void (*uart_hal_rx_callback_t)(uart_hal_channel_t channel, uint8_t data);
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief UART硬件抽象层函数指针结构体
 *
 * 提供统一的UART接口，支持多通道初始化、数据收发和中断处理
 */
typedef struct
{
    /** ***************************************************************************
     * @brief 初始化UART通道
     * @param channel UART通道号
     * @param config UART配置参数
     */
    void (*UART_HAL_Init)(uart_hal_channel_t channel, const uart_hal_config_t *config);

    /** ***************************************************************************
     * @brief 发送单个字节
     * @param channel UART通道号
     * @param data 要发送的数据
     */
    void (*UART_HAL_Send_Byte)(uart_hal_channel_t channel, uint8_t data);

    /** ***************************************************************************
     * @brief 发送数据缓冲区
     * @param channel UART通道号
     * @param buf 数据缓冲区指针
     * @param len 数据长度
     */
    void (*UART_HAL_Send_Buffer)(uart_hal_channel_t channel, const uint8_t *buf, uint16_t len);

    /** ***************************************************************************
     * @brief 设置接收数据回调函数
     * @param channel UART通道号
     * @param callback 回调函数指针
     */
    void (*UART_HAL_Set_RX_Callback)(uart_hal_channel_t channel, uart_hal_rx_callback_t callback);

    /** ***************************************************************************
     * @brief 启用UART接收中断
     * @param channel UART通道号
     */
    void (*UART_HAL_Enable_RX_IT)(uart_hal_channel_t channel);

    /** ***************************************************************************
     * @brief 禁用UART接收中断
     * @param channel UART通道号
     */
    void (*UART_HAL_Disable_RX_IT)(uart_hal_channel_t channel);

    /** ***************************************************************************
     * @brief 检查发送完成状态
     * @param channel UART通道号
     * @return 1-发送完成，0-发送中
     */
    uint8_t (*UART_HAL_Is_TX_Complete)(uart_hal_channel_t channel);

    /** ***************************************************************************
     * @brief 处理UART中断（供中断服务程序调用）
     * @param channel UART通道号
     */
    void (*UART_HAL_IRQ_Handler)(uart_hal_channel_t channel);

} uart_hal_t;

/// 全局实例（由具体平台实现）
extern const uart_hal_t uart_hal;

#endif /* __UART_HAL_H */
