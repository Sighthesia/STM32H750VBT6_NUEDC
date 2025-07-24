#include "Gyro_HAL.h"
#include "stm32h7xx_hal.h"

/* --------------------------------- 此处修改STM32平台陀螺仪接收配置 -------------------------------- */
// STM32 HAL UART句柄
static UART_HandleTypeDef g_huart;
static uint8_t g_rx_byte;
static uint8_t g_uart_channel = UART_HAL_CHANNEL_5; // 默认使用UART5

// 接收回调函数指针
static void (*g_rx_callback)(uint8_t data) = NULL;

// 内部函数声明
static void Init(uint8_t uart_channel, uint32_t baud_rate);
static void Set_RX_Callback(void (*callback)(uint8_t data));
static void Start_Receive(void);
static void Stop_Receive(void);
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief 初始化陀螺仪数据接收
 * @param uart_channel 使用的UART通道
 * @param baud_rate 波特率
 */
static void Init(uint8_t uart_channel, uint32_t baud_rate)
{
    g_uart_channel = uart_channel;
    // 注意：实际的UART初始化应由统一的UART管理模块处理
}

/** ***************************************************************************
 * @brief 设置接收回调函数
 * @param callback 接收回调函数
 */
static void Set_RX_Callback(void (*callback)(uint8_t data))
{
    g_rx_callback = callback;
}

/** ***************************************************************************
 * @brief 启动陀螺仪数据接收
 */
static void Start_Receive(void)
{
    // 使用宏定义获取UART实例
    g_huart.Instance = GET_UART_INSTANCE(g_uart_channel);
    // 启动UART接收中断
    HAL_UART_Receive_IT(&g_huart, &g_rx_byte, 1);
}

/** ***************************************************************************
 * @brief 停止陀螺仪数据接收
 */
static void Stop_Receive(void)
{
    // 停止UART接收中断
    HAL_UART_AbortReceive_IT(&g_huart);
}

// 绑定实现到全局结构体（仅包含底层硬件操作）
const gyro_hal_t gyro_hal = {
    .GYRO_HAL_Init = Init,
    .GYRO_HAL_Set_RX_Callback = Set_RX_Callback,
    .GYRO_HAL_Start_Receive = Start_Receive,
    .GYRO_HAL_Stop_Receive = Stop_Receive,
};
