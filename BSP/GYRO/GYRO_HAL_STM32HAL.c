#include "GYRO_HAL.h"
#include "GYRO_Config.h"
#include "main.h"

/* --------------------------------- 此处修改STM32平台陀螺仪接收实现 -------------------------------- */
// STM32 HAL UART相关变量
static uint8_t g_rx_byte;
static uint8_t g_uart_channel = GYRO_DEFAULT_UART_CHANNEL;

// 接收回调函数指针
static void (*g_rx_callback)(uint8_t data) = NULL;

// 静态内部函数（命名简洁，static限制作用域）
static void Init_Hardware(uint8_t uart_channel, uint32_t baud_rate);
static void Set_Receive_Callback(void (*callback)(uint8_t data));
static void Start_UART_Receive(void);
static void Stop_UART_Receive(void);
static void Delay_Ms_STM32(uint32_t ms);
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief 初始化陀螺仪数据接收
 * @param uart_channel 使用的UART通道
 * @param baud_rate 波特率
 */
static void Init_Hardware(uint8_t uart_channel, uint32_t baud_rate)
{
    g_uart_channel = uart_channel;
    // 注意：实际的UART初始化应由MX_USART3_UART_Init()在main.c中处理
    // 这里仅保存配置参数
    (void)baud_rate; // 忽略未使用参数警告，波特率由CubeMX配置
}

/** ***************************************************************************
 * @brief 设置接收回调函数
 * @param callback 接收回调函数
 */
static void Set_Receive_Callback(void (*callback)(uint8_t data))
{
    g_rx_callback = callback;
}

/** ***************************************************************************
 * @brief 启动陀螺仪数据接收
 */
static void Start_UART_Receive(void)
{
    // 启动UART中断接收模式
    HAL_UART_Receive_IT(&GYRO_UART_HANDLE, &g_rx_byte, 1);
}

/** ***************************************************************************
 * @brief 停止陀螺仪数据接收
 */
static void Stop_UART_Receive(void)
{
    // 停止UART中断接收
    HAL_UART_AbortReceive_IT(&GYRO_UART_HANDLE);
}

/** ***************************************************************************
 * @brief 延时函数（STM32 HAL实现）
 * @param ms 延时毫秒数
 */
static void Delay_Ms_STM32(uint32_t ms)
{
    HAL_Delay(ms);
}

// 初始化全局HAL结构体，确保所有函数指针非NULL
gyro_hal_t gyro_hal = {
    .GYRO_Init = Init_Hardware,
    .Set_RX_Callback = Set_Receive_Callback,
    .Start_Receive = Start_UART_Receive,
    .Stop_Receive = Stop_UART_Receive,
    .Delay_Ms = Delay_Ms_STM32};
