#include "UART_HAL.h"
#include "main.h"

/* --------------------------------- 此处修改STM32平台UART配置 -------------------------------- */
// UART句柄数组
static UART_HandleTypeDef g_uart_handles[UART_HAL_MAX_CHANNELS];

// 接收回调函数数组
static uart_hal_rx_callback_t g_rx_callbacks[UART_HAL_MAX_CHANNELS] = {NULL};

// 单字节接收缓冲区
static uint8_t g_rx_byte[UART_HAL_MAX_CHANNELS];

// 内部函数声明
static USART_TypeDef *Get_UART_Instance(uart_hal_channel_t channel);
static IRQn_Type Get_UART_IRQn(uart_hal_channel_t channel);
static void Config_UART_GPIO(uart_hal_channel_t channel);
static void Config_UART_RCC(uart_hal_channel_t channel);
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief 获取UART外设实例
 * @param channel UART通道号
 * @return UART外设指针
 */
static USART_TypeDef *Get_UART_Instance(uart_hal_channel_t channel)
{
    switch (channel)
    {
    case UART_HAL_CHANNEL_1:
        return USART1;
    case UART_HAL_CHANNEL_2:
        return USART2;
    case UART_HAL_CHANNEL_3:
        return USART3;
    case UART_HAL_CHANNEL_4:
        return UART4;
    case UART_HAL_CHANNEL_5:
        return UART5;
    case UART_HAL_CHANNEL_6:
        return USART6;
    default:
        return NULL;
    }
}

/** ***************************************************************************
 * @brief 获取UART中断号
 * @param channel UART通道号
 * @return 中断号
 */
static IRQn_Type Get_UART_IRQn(uart_hal_channel_t channel)
{
    switch (channel)
    {
    case UART_HAL_CHANNEL_1:
        return USART1_IRQn;
    case UART_HAL_CHANNEL_2:
        return USART2_IRQn;
    case UART_HAL_CHANNEL_3:
        return USART3_IRQn;
    case UART_HAL_CHANNEL_4:
        return UART4_IRQn;
    case UART_HAL_CHANNEL_5:
        return UART5_IRQn;
    case UART_HAL_CHANNEL_6:
        return USART6_IRQn;
    default:
        return NonMaskableInt_IRQn;
    }
}

/** ***************************************************************************
 * @brief 配置UART时钟
 * @param channel UART通道号
 */
static void Config_UART_RCC(uart_hal_channel_t channel)
{
    switch (channel)
    {
    case UART_HAL_CHANNEL_1:
        __HAL_RCC_USART1_CLK_ENABLE();
        break;
    case UART_HAL_CHANNEL_2:
        __HAL_RCC_USART2_CLK_ENABLE();
        break;
    case UART_HAL_CHANNEL_3:
        __HAL_RCC_USART3_CLK_ENABLE();
        break;
    case UART_HAL_CHANNEL_4:
        __HAL_RCC_UART4_CLK_ENABLE();
        break;
    case UART_HAL_CHANNEL_5:
        __HAL_RCC_UART5_CLK_ENABLE();
        break;
    case UART_HAL_CHANNEL_6:
        __HAL_RCC_USART6_CLK_ENABLE();
        break;
    default:
        break;
    }
}

/** ***************************************************************************
 * @brief 配置UART GPIO引脚
 * @param channel UART通道号
 */
static void Config_UART_GPIO(uart_hal_channel_t channel)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    switch (channel)
    {
    case UART_HAL_CHANNEL_1:
        // USART1 GPIO配置 (PA9/PA10)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;

    case UART_HAL_CHANNEL_2:
        // USART2 GPIO配置 (PA2/PA3)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;

    case UART_HAL_CHANNEL_3:
        // USART3 GPIO配置 (PB10/PB11)
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        break;

    case UART_HAL_CHANNEL_4:
        // UART4 GPIO配置 (PA0/PA1)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;

    case UART_HAL_CHANNEL_5:
        // UART5 GPIO配置 (PC12/PD2)
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;

    case UART_HAL_CHANNEL_6:
        // USART6 GPIO配置 (PC6/PC7)
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        break;

    default:
        break;
    }
}

/** ***************************************************************************
 * @brief 初始化UART通道
 * @param channel UART通道号
 * @param config UART配置参数
 */
static void Init(uart_hal_channel_t channel, const uart_hal_config_t *config)
{
    if (channel >= UART_HAL_MAX_CHANNELS || config == NULL)
        return;

    // 配置时钟
    Config_UART_RCC(channel);

    // 配置GPIO
    Config_UART_GPIO(channel);

    // 配置UART句柄
    g_uart_handles[channel].Instance = Get_UART_Instance(channel);
    g_uart_handles[channel].Init.BaudRate = config->baud_rate;
    g_uart_handles[channel].Init.WordLength = UART_WORDLENGTH_8B;
    g_uart_handles[channel].Init.StopBits = UART_STOPBITS_1;
    g_uart_handles[channel].Init.Parity = UART_PARITY_NONE;
    g_uart_handles[channel].Init.Mode = UART_MODE_TX_RX;
    g_uart_handles[channel].Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart_handles[channel].Init.OverSampling = UART_OVERSAMPLING_16;
    g_uart_handles[channel].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    g_uart_handles[channel].Init.ClockPrescaler = UART_PRESCALER_DIV1;
    g_uart_handles[channel].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    // 初始化UART
    if (HAL_UART_Init(&g_uart_handles[channel]) != HAL_OK)
    {
        return;
    }

    // 配置中断优先级
    HAL_NVIC_SetPriority(Get_UART_IRQn(channel), 5, 0);
    HAL_NVIC_EnableIRQ(Get_UART_IRQn(channel));
}

/** ***************************************************************************
 * @brief 发送单个字节
 * @param channel UART通道号
 * @param data 要发送的数据
 */
static void Send_Byte(uart_hal_channel_t channel, uint8_t data)
{
    if (channel >= UART_HAL_MAX_CHANNELS)
        return;

    HAL_UART_Transmit(&g_uart_handles[channel], &data, 1, HAL_MAX_DELAY);
}

/** ***************************************************************************
 * @brief 发送数据缓冲区
 * @param channel UART通道号
 * @param buf 数据缓冲区指针
 * @param len 数据长度
 */
static void Send_Buffer(uart_hal_channel_t channel, const uint8_t *buf, uint16_t len)
{
    if (channel >= UART_HAL_MAX_CHANNELS || buf == NULL || len == 0)
        return;

    HAL_UART_Transmit(&g_uart_handles[channel], (uint8_t *)buf, len, HAL_MAX_DELAY);
}

/** ***************************************************************************
 * @brief 设置接收数据回调函数
 * @param channel UART通道号
 * @param callback 回调函数指针
 */
static void Set_RX_Callback(uart_hal_channel_t channel, uart_hal_rx_callback_t callback)
{
    if (channel >= UART_HAL_MAX_CHANNELS)
        return;

    g_rx_callbacks[channel] = callback;
}

/** ***************************************************************************
 * @brief 启用UART接收中断
 * @param channel UART通道号
 */
static void Enable_RX_IT(uart_hal_channel_t channel)
{
    if (channel >= UART_HAL_MAX_CHANNELS)
        return;

    HAL_UART_Receive_IT(&g_uart_handles[channel], &g_rx_byte[channel], 1);
}

/** ***************************************************************************
 * @brief 禁用UART接收中断
 * @param channel UART通道号
 */
static void Disable_RX_IT(uart_hal_channel_t channel)
{
    if (channel >= UART_HAL_MAX_CHANNELS)
        return;

    HAL_UART_AbortReceive_IT(&g_uart_handles[channel]);
}

/** ***************************************************************************
 * @brief 检查发送完成状态
 * @param channel UART通道号
 * @return 1-发送完成，0-发送中
 */
static uint8_t Is_TX_Complete(uart_hal_channel_t channel)
{
    if (channel >= UART_HAL_MAX_CHANNELS)
        return 1;

    return (HAL_UART_GetState(&g_uart_handles[channel]) == HAL_UART_STATE_READY) ? 1 : 0;
}

/** ***************************************************************************
 * @brief 处理UART中断（供中断服务程序调用）
 * @param channel UART通道号
 */
static void IRQ_Handler(uart_hal_channel_t channel)
{
    if (channel >= UART_HAL_MAX_CHANNELS)
        return;

    HAL_UART_IRQHandler(&g_uart_handles[channel]);
}

/** ***************************************************************************
 * @brief UART接收完成回调函数（由HAL库调用）
 * @param huart UART句柄指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_hal_channel_t channel = UART_HAL_MAX_CHANNELS;

    // 找到对应的通道
    for (uint8_t i = 0; i < UART_HAL_MAX_CHANNELS; i++)
    {
        if (huart == &g_uart_handles[i])
        {
            channel = (uart_hal_channel_t)i;
            break;
        }
    }

    if (channel < UART_HAL_MAX_CHANNELS)
    {
        // 调用用户回调函数
        if (g_rx_callbacks[channel] != NULL)
        {
            g_rx_callbacks[channel](channel, g_rx_byte[channel]);
        }

        // 重新启动接收
        HAL_UART_Receive_IT(&g_uart_handles[channel], &g_rx_byte[channel], 1);
    }
}

// 绑定实现到全局结构体
const uart_hal_t uart_hal = {
    .UART_HAL_Init = Init,
    .UART_HAL_Send_Byte = Send_Byte,
    .UART_HAL_Send_Buffer = Send_Buffer,
    .UART_HAL_Set_RX_Callback = Set_RX_Callback,
    .UART_HAL_Enable_RX_IT = Enable_RX_IT,
    .UART_HAL_Disable_RX_IT = Disable_RX_IT,
    .UART_HAL_Is_TX_Complete = Is_TX_Complete,
    .UART_HAL_IRQ_Handler = IRQ_Handler,
};
