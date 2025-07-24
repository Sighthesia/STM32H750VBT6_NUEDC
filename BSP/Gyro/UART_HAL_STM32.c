#include "UART_HAL.h"
#include "main.h"

/* --------------------------------- 此处修改STM32平台配置 -------------------------------- */
// 外部变量声明（来自原有代码）
extern uint8 ttl_receive;
extern uint8 Fd_data[64];
extern uint8 Fd_rsimu[64];
extern uint8 Fd_rsahrs[56];
extern int rs_imutype;
extern int rs_ahrstype;

// RS485控制引脚定义
#define RS485_RX_DE PAout(11) // 485模式控制.0,接收;1,发送.
#define RS485_RX_RE PAout(12) // 485模式控制.0,接收;1,发送.

// UART通道映射到STM32外设
static USART_TypeDef *Get_UART_Instance(uart_hal_channel_t channel);
static uint32_t Get_UART_RCC_Clock(uart_hal_channel_t channel);
static uint8_t Get_UART_IRQn(uart_hal_channel_t channel);
static void Config_UART_GPIO(uart_hal_channel_t channel);

// 接收回调函数数组
static uart_hal_rx_callback_t rx_callbacks[UART_HAL_MAX_CHANNELS] = {0};
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
    default:
        return NULL;
    }
}

/** ***************************************************************************
 * @brief 获取UART RCC时钟
 * @param channel UART通道号
 * @return RCC时钟掩码
 */
static uint32_t Get_UART_RCC_Clock(uart_hal_channel_t channel)
{
    switch (channel)
    {
    case UART_HAL_CHANNEL_1:
        return RCC_APB2Periph_USART1;
    case UART_HAL_CHANNEL_2:
        return RCC_APB1Periph_USART2;
    case UART_HAL_CHANNEL_3:
        return RCC_APB1Periph_USART3;
    case UART_HAL_CHANNEL_4:
        return RCC_APB1Periph_UART4;
    case UART_HAL_CHANNEL_5:
        return RCC_APB1Periph_UART5;
    default:
        return 0;
    }
}

/** ***************************************************************************
 * @brief 获取UART中断号
 * @param channel UART通道号
 * @return 中断号
 */
static uint8_t Get_UART_IRQn(uart_hal_channel_t channel)
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
    default:
        return 0;
    }
}

/** ***************************************************************************
 * @brief 配置UART GPIO引脚
 * @param channel UART通道号
 */
static void Config_UART_GPIO(uart_hal_channel_t channel)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    switch (channel)
    {
    case UART_HAL_CHANNEL_1:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        // USART1_TX (PA9)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        // USART1_RX (PA10)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        break;

    case UART_HAL_CHANNEL_3:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
        // USART3_TX (PC10)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        // USART3_RX (PC11)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        break;

    case UART_HAL_CHANNEL_5:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
        // UART5_TX (PC12)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        // UART5_RX (PD2)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        break;

    default:
        // 其他通道的GPIO配置可以在此添加
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
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_TypeDef *uart_instance = Get_UART_Instance(channel);

    if (uart_instance == NULL || config == NULL)
        return;

    // 使能时钟
    if (channel == UART_HAL_CHANNEL_1)
    {
        RCC_APB2PeriphClockCmd(Get_UART_RCC_Clock(channel), ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(Get_UART_RCC_Clock(channel), ENABLE);
    }

    // 配置GPIO
    Config_UART_GPIO(channel);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = Get_UART_IRQn(channel);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (channel == UART_HAL_CHANNEL_1) ? 1 : 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 配置UART
    USART_InitStructure.USART_BaudRate = config->baud_rate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart_instance, &USART_InitStructure);

    USART_Cmd(uart_instance, ENABLE);
}

/** ***************************************************************************
 * @brief 发送单个字节
 * @param channel UART通道号
 * @param data 要发送的数据
 */
static void Send_Byte(uart_hal_channel_t channel, uint8_t data)
{
    USART_TypeDef *uart_instance = Get_UART_Instance(channel);
    if (uart_instance == NULL)
        return;

    USART_SendData(uart_instance, data);
    while (USART_GetFlagStatus(uart_instance, USART_FLAG_TC) == RESET)
        ;
}

/** ***************************************************************************
 * @brief 发送数据缓冲区
 * @param channel UART通道号
 * @param buf 数据缓冲区指针
 * @param len 数据长度
 */
static void Send_Buffer(uart_hal_channel_t channel, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return;

    for (uint16_t i = 0; i < len; i++)
    {
        Send_Byte(channel, buf[i]);
    }
}

/** ***************************************************************************
 * @brief 设置接收数据回调函数
 * @param channel UART通道号
 * @param callback 回调函数指针
 */
static void Set_RX_Callback(uart_hal_channel_t channel, uart_hal_rx_callback_t callback)
{
    if (channel < UART_HAL_MAX_CHANNELS)
    {
        rx_callbacks[channel] = callback;
    }
}

/** ***************************************************************************
 * @brief 启用UART接收中断
 * @param channel UART通道号
 */
static void Enable_RX_IT(uart_hal_channel_t channel)
{
    USART_TypeDef *uart_instance = Get_UART_Instance(channel);
    if (uart_instance == NULL)
        return;

    USART_ITConfig(uart_instance, USART_IT_RXNE, ENABLE);
}

/** ***************************************************************************
 * @brief 禁用UART接收中断
 * @param channel UART通道号
 */
static void Disable_RX_IT(uart_hal_channel_t channel)
{
    USART_TypeDef *uart_instance = Get_UART_Instance(channel);
    if (uart_instance == NULL)
        return;

    USART_ITConfig(uart_instance, USART_IT_RXNE, DISABLE);
}

/** ***************************************************************************
 * @brief 检查发送完成状态
 * @param channel UART通道号
 * @return 1-发送完成，0-发送中
 */
static uint8_t Is_TX_Complete(uart_hal_channel_t channel)
{
    USART_TypeDef *uart_instance = Get_UART_Instance(channel);
    if (uart_instance == NULL)
        return 1;

    return (USART_GetFlagStatus(uart_instance, USART_FLAG_TC) != RESET) ? 1 : 0;
}

/** ***************************************************************************
 * @brief 处理UART中断（供中断服务程序调用）
 * @param channel UART通道号
 */
static void IRQ_Handler(uart_hal_channel_t channel)
{
    USART_TypeDef *uart_instance = Get_UART_Instance(channel);
    if (uart_instance == NULL)
        return;

    if (USART_GetITStatus(uart_instance, USART_IT_RXNE) != RESET)
    {
        uint8_t received_data = USART_ReceiveData(uart_instance);

        // 特殊处理UART5的RS485控制
        if (channel == UART_HAL_CHANNEL_5)
        {
            RS485_RX_RE = 0;
            RS485_RX_DE = 0;
            ttl_receive = 1;
        }

        // 调用回调函数
        if (rx_callbacks[channel] != NULL)
        {
            rx_callbacks[channel](channel, received_data);
        }

        USART_ClearITPendingBit(uart_instance, USART_IT_RXNE);
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
