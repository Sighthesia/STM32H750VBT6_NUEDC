//////////////////////////////////////////////////////////////////////////////////
// SPI硬件抽象层实现文件
// 实现SPI HAL注册机制和封装函数
// 提供STM32平台的具体实现
//////////////////////////////////////////////////////////////////////////////////

#include "SPI_HAL.h"

/* --------------------------------- 此处修改SPI HAL实现相关代码 -------------------------------- */
// SPI HAL实现指针，用于存储注册的硬件实现
const SPI_HAL_t *spi_hal_ptr = NULL;  // 暴露给inline函数使用

/** ***************************************************************************
 * @brief  注册SPI硬件抽象层实现
 * @param  hal: 指向硬件抽象层实现结构体的指针
 * @retval None
 *****************************************************************************/
void SPI_HAL_Register(const SPI_HAL_t *hal)
{
    spi_hal_ptr = hal;
}

/** ***************************************************************************
 * @brief  SPI HAL层中频操作函数实现
 *****************************************************************************/
void SPI_HAL_Write_Buffer(u8 *data, u16 len)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_Write_Buffer) {
        spi_hal_ptr->SPI_Write_Buffer(data, len);
    }
}

void SPI_HAL_Read_Buffer(u8 *data, u16 len)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_Read_Buffer) {
        spi_hal_ptr->SPI_Read_Buffer(data, len);
    }
}

/** ***************************************************************************
 * @brief  SPI HAL层低频配置函数实现
 *****************************************************************************/
void SPI_HAL_Set_Speed(u8 speed)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_Set_Speed) {
        spi_hal_ptr->SPI_Set_Speed(speed);
    }
}

void SPI_HAL_Init(void)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_Init) {
        spi_hal_ptr->SPI_Init();
    }
}

void SPI_HAL_DeInit(void)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_DeInit) {
        spi_hal_ptr->SPI_DeInit();
    }
}

void SPI_HAL_MSP_Init(void)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_MSP_Init) {
        spi_hal_ptr->SPI_MSP_Init();
    }
}

void SPI_HAL_MSP_DeInit(void)
{
    if (spi_hal_ptr && spi_hal_ptr->SPI_MSP_DeInit) {
        spi_hal_ptr->SPI_MSP_DeInit();
    }
}

// ==================== STM32平台具体实现 ====================

/** ***************************************************************************
 * @brief  STM32平台SPI写入字节函数实现
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 *****************************************************************************/
static u8 STM32_SPI_Write_Byte(u8 data)
{
    return SPI_Write_Byte(&SPI_LCD_HANDLE_NAME, data);
}

/** ***************************************************************************
 * @brief  STM32平台SPI读取字节函数实现
 * @retval 接收到的数据
 *****************************************************************************/
static u8 STM32_SPI_Read_Byte(void)
{
    u8 rx_data = 0;
    HAL_SPI_Receive(&SPI_LCD_HANDLE_NAME, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

/** ***************************************************************************
 * @brief  STM32平台SPI写入缓冲区函数实现
 * @param  data: 要发送的数据缓冲区
 * @param  len: 数据长度
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Write_Buffer(u8 *data, u16 len)
{
    HAL_SPI_Transmit(&SPI_LCD_HANDLE_NAME, data, len, HAL_MAX_DELAY);
}

/** ***************************************************************************
 * @brief  STM32平台SPI读取缓冲区函数实现
 * @param  data: 接收数据缓冲区
 * @param  len: 数据长度
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Read_Buffer(u8 *data, u16 len)
{
    HAL_SPI_Receive(&SPI_LCD_HANDLE_NAME, data, len, HAL_MAX_DELAY);
}

/** ***************************************************************************
 * @brief  STM32平台SPI速度设置函数实现
 * @param  speed: SPI速度设置，0-高速，1-低速
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Set_Speed(u8 speed)
{
    SPI_Set_Speed(&SPI_LCD_HANDLE_NAME, speed);
}

/** ***************************************************************************
 * @brief  STM32平台SPI初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Init(void)
{
#if !USE_CUBEMX_SPI_CONFIG
    SPI_LCD_Init();
#endif /* !USE_CUBEMX_SPI_CONFIG */
}

/** ***************************************************************************
 * @brief  STM32平台SPI反初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_DeInit(void)
{
    HAL_SPI_DeInit(&SPI_LCD_HANDLE_NAME);
}

/** ***************************************************************************
 * @brief  STM32平台SPI MSP初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_MSP_Init(void)
{
    // HAL_SPI_MspInit会自动被HAL_SPI_Init调用
    // 这里可以留空或者添加额外的初始化代码
}

/** ***************************************************************************
 * @brief  STM32平台SPI MSP反初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_MSP_DeInit(void)
{
    // HAL_SPI_MspDeInit会自动被HAL_SPI_DeInit调用
    // 这里可以留空或者添加额外的反初始化代码
}

/** ***************************************************************************
 * @brief  STM32平台SPI硬件抽象层实现实例
 *****************************************************************************/
const SPI_HAL_t stm32_spi_hal = {
    .SPI_Write_Byte = STM32_SPI_Write_Byte,
    .SPI_Read_Byte = STM32_SPI_Read_Byte,
    .SPI_Write_Buffer = STM32_SPI_Write_Buffer,
    .SPI_Read_Buffer = STM32_SPI_Read_Buffer,
    .SPI_Set_Speed = STM32_SPI_Set_Speed,
    .SPI_Init = STM32_SPI_Init,
    .SPI_DeInit = STM32_SPI_DeInit,
    .SPI_MSP_Init = STM32_SPI_MSP_Init,
    .SPI_MSP_DeInit = STM32_SPI_MSP_DeInit};

/** ***************************************************************************
 * @brief  初始化STM32平台SPI硬件抽象层
 * @retval None
 * @note   在系统初始化时调用此函数进行HAL层注册
 *****************************************************************************/
void STM32_SPI_HAL_Init(void)
{
    SPI_HAL_Register(&stm32_spi_hal);
}
/* 自定义部分结束 */
