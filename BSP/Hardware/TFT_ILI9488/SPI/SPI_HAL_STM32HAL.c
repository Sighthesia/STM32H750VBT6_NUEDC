/** ***************************************************************************
 * @File Name: SPI_HAL_STM32HAL.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-22
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */

#include "SPI_HAL.h"

/** ***************************************************************************
 * @brief  SPI写入字节函数实现
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 */
static u8 Write_Byte(u8 data)
{
    return SPI_Write_Byte(&SPI_LCD_HANDLE_NAME, data);
}

/** ***************************************************************************
 * @brief  SPI读取字节函数实现
 * @retval 接收到的数据
 */
static u8 Read_Byte(void)
{
    u8 rx_data = 0;
    HAL_SPI_Receive(&SPI_LCD_HANDLE_NAME, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

/** ***************************************************************************
 * @brief  SPI写入缓冲区函数实现
 * @param  data: 要发送的数据缓冲区
 * @param  len: 数据长度
 * @retval None
 */
static void Write_Buffer(u8 *data, u16 len)
{
    HAL_SPI_Transmit(&SPI_LCD_HANDLE_NAME, data, len, HAL_MAX_DELAY);
}

/** ***************************************************************************
 * @brief  SPI读取缓冲区函数实现
 * @param  data: 接收数据缓冲区
 * @param  len: 数据长度
 * @retval None
 */
static void Read_Buffer(u8 *data, u16 len)
{
    HAL_SPI_Receive(&SPI_LCD_HANDLE_NAME, data, len, HAL_MAX_DELAY);
}

/** ***************************************************************************
 * @brief  SPI速度设置函数实现
 * @param  speed: SPI速度设置，0-高速，1-低速
 * @retval None
 */
static void Set_Speed(u8 speed)
{
    SPI_Set_Speed(&SPI_LCD_HANDLE_NAME, speed);
}

/** ***************************************************************************
 * @brief  SPI初始化函数实现
 * @retval None
 */
static void Init(void)
{
#if !USE_CUBEMX_SPI_CONFIG
    SPI_LCD_Init();
#endif /* !USE_CUBEMX_SPI_CONFIG */
}

/** ***************************************************************************
 * @brief  SPI反初始化函数实现
 * @retval None
 */
static void DeInit(void)
{
    HAL_SPI_DeInit(&SPI_LCD_HANDLE_NAME);
}

/** ***************************************************************************
 * @brief  SPI MSP初始化函数实现
 * @retval None
 */
static void MSP_Init(void)
{
    // HAL_SPI_MspInit会自动被HAL_SPI_Init调用
    // 这里可以留空或者添加额外的初始化代码
}

/** ***************************************************************************
 * @brief  SPI MSP反初始化函数实现
 * @retval None
 */
static void MSP_DeInit(void)
{
    // HAL_SPI_MspDeInit会自动被HAL_SPI_DeInit调用
    // 这里可以留空或者添加额外的反初始化代码
}

/** ***************************************************************************
 * @brief  SPI硬件抽象层实现实例（导出给应用层）
 */
const spi_hal_t spi_hal = {
    .SPI_Write_Byte = Write_Byte,
    .SPI_Read_Byte = Read_Byte,
    .SPI_Write_Buffer = Write_Buffer,
    .SPI_Read_Buffer = Read_Buffer,
    .SPI_Set_Speed = Set_Speed,
    .SPI_Init = Init,
    .SPI_DeInit = DeInit,
    .SPI_MSP_Init = MSP_Init,
    .SPI_MSP_DeInit = MSP_DeInit};

/* 自定义部分结束 */
