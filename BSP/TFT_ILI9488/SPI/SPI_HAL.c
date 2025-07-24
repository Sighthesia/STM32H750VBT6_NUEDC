/** ***************************************************************************
 * @File Name: SPI_HAL.c
 * @brief  SPI硬件抽象层 - 直接调用方式
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-22
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes : 删除注册机制，改为直接调用结构体
 * @Modifi Date : 2025-07-24
 */

#include "SPI_HAL.h"

/* --------------------------------- 此处为平台实现说明 -------------------------------- */
// 注意：此文件已删除，应用层直接调用 spi_hal.Function_Name()
// 具体实现由以下平台文件提供：
// - SPI_HAL_STM32.c (STM32平台)
// - SPI_HAL_ESP32.c (ESP32平台)
// - SPI_HAL_NRF52.c (NRF52平台)
//
// 使用方式：
// spi_hal.SPI_Write_Byte(data);
// spi_hal.SPI_Write_Buffer(buffer, len);
/* 自定义部分结束 */
