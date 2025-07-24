//////////////////////////////////////////////////////////////////////////////////
// SPI硬件抽象层头文件
// 提供平台无关的SPI硬件接口抽象
// 支持动态注册不同平台的SPI实现
//////////////////////////////////////////////////////////////////////////////////

#ifndef __SPI_HAL_H
#define __SPI_HAL_H

#include "main.h"
#include "lcd_spi.h"
#include "stdint.h"

#ifndef u8
#define u8 uint8_t
#endif /* u8 */

#ifndef u16
#define u16 uint16_t
#endif /* u16 */

#ifndef u32
#define u32 uint32_t
#endif /* u32 */

/* --------------------------------- 此处修改SPI HAL接口结构体定义 -------------------------------- */
/** ***************************************************************************
 * @brief  SPI硬件抽象层接口结构体
 * @note   包含所有需要平台相关实现的SPI操作函数指针
 *****************************************************************************/
typedef struct
{
    // SPI基本通信函数
    u8 (*SPI_Write_Byte)(u8 data);               // SPI写入一个字节数据
    u8 (*SPI_Read_Byte)(void);                   // SPI读取一个字节数据
    void (*SPI_Write_Buffer)(u8 *data, u16 len); // SPI写入缓冲区数据
    void (*SPI_Read_Buffer)(u8 *data, u16 len);  // SPI读取缓冲区数据

    // SPI配置函数
    void (*SPI_Set_Speed)(u8 speed); // 设置SPI速度，0-高速，1-低速
    void (*SPI_Init)(void);          // SPI初始化函数
    void (*SPI_DeInit)(void);        // SPI反初始化函数

    // SPI MSP函数 (可选)
    void (*SPI_MSP_Init)(void);   // SPI MSP初始化函数
    void (*SPI_MSP_DeInit)(void); // SPI MSP反初始化函数

} spi_hal_t;

/* --------------------------------- 此处修改外部结构体声明 -------------------------------- */
/** ***************************************************************************
 * @brief  SPI硬件抽象层实例声明
 * @note   由平台实现文件导出，应用层直接调用 spi_hal.Function_Name()
 *****************************************************************************/
extern const spi_hal_t spi_hal;
/* 自定义部分结束 */

#endif /* __SPI_HAL_H */
