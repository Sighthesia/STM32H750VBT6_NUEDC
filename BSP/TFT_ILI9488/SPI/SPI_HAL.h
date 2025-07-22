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
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

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

} SPI_HAL_t;
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief  注册SPI硬件抽象层实现
 * @param  hal: 指向硬件抽象层实现结构体的指针
 * @retval None
 *****************************************************************************/
void SPI_HAL_Register(const SPI_HAL_t *hal);

/* --------------------------------- 此处修改SPI HAL封装函数声明 -------------------------------- */
/** ***************************************************************************
 * @brief  SPI HAL层高频操作函数(内联优化)
 *****************************************************************************/
// 外部指针声明，供inline函数使用
extern const SPI_HAL_t *spi_hal_ptr;

/** ***************************************************************************
 * @brief  SPI HAL层写入字节函数(高频调用，内联优化)
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 *****************************************************************************/
static inline u8 SPI_HAL_Write_Byte(u8 data) {
    if (spi_hal_ptr && spi_hal_ptr->SPI_Write_Byte) {
        return spi_hal_ptr->SPI_Write_Byte(data);
    }
    return 0;
}

/** ***************************************************************************
 * @brief  SPI HAL层读取字节函数(高频调用，内联优化)
 * @retval 接收到的数据
 *****************************************************************************/
static inline u8 SPI_HAL_Read_Byte(void) {
    if (spi_hal_ptr && spi_hal_ptr->SPI_Read_Byte) {
        return spi_hal_ptr->SPI_Read_Byte();
    }
    return 0;
}

/** ***************************************************************************
 * @brief  SPI HAL层中频操作函数(普通函数)
 *****************************************************************************/
void SPI_HAL_Write_Buffer(u8 *data, u16 len);
void SPI_HAL_Read_Buffer(u8 *data, u16 len);

/** ***************************************************************************
 * @brief  SPI HAL层低频配置函数
 *****************************************************************************/
void SPI_HAL_Set_Speed(u8 speed);
void SPI_HAL_Init(void);
void SPI_HAL_DeInit(void);
void SPI_HAL_MSP_Init(void);
void SPI_HAL_MSP_DeInit(void);

/** ***************************************************************************
 * @brief  初始化STM32平台SPI硬件抽象层
 * @retval None
 * @note   在系统初始化时调用此函数进行HAL层注册
 *****************************************************************************/
void STM32_SPI_HAL_Init(void);
/* 自定义部分结束 */

#endif /* __SPI_HAL_H */
