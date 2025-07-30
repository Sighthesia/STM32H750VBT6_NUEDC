//////////////////////////////////////////////////////////////////////////////////
// LCD硬件抽象层头文件
// 提供平台无关的LCD硬件接口抽象
// 支持动态注册不同平台的硬件实现
//////////////////////////////////////////////////////////////////////////////////

#ifndef __LCD_HAL_H
#define __LCD_HAL_H

#include "stdint.h"
#include "lcd.h"
#include "GUI_HAL.h"

#ifndef u8
#define u8 uint8_t
#endif /* u8 */

#ifndef u16
#define u16 uint16_t
#endif /* u16 */

#ifndef u32
#define u32 uint32_t
#endif /* u32 */

/* --------------------------------- 此处修改HAL接口结构体定义 -------------------------------- */
/** ***************************************************************************
 * @brief  LCD硬件抽象层接口结构体
 * @note   包含所有需要平台相关实现的硬件操作函数指针
 *****************************************************************************/
typedef struct
{
    // GPIO控制函数
    void (*GPIO_CS_Set)(void); // CS引脚置高
    void (*GPIO_CS_Clr)(void); // CS引脚置低
    void (*GPIO_DC_Set)(void); // DC引脚置高 (数据模式)
    void (*GPIO_DC_Clr)(void); // DC引脚置低 (命令模式)

    // 可选功能的GPIO控制函数
    void (*GPIO_RST_Set)(void); // RST引脚置高 (非复位状态)
    void (*GPIO_RST_Clr)(void); // RST引脚置低 (复位状态)
    void (*GPIO_BLK_Set)(void); // 背光引脚置高 (开启背光)
    void (*GPIO_BLK_Clr)(void); // 背光引脚置低 (关闭背光)

    // SPI通信函数
    u8 (*SPI_Write_Byte)(u8 data); // SPI写入一个字节数据

    // 延时函数
    void (*Delay_Ms)(u32 ms); // 毫秒延时函数

    // SPI速度设置函数 (可选)
    void (*SPI_Set_Speed)(u8 speed); // 设置SPI速度，0-高速，1-低速

    // SPI初始化函数 (可选)
    void (*SPI_Init)(void);       // SPI初始化函数
    void (*SPI_MSP_Init)(void);   // SPI MSP初始化函数
    void (*SPI_MSP_DeInit)(void); // SPI MSP反初始化函数

} LCD_HAL_t;

/* --------------------------------- 此处修改外部结构体声明 -------------------------------- */
/** ***************************************************************************
 * @brief  LCD硬件抽象层实例声明
 * @note   由平台实现文件导出，应用层直接调用 lcd_hal.Function_Name()
 *****************************************************************************/
extern const LCD_HAL_t lcd_hal;
/* 自定义部分结束 */

#endif /* __LCD_HAL_H */
