//////////////////////////////////////////////////////////////////////////////////
// GUI硬件抽象层头文件
// 提供平台无关的GUI绘图接口抽象
// 支持动态注册不同平台的GUI实现
//////////////////////////////////////////////////////////////////////////////////

#ifndef __GUI_HAL_H
#define __GUI_HAL_H

#include "stdint.h"
#include "lcd.h"

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

/* --------------------------------- 此处修改GUI HAL接口结构体定义 -------------------------------- */
/** ***************************************************************************
 * @brief  GUI硬件抽象层接口结构体
 * @note   仅包含底层硬件操作，不包含业务逻辑
 *****************************************************************************/
typedef struct
{
    // 底层硬件操作函数
    void (*Set_Cursor)(u16 x, u16 y);                    // 设置光标
    void (*Set_Windows)(u16 x1, u16 y1, u16 x2, u16 y2); // 设置窗口
    void (*Write_Data_16Bit)(u16 data);                  // 写入16位数据
    u16 (*Read_Point)(u16 x, u16 y);                     // 读点

    // 显示控制函数
    void (*Display_On)(void);            // 显示开启
    void (*Display_Off)(void);           // 显示关闭
    void (*Set_Backlight)(u8 state);     // 背光控制
    void (*Set_Direction)(u8 direction); // 设置显示方向
    void (*Clear_Screen)(u16 color);     // 清屏

    // 寄存器操作函数
    void (*Write_Register)(u8 reg, u16 value); // 写寄存器
    u16 (*Read_Register)(u8 reg);              // 读寄存器

} GUI_HAL_t;

/* --------------------------------- 此处修改外部结构体声明 -------------------------------- */
/** ***************************************************************************
 * @brief  GUI硬件抽象层实例声明
 * @note   由平台实现文件导出，应用层直接调用 gui_hal.Function_Name()
 *****************************************************************************/
extern const GUI_HAL_t gui_hal;
/* 自定义部分结束 */

#endif /* __GUI_HAL_H */
