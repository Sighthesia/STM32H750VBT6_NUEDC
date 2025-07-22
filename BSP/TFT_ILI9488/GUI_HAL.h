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
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief  注册GUI硬件抽象层实现
 * @param  hal: 指向硬件抽象层实现结构体的指针
 * @retval None
 *****************************************************************************/
void GUI_HAL_Register(const GUI_HAL_t *hal);

/* --------------------------------- 此处修改GUI HAL封装函数声明 -------------------------------- */
// 暴露HAL指针供inline函数使用
extern const GUI_HAL_t *gui_hal_ptr;

/** ***************************************************************************
 * @brief  GUI HAL层高频操作函数 - inline优化
 *****************************************************************************/
static inline void GUI_HAL_Set_Cursor(u16 x, u16 y) {
    gui_hal_ptr->Set_Cursor(x, y);
}

static inline void GUI_HAL_Write_Data_16Bit(u16 data) {
    gui_hal_ptr->Write_Data_16Bit(data);
}

static inline void GUI_HAL_Set_Windows(u16 x1, u16 y1, u16 x2, u16 y2) {
    gui_hal_ptr->Set_Windows(x1, y1, x2, y2);
}

/** ***************************************************************************
 * @brief  GUI HAL层中频操作函数
 *****************************************************************************/
u16 GUI_HAL_Read_Point(u16 x, u16 y);
void GUI_HAL_Clear_Screen(u16 color);

/** ***************************************************************************
 * @brief  GUI HAL层低频控制函数
 *****************************************************************************/
void GUI_HAL_Display_On(void);
void GUI_HAL_Display_Off(void);
void GUI_HAL_Set_Backlight(u8 state);
void GUI_HAL_Set_Direction(u8 direction);

/** ***************************************************************************
 * @brief  GUI HAL层寄存器操作函数
 *****************************************************************************/
void GUI_HAL_Write_Register(u8 reg, u16 value);
u16 GUI_HAL_Read_Register(u8 reg);

/** ***************************************************************************
 * @brief  初始化STM32平台GUI硬件抽象层
 * @retval None
 * @note   在系统初始化时调用此函数进行HAL层注册
 *****************************************************************************/
void STM32_GUI_HAL_Init(void);
/* 自定义部分结束 */

#endif /* __GUI_HAL_H */
