//////////////////////////////////////////////////////////////////////////////////
// GUI硬件抽象层实现文件
// 移除注册机制，直接导出结构体供应用层调用
// 应用层调用方式：gui_hal.Function_Name()
//////////////////////////////////////////////////////////////////////////////////

#include "GUI_HAL.h"

// 声明外部变量
extern _lcd_dev lcddev;

/* ================================= STM32平台具体实现 ================================ */

/** ***************************************************************************
 * @brief  STM32平台GUI底层硬件操作函数实现
 *****************************************************************************/
static void STM32_Set_Cursor(u16 x, u16 y)
{
    LCD_Set_Cursor(x, y);
}

static void STM32_Set_Windows(u16 x1, u16 y1, u16 x2, u16 y2)
{
    LCD_Set_Windows(x1, y1, x2, y2);
}

static u16 STM32_Read_Point(u16 x, u16 y)
{
    // 原始LCD驱动没有读取功能，返回默认值
    // 可以根据需要实现具体的读取逻辑
    (void)x;
    (void)y;
    return 0;
}

static void STM32_Clear_Screen(u16 color)
{
    LCD_Clear(color);
}

static void STM32_Display_On(void)
{
    // 原始LCD驱动没有单独的显示开关函数
    // 可以通过背光控制或其他方式实现
#if LCD_USE_BACKLIGHT_CONTROL
    LCD_Set_Backlight(1);
#endif
}

static void STM32_Display_Off(void)
{
    // 原始LCD驱动没有单独的显示开关函数
    // 可以通过背光控制或其他方式实现
#if LCD_USE_BACKLIGHT_CONTROL
    LCD_Set_Backlight(0);
#endif
}

static void STM32_Set_Backlight(u8 state)
{
#if LCD_USE_BACKLIGHT_CONTROL
    LCD_Set_Backlight(state);
#else
    (void)state; // 避免未使用参数警告
#endif /* LCD_USE_BACKLIGHT_CONTROL */
}

static void STM32_Set_Direction(u8 direction)
{
    LCD_Direction(direction);
}

static void STM32_Write_Data_16Bit(u16 data)
{
    LCD_Write_Data_16Bit(data);
}

/** ***************************************************************************
 * @brief  STM32平台GUI硬件抽象层实现实例（导出给应用层）
 *****************************************************************************/
const GUI_HAL_t gui_hal = {
    .Set_Cursor = STM32_Set_Cursor,
    .Set_Windows = STM32_Set_Windows,
    .Write_Data_16Bit = STM32_Write_Data_16Bit,
    .Read_Point = STM32_Read_Point,
    .Clear_Screen = STM32_Clear_Screen,
    .Display_On = STM32_Display_On,
    .Display_Off = STM32_Display_Off,
    .Set_Backlight = STM32_Set_Backlight,
    .Set_Direction = STM32_Set_Direction,
};

/* 自定义部分结束 */
