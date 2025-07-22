//////////////////////////////////////////////////////////////////////////////////
// GUI硬件抽象层实现文件
// 实现GUI HAL注册机制和封装函数
// 提供STM32平台的具体实现
//////////////////////////////////////////////////////////////////////////////////

#include "GUI_HAL.h"

// 声明外部变量
extern _lcd_dev lcddev;

/* --------------------------------- 此处修改GUI HAL实现相关代码 -------------------------------- */
// GUI HAL实现指针，用于存储注册的硬件实现
const GUI_HAL_t *gui_hal_ptr = NULL;  // 暴露给inline函数使用

/** ***************************************************************************
 * @brief  注册GUI硬件抽象层实现
 * @param  hal: 指向硬件抽象层实现结构体的指针
 * @retval None
 *****************************************************************************/
void GUI_HAL_Register(const GUI_HAL_t *hal)
{
    gui_hal_ptr = hal;
}

/** ***************************************************************************
 * @brief  GUI HAL层中频操作函数实现
 *****************************************************************************/
u16 GUI_HAL_Read_Point(u16 x, u16 y)
{
    if (gui_hal_ptr && gui_hal_ptr->Read_Point) {
        return gui_hal_ptr->Read_Point(x, y);
    }
    return 0;
}

void GUI_HAL_Clear_Screen(u16 color)
{
    if (gui_hal_ptr && gui_hal_ptr->Clear_Screen) {
        gui_hal_ptr->Clear_Screen(color);
    }
}

// ==================== STM32平台具体实现 ====================

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
 * @brief  STM32平台GUI硬件抽象层实现实例
 *****************************************************************************/
const GUI_HAL_t stm32_gui_hal = {
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

/** ***************************************************************************
 * @brief  初始化STM32平台GUI硬件抽象层
 * @retval None
 * @note   在系统初始化时调用此函数进行HAL层注册
 *****************************************************************************/
void STM32_GUI_HAL_Init(void)
{
    GUI_HAL_Register(&stm32_gui_hal);
}
/* 自定义部分结束 */
