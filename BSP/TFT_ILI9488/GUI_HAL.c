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
static const GUI_HAL_t *gui_hal_ptr = NULL;

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
 * @brief  GUI HAL层底层硬件操作函数
 *****************************************************************************/
void GUI_HAL_SET_CURSOR(u16 x, u16 y)
{
    if (gui_hal_ptr && gui_hal_ptr->Set_Cursor)
    {
        gui_hal_ptr->Set_Cursor(x, y);
    }
}

void GUI_HAL_SET_WINDOWS(u16 x1, u16 y1, u16 x2, u16 y2)
{
    if (gui_hal_ptr && gui_hal_ptr->Set_Windows)
    {
        gui_hal_ptr->Set_Windows(x1, y1, x2, y2);
    }
}

u16 GUI_HAL_READ_POINT(u16 x, u16 y)
{
    if (gui_hal_ptr && gui_hal_ptr->Read_Point)
    {
        return gui_hal_ptr->Read_Point(x, y);
    }
    return 0;
}

/** ***************************************************************************
 * @brief  GUI HAL层清屏函数
 * @param  color: 填充颜色
 * @retval None
 *****************************************************************************/
void GUI_HAL_CLEAR_SCREEN(u16 color)
{
    if (gui_hal_ptr && gui_hal_ptr->Clear_Screen)
    {
        gui_hal_ptr->Clear_Screen(color);
    }
}

/** ***************************************************************************
 * @brief  GUI HAL层显示控制函数
 * @retval None
 *****************************************************************************/
void GUI_HAL_DISPLAY_ON(void)
{
    if (gui_hal_ptr && gui_hal_ptr->Display_On)
    {
        gui_hal_ptr->Display_On();
    }
}

void GUI_HAL_DISPLAY_OFF(void)
{
    if (gui_hal_ptr && gui_hal_ptr->Display_Off)
    {
        gui_hal_ptr->Display_Off();
    }
}

/** ***************************************************************************
 * @brief  GUI HAL层背光控制函数
 * @param  state: 背光状态，1-开启，0-关闭
 * @retval None
 *****************************************************************************/
void GUI_HAL_SET_BACKLIGHT(u8 state)
{
    if (gui_hal_ptr && gui_hal_ptr->Set_Backlight)
    {
        gui_hal_ptr->Set_Backlight(state);
    }
}

/** ***************************************************************************
 * @brief  GUI HAL层方向设置函数
 * @param  direction: 显示方向
 * @retval None
 *****************************************************************************/
void GUI_HAL_SET_DIRECTION(u8 direction)
{
    if (gui_hal_ptr && gui_hal_ptr->Set_Direction)
    {
        gui_hal_ptr->Set_Direction(direction);
    }
}

/** ***************************************************************************
 * @brief  GUI HAL层数据写入函数
 * @param  data: 16位数据
 * @retval None
 *****************************************************************************/
void GUI_HAL_WRITE_DATA_16BIT(u16 data)
{
    if (gui_hal_ptr && gui_hal_ptr->Write_Data_16Bit)
    {
        gui_hal_ptr->Write_Data_16Bit(data);
    }
}

/** ***************************************************************************
 * @brief  GUI HAL层寄存器操作函数
 * @param  reg: 寄存器地址
 * @param  value: 写入值
 * @retval None
 *****************************************************************************/
void GUI_HAL_WRITE_REGISTER(u8 reg, u16 value)
{
    if (gui_hal_ptr && gui_hal_ptr->Write_Register)
    {
        gui_hal_ptr->Write_Register(reg, value);
    }
}

u16 GUI_HAL_READ_REGISTER(u8 reg)
{
    if (gui_hal_ptr && gui_hal_ptr->Read_Register)
    {
        return gui_hal_ptr->Read_Register(reg);
    }
    return 0;
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
    return LCD_Read_Point(x, y);
}

static void STM32_Clear_Screen(u16 color)
{
    LCD_Clear(color);
}

static void STM32_Display_On(void)
{
    LCD_Display_On();
}

static void STM32_Display_Off(void)
{
    LCD_Display_Off();
}

static void STM32_Set_Backlight(u8 state)
{
#if LCD_USE_BACKLIGHT_CONTROL
    LCD_Set_Backlight(state);
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
    .Set_Backlight = STM32_Set_Backlight,
    .Set_Direction = STM32_Set_Direction,
    .Clear_Screen = STM32_Clear_Screen,
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
