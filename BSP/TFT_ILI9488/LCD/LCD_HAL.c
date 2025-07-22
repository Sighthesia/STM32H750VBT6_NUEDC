//////////////////////////////////////////////////////////////////////////////////
// LCD硬件抽象层实现文件
// 实现HAL注册机制和封装函数
// 提供STM32平台的具体实现
//////////////////////////////////////////////////////////////////////////////////

#include "LCD_HAL.h"
#include "SPI_HAL.h"
#include "lcd.h"
#include "lcd_spi.h"
#include "main.h"

/* --------------------------------- 此处修改HAL实现相关代码 -------------------------------- */
// HAL实现指针，用于存储注册的硬件实现
static const LCD_HAL_t *hal_ptr = NULL;

/** ***************************************************************************
 * @brief  注册LCD硬件抽象层实现
 * @param  hal: 指向硬件抽象层实现结构体的指针
 * @retval None
 *****************************************************************************/
void LCD_HAL_Register(const LCD_HAL_t *hal)
{
    hal_ptr = hal;
}

/** ***************************************************************************
 * @brief  LCD HAL层CS引脚控制函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_CS_SET(void)
{
    if (hal_ptr && hal_ptr->GPIO_CS_Set)
    {
        hal_ptr->GPIO_CS_Set();
    }
}

void LCD_HAL_CS_CLR(void)
{
    if (hal_ptr && hal_ptr->GPIO_CS_Clr)
    {
        hal_ptr->GPIO_CS_Clr();
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层DC引脚控制函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_DC_SET(void)
{
    if (hal_ptr && hal_ptr->GPIO_DC_Set)
    {
        hal_ptr->GPIO_DC_Set();
    }
}

void LCD_HAL_DC_CLR(void)
{
    if (hal_ptr && hal_ptr->GPIO_DC_Clr)
    {
        hal_ptr->GPIO_DC_Clr();
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层RST引脚控制函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_RST_SET(void)
{
    if (hal_ptr && hal_ptr->GPIO_RST_Set)
    {
        hal_ptr->GPIO_RST_Set();
    }
}

void LCD_HAL_RST_CLR(void)
{
    if (hal_ptr && hal_ptr->GPIO_RST_Clr)
    {
        hal_ptr->GPIO_RST_Clr();
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层背光控制函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_BLK_SET(void)
{
    if (hal_ptr && hal_ptr->GPIO_BLK_Set)
    {
        hal_ptr->GPIO_BLK_Set();
    }
}

void LCD_HAL_BLK_CLR(void)
{
    if (hal_ptr && hal_ptr->GPIO_BLK_Clr)
    {
        hal_ptr->GPIO_BLK_Clr();
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层SPI通信函数
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 *****************************************************************************/
u8 LCD_HAL_SPI_WRITE_BYTE(u8 data)
{
    if (hal_ptr && hal_ptr->SPI_Write_Byte)
    {
        return hal_ptr->SPI_Write_Byte(data);
    }
    return 0;
}

/** ***************************************************************************
 * @brief  LCD HAL层延时函数
 * @param  ms: 延时毫秒数
 * @retval None
 *****************************************************************************/
void LCD_HAL_DELAY_MS(u32 ms)
{
    if (hal_ptr && hal_ptr->Delay_Ms)
    {
        hal_ptr->Delay_Ms(ms);
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层SPI速度设置函数
 * @param  speed: SPI速度设置，0-高速，1-低速
 * @retval None
 *****************************************************************************/
void LCD_HAL_SPI_SET_SPEED(u8 speed)
{
    if (hal_ptr && hal_ptr->SPI_Set_Speed)
    {
        hal_ptr->SPI_Set_Speed(speed);
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层SPI初始化函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_SPI_INIT(void)
{
    if (hal_ptr && hal_ptr->SPI_Init)
    {
        hal_ptr->SPI_Init();
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层SPI MSP初始化函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_SPI_MSP_INIT(void)
{
    if (hal_ptr && hal_ptr->SPI_MSP_Init)
    {
        hal_ptr->SPI_MSP_Init();
    }
}

/** ***************************************************************************
 * @brief  LCD HAL层SPI MSP反初始化函数
 * @retval None
 *****************************************************************************/
void LCD_HAL_SPI_MSP_DEINIT(void)
{
    if (hal_ptr && hal_ptr->SPI_MSP_DeInit)
    {
        hal_ptr->SPI_MSP_DeInit();
    }
}

// ==================== STM32平台具体实现 ====================

/** ***************************************************************************
 * @brief  STM32平台GPIO控制函数实现
 * @retval None
 *****************************************************************************/
static void STM32_CS_Set(void)
{
    HAL_GPIO_WritePin(PORT_LCD_CS, GPIO_LCD_CS, GPIO_PIN_SET);
}

static void STM32_CS_Clr(void)
{
    HAL_GPIO_WritePin(PORT_LCD_CS, GPIO_LCD_CS, GPIO_PIN_RESET);
}

static void STM32_DC_Set(void)
{
    HAL_GPIO_WritePin(PORT_LCD_DC, GPIO_LCD_DC, GPIO_PIN_SET);
}

static void STM32_DC_Clr(void)
{
    HAL_GPIO_WritePin(PORT_LCD_DC, GPIO_LCD_DC, GPIO_PIN_RESET);
}

#if LCD_USE_SOFTWARE_RESET
static void STM32_RST_Set(void)
{
    HAL_GPIO_WritePin(PORT_LCD_RES, GPIO_LCD_RES, GPIO_PIN_SET);
}

static void STM32_RST_Clr(void)
{
    HAL_GPIO_WritePin(PORT_LCD_RES, GPIO_LCD_RES, GPIO_PIN_RESET);
}
#endif /* LCD_USE_SOFTWARE_RESET */

#if LCD_USE_BACKLIGHT_CONTROL
static void STM32_BLK_Set(void)
{
    HAL_GPIO_WritePin(PORT_LCD_BLK, GPIO_LCD_BLK, GPIO_PIN_SET);
}

static void STM32_BLK_Clr(void)
{
    HAL_GPIO_WritePin(PORT_LCD_BLK, GPIO_LCD_BLK, GPIO_PIN_RESET);
}
#endif /* LCD_USE_BACKLIGHT_CONTROL */

/** ***************************************************************************
 * @brief  STM32平台SPI通信函数实现
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 *****************************************************************************/
static u8 STM32_SPI_Write_Byte(u8 data)
{
    return SPI_HAL_Write_Byte(data);
}

/** ***************************************************************************
 * @brief  STM32平台延时函数实现
 * @param  ms: 延时毫秒数
 * @retval None
 *****************************************************************************/
static void STM32_Delay_Ms(u32 ms)
{
    HAL_Delay(ms);
}

/** ***************************************************************************
 * @brief  STM32平台SPI速度设置函数实现
 * @param  speed: SPI速度设置，0-高速，1-低速
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Set_Speed(u8 speed)
{
    SPI_HAL_Set_Speed(speed);
}

/** ***************************************************************************
 * @brief  STM32平台SPI初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Init(void)
{
    SPI_HAL_Init();
}

/** ***************************************************************************
 * @brief  STM32平台SPI MSP初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_MSP_Init(void)
{
    SPI_HAL_MSP_Init();
}

/** ***************************************************************************
 * @brief  STM32平台SPI MSP反初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_MSP_DeInit(void)
{
    SPI_HAL_MSP_DeInit();
}

/** ***************************************************************************
 * @brief  STM32平台LCD硬件抽象层实现实例
 *****************************************************************************/
const LCD_HAL_t stm32_lcd_hal = {
    .GPIO_CS_Set = STM32_CS_Set,
    .GPIO_CS_Clr = STM32_CS_Clr,
    .GPIO_DC_Set = STM32_DC_Set,
    .GPIO_DC_Clr = STM32_DC_Clr,
#if LCD_USE_SOFTWARE_RESET
    .GPIO_RST_Set = STM32_RST_Set,
    .GPIO_RST_Clr = STM32_RST_Clr,
#else
    .GPIO_RST_Set = NULL,
    .GPIO_RST_Clr = NULL,
#endif /* LCD_USE_SOFTWARE_RESET */
#if LCD_USE_BACKLIGHT_CONTROL
    .GPIO_BLK_Set = STM32_BLK_Set,
    .GPIO_BLK_Clr = STM32_BLK_Clr,
#else
    .GPIO_BLK_Set = NULL,
    .GPIO_BLK_Clr = NULL,
#endif /* LCD_USE_BACKLIGHT_CONTROL */
    .SPI_Write_Byte = STM32_SPI_Write_Byte,
    .Delay_Ms = STM32_Delay_Ms,
    .SPI_Set_Speed = STM32_SPI_Set_Speed,
    .SPI_Init = STM32_SPI_Init,
    .SPI_MSP_Init = STM32_SPI_MSP_Init,
    .SPI_MSP_DeInit = STM32_SPI_MSP_DeInit};

/** ***************************************************************************
 * @brief  初始化STM32平台LCD硬件抽象层
 * @retval None
 * @note   在系统初始化时调用此函数进行HAL层注册
 *****************************************************************************/
void STM32_LCD_HAL_Init(void)
{
    // 首先初始化SPI HAL层
    STM32_SPI_HAL_Init();

    // 然后注册LCD HAL层
    LCD_HAL_Register(&stm32_lcd_hal);
}
/* 自定义部分结束 */
