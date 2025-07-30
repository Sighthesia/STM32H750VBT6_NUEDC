//////////////////////////////////////////////////////////////////////////////////
// LCD硬件抽象层实现文件
// 移除注册机制，直接导出结构体供应用层调用
// 应用层调用方式：lcd_hal.Function_Name()
//////////////////////////////////////////////////////////////////////////////////

#include "LCD_HAL.h"
#include "SPI_HAL.h"
#include "lcd.h"
#include "lcd_spi.h"
#include "main.h"

/* --------------------------------- 此处修改直接实现方式 -------------------------------- */
/** ***************************************************************************
 * @brief  说明：移除注册机制，应用层直接调用 lcd_hal.Function_Name()
 * @note   平台实现在文件末尾，结构体导出为 lcd_hal
 *****************************************************************************/

/* ================================= STM32平台具体实现 ================================ */

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
    return spi_hal.SPI_Write_Byte(data);
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
    spi_hal.SPI_Set_Speed(speed);
}

/** ***************************************************************************
 * @brief  STM32平台SPI初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_Init(void)
{
    spi_hal.SPI_Init();
}

/** ***************************************************************************
 * @brief  STM32平台SPI MSP初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_MSP_Init(void)
{
    spi_hal.SPI_MSP_Init();
}

/** ***************************************************************************
 * @brief  STM32平台SPI MSP反初始化函数实现
 * @retval None
 *****************************************************************************/
static void STM32_SPI_MSP_DeInit(void)
{
    spi_hal.SPI_MSP_DeInit();
}

/** ***************************************************************************
 * @brief  STM32平台LCD硬件抽象层实现实例（导出给应用层）
 *****************************************************************************/
const LCD_HAL_t lcd_hal = {
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

/* 自定义部分结束 */
