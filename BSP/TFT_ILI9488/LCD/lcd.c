//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// 测试硬件：单片机STM32F103RCT6,正点原子MiniSTM32开发板,主频72MHZ，晶振12MHZ
// QDtech-TFT液晶驱动 for STM32 IO模拟
// xiao冯@ShenZhen QDtech co.,LTD
// 公司网站:www.qdtft.com
// 淘宝网站：http://qdtech.taobao.com
// wiki技术网站：http://www.lcdwiki.com
// 我司提供技术支持，任何技术问题欢迎随时交流学习
// 固话(传真) :+86 0755-23594567
// 手机:15989313508（冯工）
// 邮箱:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com
// 技术支持QQ:3002773612  3002778157
// 技术交流QQ群:324828016
// 创建日期:2018/08/09
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 深圳市全动电子技术有限公司 2018-2028
// All rights reserved
/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                STM32单片机
//      VCC          接        DC5V/3.3V      //电源
//      GND          接          GND          //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为SPI总线
//     LCD模块                STM32单片机
//    SDI(MOSI)      接          PB15         //液晶屏SPI总线数据写信号
//    SDO(MISO)      接          PB14         //液晶屏SPI总线数据读信号，如果不需要读，可以不接线
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 					      STM32单片机
//       LED         接          PB9          //液晶屏背光控制信号，如果不需要控制，接5V或3.3V
//       SCK         接          PB13         //液晶屏SPI总线时钟信号
//      DC/RS        接          PB10         //液晶屏数据/命令控制信号
//       RST         接          PB12         //液晶屏复位控制信号
//       CS          接          PB11         //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块                STM32单片机
//      T_IRQ        接          PC10         //触摸屏触摸中断信号
//      T_DO         接          PC2          //触摸屏SPI总线读信号
//      T_DIN        接          PC3          //触摸屏SPI总线写信号
//      T_CS         接          PC13         //触摸屏片选控制信号
//      T_CLK        接          PC0          //触摸屏SPI总线时钟信号
**************************************************************************************************/
/* @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 **************************************************************************************************/
#include "lcd.h"

// 管理LCD重要参数
// 默认为竖屏
_lcd_dev lcddev;

// 画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
u16 DeviceCode;

/** ***************************************************************************
 * @brief  写入8位命令到LCD屏幕
 * @param  data: 要写入的命令值
 * @retval None
 *****************************************************************************/
void LCD_WR_REG(u8 data)
{
	LCD_CS_CLR;
	LCD_RS_CLR;
	SPI_Write_Byte(&SPI_LCD_HANDLE_NAME, data);
	LCD_CS_SET;
}

/** ***************************************************************************
 * @brief  写入8位数据到LCD屏幕
 * @param  data: 要写入的数据值
 * @retval None
 *****************************************************************************/
void LCD_WR_DATA(u8 data)
{
	LCD_CS_CLR;
	LCD_RS_SET;
	SPI_Write_Byte(&SPI_LCD_HANDLE_NAME, data);
	LCD_CS_SET;
}

/** ***************************************************************************
 * @brief  写寄存器
 * @param  lcd_reg: 寄存器地址
 * @param  lcd_reg_value: 要写入的数据
 * @retval None
 *****************************************************************************/
void LCD_Write_Reg(u8 lcd_reg, u16 lcd_reg_value)
{
	LCD_WR_REG(lcd_reg);
	LCD_WR_DATA(lcd_reg_value);
}

/** ***************************************************************************
 * @brief  准备写GRAM
 * @retval None
 *****************************************************************************/
void LCD_Write_RAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}

/** ***************************************************************************
 * @brief  写16位数据
 * @param  data: 要写入的16位数据
 * @retval None
 *****************************************************************************/
void LCD_Write_Data_16Bit(u16 data)
{
	// 18Bit
	LCD_WR_DATA((data >> 8) & 0xF8); // RED
	LCD_WR_DATA((data >> 3) & 0xFC); // GREEN
	LCD_WR_DATA(data << 3);			 // BLUE
}

/** ***************************************************************************
 * @brief  画点
 * @param  x: X坐标
 * @param  y: Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Point(u16 x, u16 y)
{
	LCD_Set_Cursor(x, y); // 设置光标位置
	LCD_Write_Data_16Bit(POINT_COLOR);
}

/** ***************************************************************************
 * @brief  LCD清屏
 * @param  color: 填充颜色
 * @retval None
 *****************************************************************************/
void LCD_Clear(u16 color)
{
	unsigned int i, m;
	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1);
	LCD_CS_CLR;
	LCD_RS_SET;
	for (i = 0; i < lcddev.height; i++)
	{
		for (m = 0; m < lcddev.width; m++)
		{
			LCD_Write_Data_16Bit(color);
		}
	}
	LCD_CS_SET;
}

/** ***************************************************************************
 * @brief  LCD GPIO初始化
 * @param  None
 * @retval None
 *****************************************************************************/
void LCD_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* --------------------------------- 此处修改GPIO时钟和引脚配置 -------------------------------- */
	// 使能GPIO时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
#if LCD_USE_SOFTWARE_RESET
	__HAL_RCC_GPIOB_CLK_ENABLE();
#endif /* LCD_USE_SOFTWARE_RESET */

	// 配置GPIOA引脚 (SCL, SDA, CS)
#if LCD_USE_BACKLIGHT_CONTROL
	GPIO_InitStruct.Pin = GPIO_LCD_SCL | GPIO_LCD_SDA | GPIO_LCD_CS;
#else
	GPIO_InitStruct.Pin = GPIO_LCD_SCL | GPIO_LCD_SDA | GPIO_LCD_CS;
#endif											/* LCD_USE_BACKLIGHT_CONTROL */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 配置GPIOD引脚 (DC)
#if LCD_USE_BACKLIGHT_CONTROL
	GPIO_InitStruct.Pin = GPIO_LCD_DC | GPIO_LCD_BLK;
#else
	GPIO_InitStruct.Pin = GPIO_LCD_DC;
#endif											/* LCD_USE_BACKLIGHT_CONTROL */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

#if LCD_USE_SOFTWARE_RESET
	// 配置GPIOB引脚 (RES)
	GPIO_InitStruct.Pin = GPIO_LCD_RES;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif /* LCD_USE_SOFTWARE_RESET */

#if LCD_USE_TOUCH
	// 如果使用触摸屏，配置触摸屏相关引脚
	GPIO_InitStruct.Pin = GPIO_LCD_CS2 | GPIO_LCD_PEN | GPIO_LCD_TCLK | GPIO_LCD_TDIN | GPIO_LCD_TD0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif /* LCD_USE_TOUCH */
	/* 自定义部分结束 */
}

#if LCD_USE_SOFTWARE_RESET
/** ***************************************************************************
 * @brief  LCD软件复位
 * @param  None
 * @retval None
 *****************************************************************************/
void LCD_RESET(void)
{
	LCD_RST_CLR;
	HAL_Delay(100);
	LCD_RST_SET;
	HAL_Delay(50);
}
#endif /* LCD_USE_SOFTWARE_RESET */

#if LCD_USE_BACKLIGHT_CONTROL
/** ***************************************************************************
 * @brief  设置LCD背光状态
 * @param  state: 背光状态，1-开启，0-关闭
 * @retval None
 *****************************************************************************/
void LCD_Set_Backlight(u8 state)
{
	if (state)
	{
		LCD_LED_SET;
	}
	else
	{
		LCD_LED_CLR;
	}
}
#endif /* LCD_USE_BACKLIGHT_CONTROL */

/** ***************************************************************************
 * @brief  LCD初始化
 * @param  None
 * @retval None
 *****************************************************************************/
void LCD_Init(void)
{
#if !USE_CUBEMX_SPI_CONFIG
	SPI_LCD_Init(); // 硬件SPI初始化
#endif				/* !USE_CUBEMX_SPI_CONFIG */
	LCD_GPIOInit(); // LCD GPIO初始化

#if LCD_USE_SOFTWARE_RESET
	LCD_RESET(); // LCD 复位
#endif			 /* LCD_USE_SOFTWARE_RESET */

	//************* ILI9488初始化**********//
	LCD_WR_REG(0XF7);
	LCD_WR_DATA(0xA9);
	LCD_WR_DATA(0x51);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x82);
	LCD_WR_REG(0xC0);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x09);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA(0x41);
	LCD_WR_REG(0XC5);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x80);
	LCD_WR_REG(0xB1);
	LCD_WR_DATA(0xB0);
	LCD_WR_DATA(0x11);
	LCD_WR_REG(0xB4);
	LCD_WR_DATA(0x02);
	LCD_WR_REG(0xB6);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x42);
	LCD_WR_REG(0xB7);
	LCD_WR_DATA(0xc6);
	LCD_WR_REG(0xBE);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x04);
	LCD_WR_REG(0xE9);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0x36);
	LCD_WR_DATA((1 << 3) | (0 << 7) | (1 << 6) | (1 << 5));
	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x66);
	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x0B);
	LCD_WR_DATA(0x41);
	LCD_WR_DATA(0x89);
	LCD_WR_DATA(0x4B);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0x1B);
	LCD_WR_DATA(0x0F);
	LCD_WR_REG(0XE1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x1A);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x2F);
	LCD_WR_DATA(0x45);
	LCD_WR_DATA(0x43);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x32);
	LCD_WR_DATA(0x36);
	LCD_WR_DATA(0x0F);
	LCD_WR_REG(0x11);
	HAL_Delay(120);
	LCD_WR_REG(0x29);

	LCD_Direction(USE_HORIZONTAL); // 设置LCD显示方向

#if LCD_USE_BACKLIGHT_CONTROL
	LCD_Set_Backlight(1); // 点亮背光
#endif					  /* LCD_USE_BACKLIGHT_CONTROL */

	LCD_Clear(WHITE); // 清全屏白色
}

/** ***************************************************************************
 * @brief  设置LCD显示窗口
 * @param  x_star: 窗口起始X坐标
 * @param  y_star: 窗口起始Y坐标
 * @param  x_end: 窗口结束X坐标
 * @param  y_end: 窗口结束Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Set_Windows(u16 x_star, u16 y_star, u16 x_end, u16 y_end)
{
	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA(x_star >> 8);
	LCD_WR_DATA(0x00FF & x_star);
	LCD_WR_DATA(x_end >> 8);
	LCD_WR_DATA(0x00FF & x_end);

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA(y_star >> 8);
	LCD_WR_DATA(0x00FF & y_star);
	LCD_WR_DATA(y_end >> 8);
	LCD_WR_DATA(0x00FF & y_end);

	LCD_Write_RAM_Prepare(); // 开始写入GRAM
}

/** ***************************************************************************
 * @brief  设置光标位置
 * @param  x_pos: X坐标
 * @param  y_pos: Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Set_Cursor(u16 x_pos, u16 y_pos)
{
	LCD_Set_Windows(x_pos, y_pos, x_pos, y_pos);
}

/** ***************************************************************************
 * @brief  设置LCD显示方向
 * @param  direction: 显示方向 0-0度，1-90度，2-180度，3-270度
 * @retval None
 *****************************************************************************/
void LCD_Direction(u8 direction)
{
	lcddev.setxcmd = 0x2A;
	lcddev.setycmd = 0x2B;
	lcddev.wramcmd = 0x2C;
	switch (direction)
	{
	case 0:
		lcddev.width = LCD_W;
		lcddev.height = LCD_H;
		LCD_Write_Reg(0x36, (1 << 3) | (0 << 6) | (0 << 7)); // BGR==1,MY==0,MX==0,MV==0
		break;
	case 1:
		lcddev.width = LCD_H;
		lcddev.height = LCD_W;
		LCD_Write_Reg(0x36, (1 << 3) | (0 << 7) | (1 << 6) | (1 << 5)); // BGR==1,MY==1,MX==0,MV==1
		break;
	case 2:
		lcddev.width = LCD_W;
		lcddev.height = LCD_H;
		LCD_Write_Reg(0x36, (1 << 3) | (1 << 6) | (1 << 7)); // BGR==1,MY==0,MX==0,MV==0
		break;
	case 3:
		lcddev.width = LCD_H;
		lcddev.height = LCD_W;
		LCD_Write_Reg(0x36, (1 << 3) | (1 << 7) | (1 << 5)); // BGR==1,MY==1,MX==0,MV==1
		break;
	default:
		break;
	}
}