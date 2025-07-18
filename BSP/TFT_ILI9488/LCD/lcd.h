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
#ifndef __LCD_H
#define __LCD_H
#include "stdlib.h"
#include "stdint.h"
#include "lcd_spi.h"
#include "main.h"

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

// LCD重要参数集
typedef struct
{
	u16 width;	 // LCD 宽度
	u16 height;	 // LCD 高度
	u16 id;		 // LCD ID
	u8 dir;		 // 横屏还是竖屏控制：0，竖屏；1，横屏。
	u16 wramcmd; // 开始写gram指令
	u16 setxcmd; // 设置x坐标指令
	u16 setycmd; // 设置y坐标指令
} _lcd_dev;

// LCD参数
extern _lcd_dev lcddev; // 管理LCD重要参数
/////////////////////////////////////用户配置区///////////////////////////////////
#define USE_HORIZONTAL 0 // 定义液晶屏顺时针旋转方向 	0-0度旋转，1-90度旋转，2-180度旋转，3-270度旋转

/* --------------------------------- 此处修改功能开关配置 -------------------------------- */
#define LCD_USE_BACKLIGHT_CONTROL 0 // 是否使用背光控制功能，1-使用，0-不使用
#define LCD_USE_SOFTWARE_RESET 0	// 是否使用软件复位功能，1-使用，0-不使用
#define LCD_USE_TOUCH 0				// 是否使用触摸屏功能，1-使用，0-不使用
/* 自定义部分结束 */

//////////////////////////////////////////////////////////////////////////////////
// 定义LCD的尺寸
#define LCD_W 320
#define LCD_H 480

// TFTLCD部分外要调用的函数
extern u16 POINT_COLOR; // 默认红色
extern u16 BACK_COLOR;	// 背景颜色.默认为白色

/* --------------------------------- 此处修改LCD端口配置 -------------------------------- */
// SCK
#define PORT_LCD_SCL GPIOA
#define GPIO_LCD_SCL GPIO_PIN_7

// MOSI
#define PORT_LCD_SDA GPIOA
#define GPIO_LCD_SDA GPIO_PIN_5

////MISO
// #define PORT_LCD_MIS    GPIOB
// #define GPIO_LCD_MIS    GPIO_PIN_14

// NSS
#define PORT_LCD_CS GPIOA
#define GPIO_LCD_CS GPIO_PIN_15

// DC
#define PORT_LCD_DC GPIOD
#define GPIO_LCD_DC GPIO_PIN_2

#if LCD_USE_SOFTWARE_RESET
// RES
#define PORT_LCD_RES GPIOB
#define GPIO_LCD_RES GPIO_PIN_1
#endif /* LCD_USE_SOFTWARE_RESET */

#if LCD_USE_BACKLIGHT_CONTROL
// BLK/LED
#define PORT_LCD_BLK GPIOD
#define GPIO_LCD_BLK GPIO_PIN_10
#endif /* LCD_USE_BACKLIGHT_CONTROL */

#if LCD_USE_TOUCH
// CS2/T_CS
#define PORT_LCD_CS2 GPIOA
#define GPIO_LCD_CS2 GPIO_PIN_3

// PEN/T_IRQ
#define PORT_LCD_PEN GPIOA
#define GPIO_LCD_PEN GPIO_PIN_0

// T_CLK
#define PORT_LCD_TCLK GPIOA
#define GPIO_LCD_TCLK GPIO_PIN_4

// T_DIN
#define PORT_LCD_TDIN GPIOA
#define GPIO_LCD_TDIN GPIO_PIN_2

// T_DO
#define PORT_LCD_TD0 GPIOA
#define GPIO_LCD_TD0 GPIO_PIN_1
#endif /* LCD_USE_TOUCH */
/* 自定义部分结束 */

/* --------------------------------- 此处修改GPIO操作宏定义 -------------------------------- */
// 液晶控制口置1操作语句宏定义
#define LCD_CS_SET HAL_GPIO_WritePin(PORT_LCD_CS, GPIO_LCD_CS, GPIO_PIN_SET)
#define LCD_RS_SET HAL_GPIO_WritePin(PORT_LCD_DC, GPIO_LCD_DC, GPIO_PIN_SET)
#define LCD_SDA_SET HAL_GPIO_WritePin(PORT_LCD_SDA, GPIO_LCD_SDA, GPIO_PIN_SET)
#define LCD_SCL_SET HAL_GPIO_WritePin(PORT_LCD_SCL, GPIO_LCD_SCL, GPIO_PIN_SET)

#if LCD_USE_SOFTWARE_RESET
#define LCD_RST_SET HAL_GPIO_WritePin(PORT_LCD_RES, GPIO_LCD_RES, GPIO_PIN_SET)
#endif /* LCD_USE_SOFTWARE_RESET */

#if LCD_USE_BACKLIGHT_CONTROL
#define LCD_LED_SET HAL_GPIO_WritePin(PORT_LCD_BLK, GPIO_LCD_BLK, GPIO_PIN_SET)
#endif /* LCD_USE_BACKLIGHT_CONTROL */

// 液晶控制口置0操作语句宏定义
#define LCD_CS_CLR HAL_GPIO_WritePin(PORT_LCD_CS, GPIO_LCD_CS, GPIO_PIN_RESET)	  // CS
#define LCD_RS_CLR HAL_GPIO_WritePin(PORT_LCD_DC, GPIO_LCD_DC, GPIO_PIN_RESET)	  // DC
#define LCD_SDA_CLR HAL_GPIO_WritePin(PORT_LCD_SDA, GPIO_LCD_SDA, GPIO_PIN_RESET) // SDA=MOSI
#define LCD_SCL_CLR HAL_GPIO_WritePin(PORT_LCD_SCL, GPIO_LCD_SCL, GPIO_PIN_RESET) // SCL=SCLK

#if LCD_USE_SOFTWARE_RESET
#define LCD_RST_CLR HAL_GPIO_WritePin(PORT_LCD_RES, GPIO_LCD_RES, GPIO_PIN_RESET) // RES
#endif																			  /* LCD_USE_SOFTWARE_RESET */

#if LCD_USE_BACKLIGHT_CONTROL
#define LCD_LED_CLR HAL_GPIO_WritePin(PORT_LCD_BLK, GPIO_LCD_BLK, GPIO_PIN_RESET)
#endif /* LCD_USE_BACKLIGHT_CONTROL */
/* 自定义部分结束 */

// 画笔颜色
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40 // 棕色
#define BRRED 0XFC07 // 棕红色
#define GRAY 0X8430	 // 灰色

// GUI颜色
#define DARKBLUE 0X01CF	 // 深蓝色
#define LIGHTBLUE 0X7D7C // 浅蓝色
#define GRAYBLUE 0X5458	 // 灰蓝色
// 以上三色为PANEL的颜色

#define LIGHTGREEN 0X841F // 浅绿色
#define LIGHTGRAY 0XEF5B  // 浅灰色(PANNEL)
#define LGRAY 0XC618	  // 浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE 0XA651 // 浅灰蓝色(中间层颜色)
#define LBBLUE 0X2B12	 // 浅棕蓝色(选择条目的反色)

/** ***************************************************************************
 * @brief  LCD初始化
 * @retval None
 *****************************************************************************/
void LCD_Init(void);

/** ***************************************************************************
 * @brief  LCD显示开启
 * @retval None
 *****************************************************************************/
void LCD_Display_On(void);

/** ***************************************************************************
 * @brief  LCD显示关闭
 * @retval None
 *****************************************************************************/
void LCD_Display_Off(void);

/** ***************************************************************************
 * @brief  LCD清屏
 * @param  color: 填充颜色
 * @retval None
 *****************************************************************************/
void LCD_Clear(u16 color);

/** ***************************************************************************
 * @brief  设置光标位置
 * @param  x_pos: X坐标
 * @param  y_pos: Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Set_Cursor(u16 x_pos, u16 y_pos);

/** ***************************************************************************
 * @brief  画点
 * @param  x: X坐标
 * @param  y: Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Point(u16 x, u16 y);

/** ***************************************************************************
 * @brief  读点
 * @param  x: X坐标
 * @param  y: Y坐标
 * @retval 点的颜色值
 *****************************************************************************/
u16 LCD_Read_Point(u16 x, u16 y);

/** ***************************************************************************
 * @brief  画线
 * @param  x1: 起点X坐标
 * @param  y1: 起点Y坐标
 * @param  x2: 终点X坐标
 * @param  y2: 终点Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Line(u16 x1, u16 y1, u16 x2, u16 y2);

/** ***************************************************************************
 * @brief  画矩形
 * @param  x1: 左上角X坐标
 * @param  y1: 左上角Y坐标
 * @param  x2: 右下角X坐标
 * @param  y2: 右下角Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Rectangle(u16 x1, u16 y1, u16 x2, u16 y2);

/** ***************************************************************************
 * @brief  设置LCD显示窗口
 * @param  x_star: 窗口起始X坐标
 * @param  y_star: 窗口起始Y坐标
 * @param  x_end: 窗口结束X坐标
 * @param  y_end: 窗口结束Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Set_Windows(u16 x_star, u16 y_star, u16 x_end, u16 y_end);

/** ***************************************************************************
 * @brief  读取LCD数据
 * @retval LCD数据
 *****************************************************************************/
u16 LCD_RD_DATA(void);

/** ***************************************************************************
 * @brief  写寄存器
 * @param  lcd_reg: 寄存器地址
 * @param  lcd_reg_value: 要写入的数据
 * @retval None
 *****************************************************************************/
void LCD_Write_Reg(u8 lcd_reg, u16 lcd_reg_value);

/** ***************************************************************************
 * @brief  写数据
 * @param  data: 要写入的数据
 * @retval None
 *****************************************************************************/
void LCD_WR_DATA(u8 data);

/** ***************************************************************************
 * @brief  读寄存器
 * @param  lcd_reg: 寄存器地址
 * @retval 寄存器值
 *****************************************************************************/
u16 LCD_Read_Reg(u8 lcd_reg);

/** ***************************************************************************
 * @brief  准备写GRAM
 * @retval None
 *****************************************************************************/
void LCD_Write_RAM_Prepare(void);

/** ***************************************************************************
 * @brief  写GRAM
 * @param  rgb_code: 颜色代码
 * @retval None
 *****************************************************************************/
void LCD_Write_RAM(u16 rgb_code);

/** ***************************************************************************
 * @brief  读GRAM
 * @retval 颜色代码
 *****************************************************************************/
u16 LCD_Read_RAM(void);

/** ***************************************************************************
 * @brief  BGR转RGB
 * @param  c: 颜色值
 * @retval 转换后的颜色值
 *****************************************************************************/
u16 LCD_BGR2RGB(u16 c);

/** ***************************************************************************
 * @brief  设置LCD参数
 * @retval None
 *****************************************************************************/
void LCD_Set_Param(void);

/** ***************************************************************************
 * @brief  写16位数据
 * @param  data: 要写入的16位数据
 * @retval None
 *****************************************************************************/
void LCD_Write_Data_16Bit(u16 data);

/** ***************************************************************************
 * @brief  设置LCD显示方向
 * @param  direction: 显示方向 0-0度，1-90度，2-180度，3-270度
 * @retval None
 *****************************************************************************/
void LCD_Direction(u8 direction);

#if LCD_USE_BACKLIGHT_CONTROL
/** ***************************************************************************
 * @brief  设置LCD背光状态
 * @param  state: 背光状态，1-开启，0-关闭
 * @retval None
 *****************************************************************************/
void LCD_Set_Backlight(u8 state);
#endif /* LCD_USE_BACKLIGHT_CONTROL */

#if LCD_USE_SOFTWARE_RESET
/** ***************************************************************************
 * @brief  LCD软件复位
 * @retval None
 *****************************************************************************/
void LCD_RESET(void);
#endif /* LCD_USE_SOFTWARE_RESET */

#endif /* __LCD_H */
