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
#ifndef _SPI_H_
#define _SPI_H_

#include "lcd.h"
#include "main.h"
#include "stdlib.h"

/* --------------------------------- 此处修改SPI配置参数 -------------------------------- */
// SPI实例配置宏定义
#define SPI_LCD_INSTANCE SPI1
#define SPI_LCD_HANDLE_NAME hspi1

// SPI速度配置
#define SPI_SPEED_HIGH 0
#define SPI_SPEED_LOW 1

// 是否使用CubeMX配置SPI，1-使用CubeMX配置，0-手动配置
#define USE_CUBEMX_SPI_CONFIG 1
/* 自定义部分结束 */

extern SPI_HandleTypeDef SPI_LCD_HANDLE_NAME;

/** ***************************************************************************
 * @brief  写入一个字节数据到SPI总线
 * @param  hspi: SPI句柄指针
 * @param  byte: 要写入的数据
 * @retval 总线接收到的数据
 *****************************************************************************/
uint8_t SPI_Write_Byte(SPI_HandleTypeDef *hspi, uint8_t byte);

#if !USE_CUBEMX_SPI_CONFIG
/** ***************************************************************************
 * @brief  初始化SPI外设
 * @param  None
 * @retval None
 *****************************************************************************/
void SPI_LCD_Init(void);
#endif /* !USE_CUBEMX_SPI_CONFIG */

/** ***************************************************************************
 * @brief  设置SPI总线速度
 * @param  hspi: SPI句柄指针
 * @param  speed_set: 速度设置，0-高速，1-低速
 * @retval None
 *****************************************************************************/
void SPI_Set_Speed(SPI_HandleTypeDef *hspi, uint8_t speed_set);

#endif /* _SPI_H_ */
