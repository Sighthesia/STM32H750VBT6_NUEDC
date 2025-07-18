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
#include "lcd_spi.h"

/* 自定义部分结束 */

/** ***************************************************************************
 * @brief  写入一个字节数据到SPI总线
 * @param  hspi: SPI句柄指针
 * @param  byte: 要写入的数据
 * @retval 总线接收到的数据
 *****************************************************************************/
uint8_t SPI_Write_Byte(SPI_HandleTypeDef *hspi, uint8_t byte)
{
	uint8_t rx_data = 0;
	HAL_SPI_TransmitReceive(hspi, &byte, &rx_data, 1, HAL_MAX_DELAY);
	return rx_data;
}

/** ***************************************************************************
 * @brief  设置SPI总线速度
 * @param  hspi: SPI句柄指针
 * @param  speed_set: 速度设置，0-高速，1-低速
 * @retval None
 *****************************************************************************/
void SPI_Set_Speed(SPI_HandleTypeDef *hspi, uint8_t speed_set)
{
	__HAL_SPI_DISABLE(hspi);

	if (speed_set == SPI_SPEED_HIGH) // 高速
	{
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // Fsck=Fpclk/2
	}
	else // 低速
	{
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // Fsck=Fpclk/32
	}

	HAL_SPI_Init(hspi);
	__HAL_SPI_ENABLE(hspi);
}

#if !USE_CUBEMX_SPI_CONFIG
/** ***************************************************************************
 * @brief  初始化SPI外设
 * @param  None
 * @retval None
 *****************************************************************************/
void SPI_LCD_Init(void)
{
	/* --------------------------------- 此处修改SPI初始化配置 -------------------------------- */
	// 注意：以下配置已在CubeMX中配置，如需手动配置请设置USE_CUBEMX_SPI_CONFIG为0
	SPI_LCD_HANDLE_NAME.Instance = SPI_LCD_INSTANCE;
	SPI_LCD_HANDLE_NAME.Init.Mode = SPI_MODE_MASTER;
	SPI_LCD_HANDLE_NAME.Init.Direction = SPI_DIRECTION_2LINES;
	SPI_LCD_HANDLE_NAME.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_LCD_HANDLE_NAME.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_LCD_HANDLE_NAME.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_LCD_HANDLE_NAME.Init.NSS = SPI_NSS_SOFT;
	SPI_LCD_HANDLE_NAME.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	SPI_LCD_HANDLE_NAME.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_LCD_HANDLE_NAME.Init.TIMode = SPI_TIMODE_DISABLE;
	SPI_LCD_HANDLE_NAME.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPI_LCD_HANDLE_NAME.Init.CRCPolynomial = 0x0;
	SPI_LCD_HANDLE_NAME.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	SPI_LCD_HANDLE_NAME.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	SPI_LCD_HANDLE_NAME.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	SPI_LCD_HANDLE_NAME.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	SPI_LCD_HANDLE_NAME.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	SPI_LCD_HANDLE_NAME.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	SPI_LCD_HANDLE_NAME.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	SPI_LCD_HANDLE_NAME.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	SPI_LCD_HANDLE_NAME.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	SPI_LCD_HANDLE_NAME.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	/* 自定义部分结束 */

	if (HAL_SPI_Init(&SPI_LCD_HANDLE_NAME) != HAL_OK)
	{
		Error_Handler();
	}
}

/** ***************************************************************************
 * @brief  SPI MSP初始化回调函数
 * @param  hspi: SPI句柄指针
 * @retval None
 *****************************************************************************/
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (hspi->Instance == SPI_LCD_INSTANCE)
	{
		/* --------------------------------- 此处修改GPIO和时钟配置 -------------------------------- */
		// 使能SPI和GPIOB时钟
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		// 配置SPI引脚
		// PB13: SCK, PB14: MISO, PB15: MOSI
		GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* 自定义部分结束 */
	}
}

/** ***************************************************************************
 * @brief  SPI MSP反初始化回调函数
 * @param  hspi: SPI句柄指针
 * @retval None
 *****************************************************************************/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI_LCD_INSTANCE)
	{
		// 禁用SPI时钟
		__HAL_RCC_SPI1_CLK_DISABLE();

		// 反初始化GPIO引脚
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
	}
}
#endif /* !USE_CUBEMX_SPI_CONFIG */
