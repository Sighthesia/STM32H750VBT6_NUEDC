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
#include "string.h"
#include "font.h"
#include "gui.h"

/** ***************************************************************************
 * @brief  在LCD屏幕上画点
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  color: 颜色值
 * @retval None
 *****************************************************************************/
void GUI_Draw_Point(u16 x, u16 y, u16 color)
{
	LCD_Set_Cursor(x, y); // 设置光标位置
	LCD_Write_Data_16Bit(color);
}

/** ***************************************************************************
 * @brief  填充指定区域
 * @param  sx: 起始X坐标
 * @param  sy: 起始Y坐标
 * @param  ex: 结束X坐标
 * @param  ey: 结束Y坐标
 * @param  color: 填充颜色
 * @retval None
 *****************************************************************************/
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color)
{
	u16 i, j;
	u16 width = ex - sx + 1;		 // 得到填充的宽度
	u16 height = ey - sy + 1;		 // 高度
	LCD_Set_Windows(sx, sy, ex, ey); // 设置显示窗口
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
			LCD_Write_Data_16Bit(color); // 写入数据
	}
	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复窗口设置为全屏
}

/** ***************************************************************************
 * @brief  画线
 * @param  x1: 起点X坐标
 * @param  y1: 起点Y坐标
 * @param  x2: 终点X坐标
 * @param  y2: 终点Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Line(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;

	delta_x = x2 - x1; // 计算坐标增量
	delta_y = y2 - y1;
	uRow = x1;
	uCol = y1;
	if (delta_x > 0)
		incx = 1; // 设置单步方向
	else if (delta_x == 0)
		incx = 0; // 垂直线
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0)
		incy = 1;
	else if (delta_y == 0)
		incy = 0; // 水平线
	else
	{
		incy = -1;
		delta_y = -delta_y;
	}
	if (delta_x > delta_y)
		distance = delta_x; // 选取基本增量坐标轴
	else
		distance = delta_y;
	for (t = 0; t <= distance + 1; t++) // 画线输出
	{
		LCD_Draw_Point(uRow, uCol); // 画点
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

/** ***************************************************************************
 * @brief  画矩形
 * @param  x1: 左上角X坐标
 * @param  y1: 左上角Y坐标
 * @param  x2: 右下角X坐标
 * @param  y2: 右下角Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Rectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_Draw_Line(x1, y1, x2, y1);
	LCD_Draw_Line(x1, y1, x1, y2);
	LCD_Draw_Line(x1, y2, x2, y2);
	LCD_Draw_Line(x2, y1, x2, y2);
}

/** ***************************************************************************
 * @brief  画填充矩形
 * @param  x1: 左上角X坐标
 * @param  y1: 左上角Y坐标
 * @param  x2: 右下角X坐标
 * @param  y2: 右下角Y坐标
 * @retval None
 *****************************************************************************/
void LCD_Draw_Fill_Rectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_Fill(x1, y1, x2, y2, POINT_COLOR);
}

/** ***************************************************************************
 * @brief  8对称圆绘制算法（内部调用）
 * @param  xc: 圆心X坐标
 * @param  yc: 圆心Y坐标
 * @param  x: 相对圆心的X坐标
 * @param  y: 相对圆心的Y坐标
 * @param  c: 颜色值
 * @retval None
 *****************************************************************************/
void _draw_circle_8(int xc, int yc, int x, int y, u16 c)
{
	GUI_Draw_Point(xc + x, yc + y, c);
	GUI_Draw_Point(xc - x, yc + y, c);
	GUI_Draw_Point(xc + x, yc - y, c);
	GUI_Draw_Point(xc - x, yc - y, c);
	GUI_Draw_Point(xc + y, yc + x, c);
	GUI_Draw_Point(xc - y, yc + x, c);
	GUI_Draw_Point(xc + y, yc - x, c);
	GUI_Draw_Point(xc - y, yc - x, c);
}

/** ***************************************************************************
 * @brief  画圆
 * @param  xc: 圆心X坐标
 * @param  yc: 圆心Y坐标
 * @param  c: 颜色
 * @param  r: 半径
 * @param  fill: 是否填充
 * @retval None
 *****************************************************************************/
void GUI_Circle(int xc, int yc, u16 c, int r, int fill)
{
	int x = 0, y = r, yi, d;

	d = 3 - 2 * r;

	if (fill)
	{
		// 如果填充（画实心圆）
		while (x <= y)
		{
			for (yi = x; yi <= y; yi++)
				_draw_circle_8(xc, yc, x, yi, c);

			if (d < 0)
			{
				d = d + 4 * x + 6;
			}
			else
			{
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
	else
	{
		// 如果不填充（画空心圆）
		while (x <= y)
		{
			_draw_circle_8(xc, yc, x, y, c);
			if (d < 0)
			{
				d = d + 4 * x + 6;
			}
			else
			{
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
}

/** ***************************************************************************
 * @brief  画三角形
 * @param  x0: 第一个顶点X坐标
 * @param  y0: 第一个顶点Y坐标
 * @param  x1: 第二个顶点X坐标
 * @param  y1: 第二个顶点Y坐标
 * @param  x2: 第三个顶点X坐标
 * @param  y2: 第三个顶点Y坐标
 * @retval None
 *****************************************************************************/
void Draw_Triangle(u16 x0, u16 y0, u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_Draw_Line(x0, y0, x1, y1);
	LCD_Draw_Line(x1, y1, x2, y2);
	LCD_Draw_Line(x2, y2, x0, y0);
}

static void _swap(u16 *a, u16 *b)
{
	u16 tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}

/** ***************************************************************************
 * @brief  填充三角形
 * @param  x0: 第一个顶点X坐标
 * @param  y0: 第一个顶点Y坐标
 * @param  x1: 第二个顶点X坐标
 * @param  y1: 第二个顶点Y坐标
 * @param  x2: 第三个顶点X坐标
 * @param  y2: 第三个顶点Y坐标
 * @retval None
 *****************************************************************************/
void Fill_Triangle(u16 x0, u16 y0, u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 a, b, y, last;
	int dx01, dy01, dx02, dy02, dx12, dy12;
	long sa = 0;
	long sb = 0;
	if (y0 > y1)
	{
		_swap(&y0, &y1);
		_swap(&x0, &x1);
	}
	if (y1 > y2)
	{
		_swap(&y2, &y1);
		_swap(&x2, &x1);
	}
	if (y0 > y1)
	{
		_swap(&y0, &y1);
		_swap(&x0, &x1);
	}
	if (y0 == y2)
	{
		a = b = x0;
		if (x1 < a)
		{
			a = x1;
		}
		else if (x1 > b)
		{
			b = x1;
		}
		if (x2 < a)
		{
			a = x2;
		}
		else if (x2 > b)
		{
			b = x2;
		}
		LCD_Fill(a, y0, b, y0, POINT_COLOR);
		return;
	}
	dx01 = x1 - x0;
	dy01 = y1 - y0;
	dx02 = x2 - x0;
	dy02 = y2 - y0;
	dx12 = x2 - x1;
	dy12 = y2 - y1;

	if (y1 == y2)
	{
		last = y1;
	}
	else
	{
		last = y1 - 1;
	}
	for (y = y0; y <= last; y++)
	{
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		if (a > b)
		{
			_swap(&a, &b);
		}
		LCD_Fill(a, y, b, y, POINT_COLOR);
	}
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for (; y <= y2; y++)
	{
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if (a > b)
		{
			_swap(&a, &b);
		}
		LCD_Fill(a, y, b, y, POINT_COLOR);
	}
}

/** ***************************************************************************
 * @brief  显示字符
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  fc: 前景色
 * @param  bc: 背景色
 * @param  num: 字符ASCII码
 * @param  size: 字符大小
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void LCD_Show_Char(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 size, u8 mode)
{
	u8 temp;
	u8 pos, t;
	u16 color_temp = POINT_COLOR;

	num = num - ' ';									   // 得到偏移后的值
	LCD_Set_Windows(x, y, x + size / 2 - 1, y + size - 1); // 设置单个文字显示窗口
	if (!mode)											   // 非叠加方式
	{
		for (pos = 0; pos < size; pos++)
		{
			if (size == 12)
				temp = asc2_1206[num][pos]; // 调用1206字体
			else
				temp = asc2_1608[num][pos]; // 调用1608字体
			for (t = 0; t < size / 2; t++)
			{
				if (temp & 0x01)
					LCD_Write_Data_16Bit(fc);
				else
					LCD_Write_Data_16Bit(bc);
				temp >>= 1;
			}
		}
	}
	else // 叠加方式
	{
		for (pos = 0; pos < size; pos++)
		{
			if (size == 12)
				temp = asc2_1206[num][pos]; // 调用1206字体
			else
				temp = asc2_1608[num][pos]; // 调用1608字体
			for (t = 0; t < size / 2; t++)
			{
				POINT_COLOR = fc;
				if (temp & 0x01)
					LCD_Draw_Point(x + t, y + pos); // 画一个点
				temp >>= 1;
			}
		}
	}
	POINT_COLOR = color_temp;
	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复窗口为全屏
}

/** ***************************************************************************
 * @brief  显示字符串
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  size: 字符大小
 * @param  p: 字符串指针
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void LCD_Show_String(u16 x, u16 y, u8 size, u8 *p, u8 mode)
{
	while ((*p <= '~') && (*p >= ' ')) // 判断是不是非法字符!
	{
		if (x > (lcddev.width - 1) || y > (lcddev.height - 1))
			return;
		LCD_Show_Char(x, y, POINT_COLOR, BACK_COLOR, *p, size, mode);
		x += size / 2;
		p++;
	}
}

/** ***************************************************************************
 * @brief  取m的n次方（内部调用）
 * @param  m: 底数
 * @param  n: 指数
 * @retval m的n次方
 *****************************************************************************/
u32 mypow(u8 m, u8 n)
{
	u32 result = 1;
	while (n--)
		result *= m;
	return result;
}

/** ***************************************************************************
 * @brief  显示数字
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  num: 数字
 * @param  len: 显示长度
 * @param  size: 字符大小
 * @retval None
 *****************************************************************************/
void LCD_Show_Num(u16 x, u16 y, u32 num, u8 len, u8 size)
{
	u8 t, temp;
	u8 enshow = 0;
	for (t = 0; t < len; t++)
	{
		temp = (num / mypow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				LCD_Show_Char(x + (size / 2) * t, y, POINT_COLOR, BACK_COLOR, ' ', size, 0);
				continue;
			}
			else
				enshow = 1;
		}
		LCD_Show_Char(x + (size / 2) * t, y, POINT_COLOR, BACK_COLOR, temp + '0', size, 0);
	}
}

/** ***************************************************************************
 * @brief  显示16x16中文字符
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  fc: 前景色
 * @param  bc: 背景色
 * @param  s: 字符指针
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void GUI_Draw_Font16(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode)
{
	u8 i, j;
	u16 k;
	u16 hz_num;
	u16 x0 = x;
	hz_num = sizeof(tfont16) / sizeof(typFNT_GB16); // 自动统计汉字数目

	for (k = 0; k < hz_num; k++)
	{
		if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1)))
		{
			LCD_Set_Windows(x, y, x + 16 - 1, y + 16 - 1);
			for (i = 0; i < 16 * 2; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (!mode) // 非叠加方式
					{
						if (tfont16[k].Msk[i] & (0x80 >> j))
							LCD_Write_Data_16Bit(fc);
						else
							LCD_Write_Data_16Bit(bc);
					}
					else
					{
						POINT_COLOR = fc;
						if (tfont16[k].Msk[i] & (0x80 >> j))
							LCD_Draw_Point(x, y); // 画一个点
						x++;
						if ((x - x0) == 16)
						{
							x = x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}

	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复窗口为全屏
}

/** ***************************************************************************
 * @brief  显示24x24中文字符
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  fc: 前景色
 * @param  bc: 背景色
 * @param  s: 字符指针
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void GUI_Draw_Font24(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode)
{
	u8 i, j;
	u16 k;
	u16 hz_num;
	u16 x0 = x;
	hz_num = sizeof(tfont24) / sizeof(typFNT_GB24); // 自动统计汉字数目

	for (k = 0; k < hz_num; k++)
	{
		if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1)))
		{
			LCD_Set_Windows(x, y, x + 24 - 1, y + 24 - 1);
			for (i = 0; i < 24 * 3; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (!mode) // 非叠加方式
					{
						if (tfont24[k].Msk[i] & (0x80 >> j))
							LCD_Write_Data_16Bit(fc);
						else
							LCD_Write_Data_16Bit(bc);
					}
					else
					{
						POINT_COLOR = fc;
						if (tfont24[k].Msk[i] & (0x80 >> j))
							LCD_Draw_Point(x, y); // 画一个点
						x++;
						if ((x - x0) == 24)
						{
							x = x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}

	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复窗口为全屏
}

/** ***************************************************************************
 * @brief  显示32x32中文字符
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  fc: 前景色
 * @param  bc: 背景色
 * @param  s: 字符指针
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void GUI_Draw_Font32(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode)
{
	u8 i, j;
	u16 k;
	u16 hz_num;
	u16 x0 = x;
	hz_num = sizeof(tfont32) / sizeof(typFNT_GB32); // 自动统计汉字数目
	for (k = 0; k < hz_num; k++)
	{
		if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1)))
		{
			LCD_Set_Windows(x, y, x + 32 - 1, y + 32 - 1);
			for (i = 0; i < 32 * 4; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (!mode) // 非叠加方式
					{
						if (tfont32[k].Msk[i] & (0x80 >> j))
							LCD_Write_Data_16Bit(fc);
						else
							LCD_Write_Data_16Bit(bc);
					}
					else
					{
						POINT_COLOR = fc;
						if (tfont32[k].Msk[i] & (0x80 >> j))
							LCD_Draw_Point(x, y); // 画一个点
						x++;
						if ((x - x0) == 32)
						{
							x = x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}

	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复窗口为全屏
}

/** ***************************************************************************
 * @brief  显示中英文字符串
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  fc: 前景色
 * @param  bc: 背景色
 * @param  str: 字符串指针
 * @param  size: 字符大小
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void Show_Str(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode)
{
	u16 x0 = x;
	u8 b_hz = 0;	  // 字符或者中文
	while (*str != 0) // 数据未结束
	{
		if (!b_hz)
		{
			if (x > (lcddev.width - size / 2) || y > (lcddev.height - size))
				return;
			if (*str > 0x80)
				b_hz = 1; // 中文
			else		  // 字符
			{
				if (*str == 0x0D) // 换行符号
				{
					y += size;
					x = x0;
					str++;
				}
				else
				{
					if (size > 16) // 字库中没有集成12X24 16X32的英文字体,用8X16代替
					{
						LCD_Show_Char(x, y, fc, bc, *str, 16, mode);
						x += 8; // 字符,为全字的一半
					}
					else
					{
						LCD_Show_Char(x, y, fc, bc, *str, size, mode);
						x += size / 2; // 字符,为全字的一半
					}
				}
				str++;
			}
		}
		else // 中文
		{
			if (x > (lcddev.width - size) || y > (lcddev.height - size))
				return;
			b_hz = 0; // 有汉字库
			if (size == 32)
				GUI_Draw_Font32(x, y, fc, bc, str, mode);
			else if (size == 24)
				GUI_Draw_Font24(x, y, fc, bc, str, mode);
			else
				GUI_Draw_Font16(x, y, fc, bc, str, mode);

			str += 2;
			x += size; // 下一个汉字偏移
		}
	}
}

/** ***************************************************************************
 * @brief  居中显示字符串
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  fc: 前景色
 * @param  bc: 背景色
 * @param  str: 字符串指针
 * @param  size: 字符大小
 * @param  mode: 显示模式
 * @retval None
 *****************************************************************************/
void GUI_Str_Center(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode)
{
	u16 len = strlen((const char *)str);
	u16 x1 = (lcddev.width - len * 8) / 2;
	Show_Str(x1, y, fc, bc, str, size, mode);
}

/** ***************************************************************************
 * @brief  显示16位BMP图片
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  p: 图片数据指针
 * @retval None
 *****************************************************************************/
void GUI_Draw_BMP16(u16 x, u16 y, const unsigned char *p) // 显示40*40 QQ图片
{
	int i;
	unsigned char pic_h, pic_l;
	LCD_Set_Windows(x, y, x + 40 - 1, y + 40 - 1); // 窗口设置
	for (i = 0; i < 40 * 40; i++)
	{
		pic_l = *(p + i * 2); // 数据低位在前
		pic_h = *(p + i * 2 + 1);
		LCD_Write_Data_16Bit(pic_h << 8 | pic_l);
	}
	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复显示窗口为全屏
}

/*****************************************************************************
 * @name       :void Gui_StrCenter(u16 x, u16 y, u16 fc, u16 bc, u8 *str,u8 size,u8 mode)
 * @date       :2018-08-09
 * @function   :Centered display of English and Chinese strings
 * @parameters :x:the bebinning x coordinate of the Chinese and English strings
				y:the bebinning y coordinate of the Chinese and English strings
								fc:the color value of Chinese and English strings
								bc:the background color of Chinese and English strings
								str:the start address of the Chinese and English strings
								size:the size of Chinese and English strings
								mode:0-no overlying,1-overlying
 * @retvalue   :None
******************************************************************************/
void Gui_StrCenter(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode)
{
	u16 len = strlen((const char *)str);
	u16 x1 = (lcddev.width - len * 8) / 2;
	Show_Str(x1, y, fc, bc, str, size, mode);
}

/*****************************************************************************
 * @name       :void Gui_Drawbmp16(u16 x,u16 y,const unsigned char *p)
 * @date       :2018-08-09
 * @function   :Display a 16-bit BMP image
 * @parameters :x:the bebinning x coordinate of the BMP image
				y:the bebinning y coordinate of the BMP image
								p:the start address of image array
 * @retvalue   :None
******************************************************************************/
void Gui_Drawbmp16(u16 x, u16 y, const unsigned char *p) // 显示40*40 QQ图片
{
	int i;
	unsigned char picH, picL;
	LCD_Set_Windows(x, y, x + 40 - 1, y + 40 - 1); // 窗口设置
	for (i = 0; i < 40 * 40; i++)
	{
		picL = *(p + i * 2); // 数据低位在前
		picH = *(p + i * 2 + 1);
		LCD_Write_Data_16Bit(picH << 8 | picL);
	}
	LCD_Set_Windows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复显示窗口为全屏
}
