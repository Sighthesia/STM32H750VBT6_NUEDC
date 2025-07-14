/** ***************************************************************************
 * @File Name: u8g2_Custom_Display.c
 * @brief  u8g2自定义显示函数
 * @Author : Sighthesia
 * @Version : 1.0
 * @Creat Date : 2025-04-26
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "u8g2_Custom_Display.h"

/// @brief u8g2显示变量
/// @param u8g2 u8g2结构体
/// @param x x坐标
/// @param y y坐标
/// @param font 字体
/// @param var 要显示的变量
void PrintVarFormat(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, const uint8_t *font, short var)
{
    char var_buff[100];                 // 用来存ASCII码
    u8g2_SetFont(u8g2, font);           // 设定字体
    sprintf(var_buff, "%4d", var);      // 传递变量（十进制（占四格））
    u8g2_DrawStr(u8g2, x, y, var_buff); // 显示
}