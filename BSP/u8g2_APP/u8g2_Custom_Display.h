/** ***************************************************************************
 * @File Name: u8g2_Custom_Display.h
 * @brief  u8g2自定义显示函数
 * @Author : Sighthesia
 * @Version : 1.0
 * @Creat Date : 2025-05-06
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __U8G2_DISPLAY_CUSTOM_H__
#define __U8G2_DISPLAY_CUSTOM_H__

#include "stdio.h"
#include "u8g2.h"
#include "u8x8.h"

/* ---------------------------------- 常用坐标 ---------------------------------- */
#define SCREEN_WIDTH       128 // 屏幕宽度
#define SCREEN_HEIGHT      64  // 屏幕高度
#define SCREEN_WIDTH_1_2   SCREEN_WIDTH / 2
#define SCREEN_HEIGHT_1_2  SCREEN_HEIGHT / 2
#define SCREEN_WIDTH_1_4   SCREEN_WIDTH / 4
#define SCREEN_HEIGHT_1_4  SCREEN_HEIGHT / 4
#define SCREEN_WIDTH_1_8   SCREEN_WIDTH / 8
#define SCREEN_HEIGHT_1_8  SCREEN_HEIGHT / 8
#define SCREEN_WIDTH_1_16  SCREEN_WIDTH / 16
#define SCREEN_HEIGHT_1_16 SCREEN_HEIGHT / 16

void PrintVarFormat(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, const uint8_t *font, short var);

#endif
