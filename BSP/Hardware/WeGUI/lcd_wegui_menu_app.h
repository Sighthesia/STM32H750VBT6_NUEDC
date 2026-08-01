/*
	Copyright 2025 Lu Zhihao
	本程序仅供学习用途, 暂不公开对其他用途的授权
*/

#ifndef LCD_WEGUI_MENU_APP_H
#define LCD_WEGUI_MENU_APP_H

#include <stdint.h>

void Competition_Test_1_Begin(void);
void Competition_Test_2_Begin(void);
void Competition_Test_3_Begin(void);
void Competition_Test_4_Begin(void);
void Competition_Test_5_Begin(void);
void Competition_Test_6_Begin(void);
void Competition_Test_Quit(void);
void Competition_Test_Stop(void);
uint8_t Competition_Test_Refresh(uint8_t ui_frames, uint16_t elapsed_ms);

#endif
