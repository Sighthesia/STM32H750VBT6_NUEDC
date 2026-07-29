/*
    Copyright 2025 Lu Zhihao

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0
*/
#ifndef LCD_WEGUI_CONFIG_H
#define LCD_WEGUI_CONFIG_H

#define MENU_CONST_DIS 0
#define MENU_DEEP 3
#define WEGUI_PORT 0
#define WEGUI_UART_EN 0
#define MLIST_MENU_CURSOR_UP_LOOP 0
#define MLIST_MENU_CURSOR_DOWN_LOOP 0
#define STARTUP_DEBUG_WINDOWS_EN 0

#include "TFT_Color.h"

#define COLOUR_MLIST_DEFAULT_0 RGB565_Navy
#define COLOUR_MLIST_DEFAULT_1 RGB565_White
#define COLOUR_MLIST_NORMAL_TEXT write_1
#define COLOUR_MLIST_WMESSAGE_TEXT write_1
#define COLOUR_MLIST_WSLIDER_NUM write_1
#define COLOUR_MLIST_WCHECKBOX_BORDER write_1
#define COLOUR_MLIST_WCHECKBOX_INTER write_1
#define COLOUR_MLIST_SCROOL_BAR write_1
#define COLOUR_DEBUG_BAR_BORDER write_1
#define COLOUR_DEBUG_BAR_BG write_0
#define COLOUR_DEBUG_BAR_TEXT write_1
#define COLOUR_MESS_TIP_BOX_BORDER write_1
#define COLOUR_MESS_TIP_TEXT write_1
#define COLOUR_SLIDER_TIP_BOX_BORDER write_1
#define COLOUR_SLIDER_TIP_TEXT write_1
#define COLOUR_SLIDER_TIP_NUM write_1
#define COLOUR_SLIDER_TIP_BAR_PROG write_1
#define COLOUR_SLIDER_TIP_BAR_BORDER write_1

#endif /* LCD_WEGUI_CONFIG_H */
