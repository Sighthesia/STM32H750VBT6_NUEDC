#ifndef LCD_WEGUI_CONFIG_H
#define LCD_WEGUI_CONFIG_H

#include <stdint.h>

#include "lcd_driver_config.h"

#define STR_MCU_CLASS "STM32H7"
#define STR_MCU_MODEL "STM32H750"
#define STR_WEGUI_VERSION_CLASS "V0.5"
#define STR_WEGUI_VERSION "V0.5.7"

#define MENU_CONST_DIS 1
#define MENU_DEEP 3
#define MLIST_MENU_CURSOR_UP_LOOP 0
#define MLIST_MENU_CURSOR_DOWN_LOOP 0
#define MLIST_MENU_YSCAPE (mList_par.list_font_high / 2)
#define STARTUP_DEBUG_WINDOWS_EN 0

#define _NO_GUI_PORT 0
#define _GUI_KEY_PORT 1
#define WEGUI_PORT _GUI_KEY_PORT

void wegui_stm32_input_init(void);
void wegui_stm32_input_tick_1ms(void);
void wegui_stm32_input_process(uint16_t elapsed_ms);

#define wegui_port_init() wegui_stm32_input_init()
#define wegui_port_task(stick) wegui_stm32_input_process(stick)
#define wegui_port_ms_irq() wegui_stm32_input_tick_1ms()

#define COLOUR_MLIST_DEFAULT_0 0x530D
#define COLOUR_MLIST_DEFAULT_1 0xEFBE
#define COLOUR_MLIST_DEFAULT_2 0xC6BB
#define COLOUR_MLIST_DEFAULT_3 0x2104
#define COLOUR_MLIST_NORMAL_TEXT 1
#define COLOUR_MLIST_WSLIDER_NUM 1
#define COLOUR_MLIST_WMESSAGE_TEXT 1
#define COLOUR_MLIST_WCHECKBOX_BORDER 1
#define COLOUR_MLIST_WCHECKBOX_INTER 3
#define COLOUR_MLIST_SCROOL_BAR 1
#define COLOUR_MESS_TIP_BOX_BORDER 2
#define COLOUR_MESS_TIP_TEXT 2
#define COLOUR_SLIDER_TIP_BOX_BORDER 2
#define COLOUR_SLIDER_TIP_TEXT 2
#define COLOUR_SLIDER_TIP_NUM 2
#define COLOUR_SLIDER_TIP_BAR_BORDER 2
#define COLOUR_SLIDER_TIP_BAR_PROG 1
#define COLOUR_DEBUG_BAR_BG 0
#define COLOUR_DEBUG_BAR_BORDER 1
#define COLOUR_DEBUG_BAR_TEXT 1

#endif
