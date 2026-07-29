/*
    Copyright 2025 Lu Zhihao

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0
*/
#ifndef LCD_DRIVER_CONFIG_H
#define LCD_DRIVER_CONFIG_H

#define _LCD_NO_PORT 0
#define LCD_PORT 1

#define SCREEN_WIDTH 240U
#define SCREEN_HIGH 135U

#define _FULL_BUFF_FULL_UPDATE 0
#define _FULL_BUFF_DYNA_UPDATE 1
#define _PAGE_BUFF_FULL_UPDATE 2
#define _PAGE_BUFF_DYNA_UPDATE 3
#define LCD_MODE _FULL_BUFF_FULL_UPDATE
#define GRAM_YPAGE_NUM ((SCREEN_HIGH + 7U) / 8U)
/* lcd_driver_t declares the CRC array for every refresh mode. */
#define PAGE_CRC_NUM 1U

#define _FLASH_NO_PORT 0
#define FLASH_PORT _FLASH_NO_PORT

#define LCD_RGB565 3
#define LCD_TYPE LCD_RGB565
#define LCD_COLOUR_BIT 1

#include "lcd_port_template.h"
#include "lcd_res.h"

#define STARTUP_FONTS_ASCII (&mcu_fonts_ascii_songti_6X12)
#define STARTUP_FONTS_UTF8 (&mcu_fonts_utf8_songti_12X12)

#endif /* LCD_DRIVER_CONFIG_H */
