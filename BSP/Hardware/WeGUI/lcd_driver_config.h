#ifndef LCD_DRIVER_CONFIG_H
#define LCD_DRIVER_CONFIG_H

#include <stdint.h>

#define _LCD_NO_PORT 0
#define _LCD_PORT_STM32H7 1
#define LCD_PORT _LCD_PORT_STM32H7

#define SCREEN_WIDTH 128
#define SCREEN_HIGH 160
#define SCREEN_X_OFFSET 0
#define SCREEN_Y_OFFSET 0

#define _FULL_BUFF_FULL_UPDATE 0
#define _FULL_BUFF_DYNA_UPDATE 1
#define _PAGE_BUFF_FULL_UPDATE 2
#define _PAGE_BUFF_DYNA_UPDATE 3
#define LCD_MODE _FULL_BUFF_FULL_UPDATE
#define GRAM_YPAGE_NUM ((SCREEN_HIGH + 7) / 8)

#define STARTUP_FONTS_ASCII (&mcu_fonts_ascii_songti_8X16)
#define STARTUP_FONTS_UTF8 (&mcu_fonts_utf8_songti_16X16)

#define _FLASH_NO_PORT 0
#define FLASH_PORT _FLASH_NO_PORT
#define _FLASH_NO_IC 0
#define FLASH_IC _FLASH_NO_IC

#define LCD_UNKNOW 0
#define LCD_OLED 1
#define LCD_GRAY 2
#define _ST7735 10
#define LCD_IC _ST7735
#define LCD_RGB565 3
#define LCD_TYPE LCD_RGB565
#define LCD_COLOUR_BIT 2
#define WEGUI_PANEL_SELF_TEST 0

#include "st7735.h"

void wegui_stm32_port_init(void);
void wegui_stm32_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void wegui_stm32_panel_self_test(void);
uint8_t lcd_is_busy(void);
void lcd_send_1Cmd(uint8_t value);
void lcd_send_nCmd(uint8_t *data, uint16_t length);
void lcd_send_1Dat(uint8_t value);
void lcd_send_nDat(uint8_t *data, uint16_t length);
void lcd_bl_on(void);
void lcd_bl_off(void);
void lcd_delay_ms(volatile uint32_t ms);
void lcd_rgb565_port(uint16_t x0, uint16_t x1, uint16_t page, uint8_t *gram);

#define lcd_port_init() wegui_stm32_port_init()
#define lcd_ic_init() ST7735_Init()
#define lcd_set_addr(x0, y0, x1, y1) ST7735_Set_Addr((x0), (y0), (x1), (y1))
#define lcd_set_bright(x) do { (void)(x); } while (0)

#endif
