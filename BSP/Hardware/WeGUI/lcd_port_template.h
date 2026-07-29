#ifndef LCD_PORT_TEMPLATE_H
#define LCD_PORT_TEMPLATE_H

#include <stdint.h>

void lcd_port_init(void);
void lcd_delay_ms(volatile uint32_t ms);
void lcd_bl_on(void);
void lcd_bl_off(void);
uint8_t lcd_is_busy(void);
void lcd_send_1Cmd(uint8_t dat);
void lcd_send_nCmd(uint8_t *p, uint16_t num);
void lcd_send_1Dat(uint8_t dat);
void lcd_send_nDat(uint8_t *p, uint16_t num);
void lcd_rgb565_port(uint16_t x0, uint16_t x1, uint16_t page, uint8_t *page_gram);

#endif /* LCD_PORT_TEMPLATE_H */
