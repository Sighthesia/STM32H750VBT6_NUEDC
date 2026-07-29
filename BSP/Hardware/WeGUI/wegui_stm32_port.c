#include "lcd_driver.h"
#include "ips114_lcd.h"
#include "main.h"

void lcd_port_init(void)
{
    IPS114_LCD_Init();
}

void lcd_delay_ms(volatile uint32_t ms)
{
    HAL_Delay(ms);
}

void lcd_bl_on(void)
{
}

void lcd_bl_off(void)
{
}

uint8_t lcd_is_busy(void)
{
    return 0U;
}

void lcd_send_1Cmd(uint8_t dat)
{
    (void)dat;
}

void lcd_send_nCmd(uint8_t *p, uint16_t num)
{
    (void)p;
    (void)num;
}

void lcd_send_1Dat(uint8_t dat)
{
    (void)dat;
}

void lcd_send_nDat(uint8_t *p, uint16_t num)
{
    (void)p;
    (void)num;
}

void lcd_rgb565_port(uint16_t x0, uint16_t x1, uint16_t page, uint8_t *page_gram)
{
    static uint8_t pixels[SCREEN_WIDTH * 8U * 2U];
    uint16_t y0 = page * 8U;
    uint16_t y1;
    uint16_t y;
    uint16_t length = 0U;

    if (page_gram == NULL || x0 > x1 || x1 >= SCREEN_WIDTH || y0 >= SCREEN_HIGH)
    {
        return;
    }

    y1 = y0 + 7U;
    if (y1 >= SCREEN_HIGH)
    {
        y1 = SCREEN_HIGH - 1U;
    }

    for (y = y0; y <= y1; ++y)
    {
        uint16_t x;
        uint8_t mask = (uint8_t)(1U << (y - y0));

        for (x = x0; x <= x1; ++x)
        {
            uint16_t color = (page_gram[(x - x0) * LCD_COLOUR_BIT] & mask) != 0U ?
                                 lcd_driver.colour[1] : lcd_driver.colour[0];
            pixels[length++] = (uint8_t)(color >> 8);
            pixels[length++] = (uint8_t)color;
        }
    }

    IPS114_LCD_WritePixels(x0, y0, x1, y1, pixels, length);
}
