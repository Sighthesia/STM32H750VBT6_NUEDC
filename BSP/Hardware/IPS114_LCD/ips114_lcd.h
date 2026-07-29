#ifndef IPS114_LCD_H
#define IPS114_LCD_H

#include <stdint.h>

#define IPS114_LCD_WIDTH  240U
#define IPS114_LCD_HEIGHT 135U

#define IPS114_COLOR_BLACK 0x0000U
#define IPS114_COLOR_WHITE 0xFFFFU
#define IPS114_COLOR_RED   0xF800U
#define IPS114_COLOR_GREEN 0x07E0U
#define IPS114_COLOR_BLUE  0x001FU
#define IPS114_COLOR_YELLOW 0xFFE0U
#define IPS114_COLOR_CYAN  0x07FFU

void IPS114_LCD_Init(void);
void IPS114_LCD_Fill(uint16_t color);
void IPS114_LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void IPS114_LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void IPS114_LCD_DrawString(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t background);
void IPS114_LCD_ShowSelfTest(void);

#endif /* IPS114_LCD_H */
