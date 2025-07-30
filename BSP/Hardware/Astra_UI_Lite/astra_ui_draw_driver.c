//
// Created by Fir on 24-11-30.
//

#include "astra_ui_draw_driver.h"

/* 此处自行编写oled及图形库初始化函数所需的函数 */
u8g2_t u8g2;

void u8g2_init(u8g2_t *u8g2)
{
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2,
                                           U8G2_R0,
                                           u8x8_byte_hw_i2c,
                                           u8g2_gpio_and_delay); // 初始化u8g2 结构体
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    u8g2_ClearBuffer(u8g2);
    u8g2_SetFontMode(u8g2, 1);
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, wenquanyi_Custom_16px);
}

/* 此处自行编写oled及图形库初始化函数所需的函数 */

/* 此处自行修改内部函数名 */
void astra_ui_driver_init()
{
    u8g2_init(&u8g2);
}

/* 此处自行修改内部函数名 */
