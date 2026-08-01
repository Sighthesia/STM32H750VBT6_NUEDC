/*
	Copyright 2025 Lu Zhihao
	本程序仅供学习用途, 暂不公开对其他用途的授权
*/

#include "lcd_wegui_menu_app.h"
#include "Board_Link.h"
#include "lcd_driver.h"

static uint8_t active_test;

static void draw_hex(uint8_t y, const char *label, uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    char text[12] = {0};
    uint8_t i;

    text[0] = label[0];
    text[1] = label[1];
    text[2] = ':';
    for (i = 0U; i < 8U; i++) text[3U + i] = hex[(value >> (28U - i * 4U)) & 0x0FU];
    lcd_draw_utf8_string(4, y, text);
}

static void begin_test(uint8_t test_id)
{
    active_test = test_id;
    Board_Link_Select_Test(test_id);
    Board_Link_Start_Test();
}

void Competition_Test_1_Begin(void) { begin_test(1U); }
void Competition_Test_2_Begin(void) { begin_test(2U); }
void Competition_Test_3_Begin(void) { begin_test(3U); }
void Competition_Test_4_Begin(void) { begin_test(4U); }
void Competition_Test_5_Begin(void) { begin_test(5U); }
void Competition_Test_6_Begin(void) { begin_test(6U); }
void Competition_Test_Quit(void) { Board_Link_Stop_Test(); }
void Competition_Test_Stop(void) { Board_Link_Stop_Test(); }

uint8_t Competition_Test_Refresh(uint8_t ui_frames, uint16_t elapsed_ms)
{
    const char *state = "IDLE";
    (void)ui_frames;
    (void)elapsed_ms;
    if (!Board_Link_Is_Online()) state = "LINK OFF";
    else if (Board_Link_Get_Command_Status() == BOARD_LINK_COMMAND_WAITING) state = "NO ACK";
    else if (Board_Link_Get_Command_Status() == BOARD_LINK_COMMAND_OK) state = "RUNNING";
    else if (Board_Link_Get_Command_Status() == BOARD_LINK_COMMAND_REJECTED) state = "REJECTED";
    else if (Board_Link_Get_Command_Status() == BOARD_LINK_COMMAND_ERROR) state = "SPI ERROR";
    lcd_clear_gram();
    lcd_draw_utf8_string(4, 4, "TEST");
    lcd_draw_utf8_string(4, 26, active_test == 1U ? "1" : active_test == 2U ? "2" : active_test == 3U ? "3" : active_test == 4U ? "4" : active_test == 5U ? "5" : "6");
    lcd_draw_utf8_string(4, 48, (char *)state);
    draw_hex(70, "CM", Board_Link_Get_Last_Command());
    draw_hex(82, "AK", Board_Link_Get_Last_Ack());
    draw_hex(94, "CR", Board_Link_Get_Peer_Diagnostic());
    draw_hex(106, "RX", Board_Link_Get_Valid_Frame_Count());
    draw_hex(118, "ER", Board_Link_Get_Last_Spi_Error());
    return 1U;
}
