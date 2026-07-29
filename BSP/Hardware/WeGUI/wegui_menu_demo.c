#include "wegui_menu_demo.h"

static uint8_t g_demo_enabled = 1U;
static int16_t g_demo_level = 50;
static char g_demo_status[] = "READY";
static char g_demo_version[] = "STM32H7";

extern menu_t m_main;
extern menu_t m_display;
extern menu_t m_system;
extern menu_t m_display_enable;
extern menu_t m_display_level;
extern menu_t m_system_status;
extern menu_t m_system_version;

menu_t m_display_level = {
    .fatherMenu = &m_display,
    .subMenu = NULL,
    .nextMenu = NULL,
    .titel = {.str_en_US = "DISPLAY", .str_zh_CN = "DISPLAY", .str_zh_TC = "DISPLAY"},
    .discribe = {.str_en_US = "Brightness", .str_zh_CN = "Brightness", .str_zh_TC = "Brightness"},
    .menuType = wSlider,
    .menuPar.wSliderTip_Par = {.Push_tip_func = NULL, .Change_Value_func = NULL, .End_tip_func = NULL,
                                .tip_string = {.str_en_US = "Brightness", .str_zh_CN = "Brightness", .str_zh_TC = "Brightness"},
                                .pvalue = &g_demo_level, .min = 0, .max = 100}
};

menu_t m_display_enable = {
    .fatherMenu = &m_display,
    .subMenu = NULL,
    .nextMenu = &m_display_level,
    .titel = {.str_en_US = "DISPLAY", .str_zh_CN = "DISPLAY", .str_zh_TC = "DISPLAY"},
    .discribe = {.str_en_US = "Enabled", .str_zh_CN = "Enabled", .str_zh_TC = "Enabled"},
    .menuType = wCheckBox,
    .menuPar.wCheckBox_Par = {.pstr = &g_demo_enabled, .Change_Value = NULL}
};

menu_t m_system_version = {
    .fatherMenu = &m_system,
    .subMenu = NULL,
    .nextMenu = NULL,
    .titel = {.str_en_US = "SYSTEM", .str_zh_CN = "SYSTEM", .str_zh_TC = "SYSTEM"},
    .discribe = {.str_en_US = "Platform", .str_zh_CN = "Platform", .str_zh_TC = "Platform"},
    .menuType = wMessage,
    .menuPar.wMessage_Par = {.Press_func = NULL, .Value_string = g_demo_version,
                              .Tip_string = {.str_en_US = "Platform", .str_zh_CN = "Platform", .str_zh_TC = "Platform"}}
};

menu_t m_system_status = {
    .fatherMenu = &m_system,
    .subMenu = NULL,
    .nextMenu = &m_system_version,
    .titel = {.str_en_US = "SYSTEM", .str_zh_CN = "SYSTEM", .str_zh_TC = "SYSTEM"},
    .discribe = {.str_en_US = "Status", .str_zh_CN = "Status", .str_zh_TC = "Status"},
    .menuType = wMessage,
    .menuPar.wMessage_Par = {.Press_func = NULL, .Value_string = g_demo_status,
                              .Tip_string = {.str_en_US = "System status", .str_zh_CN = "System status", .str_zh_TC = "System status"}}
};

menu_t m_system = {
    .fatherMenu = &m_main,
    .subMenu = &m_system_status,
    .nextMenu = NULL,
    .titel = {.str_en_US = "SYSTEM", .str_zh_CN = "SYSTEM", .str_zh_TC = "SYSTEM"},
    .discribe = {.str_en_US = "System", .str_zh_CN = "System", .str_zh_TC = "System"},
    .menuType = mList,
    .menuPar.mList_Par = {.begin_fun = NULL, .loop_fun = NULL, .quit_fun = NULL, .pstr_control_func = NULL}
};

menu_t m_display = {
    .fatherMenu = &m_main,
    .subMenu = &m_display_enable,
    .nextMenu = &m_system,
    .titel = {.str_en_US = "DISPLAY", .str_zh_CN = "DISPLAY", .str_zh_TC = "DISPLAY"},
    .discribe = {.str_en_US = "Display", .str_zh_CN = "Display", .str_zh_TC = "Display"},
    .menuType = mList,
    .menuPar.mList_Par = {.begin_fun = NULL, .loop_fun = NULL, .quit_fun = NULL, .pstr_control_func = NULL}
};

menu_t m_main = {
    .fatherMenu = NULL,
    .subMenu = &m_display,
    .nextMenu = NULL,
    .titel = {.str_en_US = "WEGUI DEMO", .str_zh_CN = "WEGUI DEMO", .str_zh_TC = "WEGUI DEMO"},
    .discribe = {.str_en_US = "", .str_zh_CN = "", .str_zh_TC = ""},
    .menuType = mList,
    .menuPar.mList_Par = {.begin_fun = NULL, .loop_fun = NULL, .quit_fun = NULL, .pstr_control_func = NULL}
};
