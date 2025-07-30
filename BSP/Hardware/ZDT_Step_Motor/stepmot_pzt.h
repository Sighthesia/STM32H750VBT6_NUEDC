#pragma once

// 头文件
#include "zdt_stepmot_driver.h"
#include "zdt_stepmot_driver_ad1.h"

// 云台初始化
void stepmot_pzt_init();
// 云台设置
void stepmot_pzt_set(uint32_t stp1, float spd1, uint32_t stp2, float spd2);
// 云台设置2
void stepmot_pzt_set_2(int32_t stp1, int32_t stp2);
// 云台检查
uint8_t stepmot_pzt_check();
// 云台复位
void stepmot_pzt_reset();
// 云台绘制
void stepmot_pzt_draw(int32_t x, int32_t y);
