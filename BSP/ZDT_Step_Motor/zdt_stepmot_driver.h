#pragma once

//头文件
#include<stdint.h>
// #include"driver/gpio.h"
#include"sp_sys.h"

//电机初始化
void zdt_stepmot_init();
//电机使能
void zdt_stepmot_en(uint8_t en);
//电机设置
void zdt_stepmot_set(uint8_t dir,uint32_t stp,float spd);
//电机状态检查（仅在电机空闲时可操作）
uint8_t zdt_stepmot_check();
