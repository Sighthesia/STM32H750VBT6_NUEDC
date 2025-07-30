#pragma once

//头文件
#include"sp_sys.h"
#include"zdt_stepmot_driver.h"
#include"zdt_stepmot_driver_ad1.h"

//云台初始化
void stepmot_pzt_init();
//云台设置
void stepmot_pzt_set(u32 stp1,float spd1,u32 stp2,float spd2);
//云台设置2
void stepmot_pzt_set_2(i32 stp1,i32 stp2);
//云台检查
u8 stepmot_pzt_check();
//云台复位
void stepmot_pzt_reset();
//云台绘制
void stepmot_pzt_draw(i32 x,i32 y);
