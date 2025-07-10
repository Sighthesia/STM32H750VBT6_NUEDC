#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "stdint.h"

int GFP_abs(int p);
void mode_1(void);
void mode_2(void);

void urat_fx(void);
void test(void);

uint8_t get_point1_p();
uint8_t get_point2_p(); // 处理坐标函数
void go_to_point(uint16_t now_x, uint16_t now_y, uint16_t targ_x, uint16_t targ_y);
void write_point_to_flash(int16_t *arr);
void read_point_from_flash(int16_t *arr);

int abs(int x);

#endif /* __CONTROL_H */
