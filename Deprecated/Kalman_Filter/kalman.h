#ifndef __Kalman_H
#define __Kalman_H

#include "main.h"

typedef struct
{
    float Last_P; // 上次估算协方差
    float Now_P;  // 当前估算协方差
    float out;    // 卡尔曼滤波输出
    float Kg;     // 卡尔曼增益
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
    float source; // 测量值
} Kalman;

void Kalman_Init(Kalman *kalman);
float KalmanFilter(Kalman *kfp, float input);
extern Kalman kfp_x, kfp_y;

#endif
