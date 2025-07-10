#include "kalman.h"

Kalman kfp_x, kfp_y;

void Kalman_Init(Kalman *kalman)
{
   kalman->Last_P = 1;
   kalman->Now_P = 0;
   kalman->out = 0;
   kalman->Kg = 0;
   kalman->Q = 0.009; // 过程噪声协方差，越小越相信模型
   kalman->R = 0.10;  // 测量噪声协方差，越小越相信测量
   kalman->source = 0;
}

/**
 * 卡尔曼滤波器初始化
 * @param   Kalman *kfp 卡尔曼结构体指针
 *          float input 当前测量值（传感器输出）
 * @return  滤波后的值（估计值）
 */
float KalmanFilter(Kalman *kfp, float input)
{
   // 预测协方差: k时刻的先验估计协方差 = k-1时刻的后验估计协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   // 计算卡尔曼增益: 卡尔曼增益 = 先验估计协方差 / (先验估计协方差 + 测量噪声协方差)
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   // 更新估计值: 后验估计 = 先验估计 + 卡尔曼增益 * (测量值 - 先验估计)
   kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 估计值更新
   // 更新后验协方差: 后验协方差 = (1 - 卡尔曼增益) * 先验协方差
   kfp->Last_P = (1 - kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
