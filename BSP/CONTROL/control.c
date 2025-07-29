#include <math.h>

#include "control.h"

// #include "adc.h"
// #include "key.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>

#include "Servo_APP.h"

/********************************************
 *                                           *
 *               内部变量                     *
 *                                           *
 ********************************************/
// 电池电压采集辅组变量
static int Voltage_All = 0;
static uint8_t Voltage_Count = 0;

/********************************************
 *                                           *
 *               全局变量                     *
 *                                           *
 ********************************************/
float Voltage; // 电池电压

// 将角度转换为PWM值（适用于270度舵机）
int angle_to_pwm_270(float angle)
{
	// 限制角度在0到270度之间
	if (angle < 0.0f)
	{
		angle = 0.0f;
	}
	else if (angle > 270.0f)
	{
		angle = 270.0f;
	}

	// 线性转换公式：PWM = (angle * 1000 / 270) + 250
	float pwm = (angle * 1000.0f / 270.0f) + 250.0f;
	return (int)roundf(pwm); // 四舍五入到整数
}

int angle_to_pwm_180(float angle)
{

	if (angle < 0.0f)
	{
		angle = 0.0f;
	}
	else if (angle > 180.0f)
	{
		angle = 180.0f;
	}

	float pwm = (angle * 1000.0f / 180.0f) + 250.0f;
	return (int)roundf(pwm); // 四舍五入到整数
}

float base_x(float ts, float x_point)
{
	float angle = 0;
	angle = 135.0f - (atan(x_point / ts)) * 57.3f;
	return angle;
}

float base_y(float ts, float y_point)
{
	float angle = 0;
	angle = 90.0f - (atan(y_point / ts)) * 57.3f;
	return angle;
}

float real_arm_angle;
float real_base_angle;

float step_base;
float step_arm;

// 激光笔在UV纸板的投射距离，单位是米
float ts_dis = 0.4f;

// 定时器2更新中断
// void TIM2_IRQHandler(void)
// {
// 	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
// 	{
// 		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
void Draw(void)
{
	static float target_arm_angle = 90;
	static float target_base_angle = 135;

	static float last_arm_angle = 90;
	static float last_base_angle = 135;

	static float real_arm_angle = 90;
	static float real_base_angle = 135;

	static float step_base;
	static float step_arm;

	static int32_t mode = 0;

	///////////////////////////// 三角形绘制  使用打开此部分注释后,再注释其他部分 ////////////////////////////
	static int interp_step = 0; // 插值步数计数器

	const int cnt = 4;

	if (interp_step == 0)
	{ // 仅在插值开始时更新目标角度
		if (mode <= 0)
		{
			target_base_angle = base_x(ts_dis, 0);
			target_arm_angle = base_y(ts_dis, 0);
		}
		else if (mode == 30)
		{
			target_base_angle = base_x(ts_dis, 0.0);
			target_arm_angle = base_y(ts_dis, 0.05);
		}
		else if (mode == 30 * 2)
		{
			target_base_angle = base_x(ts_dis, -0.05);
			target_arm_angle = base_y(ts_dis, 0.0);
		}
		else if (mode == 30 * 3)
		{
			target_base_angle = base_x(ts_dis, 0.05);
			target_arm_angle = base_y(ts_dis, 0.0);
			mode = 0; // 重置mode以循环绘制三角形
		}

		// 计算插值步长
		step_base = (target_base_angle - last_base_angle) / cnt;
		step_arm = (target_arm_angle - last_arm_angle) / cnt;
		last_base_angle = target_base_angle;
		last_arm_angle = target_arm_angle;
	}

	// 插值
	real_arm_angle = real_arm_angle + step_arm;
	real_base_angle = real_base_angle + step_base;

	// 角度转换为PWM
	int pwmA = angle_to_pwm_180(real_arm_angle);
	int pwmB = angle_to_pwm_270(real_base_angle);

	// 输出到舵机
	// TIM4->CCR3 = pwmB;
	// TIM4->CCR4 = pwmA;
	Servo_Set_PWM(SERVO_0X, pwmB);
	Servo_Set_PWM(SERVO_0Y, pwmA);

	// 更新插值步数和mode
	interp_step++;
	if (interp_step >= cnt)
	{ // 插值完成，切换到下一个目标点
		interp_step = 0;
		mode++;
	}
	///////////////////////////// 三角形 END ////////////////////////////

	///////////////////////////// 圆形绘制  使用打开此部分注释后,再注释其他部分  //////////////////////////
	// static float theta = 0.0f;									  // 圆的参数化角度（弧度）
	// static const float radius = 0.06f;							  // 圆的半径（假设与你的偏移量单位一致）
	// static const int steps = 1000;								  // 每圈的步数
	// static const float theta_step = 2.0f * 3.1415926535f / steps; // 每步的角度增量

	// if (mode < steps)
	// {
	// 	// 计算目标角度
	// 	target_base_angle = base_x(ts_dis, radius * cos(theta));
	// 	target_arm_angle = base_y(ts_dis, radius * sin(theta));

	// 	// 计算每步的增量
	// 	step_base = (target_base_angle - last_base_angle) / 100.0f;
	// 	step_arm = (target_arm_angle - last_arm_angle) / 100.0f;

	// 	// 更新上一次的角度
	// 	last_base_angle = target_base_angle;
	// 	last_arm_angle = target_arm_angle;

	// 	// 增加角度参数
	// 	theta += theta_step;
	// 	mode++;
	// }
	// else
	// {
	// 	// 一圈完成后重置或进入其他状态
	// 	mode = 0;
	// 	theta = 0.0f; // 重置角度以重复绘制圆形
	// }

	// // 计算实际角度（插值）
	// real_arm_angle = last_arm_angle + step_arm;
	// real_base_angle = last_base_angle + step_base;

	// // 转换为 PWM 信号
	// int pwmA = angle_to_pwm_180(real_arm_angle);
	// int pwmB = angle_to_pwm_270(real_base_angle);

	// // 输出到舵机
	// // TIM4->CCR3 = pwmB;
	// // TIM4->CCR4 = pwmA;
	// Servo_Set_PWM(SERVO_0X, pwmB);
	// Servo_Set_PWM(SERVO_0Y, pwmA);
	// ///////////////////////////// 圆形 END //////////////////////////

	// ///////////////////////////// 正弦波 绘制  使用打开此部分注释后,再注释其他部分  ////////////////////////////////
	// static float theta = 0.0f;					 // 坐标值
	// static const float radius = 0.03f;			 // 圆的半径（假设与你的偏移量单位一致）
	// static const int steps = 500;				 // 每圈的步数
	// static const float theta_step = 0.4 / steps; // 每步的角度增量

	// static uint8_t low_fre = 0;

	// low_fre = !low_fre;

	// if (1)
	// {
	// 	if (mode < steps)
	// 	{
	// 		// 计算目标角度
	// 		target_base_angle = base_x(ts_dis, -0.2 + theta);
	// 		target_arm_angle = base_y(ts_dis, radius * sin(30 * 3.1415926f * theta));

	// 		// 计算每步的增量
	// 		step_base = (target_base_angle - last_base_angle) / 100.0f;
	// 		step_arm = (target_arm_angle - last_arm_angle) / 100.0f;

	// 		// 更新上一次的角度
	// 		last_base_angle = target_base_angle;
	// 		last_arm_angle = target_arm_angle;

	// 		// 增加角度参数
	// 		theta += theta_step;
	// 		mode++;
	// 	}
	// 	else if (mode >= steps && mode <= steps + 100)
	// 	{
	// 		mode++;
	// 		// TIM4->CCR4 = 1000;
	// 		// TIM4->CCR3 = 1100;
	// 		Servo_Set_PWM(SERVO_0X, 1100);
	// 		Servo_Set_PWM(SERVO_0Y, 1000);
	// 		return;
	// 	}
	// 	else
	// 	{
	// 		// 一圈完成后重置或进入其他状态
	// 		mode = 0;
	// 		theta = 0.0f; // 重置角度以重复绘制圆形
	// 	}

	// 	// 计算实际角度（插值）
	// 	real_arm_angle = last_arm_angle + step_arm;
	// 	real_base_angle = last_base_angle + step_base;
	// }

	// // 转换为 PWM 信号
	// int pwmA = angle_to_pwm_180(real_arm_angle);
	// int pwmB = angle_to_pwm_270(real_base_angle);

	// // 输出到舵机
	// // TIM4->CCR3 = pwmB;
	// // TIM4->CCR4 = pwmA;
	// Servo_Set_PWM(SERVO_0X, pwmB);
	// Servo_Set_PWM(SERVO_0Y, pwmA);
	// /////////////////////////////// 正弦波 /////////////////////////////////

	// // 采集电池电压
	// Voltage_All += Get_battery_volt();
	// if (++Voltage_Count == 100)
	// 	Voltage = (float)Voltage_All / 10000.0f, Voltage_All = 0, Voltage_Count = 0, LED = !LED;
}
