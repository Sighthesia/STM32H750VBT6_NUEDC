#include "pid.h"
#include "kalman.h"
#include "control.h"

PID_TypeDef x_pid, y_pid;

/**
 * @brief       PID参数初始化
 * @param       pid: PID结构体指针
 * @retval      无
 */
void pid_param_init(PID_TypeDef *pid)
{
	pid->Target = 0; // 目标值
	pid->PID_out = 0;
	pid->Kp = 0;
	pid->Ki = 0;
	pid->Kd = 0;
	pid->Err = 0.0f;
	pid->LastErr = 0.0f;
	pid->PenultErr = 0.0f;
	pid->Integral = 0.0f; // 积分项

	pid->KP_polarity = 1;
	pid->KI_polarity = 1;
	pid->KD_polarity = 1;
}

/**
 * @brief       PID位置式计算
 * @param       PID: PID结构体指针
 * @param       CurrentValue: 当前值
 * @retval      PID输出
 */
float pid_calculate(PID_TypeDef *PID, float CurrentValue)
{
	PID->Err = PID->Target - CurrentValue;
	PID->Integral += PID->Err;
	/*	// 积分分离（可选）
		if( (PID->Err > 36) || (PID->Err < -36) ){
			PID->Integral = 0;
			//PID->PID_out += PID->IntegralConstant * PID->Integral10
		}
	*/
	/* 积分限幅 */
	if (PID->Integral > 7000)
	{
		PID->Integral = 7000;
	}
	if (PID->Integral < -7000)
	{
		PID->Integral = -7000;
	}
	PID->PID_out = PID->Kp * PID->Err					  // 比例项
				   + PID->Ki * PID->Integral			  // 积分项
				   + PID->Kd * (PID->Err - PID->LastErr); // 微分项

	PID->LastErr = PID->Err;
	return PID->PID_out;
}

/**
 * @brief       PID增量式计算
 * @param       PID: PID结构体指针
 * @param       CurrentValue: 当前值
 * @retval      PID输出
 */
float pid_calculate_inc(PID_TypeDef *PID, float CurrentValue)
{
	float increment_val;
	PID->Err = PID->Target - CurrentValue;

	increment_val = PID->Kp * (PID->Err - PID->LastErr)							// 比例项
					+ PID->Ki * PID->Err										// 积分项
					+ PID->Kd * (PID->Err - 2 * PID->LastErr + PID->PenultErr); // 微分项

	PID->PID_out += increment_val;

	PID->PenultErr = PID->LastErr;
	PID->LastErr = PID->Err;
	return PID->PID_out;
}

/**
 * @brief  获取PID目标值
 * @param  pid: PID结构体指针
 * @retval pid->Target
 */
float get_pid_target(PID_TypeDef *pid)
{
	return pid->Target; // 返回目标值
}

/**
 * @brief  设置PID目标值
 * @param  pid: PID结构体指针
 * @param  target: 目标值
 * @retval 无
 */
void set_pid_target(PID_TypeDef *pid, float target)
{
	pid->Target = target;
}

/**
 * @brief  设置PID参数
 * @param  pid: PID结构体指针
 * @param  p: 比例系数P
 * @param  i: 积分系数I
 * @param  d: 微分系数D
 * @retval 无
 */
void set_pid_param(PID_TypeDef *pid, float p, float i, float d)
{
	pid->Kp = p * (pid->KP_polarity); // 设置比例系数P
	pid->Ki = i * (pid->KI_polarity); // 设置积分系数I
	pid->Kd = d * (pid->KD_polarity); // 设置微分系数D
}

/**
 * @brief  设置PID极性
 * @param  pid: PID结构体指针
 * @param  p_polarity: P极性
 * @param  i_polarity: I极性
 * @param  d_polarity: D极性
 * @retval 无
 */
void set_pid_polarity(PID_TypeDef *pid, int8_t p_polarity, int8_t i_polarity, int8_t d_polarity)
{
	pid->KP_polarity = p_polarity;
	pid->KI_polarity = i_polarity;
	pid->KD_polarity = d_polarity;
}
