#include "control.h"
#include "kalman.h"
#include "bsp_usart.h"
#include "pid.h"
#include "kalman.h"
#include "bsp_servo.h"

uint8_t mode_flag = 0;
uint16_t rec_step_point[4][2] = {0};
uint16_t rec_step_p[1][2] = {0};
uint8_t step_flag = 0;

int abs(int x)
{
	int y;
	if (y < 0)
		y = -y;
	return y;
}

void set_target(float x, float y)
{
	x_pid.Target = x;
	y_pid.Target = y;
}

void mode_1(void)
{

	static uint8_t step_flag = 0;
	if (step_flag == 0)
	{

		get_point1_p();
	}

	if (mode_flag == 4 && step_flag == 0)
	{
		HAL_UART_Abort(&huart3); // HAL库关闭USART3
		HAL_TIM_Base_Start_IT(&SERVO_TIM_HANDLE);
		HAL_TIM_Base_Start_IT(&PID_TIM_HANDLE);

		go_to_point(rec_step_point[0][0], rec_step_point[0][1], rec_step_point[1][0], rec_step_point[1][1]);
		go_to_point(rec_step_point[1][0], rec_step_point[1][1], rec_step_point[2][0], rec_step_point[2][1]);
		go_to_point(rec_step_point[2][0], rec_step_point[2][1], rec_step_point[3][0], rec_step_point[3][1]);
		go_to_point(rec_step_point[3][0], rec_step_point[3][1], rec_step_point[0][0], rec_step_point[0][1]);

		go_to_point(rec_step_point[0][0], rec_step_point[0][1], rec_step_point[1][0], rec_step_point[1][1]);
		go_to_point(rec_step_point[1][0], rec_step_point[1][1], rec_step_point[2][0], rec_step_point[2][1]);
		go_to_point(rec_step_point[2][0], rec_step_point[2][1], rec_step_point[3][0], rec_step_point[3][1]);
		go_to_point(rec_step_point[3][0], rec_step_point[3][1], rec_step_point[0][0], rec_step_point[0][1]);
		step_flag += 1;
	}
}

void mode_2(void)
{

	int a_Err = (int)x_pid.Err, b_Err = (int)y_pid.Err;

	get_point2_p();

	// if (abs(a_Err) + abs(b_Err) < 800)
	// { /* 声光提示 */
	// 	BEEP_ON;
	// }
	// else
	// {
	// 	BEEP_OFF;
	// }
	set_target(rec_step_p[0][0], rec_step_p[0][1]);
	//				printf("a_Err:%d,b_Err:%d \r\n",a_Err,b_Err);
}

void go_to_point(uint16_t now_x, uint16_t now_y, uint16_t targ_x, uint16_t targ_y)
{
	uint16_t i;
	float step_num = 1000;					/* 1000步 */
	float step_x, step_y;					/* 步进值  */
	float going_x = now_x, going_y = now_y; /* 当前目标值*/

	step_x = (targ_x - now_x) / step_num;
	step_y = (targ_y - now_y) / step_num;

	for (i = 0; i < step_num; i++)
	{

		kfp_x.source = step_x;
		kfp_y.source = step_y;
		kfp_x.out = KalmanFilter(&kfp_x, kfp_x.source);
		kfp_y.out = KalmanFilter(&kfp_y, kfp_y.source);

		going_x += kfp_x.out;
		going_y += kfp_y.out;

		set_target(going_x, going_y);
	}
}

uint8_t get_point2_p() // 处理坐标函数
{
	uint8_t lable, load_1, load_2, load_3, ret = 0;
	if (DataDecode1(&Uart3_RingBuff, &lable, &load_1, &load_2, &load_3) == 0)
	{
		if (lable == 0x00)
		{
			rec_step_p[0][0] = load_1 + load_2 + load_3;
			//			ret=1;
			//		 printf("x:%2x\r\n",x);
		}
		else if (lable == 0xff)
		{

			rec_step_p[0][1] = load_1 + load_2 + load_3;
			//			ret=1;
			//		 printf("y:%2x\r\n",y);
		}

		return ret;
	}
}

uint8_t get_point1_p()
{
	uint8_t lable, load_1, load_2, load_3, ret = 0;
	if (DataDecode1(&Uart3_RingBuff, &lable, &load_1, &load_2, &load_3) == 0)
	{

		if (lable == 0x10)
		{
			rec_step_point[3][0] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[3][0]: %d\r\n", rec_step_point[3][0]);
		}
		else if (lable == 0x11)
		{
			rec_step_point[3][1] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[3][1]: %d\r\n", rec_step_point[3][1]);
		}
		else if (lable == 0x12)
		{
			rec_step_point[0][0] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[0][0]: %d\r\n", rec_step_point[0][0]);
		}
		else if (lable == 0x13)
		{
			rec_step_point[0][1] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[0][1]: %d\r\n", rec_step_point[0][1]);
		}
		else if (lable == 0x14)
		{
			rec_step_point[1][0] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[1][0]: %d\r\n", rec_step_point[1][0]);
		}
		else if (lable == 0x15)
		{
			rec_step_point[1][1] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[1][1]: %d\r\n", rec_step_point[1][1]);
		}
		else if (lable == 0x16)
		{
			rec_step_point[2][0] = (load_1 + load_2 + load_3);
			//            printf("rec_step_point[2][0]: %d\r\n", rec_step_point[2][0]);
		}
		else if (lable == 0x17)
		{
			rec_step_point[2][1] = (load_1 + load_2 + load_3);
			//           printf("rec_step_point[2][1]: %d\r\n", rec_step_point[2][1]);
			mode_flag = 4;
		}
	}
	return ret;
}