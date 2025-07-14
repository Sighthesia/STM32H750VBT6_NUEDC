/** ***************************************************************************
 * @File_Name: control.c
 * @brief 舵机控制模块 - 轨迹跟踪和目标跟踪控制
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-14
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author: Sighthesia
 * @Changes: 移植到新的HAL架构，添加舵机控制接口
 * @Modifi_Date: 2025-07-14
 */

#include "control.h"
#include "kalman.h"
#include "bsp_usart.h"
#include "pid.h"
#include "Servo_PWM.h" // 添加舵机控制头文件

/* --------------------------------- 此处修改全局变量定义 --------------------------------- */
uint8_t mode_flag = 0;
uint16_t rec_step_point[4][2] = {0}; // 轨迹跟踪模式的4个目标点坐标
uint16_t rec_step_p[1][2] = {0};     // 实时跟踪模式的目标点坐标
uint8_t step_flag = 0;

/* --------------------------------- 此处修改私有函数定义 --------------------------------- */

/** ***************************************************************************
 * @brief 计算绝对值
 * @param x: 输入值
 * @return int: 绝对值
 */
int Control_Abs(int x)
{
    if (x < 0)
        return -x;
    return x;
}

/** ***************************************************************************
 * @brief 设置PID控制器目标值
 * @param x: X轴目标值
 * @param y: Y轴目标值
 */
void Control_Set_Target(float x, float y)
{
    x_pid.Target = x;
    y_pid.Target = y;
}

/** ***************************************************************************
 * @brief 将PID输出转换为舵机角度并控制舵机
 * @param x_pid_output: X轴PID输出
 * @param y_pid_output: Y轴PID输出
 */
static void Control_Update_Servo_Position(float x_pid_output, float y_pid_output)
{
    // 将PID输出映射到舵机角度范围 (0-180度)
    // 这里需要根据实际的映射关系进行调整
    float x_angle = 90.0f + (x_pid_output * 0.1f); // 中心位置90度，PID输出乘以比例系数
    float y_angle = 90.0f + (y_pid_output * 0.1f);

    // 限制角度范围
    if (x_angle < 0)
        x_angle = 0;
    if (x_angle > 180)
        x_angle = 180;
    if (y_angle < 0)
        y_angle = 0;
    if (y_angle > 180)
        y_angle = 180;

    // 控制舵机到指定角度
    Servo_Set_Angle_By_ID(SERVO_ID_J0, (uint8_t)x_angle); // X轴舵机
    Servo_Set_Angle_By_ID(SERVO_ID_J1, (uint8_t)y_angle); // Y轴舵机
}

/* --------------------------------- 此处修改公共函数实现 --------------------------------- */

/** ***************************************************************************
 * @brief 模式1 - 轨迹跟踪模式
 * @note 按照预设的4个点进行循环运动
 */
void Control_Mode_1(void)
{
    static uint8_t step_flag = 0;

    if (step_flag == 0)
    {
        // 启用串口接收坐标数据
        // USART_Cmd(USART3, ENABLE);  // 使用HAL库时不需要此函数
        Control_Get_Point1_P();
    }

    if (mode_flag == 4 && step_flag == 0)
    {
        // 禁用串口接收
        // USART_Cmd(USART3, DISABLE);  // 使用HAL库时不需要此函数

        // 启用定时器中断和PID计算
        // TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  // 已由HAL库处理
        // TIM_Cmd(TIM4, ENABLE);  // 已由HAL库处理

        // 执行轨迹跟踪
        Control_Go_To_Point(rec_step_point[0][0], rec_step_point[0][1],
                            rec_step_point[1][0], rec_step_point[1][1]);
        Control_Go_To_Point(rec_step_point[1][0], rec_step_point[1][1],
                            rec_step_point[2][0], rec_step_point[2][1]);
        Control_Go_To_Point(rec_step_point[2][0], rec_step_point[2][1],
                            rec_step_point[3][0], rec_step_point[3][1]);
        Control_Go_To_Point(rec_step_point[3][0], rec_step_point[3][1],
                            rec_step_point[0][0], rec_step_point[0][1]);

        // 重复执行轨迹
        Control_Go_To_Point(rec_step_point[0][0], rec_step_point[0][1],
                            rec_step_point[1][0], rec_step_point[1][1]);
        Control_Go_To_Point(rec_step_point[1][0], rec_step_point[1][1],
                            rec_step_point[2][0], rec_step_point[2][1]);
        Control_Go_To_Point(rec_step_point[2][0], rec_step_point[2][1],
                            rec_step_point[3][0], rec_step_point[3][1]);
        Control_Go_To_Point(rec_step_point[3][0], rec_step_point[3][1],
                            rec_step_point[0][0], rec_step_point[0][1]);
        step_flag += 1;
    }
}

/** ***************************************************************************
 * @brief 模式2 - 实时跟踪模式
 * @note 根据串口接收的坐标进行实时跟踪，带声光提示
 */
void Control_Mode_2(void)
{
    int a_err = (int)x_pid.Err;
    int b_err = (int)y_pid.Err;

    Control_Get_Point2_P();

    // 误差小于阈值时进行声光提示
    if (Control_Abs(a_err) + Control_Abs(b_err) < 800)
    {
        // 声光提示 - 目标接近
        // BEEP_ON;  // 如果有蜂鸣器可以启用
    }
    else
    {
        // BEEP_OFF;  // 如果有蜂鸣器可以启用
    }

    // 设置新的目标位置
    Control_Set_Target(rec_step_p[0][0], rec_step_p[0][1]);

    // 更新舵机位置
    Control_Update_Servo_Position(x_pid.PID_out, y_pid.PID_out);

    // 调试输出 (可选)
    // printf("a_Err:%d,b_Err:%d \r\n", a_err, b_err);
}

/** ***************************************************************************
 * @brief 从一个点移动到另一个点 (插值移动)
 * @param now_x: 当前X坐标
 * @param now_y: 当前Y坐标
 * @param targ_x: 目标X坐标
 * @param targ_y: 目标Y坐标
 * @note 使用卡尔曼滤波进行平滑插值移动
 */
void Control_Go_To_Point(uint16_t now_x, uint16_t now_y, uint16_t targ_x, uint16_t targ_y)
{
    uint16_t i;
    float step_num = 1000;                  // 1000步插值
    float step_x, step_y;                   // 每步的增量
    float going_x = now_x, going_y = now_y; // 当前插值位置

    // 计算每步的增量
    step_x = (targ_x - now_x) / step_num;
    step_y = (targ_y - now_y) / step_num;

    for (i = 0; i < step_num; i++)
    {
        // 使用卡尔曼滤波进行平滑
        kfp_x.source = step_x;
        kfp_y.source = step_y;
        kfp_x.out = KalmanFilter(&kfp_x, kfp_x.source);
        kfp_y.out = KalmanFilter(&kfp_y, kfp_y.source);

        // 更新当前位置
        going_x += kfp_x.out;
        going_y += kfp_y.out;

        // 设置新的目标并更新舵机
        Control_Set_Target(going_x, going_y);
        Control_Update_Servo_Position(x_pid.PID_out, y_pid.PID_out);

        // 适当延时确保运动平滑
        HAL_Delay(1); // 1ms延时
    }
}

/** ***************************************************************************
 * @brief 解析串口数据获取实时跟踪目标坐标
 * @return uint8_t: 解析状态 (0-成功, 其他-失败)
 * @note 用于模式2的实时跟踪
 */
uint8_t Control_Get_Point2_P(void)
{
    uint8_t label, load_1, load_2, load_3, ret = 0;

    if (DataDecode1(&Uart3_RingBuff, &label, &load_1, &load_2, &load_3) == 0)
    {
        if (label == 0x00)
        {
            rec_step_p[0][0] = load_1 + load_2 + load_3; // X坐标
            // ret = 1;
            // printf("x:%2x\r\n", rec_step_p[0][0]);
        }
        else if (label == 0xff)
        {
            rec_step_p[0][1] = load_1 + load_2 + load_3; // Y坐标
            // ret = 1;
            // printf("y:%2x\r\n", rec_step_p[0][1]);
        }
    }
    return ret;
}

/** ***************************************************************************
 * @brief 解析串口数据获取轨迹跟踪的4个目标点坐标
 * @return uint8_t: 解析状态 (0-成功, 其他-失败)
 * @note 用于模式1的轨迹跟踪，按顺序接收4个点的坐标
 */
uint8_t Control_Get_Point1_P(void)
{
    uint8_t label, load_1, load_2, load_3, ret = 0;

    if (DataDecode1(&Uart3_RingBuff, &label, &load_1, &load_2, &load_3) == 0)
    {
        switch (label)
        {
        case 0x10: // 第4个点X坐标
            rec_step_point[3][0] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[3][0]: %d\r\n", rec_step_point[3][0]);
            break;
        case 0x11: // 第4个点Y坐标
            rec_step_point[3][1] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[3][1]: %d\r\n", rec_step_point[3][1]);
            break;
        case 0x12: // 第1个点X坐标
            rec_step_point[0][0] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[0][0]: %d\r\n", rec_step_point[0][0]);
            break;
        case 0x13: // 第1个点Y坐标
            rec_step_point[0][1] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[0][1]: %d\r\n", rec_step_point[0][1]);
            break;
        case 0x14: // 第2个点X坐标
            rec_step_point[1][0] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[1][0]: %d\r\n", rec_step_point[1][0]);
            break;
        case 0x15: // 第2个点Y坐标
            rec_step_point[1][1] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[1][1]: %d\r\n", rec_step_point[1][1]);
            break;
        case 0x16: // 第3个点X坐标
            rec_step_point[2][0] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[2][0]: %d\r\n", rec_step_point[2][0]);
            break;
        case 0x17: // 第3个点Y坐标
            rec_step_point[2][1] = (load_1 + load_2 + load_3);
            // printf("rec_step_point[2][1]: %d\r\n", rec_step_point[2][1]);
            mode_flag = 4; // 所有点接收完成，启动轨迹跟踪
            break;
        default:
            break;
        }
    }
    return ret;
}

/** ***************************************************************************
 * @brief 舵机控制系统初始化
 * @note 初始化舵机、PID控制器等
 */
void Control_System_Init(void)
{
    // 初始化舵机系统
    Servo_TIM_PWM_Init(); // 初始化舵机PWM

    // 设置PID参数 (需要根据实际系统调节)
    set_pid_param(&x_pid, 1.0f, 0.1f, 0.05f); // X轴PID参数
    set_pid_param(&y_pid, 1.0f, 0.1f, 0.05f); // Y轴PID参数

    // 设置舵机到中心位置
    Servo_Set_Angle_By_ID(SERVO_ID_J0, 90); // X轴舵机到中心
    Servo_Set_Angle_By_ID(SERVO_ID_J1, 90); // Y轴舵机到中心

    // 延时确保舵机到位
    HAL_Delay(1000);
}