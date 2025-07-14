/** ***************************************************************************
 * @File_Name: control.h
 * @brief 舵机控制模块头文件 - 轨迹跟踪和目标跟踪控制
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-14
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author: Sighthesia
 * @Changes: 移植到新的HAL架构，更新函数命名规范
 * @Modifi_Date: 2025-07-14
 */

#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

/* --------------------------------- 此处修改公共函数声明 --------------------------------- */

/** ***************************************************************************
 * @brief 计算绝对值
 * @param p: 输入值
 * @return int: 绝对值
 */
int Control_Abs(int p);

/** ***************************************************************************
 * @brief 模式1 - 轨迹跟踪模式
 */
void Control_Mode_1(void);

/** ***************************************************************************
 * @brief 模式2 - 实时跟踪模式
 */
void Control_Mode_2(void);

/** ***************************************************************************
 * @brief 设置PID控制器目标值
 * @param x: X轴目标值
 * @param y: Y轴目标值
 */
void Control_Set_Target(float x, float y);

/** ***************************************************************************
 * @brief 解析串口数据获取轨迹跟踪的4个目标点坐标
 * @return uint8_t: 解析状态
 */
uint8_t Control_Get_Point1_P(void);

/** ***************************************************************************
 * @brief 解析串口数据获取实时跟踪目标坐标
 * @return uint8_t: 解析状态
 */
uint8_t Control_Get_Point2_P(void);

/** ***************************************************************************
 * @brief 从一个点移动到另一个点 (插值移动)
 * @param now_x: 当前X坐标
 * @param now_y: 当前Y坐标
 * @param targ_x: 目标X坐标
 * @param targ_y: 目标Y坐标
 */
void Control_Go_To_Point(uint16_t now_x, uint16_t now_y, uint16_t targ_x, uint16_t targ_y);

/** ***************************************************************************
 * @brief 写入坐标点到闪存 (保留接口)
 * @param arr: 坐标数组指针
 */
void Control_Write_Point_To_Flash(int16_t *arr);

/** ***************************************************************************
 * @brief 从闪存读取坐标点 (保留接口)
 * @param arr: 坐标数组指针
 */
void Control_Read_Point_From_Flash(int16_t *arr);

void Control_System_Init(void);

#endif /* __CONTROL_H */
