#ifndef __GYRO_EXAMPLE_H
#define __GYRO_EXAMPLE_H

#include "Gyro_APP.h"

/** ***************************************************************************
 * @brief 陀螺仪使用示例初始化
 */
void Gyro_Example_Init(void);

/** ***************************************************************************
 * @brief 陀螺仪数据处理示例（在主循环中调用）
 */
void Gyro_Example_Process(void);

/** ***************************************************************************
 * @brief 打印陀螺仪数据
 */
void Print_Gyro_Data(void);

/** ***************************************************************************
 * @brief 获取当前姿态角度（简化接口）
 * @param roll 横滚角指针（度）
 * @param pitch 俯仰角指针（度）
 * @param yaw 偏航角指针（度）
 * @return 获取结果
 */
gyro_app_status_t Get_Current_Attitude(float *roll, float *pitch, float *yaw);

/** ***************************************************************************
 * @brief 检查陀螺仪是否正常工作
 * @return 1-正常，0-异常
 */
uint8_t Is_Gyro_Working(void);

/** ***************************************************************************
 * @brief 陀螺仪自检
 * @return 自检结果
 */
gyro_app_status_t Gyro_Self_Test(void);

/** ***************************************************************************
 * @brief 演示结构体方式获取陀螺仪数据的示例
 */
void Demo_Struct_Data_Access(void);

/** ***************************************************************************
 * @brief 演示一次性获取所有数据的示例（推荐使用）
 */
void Demo_All_Data_Access(void);

#endif /* __GYRO_EXAMPLE_H */
