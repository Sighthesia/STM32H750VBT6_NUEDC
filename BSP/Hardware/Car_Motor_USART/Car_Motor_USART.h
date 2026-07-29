#ifndef CAR_MOTOR_USART_H
#define CAR_MOTOR_USART_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAR_MOTOR_TYPE_520 = 1,
    CAR_MOTOR_TYPE_310,
    CAR_MOTOR_TYPE_TT_ENCODER,
    CAR_MOTOR_TYPE_TT
} car_motor_type_t;

typedef struct
{
    int32_t total_encoder[4];
    int32_t encoder_delta_10ms[4];
    float speed[4];
} car_motor_telemetry_t;

void Car_Motor_Init(void);
void Car_Motor_Process(void);
void Car_Motor_UART_RxCplt_Callback(void);
void Car_Motor_UART_TxCplt_Callback(void);
void Car_Motor_UART_Error_Callback(void);

bool Car_Motor_Set_Type(car_motor_type_t type);
bool Car_Motor_Set_Deadzone(uint16_t deadzone);
bool Car_Motor_Set_Line(uint16_t line);
bool Car_Motor_Set_Wheel_Diameter(float diameter_mm);
bool Car_Motor_Set_PID(float kp, float ki, float kd);
bool Car_Motor_Factory_Reset(void);
bool Car_Motor_Set_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool Car_Motor_Set_PWM(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool Car_Motor_Set_Upload(bool total_encoder, bool encoder_delta_10ms, bool speed);
bool Car_Motor_Read_Flash(void);
bool Car_Motor_Read_Voltage(void);

const car_motor_telemetry_t *Car_Motor_Get_Telemetry(void);

#endif /* CAR_MOTOR_USART_H */
