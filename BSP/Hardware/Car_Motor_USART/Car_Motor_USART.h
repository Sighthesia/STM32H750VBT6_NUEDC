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

typedef enum
{
    CAR_MOTOR_COMMAND_NONE,
    CAR_MOTOR_COMMAND_QUEUED,
    CAR_MOTOR_COMMAND_TRANSMITTED,
    CAR_MOTOR_COMMAND_DRIVER_FRAME,
    CAR_MOTOR_COMMAND_FAILED
} car_motor_command_status_t;

typedef enum
{
    CAR_MOTOR_DRIVER_FRAME_NONE,
    CAR_MOTOR_DRIVER_FRAME_TELEMETRY,
    CAR_MOTOR_DRIVER_FRAME_VOLTAGE
} car_motor_driver_frame_t;

#define CAR_MOTOR_LAST_COMMAND_LENGTH 23U

typedef struct
{
    int32_t total_encoder[4];
    int32_t encoder_delta_10ms[4];
    float speed[4];
    char last_command[CAR_MOTOR_LAST_COMMAND_LENGTH];
    float voltage;
    uint8_t command_status;
    uint8_t last_driver_frame;
    uint8_t voltage_valid;
    uint8_t encoder_upload_active;
    uint8_t total_encoder_valid;
    uint32_t total_encoder_ms;
    uint32_t rx_byte_count;
    uint32_t rx_frame_count;
    uint32_t rx_unmatched_frame_count;
    uint32_t rx_parse_error_count;
    uint32_t rx_overwrite_count;
    uint32_t uart_error_count;
    int16_t applied_speed[4];
} car_motor_telemetry_t;

typedef struct
{
    car_motor_type_t type;
    uint16_t deadzone;
    uint16_t line;
    uint16_t reduction_ratio;
    float wheel_diameter_mm;
    float pid_kp;
    float pid_ki;
    float pid_kd;
} car_motor_config_t;

void Car_Motor_Init(void);
void Car_Motor_Process(void);
void Car_Motor_UART_RxCplt_Callback(void);
void Car_Motor_UART_TxCplt_Callback(void);
void Car_Motor_UART_Error_Callback(void);

bool Car_Motor_Set_Type(car_motor_type_t type);
bool Car_Motor_Set_Deadzone(uint16_t deadzone);
bool Car_Motor_Set_Line(uint16_t line);
bool Car_Motor_Set_Reduction_Ratio(uint16_t reduction_ratio);
bool Car_Motor_Set_Wheel_Diameter(float diameter_mm);
bool Car_Motor_Set_PID(float kp, float ki, float kd);
bool Car_Motor_Apply_Config(const car_motor_config_t *config);
bool Car_Motor_Factory_Reset(void);
bool Car_Motor_Restore_Defaults(void);
bool Car_Motor_Set_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool Car_Motor_Set_PWM(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool Car_Motor_Set_Direction(uint8_t motor_index, bool reversed);
bool Car_Motor_Set_Upload(bool total_encoder, bool encoder_delta_10ms, bool speed);
bool Car_Motor_Read_Flash(void);
bool Car_Motor_Read_Voltage(void);

const car_motor_telemetry_t *Car_Motor_Get_Telemetry(void);

#endif /* CAR_MOTOR_USART_H */
