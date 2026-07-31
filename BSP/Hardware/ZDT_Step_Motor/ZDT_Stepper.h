#ifndef _ZDT_STEPPER_H_
#define _ZDT_STEPPER_H_

#include "ZDT_Stepper_Config.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "fdcan.h"
#include "stdint.h"
#include "stdbool.h"

#define ZDT_STEPPER_MOTOR_CONFIG_NUM (sizeof(zdt_stepper_motor_config_table) / sizeof(zdt_stepper_motor_config_table[0]))

typedef enum
{
    ZDT_STEPPER_COMMAND_NONE = 0U,
    ZDT_STEPPER_COMMAND_QUEUED,
    ZDT_STEPPER_COMMAND_CAN_SENT,
    ZDT_STEPPER_COMMAND_ACK_OK,
    ZDT_STEPPER_COMMAND_ACK_ERROR,
    ZDT_STEPPER_COMMAND_TIMEOUT,
    ZDT_STEPPER_COMMAND_RX_INVALID,
    ZDT_STEPPER_COMMAND_RX_UNMATCHED
} zdt_stepper_command_status_t;

typedef struct
{
    uint16_t firmware_version;         // 固件版本
    uint16_t hardware_version;         // 硬件版本
    uint16_t phase_resistance;         // 相电阻 (mΩ)
    uint16_t phase_inductance;         // 相电感 (µH)
    uint16_t bus_voltage;              // 总线电压 (mV)
    uint16_t bus_average_current;      // 总线平均电流 (mA)
    uint16_t phase_current;            // 相电流 (A)
    uint16_t encoder_raw_value;        // 编码器原始值
    uint16_t encoder_calibrated_value; // 经过线性化校准后的编码器值
    float target_position;             // 电机目标位置 (°)
    float current_speed;               // 电机实时转速 (RPM)
    float current_position;            // 电机实时位置 (°)
    float position_error;              // 电机位置误差 (°)
    int8_t driver_temperature;         // 驱动器实时温度 (°C)
    uint8_t motor_status_flags;        // 电机状态标志位
    uint8_t motor_enabled;             // 电机使能状态
    uint8_t motor_position_reached;    // 电机位置到位标志位
    uint8_t motor_stall;               // 电机堵转标志位
    uint8_t motor_stall_protection;    // 电机编码器校准标志位
    uint8_t last_command;
    uint8_t last_command_status;
    uint8_t command_lifecycle;
    uint8_t pending_command;
    uint8_t microstepping;
    uint8_t homing_status_flags;
    uint32_t last_command_ms;
    uint32_t command_sent_ms;
    uint32_t command_timeout_count;
    uint32_t rx_invalid_count;
    uint32_t rx_unmatched_count;
    uint32_t tx_dropped_count;
    uint32_t tx_error_count;
    uint32_t current_position_ms;
    uint32_t current_speed_ms;
    uint32_t motor_status_ms;
    uint32_t homing_status_ms;
    float limit_min;
    float limit_max;
    uint8_t limits_enabled;
} ZDTStepperData;

// 电机信息结构体
typedef struct
{
    uint8_t id;                 // 电机ID
    FDCAN_HandleTypeDef *hfdcan;
    ZDTStepperData data;
} ZDTStepperMotor;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    ZDTStepperMotor motors[ZDT_STEPPER_PER_CAN_MAX_MOTORS];
    uint8_t motor_count;
} ZDTStepperCan;

bool ZDT_Stepper_Init(void);
void ZDT_Stepper_Register_Motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
ZDTStepperMotor *ZDT_Stepper_Get_Motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void ZDT_Stepper_UART_Send_Data(ZDTStepperMotor *motor, uint8_t *data, size_t length);
void ZDT_Stepper_FDCAN_RxFifo0_Callback(FDCAN_HandleTypeDef *hfdcan);
void ZDT_Stepper_FDCAN_Error_Callback(FDCAN_HandleTypeDef *hfdcan);
void ZDT_Stepper_Update_Communication_Status(ZDTStepperMotor *motor);
void ZDT_Stepper_Set_Software_Limits(ZDTStepperMotor *motor, float minimum, float maximum, bool enabled);

void ZDT_Stepper_Enable(ZDTStepperMotor *motor, uint8_t enable, uint8_t sync_flag);
void ZDT_Stepper_Set_Speed(ZDTStepperMotor *motor, uint8_t dir, float speed_f, uint8_t accel_accel, uint8_t sync_flag);
void ZDT_Stepper_Set_position(ZDTStepperMotor *motor, uint8_t dir, float speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag); // 设置步进电机位置
void ZDT_Stepper_Set_position_accel(ZDTStepperMotor *motor, uint8_t dir, float speed_f, uint8_t accel, float position_angle_f, uint8_t position_mode, uint8_t sync_flag);
void Emm_V5_Pos_Control(ZDTStepperMotor *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void ZDT_Stepper_Set_T_position(ZDTStepperMotor *motor, uint8_t dir, float speed_f, uint8_t accel_accel, uint32_t pulse_count, uint8_t position_mode, uint8_t sync_flag);
void ZDT_Stepper_torque_control(ZDTStepperMotor *motor, uint8_t dir, uint16_t current_rate, uint16_t current, uint8_t sync_flag); // 力矩控制模式
void ZDT_Stepper_start_sync_motion(ZDTStepperMotor *motor);                                                                       // 开启多机同步运动
void ZDT_Stepper_stop(ZDTStepperMotor *motor, uint8_t sync_flag);                                                                 // 立即停止
void ZDT_Stepper_clear_current_position(ZDTStepperMotor *motor);                                                                  // 角度清零
void ZDT_Stepper_trigger_zero_return(ZDTStepperMotor *motor, uint8_t return_mode, uint8_t sync_flag);                             // 触发回零
void ZDT_Stepper_abort_zero_return(ZDTStepperMotor *motor);

void ZDT_Stepper_release_stall_protection(ZDTStepperMotor *motor);    // 解除堵转保护
void ZDT_Stepper_trigger_encoder_calibration(ZDTStepperMotor *motor); // 触发编码器校准
void ZDT_Stepper_set_microstepping(ZDTStepperMotor *motor, uint8_t store_flag, uint8_t microstepping_value);

void ZDT_Stepper_Read_version(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_resistance_and_inductance(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_bus_voltage(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_bus_average_current(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_phase_current(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_encoder_raw_value(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_encoder_calibrated_value(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_target_position(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_current_speed(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_current_position(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_position_error(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_driver_temperature(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_motor_status_flags(ZDTStepperMotor *motor);
void ZDT_Stepper_Read_homing_status_flags(ZDTStepperMotor *motor);

extern ZDTStepperMotor *zdt_stepper_motors[];

#endif // _ZDT_STEPPER_H_
