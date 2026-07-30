#ifndef WHEELTEC_H
#define WHEELTEC_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} wheeltec_vector3_t;

typedef struct
{
    wheeltec_vector3_t angular_velocity;
    wheeltec_vector3_t acceleration;
    wheeltec_vector3_t magnetic_field;
    float imu_temperature;
    float pressure;
    float pressure_temperature;
    uint64_t timestamp_us;
    uint32_t sequence;
} wheeltec_imu_data_t;

typedef struct
{
    wheeltec_vector3_t angular_velocity;
    wheeltec_vector3_t angle;
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    uint64_t timestamp_us;
    uint32_t sequence;
} wheeltec_ahrs_data_t;

typedef struct
{
    uint8_t online;
    uint8_t imu_valid;
    uint8_t ahrs_valid;
    uint8_t reserved;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t uart_error_count;
} wheeltec_status_t;

void WheelTec_Init(void);
bool WheelTec_Get_IMU(wheeltec_imu_data_t *data);
bool WheelTec_Get_AHRS(wheeltec_ahrs_data_t *data);
void WheelTec_Get_Status(wheeltec_status_t *status);
void WheelTec_UART_RxCplt_Callback(void);
void WheelTec_UART_Error_Callback(void);

#endif /* WHEELTEC_H */
