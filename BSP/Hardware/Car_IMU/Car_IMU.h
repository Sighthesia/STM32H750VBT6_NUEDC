#ifndef CAR_IMU_H
#define CAR_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAR_IMU_SOURCE_WHEELTEC = 0
} car_imu_source_t;

typedef struct
{
    float x;
    float y;
    float z;
} car_imu_vector3_t;

typedef struct
{
    car_imu_vector3_t acceleration_mps2;
    car_imu_vector3_t angular_velocity_rad_s;
    car_imu_vector3_t magnetic_field_mg;
    car_imu_vector3_t angle_rad;
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    float temperature_c;
    float pressure_pa;
    uint64_t timestamp_us;
    uint32_t sequence;
    car_imu_source_t source;
} car_imu_sample_t;

void Car_IMU_Init(void);
car_imu_source_t Car_IMU_Get_Source(void);
bool Car_IMU_Get_Sample(car_imu_sample_t *sample);

#endif /* CAR_IMU_H */
