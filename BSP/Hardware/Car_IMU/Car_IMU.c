#include "Car_IMU.h"

#include "JY61P.h"
#include "WheelTec.h"
#include "main.h"

#define CAR_IMU_GRAVITY_MPS2       9.80665f
#define CAR_IMU_JY61P_ACCEL_SCALE  (16.0f * CAR_IMU_GRAVITY_MPS2 / 32768.0f)
#define CAR_IMU_JY61P_GYRO_SCALE   (34.906585f / 32768.0f)
#define CAR_IMU_JY61P_ANGLE_SCALE  (3.14159265358979323846f / 32768.0f)
#define CAR_IMU_JY61P_QUAT_SCALE   (1.0f / 32768.0f)

static car_imu_source_t active_source;

void Car_IMU_Init(void)
{
    active_source = CAR_IMU_SOURCE_JY61P;
}

bool Car_IMU_Set_Source(car_imu_source_t source)
{
    if (source != CAR_IMU_SOURCE_JY61P && source != CAR_IMU_SOURCE_WHEELTEC)
    {
        return false;
    }
    active_source = source;
    return true;
}

car_imu_source_t Car_IMU_Get_Source(void)
{
    return active_source;
}

static bool Car_IMU_Get_JY61P(car_imu_sample_t *sample)
{
    const jy61p_status_t *status = JY61P_Get_Status();
    const jy61p_sample_t *raw;

    if (status->available == 0U || status->sample_valid == 0U)
    {
        return false;
    }
    raw = JY61P_Get_Sample();
    sample->acceleration_mps2.x = (float)raw->acceleration.x * CAR_IMU_JY61P_ACCEL_SCALE;
    sample->acceleration_mps2.y = (float)raw->acceleration.y * CAR_IMU_JY61P_ACCEL_SCALE;
    sample->acceleration_mps2.z = (float)raw->acceleration.z * CAR_IMU_JY61P_ACCEL_SCALE;
    sample->angular_velocity_rad_s.x = (float)raw->angular_velocity.x * CAR_IMU_JY61P_GYRO_SCALE;
    sample->angular_velocity_rad_s.y = (float)raw->angular_velocity.y * CAR_IMU_JY61P_GYRO_SCALE;
    sample->angular_velocity_rad_s.z = (float)raw->angular_velocity.z * CAR_IMU_JY61P_GYRO_SCALE;
    sample->magnetic_field_mg.x = (float)raw->magnetic_field.x;
    sample->magnetic_field_mg.y = (float)raw->magnetic_field.y;
    sample->magnetic_field_mg.z = (float)raw->magnetic_field.z;
    sample->angle_rad.x = (float)raw->angle.x * CAR_IMU_JY61P_ANGLE_SCALE;
    sample->angle_rad.y = (float)raw->angle.y * CAR_IMU_JY61P_ANGLE_SCALE;
    sample->angle_rad.z = (float)raw->angle.z * CAR_IMU_JY61P_ANGLE_SCALE;
    sample->quaternion_w = (float)raw->quaternion.w * CAR_IMU_JY61P_QUAT_SCALE;
    sample->quaternion_x = (float)raw->quaternion.x * CAR_IMU_JY61P_QUAT_SCALE;
    sample->quaternion_y = (float)raw->quaternion.y * CAR_IMU_JY61P_QUAT_SCALE;
    sample->quaternion_z = (float)raw->quaternion.z * CAR_IMU_JY61P_QUAT_SCALE;
    sample->temperature_c = (float)raw->temperature * 0.01f;
    sample->pressure_pa = 0.0f;
    sample->timestamp_us = (uint64_t)HAL_GetTick() * 1000U;
    sample->sequence = raw->sequence & 0x7FFFFFFFU;
    sample->source = CAR_IMU_SOURCE_JY61P;
    return true;
}

static bool Car_IMU_Get_WheelTec(car_imu_sample_t *sample)
{
    wheeltec_status_t status;
    wheeltec_imu_data_t imu;
    wheeltec_ahrs_data_t ahrs;

    WheelTec_Get_Status(&status);
    if (status.online == 0U || !WheelTec_Get_IMU(&imu) || !WheelTec_Get_AHRS(&ahrs))
    {
        return false;
    }
    sample->acceleration_mps2.x = imu.acceleration.x;
    sample->acceleration_mps2.y = imu.acceleration.y;
    sample->acceleration_mps2.z = imu.acceleration.z;
    sample->angular_velocity_rad_s.x = ahrs.angular_velocity.x;
    sample->angular_velocity_rad_s.y = ahrs.angular_velocity.y;
    sample->angular_velocity_rad_s.z = ahrs.angular_velocity.z;
    sample->magnetic_field_mg.x = imu.magnetic_field.x;
    sample->magnetic_field_mg.y = imu.magnetic_field.y;
    sample->magnetic_field_mg.z = imu.magnetic_field.z;
    sample->angle_rad.x = ahrs.angle.x;
    sample->angle_rad.y = ahrs.angle.y;
    sample->angle_rad.z = ahrs.angle.z;
    sample->quaternion_w = ahrs.quaternion_w;
    sample->quaternion_x = ahrs.quaternion_x;
    sample->quaternion_y = ahrs.quaternion_y;
    sample->quaternion_z = ahrs.quaternion_z;
    sample->temperature_c = imu.imu_temperature;
    sample->pressure_pa = imu.pressure;
    sample->timestamp_us = ahrs.timestamp_us;
    sample->sequence = (ahrs.sequence & 0x7FFFFFFFU) | 0x80000000U;
    sample->source = CAR_IMU_SOURCE_WHEELTEC;
    return true;
}

bool Car_IMU_Get_Sample(car_imu_sample_t *sample)
{
    if (sample == NULL)
    {
        return false;
    }
    return active_source == CAR_IMU_SOURCE_JY61P ?
           Car_IMU_Get_JY61P(sample) : Car_IMU_Get_WheelTec(sample);
}
