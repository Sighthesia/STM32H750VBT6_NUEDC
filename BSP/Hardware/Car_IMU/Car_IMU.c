#include "Car_IMU.h"

#include "WheelTec.h"

static car_imu_source_t active_source;

void Car_IMU_Init(void)
{
    active_source = CAR_IMU_SOURCE_WHEELTEC;
}

car_imu_source_t Car_IMU_Get_Source(void)
{
    return active_source;
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
    return Car_IMU_Get_WheelTec(sample);
}
