#include "Car_Attitude_Control.h"

#include "Car_Motor_USART.h"
#include "JY61P.h"
#include "PID.h"
#include "Tracking_Link.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define CAR_CONTROL_PERIOD_S             0.01f
#define CAR_CONTROL_IMU_TIMEOUT_MS       50U
#define CAR_CONTROL_GYRO_SCALE_DPS       (2000.0f / 32768.0f)

typedef struct
{
    float z1;
    float z2;
    float previous_output;
    uint8_t initialized;
} car_ladrc_t;

static car_attitude_control_config_t config;
static PIDController heading_pid;
static PIDController rate_pid;
static car_ladrc_t rate_ladrc;
static uint32_t last_sequence;
static uint32_t last_sample_ms;
static uint8_t enabled;
static uint8_t stopped;
static uint8_t heading_pid_initialized;
static uint8_t rate_pid_initialized;

static float clampf(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static void reset_controllers(void)
{
    PIDController_Init(&heading_pid);
    PIDController_Init(&rate_pid);
    memset(&rate_ladrc, 0, sizeof(rate_ladrc));
    heading_pid_initialized = 0U;
    rate_pid_initialized = 0U;
}

static void configure_pid(void)
{
    heading_pid.Kp = config.heading_kp;
    heading_pid.Ki = config.heading_ki;
    heading_pid.Kd = config.heading_kd;
    heading_pid.tau = 0.02f;
    heading_pid.limMin = -config.max_rate;
    heading_pid.limMax = config.max_rate;
    heading_pid.limMinInt = -config.max_rate;
    heading_pid.limMaxInt = config.max_rate;
    heading_pid.T = CAR_CONTROL_PERIOD_S;

    rate_pid.Kp = config.rate_kp;
    rate_pid.Ki = config.rate_ki;
    rate_pid.Kd = config.rate_kd;
    rate_pid.tau = 0.02f;
    rate_pid.limMin = -config.max_differential;
    rate_pid.limMax = config.max_differential;
    rate_pid.limMinInt = -config.max_differential;
    rate_pid.limMaxInt = config.max_differential;
    rate_pid.T = CAR_CONTROL_PERIOD_S;
}

static void stop_output(void)
{
    if (!stopped)
    {
        if (Car_Motor_Set_Speed(0, 0, 0, 0)) stopped = 1U;
    }
}

static float update_ladrc(float target_rate, float measured_rate)
{
    const float error = measured_rate - rate_ladrc.z1;
    const float observer = config.ladrc_observer_bandwidth;
    const float gain = config.ladrc_plant_gain;

    if (!rate_ladrc.initialized)
    {
        rate_ladrc.z1 = measured_rate;
        rate_ladrc.initialized = 1U;
    }

    rate_ladrc.z1 += CAR_CONTROL_PERIOD_S *
                     (rate_ladrc.z2 + gain * rate_ladrc.previous_output + 2.0f * observer * error);
    rate_ladrc.z2 += CAR_CONTROL_PERIOD_S * observer * observer * error;
    rate_ladrc.previous_output =
        (config.ladrc_controller_bandwidth * (target_rate - rate_ladrc.z1) - rate_ladrc.z2) / gain;
    rate_ladrc.previous_output = clampf(rate_ladrc.previous_output,
                                       -config.max_differential, config.max_differential);
    return rate_ladrc.previous_output;
}

void Car_Attitude_Control_Reset_Defaults(void)
{
    config.mode = (float)CAR_ATTITUDE_CONTROL_PID;
    config.base_speed = 150.0f;
    config.heading_kp = 3.0f;
    config.heading_ki = 0.0f;
    config.heading_kd = 0.0f;
    config.rate_kp = 1.2f;
    config.rate_ki = 0.2f;
    config.rate_kd = 0.0f;
    config.ladrc_observer_bandwidth = 20.0f;
    config.ladrc_controller_bandwidth = 8.0f;
    config.ladrc_plant_gain = 1.0f;
    config.max_rate = 180.0f;
    config.max_differential = 300.0f;
    config.invert_x = 0.0f;
    config.invert_y = 0.0f;
    config.invert_z = 0.0f;
    configure_pid();
    JY61P_Set_Axis_Inverted(false, false, false);
    reset_controllers();
}

bool Car_Attitude_Control_Apply_Config(const car_attitude_control_config_t *next)
{
    if (next == NULL || !isfinite(next->mode) || !isfinite(next->base_speed) ||
        !isfinite(next->heading_kp) || !isfinite(next->heading_ki) || !isfinite(next->heading_kd) ||
        !isfinite(next->rate_kp) || !isfinite(next->rate_ki) || !isfinite(next->rate_kd) ||
        !isfinite(next->ladrc_observer_bandwidth) || !isfinite(next->ladrc_controller_bandwidth) ||
        !isfinite(next->ladrc_plant_gain) || !isfinite(next->max_rate) || !isfinite(next->max_differential) ||
        (next->mode != 0.0f && next->mode != 1.0f) || next->base_speed < 0.0f || next->base_speed > 1000.0f ||
        next->heading_kp < 0.0f || next->heading_ki < 0.0f || next->heading_kd < 0.0f ||
        next->rate_kp < 0.0f || next->rate_ki < 0.0f || next->rate_kd < 0.0f ||
        next->ladrc_observer_bandwidth <= 0.0f || next->ladrc_controller_bandwidth <= 0.0f ||
        next->ladrc_plant_gain <= 0.0f || next->max_rate <= 0.0f || next->max_rate > 2000.0f ||
        next->max_differential <= 0.0f || next->max_differential > 1000.0f ||
        (next->invert_x != 0.0f && next->invert_x != 1.0f) ||
        (next->invert_y != 0.0f && next->invert_y != 1.0f) ||
        (next->invert_z != 0.0f && next->invert_z != 1.0f))
    {
        return false;
    }

    config = *next;
    configure_pid();
    JY61P_Set_Axis_Inverted(config.invert_x != 0.0f,
                            config.invert_y != 0.0f,
                            config.invert_z != 0.0f);
    reset_controllers();
    return true;
}

void Car_Attitude_Control_Init(void)
{
    enabled = 0U;
    stopped = 0U;
    last_sequence = 0U;
    last_sample_ms = HAL_GetTick();
    Car_Attitude_Control_Reset_Defaults();
    stop_output();
}

bool Car_Attitude_Control_Set_Enabled(bool state)
{
    enabled = state ? 1U : 0U;
    stopped = 0U;
    reset_controllers();
    if (!enabled) stop_output();
    return true;
}

bool Car_Attitude_Control_Is_Enabled(void)
{
    return enabled != 0U;
}

car_attitude_control_config_t *Car_Attitude_Control_Get_Config(void)
{
    return &config;
}

void Car_Attitude_Control_Process(void)
{
    const jy61p_status_t *imu_status = JY61P_Get_Status();
    uint32_t now = HAL_GetTick();
    int16_t tracking_angle_x10;
    bool new_sample = imu_status->sample_valid && imu_status->sequence != last_sequence;

    if (!enabled)
    {
        stop_output();
        return;
    }
    if (new_sample)
    {
        last_sequence = imu_status->sequence;
        last_sample_ms = now;
    }
    else if ((uint32_t)(now - last_sample_ms) > CAR_CONTROL_IMU_TIMEOUT_MS)
    {
        stop_output();
        reset_controllers();
        return;
    }
    else
    {
        return;
    }
    if (!Tracking_Link_Get_Angle(&tracking_angle_x10))
    {
        stop_output();
        reset_controllers();
        return;
    }

    const jy61p_sample_t *imu = JY61P_Get_Sample();
    const float heading_error = (float)tracking_angle_x10 * 0.1f;
    const float measured_rate = (float)imu->angular_velocity.z * CAR_CONTROL_GYRO_SCALE_DPS;
    if (!heading_pid_initialized)
    {
        heading_pid.prevMeasurement = heading_error;
        heading_pid_initialized = 1U;
    }
    const float target_rate = PIDController_Update(&heading_pid, 0.0f, heading_error);
    if (!rate_pid_initialized)
    {
        rate_pid.prevMeasurement = measured_rate;
        rate_pid_initialized = 1U;
    }
    float differential = config.mode == (float)CAR_ATTITUDE_CONTROL_LADRC ?
                         update_ladrc(target_rate, measured_rate) :
                         PIDController_Update(&rate_pid, target_rate, measured_rate);
    differential = clampf(differential, -config.max_differential, config.max_differential);

    const int16_t left = (int16_t)clampf(config.base_speed - differential, -1000.0f, 1000.0f);
    const int16_t right = (int16_t)clampf(config.base_speed + differential, -1000.0f, 1000.0f);
    if (Car_Motor_Set_Speed(left, right, 0, 0)) stopped = 0U;
}
