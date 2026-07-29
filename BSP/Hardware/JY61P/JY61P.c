#include "JY61P.h"

#include "i2c.h"

#include <limits.h>
#include <string.h>

#define JY61P_I2C_ADDRESS          (0x50U << 1U)
#define JY61P_REGISTER_AX          0x34U
#define JY61P_REGISTER_VERSION     0x2EU
#define JY61P_REGISTER_QUATERNION  0x51U
#define JY61P_SAMPLE_PERIOD_MS     10U
#define JY61P_I2C_TIMEOUT_MS       5U

static jy61p_sample_t sample;
static jy61p_status_t status;
static uint32_t last_poll_ms;

static int16_t decode_i16(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

static jy61p_raw_vector_t decode_vector(const uint8_t *data)
{
    jy61p_raw_vector_t vector = {
        .x = decode_i16(&data[0]),
        .y = decode_i16(&data[2]),
        .z = decode_i16(&data[4])
    };
    return vector;
}

static uint8_t read_registers(uint8_t start, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c1, JY61P_I2C_ADDRESS, start,
                            I2C_MEMADD_SIZE_8BIT, data, length,
                            JY61P_I2C_TIMEOUT_MS) == HAL_OK;
}

static uint8_t read_sample(jy61p_sample_t *next)
{
    uint8_t sensor_data[26];
    uint8_t quaternion_data[8];

    if (!read_registers(JY61P_REGISTER_AX, sensor_data, sizeof(sensor_data)) ||
        !read_registers(JY61P_REGISTER_QUATERNION, quaternion_data, sizeof(quaternion_data)))
    {
        return 0U;
    }

    next->acceleration = decode_vector(&sensor_data[0]);
    next->angular_velocity = decode_vector(&sensor_data[6]);
    next->magnetic_field = decode_vector(&sensor_data[12]);
    next->angle = decode_vector(&sensor_data[18]);
    next->temperature = decode_i16(&sensor_data[24]);
    next->quaternion.w = decode_i16(&quaternion_data[0]);
    next->quaternion.x = decode_i16(&quaternion_data[2]);
    next->quaternion.y = decode_i16(&quaternion_data[4]);
    next->quaternion.z = decode_i16(&quaternion_data[6]);
    return 1U;
}

void JY61P_Init(void)
{
    uint8_t version_data[2];

    memset(&sample, 0, sizeof(sample));
    memset(&status, 0, sizeof(status));
    last_poll_ms = HAL_GetTick() - JY61P_SAMPLE_PERIOD_MS;
    if (read_registers(JY61P_REGISTER_VERSION, version_data, sizeof(version_data)))
    {
        status.version = (uint16_t)version_data[0] | ((uint16_t)version_data[1] << 8U);
        status.available = 1U;
    }
}

void JY61P_Process(void)
{
    jy61p_sample_t next;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_poll_ms) < JY61P_SAMPLE_PERIOD_MS)
    {
        return;
    }
    last_poll_ms = now;

    if (!read_sample(&next))
    {
        status.available = 0U;
        status.read_error_count++;
        if (status.consecutive_failures != UINT32_MAX)
        {
            status.consecutive_failures++;
        }
        return;
    }

    next.sequence = status.sequence + 1U;
    sample = next;
    status.available = 1U;
    status.sample_valid = 1U;
    status.sequence = next.sequence;
    status.consecutive_failures = 0U;
}

const jy61p_sample_t *JY61P_Get_Sample(void)
{
    return &sample;
}

const jy61p_status_t *JY61P_Get_Status(void)
{
    return &status;
}
