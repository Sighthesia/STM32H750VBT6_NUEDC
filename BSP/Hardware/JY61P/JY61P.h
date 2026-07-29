#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} jy61p_raw_vector_t;

typedef struct
{
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} jy61p_raw_quaternion_t;

typedef struct
{
    jy61p_raw_vector_t acceleration;
    jy61p_raw_vector_t angular_velocity;
    jy61p_raw_vector_t magnetic_field;
    jy61p_raw_vector_t angle;
    int16_t temperature;
    jy61p_raw_quaternion_t quaternion;
    uint32_t sequence;
} jy61p_sample_t;

typedef struct
{
    uint8_t available;
    uint8_t sample_valid;
    uint16_t version;
    uint32_t sequence;
    uint32_t consecutive_failures;
    uint32_t read_error_count;
} jy61p_status_t;

void JY61P_Init(void);
void JY61P_Process(void);
void JY61P_Set_Axis_Inverted(bool invert_x, bool invert_y, bool invert_z);
const jy61p_sample_t *JY61P_Get_Sample(void);
const jy61p_status_t *JY61P_Get_Status(void);

#endif /* JY61P_H */
