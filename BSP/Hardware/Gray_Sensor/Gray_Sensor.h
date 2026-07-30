#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdint.h>

static inline uint8_t Gray_Sensor_Normalize(uint8_t raw_value, uint8_t active_low)
{
    return active_low != 0U ? ((uint8_t)~raw_value & 0x0FU) : (raw_value & 0x0FU);
}

void Gray_Sensor_Init(void);
void Gray_Sensor_Process(void);
uint8_t Gray_Sensor_Read(void);
uint8_t Gray_Sensor_Get_Stable(void);
void Gray_Sensor_Set_Active_Low(uint8_t active_low);
uint8_t Gray_Sensor_Get_Active_Low(void);

#endif /* GRAY_SENSOR_H */
