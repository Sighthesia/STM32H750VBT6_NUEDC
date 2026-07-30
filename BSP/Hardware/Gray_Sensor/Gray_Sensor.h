#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdint.h>

void Gray_Sensor_Init(void);
void Gray_Sensor_Process(void);
uint8_t Gray_Sensor_Read(void);
uint8_t Gray_Sensor_Get_Stable(void);

#endif /* GRAY_SENSOR_H */
