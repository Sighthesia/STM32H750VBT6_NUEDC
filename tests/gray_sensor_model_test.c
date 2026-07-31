#include "Gray_Sensor.h"

#include <assert.h>
int main(void)
{
    assert(Gray_Sensor_Normalize(0xFFU, 1U) == 0x00U);
    assert(Gray_Sensor_Normalize(0xBFU, 1U) == 0x40U);
    assert(Gray_Sensor_Normalize(0x04U, 0U) == 0x04U);
    assert(Gray_Sensor_Normalize(0x00U, 1U) == 0xFFU);

    return 0;
}
