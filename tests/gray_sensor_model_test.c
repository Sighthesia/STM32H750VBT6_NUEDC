#include "Gray_Sensor.h"

#include <assert.h>
int main(void)
{
    assert(Gray_Sensor_Normalize(0x0FU, 1U) == 0x00U);
    assert(Gray_Sensor_Normalize(0x0BU, 1U) == 0x04U);
    assert(Gray_Sensor_Normalize(0x04U, 0U) == 0x04U);
    assert(Gray_Sensor_Normalize(0xF0U, 1U) == 0x0FU);

    return 0;
}
