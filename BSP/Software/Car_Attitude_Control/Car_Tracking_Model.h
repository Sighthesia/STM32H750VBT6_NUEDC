#ifndef CAR_TRACKING_MODEL_H
#define CAR_TRACKING_MODEL_H

#include <stdint.h>

static inline float Car_Tracking_Target_Offset(uint8_t sensor_state,
                                               float inner_weight,
                                               float outer_weight,
                                               float lookahead_time_s)
{
    float heading_rate = 0.0f;

    if ((sensor_state & 0x01U) != 0U) heading_rate += outer_weight;
    if ((sensor_state & 0x02U) != 0U) heading_rate += inner_weight;
    if ((sensor_state & 0x04U) != 0U) heading_rate -= inner_weight;
    if ((sensor_state & 0x08U) != 0U) heading_rate -= outer_weight;
    return heading_rate * lookahead_time_s;
}

static inline float Car_Tracking_Display_Correction(float target_offset)
{
    return target_offset;
}

#endif /* CAR_TRACKING_MODEL_H */
