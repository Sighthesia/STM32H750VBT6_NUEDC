#include "Tracking_Line_Model.h"

#include <stddef.h>

#define TRACKING_LINE_MAX_ANGLE_X10 300

tracking_pattern_t Tracking_Line_Decode(uint8_t state, int16_t *offset_x1000,
                                        int16_t *angle_x10)
{
    static const int8_t positions[] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t groups = 0U;
    uint8_t previous_active = 0U;

    if (offset_x1000 == NULL || angle_x10 == NULL) return TRACKING_PATTERN_LOST;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        uint8_t active = (state & (uint8_t)(1U << bit)) != 0U ? 1U : 0U;
        if (active != 0U)
        {
            sum += positions[bit];
            count++;
            if (previous_active == 0U) groups++;
        }
        previous_active = active;
    }
    if (count == 0U)
    {
        *offset_x1000 = 0;
        *angle_x10 = 0;
        return TRACKING_PATTERN_LOST;
    }
    if (state == 0xFFU)
    {
        *offset_x1000 = 0;
        *angle_x10 = 0;
        return TRACKING_PATTERN_FULL;
    }

    *offset_x1000 = (int16_t)((sum * 1000) / ((int16_t)count * 7));
    *angle_x10 = (int16_t)((*offset_x1000 * TRACKING_LINE_MAX_ANGLE_X10) / 1000);
    return groups == 1U ? TRACKING_PATTERN_LINE : TRACKING_PATTERN_SPLIT;
}
