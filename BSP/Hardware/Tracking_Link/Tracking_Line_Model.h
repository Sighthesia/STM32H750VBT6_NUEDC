#ifndef TRACKING_LINE_MODEL_H
#define TRACKING_LINE_MODEL_H

#include <stdint.h>

typedef enum
{
    TRACKING_PATTERN_LOST = 0,
    TRACKING_PATTERN_LINE,
    TRACKING_PATTERN_FULL,
    TRACKING_PATTERN_SPLIT
} tracking_pattern_t;

tracking_pattern_t Tracking_Line_Decode(uint8_t state, int16_t *offset_x1000,
                                        int16_t *angle_x10);

#endif
