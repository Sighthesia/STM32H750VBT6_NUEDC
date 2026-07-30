#include "Tracking_Line_Model.h"

#include <assert.h>
#include <stdio.h>

static void expect_line(uint8_t state, int16_t offset, int16_t angle)
{
    int16_t actual_offset;
    int16_t actual_angle;

    assert(Tracking_Line_Decode(state, &actual_offset, &actual_angle) == TRACKING_PATTERN_LINE);
    assert(actual_offset == offset);
    assert(actual_angle == angle);
}

int main(void)
{
    int16_t offset;
    int16_t angle;

    expect_line(0x01U, -1000, -300);
    expect_line(0x80U, 1000, 300);
    expect_line(0x18U, 0, 0);
    assert(Tracking_Line_Decode(0x00U, &offset, &angle) == TRACKING_PATTERN_LOST);
    assert(Tracking_Line_Decode(0xFFU, &offset, &angle) == TRACKING_PATTERN_FULL);
    assert(Tracking_Line_Decode(0x81U, &offset, &angle) == TRACKING_PATTERN_SPLIT);
    assert(offset == 0 && angle == 0);
    assert(Tracking_Line_Decode(0x18U, NULL, &angle) == TRACKING_PATTERN_LOST);

    puts("tracking line model tests passed");
    return 0;
}
