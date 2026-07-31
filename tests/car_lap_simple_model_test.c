#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static int should_arm(float relative_yaw_deg)
{
    if (relative_yaw_deg < 0.0f) relative_yaw_deg = -relative_yaw_deg;
    return relative_yaw_deg >= 180.0f;
}

static int cross_confirmed(int armed, uint8_t gray_state, uint8_t pattern, uint32_t held_ms)
{
    return armed && (gray_state & pattern) == pattern && held_ms >= 50U;
}

static int arc_complete(float distance_m, float yaw_deg, int odometry_valid)
{
    if (yaw_deg < 0.0f) yaw_deg = -yaw_deg;
    return odometry_valid && distance_m >= 1.5707963f && yaw_deg >= 165.0f;
}

static int geometry_remains_valid(int was_valid, int odometry_valid)
{
    return was_valid && odometry_valid;
}

int main(void)
{
    assert(!should_arm(179.9f));
    assert(should_arm(180.0f));
    assert(should_arm(-180.0f));
    assert(!cross_confirmed(0, 0x3CU, 0x3CU, 100U));
    assert(!cross_confirmed(1, 0x00U, 0x3CU, 100U));
    assert(!cross_confirmed(1, 0x38U, 0x3CU, 100U));
    assert(!cross_confirmed(1, 0x3CU, 0x3CU, 49U));
    assert(cross_confirmed(1, 0x3CU, 0x3CU, 50U));
    assert(cross_confirmed(1, 0xFFU, 0x3CU, 50U));
    assert(!arc_complete(1.57f, 180.0f, 1));
    assert(!arc_complete(1.58f, 164.9f, 1));
    assert(!arc_complete(2.00f, 180.0f, 0));
    assert(arc_complete(1.58f, 165.0f, 1));
    assert(geometry_remains_valid(1, 1));
    assert(!geometry_remains_valid(1, 0));
    assert(!geometry_remains_valid(0, 1));

    puts("car lap simple model tests passed");
    return 0;
}
