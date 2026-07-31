#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static int should_arm(float relative_yaw_deg)
{
    if (relative_yaw_deg < 0.0f) relative_yaw_deg = -relative_yaw_deg;
    return relative_yaw_deg >= 180.0f;
}

static int cross_confirmed(int armed, int cross, uint32_t held_ms)
{
    return armed && cross && held_ms >= 50U;
}

int main(void)
{
    assert(!should_arm(179.9f));
    assert(should_arm(180.0f));
    assert(should_arm(-180.0f));
    assert(!cross_confirmed(0, 1, 100U));
    assert(!cross_confirmed(1, 0, 100U));
    assert(!cross_confirmed(1, 1, 49U));
    assert(cross_confirmed(1, 1, 50U));

    puts("car lap simple model tests passed");
    return 0;
}
