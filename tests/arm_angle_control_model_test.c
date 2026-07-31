#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float angle_output(float target, float measured, float kp,
                          float deadzone, float speed_limit, int inverted)
{
    float error = target - measured;
    float output = fabsf(error) <= deadzone ? 0.0f : clampf(kp * error, speed_limit);
    return inverted ? -output : output;
}

static int limit_allows(float motor_position, float minimum, float maximum, float output)
{
    return !((motor_position <= minimum && output < 0.0f) ||
             (motor_position >= maximum && output > 0.0f));
}

int main(void)
{
    assert(angle_output(0.0f, -5.0f, 1.0f, 0.2f, 10.0f, 0) > 0.0f);
    assert(angle_output(0.0f, 5.0f, 1.0f, 0.2f, 10.0f, 0) < 0.0f);
    assert(angle_output(0.0f, 0.1f, 1.0f, 0.2f, 10.0f, 0) == 0.0f);
    assert(angle_output(20.0f, 0.0f, 1.0f, 0.2f, 10.0f, 0) == 10.0f);
    assert(angle_output(0.0f, -5.0f, 1.0f, 0.2f, 10.0f, 1) < 0.0f);

    assert(!limit_allows(-30.0f, -30.0f, 30.0f, -1.0f));
    assert(limit_allows(-30.0f, -30.0f, 30.0f, 1.0f));
    assert(!limit_allows(30.0f, -30.0f, 30.0f, 1.0f));
    assert(limit_allows(30.0f, -30.0f, 30.0f, -1.0f));

    puts("arm angle control model tests passed");
    return 0;
}
