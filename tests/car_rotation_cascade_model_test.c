#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define ANGLE_TOLERANCE_DEG 1.0f
#define RATE_TOLERANCE_DEG_S 5.0f
#define SETTLE_CONFIRM_MS 200U

static float clampf(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static void cascade(float target_angle, float measured_angle, float measured_rate,
                    float *target_rate, float *differential,
                    float *left, float *right)
{
    const float heading_error = measured_angle - target_angle;
    *target_rate = clampf(-3.0f * heading_error, -180.0f, 180.0f);
    *differential = clampf(1.2f * (*target_rate - measured_rate), -300.0f, 300.0f);
    *left = -*differential;
    *right = *differential;
}

static int settled(float angle_error, float measured_rate, uint32_t held_ms)
{
    return fabsf(angle_error) <= ANGLE_TOLERANCE_DEG &&
           fabsf(measured_rate) <= RATE_TOLERANCE_DEG_S &&
           held_ms >= SETTLE_CONFIRM_MS;
}

int main(void)
{
    float positive_rate, positive_diff, positive_left, positive_right;
    float negative_rate, negative_diff, negative_left, negative_right;

    cascade(90.0f, 0.0f, 0.0f, &positive_rate, &positive_diff,
            &positive_left, &positive_right);
    cascade(-90.0f, 0.0f, 0.0f, &negative_rate, &negative_diff,
            &negative_left, &negative_right);

    assert(positive_rate > 0.0f);
    assert(negative_rate < 0.0f);
    assert(positive_diff > 0.0f);
    assert(negative_diff < 0.0f);
    assert(positive_left == -positive_right);
    assert(negative_left == -negative_right);
    assert(positive_diff == -negative_diff);

    assert(!settled(0.5f, 2.0f, 199U));
    assert(settled(0.5f, 2.0f, 200U));
    assert(!settled(1.1f, 2.0f, 200U));
    assert(!settled(0.5f, 5.1f, 200U));

    puts("car rotation cascade model tests passed");
    return 0;
}
