#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define MIN_SPAN_DEG 0.5f
#define STABILITY_DEG 0.3f

static int points_valid(const float values[6])
{
    int pitch_increasing;
    int pitch_decreasing;

    for (uint8_t i = 0U; i < 6U; i++)
        if (!isfinite(values[i])) return 0;
    if (!(values[0] + MIN_SPAN_DEG < values[2] &&
          values[2] + MIN_SPAN_DEG < values[4])) return 0;
    pitch_increasing = values[1] + MIN_SPAN_DEG < values[3] &&
                       values[3] + MIN_SPAN_DEG < values[5];
    pitch_decreasing = values[1] > values[3] + MIN_SPAN_DEG &&
                       values[3] > values[5] + MIN_SPAN_DEG;
    return pitch_increasing || pitch_decreasing;
}

static int samples_stable(float minimum, float maximum)
{
    return maximum - minimum <= STABILITY_DEG;
}

static void rolling_sample(float samples[3], uint8_t *index, uint8_t *count,
                           float *sum, float value)
{
    if (*count < 3U)
    {
        samples[*index] = value;
        *sum += value;
        (*count)++;
    }
    else
    {
        *sum -= samples[*index];
        samples[*index] = value;
        *sum += value;
    }
    *index = (uint8_t)((*index + 1U) % 3U);
}

static int first_sequence_is_newer(uint32_t first, uint32_t second)
{
    return (int32_t)(first - second) > 0;
}

int main(void)
{
    const float increasing[6] = {-30.0f, -8.0f, 0.0f, 0.0f, 30.0f, 8.0f};
    const float decreasing[6] = {-30.0f, 8.0f, 0.0f, 0.0f, 30.0f, -8.0f};
    const float bad_motor_order[6] = {-30.0f, -8.0f, -30.0f, 0.0f, 30.0f, 8.0f};
    const float bad_pitch_order[6] = {-30.0f, -8.0f, 0.0f, 2.0f, 30.0f, 1.0f};
    const float too_close[6] = {-0.2f, -8.0f, 0.0f, 0.0f, 0.2f, 8.0f};
    float samples[3] = {0.0f};
    float sum = 0.0f;
    uint8_t index = 0U;
    uint8_t count = 0U;

    assert(points_valid(increasing));
    assert(points_valid(decreasing));
    assert(!points_valid(bad_motor_order));
    assert(!points_valid(bad_pitch_order));
    assert(!points_valid(too_close));

    assert(samples_stable(-0.1f, 0.2f));
    assert(!samples_stable(-0.1f, 0.21f));

    rolling_sample(samples, &index, &count, &sum, 1.0f);
    rolling_sample(samples, &index, &count, &sum, 2.0f);
    rolling_sample(samples, &index, &count, &sum, 3.0f);
    assert(count == 3U && fabsf(sum - 6.0f) < 0.001f);
    rolling_sample(samples, &index, &count, &sum, 4.0f);
    assert(count == 3U && fabsf(sum - 9.0f) < 0.001f);

    assert(first_sequence_is_newer(11U, 10U));
    assert(!first_sequence_is_newer(10U, 11U));
    assert(first_sequence_is_newer(1U, UINT32_MAX));

    puts("arm three-point calibration model tests passed");
    return 0;
}
