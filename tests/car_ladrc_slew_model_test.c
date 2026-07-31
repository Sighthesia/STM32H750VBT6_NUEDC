#include <assert.h>
#include <math.h>
#include <stdio.h>

#define CONTROL_PERIOD_S 0.01f

static float approach(float current, float target, float maximum_step)
{
    if (current < target)
    {
        current += maximum_step;
        return current > target ? target : current;
    }
    if (current > target)
    {
        current -= maximum_step;
        return current < target ? target : current;
    }
    return current;
}

static float select_rate_target(int ladrc_mode, float *commanded_rate, float raw_target)
{
    if (!ladrc_mode) return raw_target;
    *commanded_rate = approach(*commanded_rate, raw_target, 60.0f * CONTROL_PERIOD_S);
    return *commanded_rate;
}

static float select_output(int ladrc_mode, float *commanded_output, float raw_output)
{
    if (!ladrc_mode) return raw_output;
    *commanded_output = approach(*commanded_output, raw_output, 250.0f * CONTROL_PERIOD_S);
    return *commanded_output;
}

int main(void)
{
    float commanded_rate = 0.0f;
    float differential = 0.0f;
    const float rate_step = 60.0f * CONTROL_PERIOD_S;
    const float differential_step = 250.0f * CONTROL_PERIOD_S;

    for (unsigned int sample = 0U; sample < 75U; sample++)
    {
        float previous = commanded_rate;
        commanded_rate = approach(commanded_rate, 45.0f, rate_step);
        assert(commanded_rate >= previous);
        assert(commanded_rate - previous <= rate_step + 1.0e-6f);
    }
    assert(fabsf(commanded_rate - 45.0f) < 1.0e-4f);

    for (unsigned int sample = 0U; sample < 75U; sample++)
    {
        float previous = commanded_rate;
        commanded_rate = approach(commanded_rate, 0.0f, rate_step);
        assert(commanded_rate <= previous);
        assert(previous - commanded_rate <= rate_step + 1.0e-6f);
    }
    assert(fabsf(commanded_rate) < 1.0e-4f);

    for (unsigned int sample = 0U; sample < 24U; sample++)
    {
        float previous = differential;
        differential = approach(differential, 60.0f, differential_step);
        assert(differential >= previous);
        assert(differential - previous <= differential_step + 1.0e-6f);
    }
    assert(fabsf(differential - 60.0f) < 1.0e-5f);

    for (unsigned int sample = 0U; sample < 48U; sample++)
    {
        float previous = differential;
        differential = approach(differential, -60.0f, differential_step);
        assert(differential <= previous);
        assert(previous - differential <= differential_step + 1.0e-6f);
    }
    assert(fabsf(differential + 60.0f) < 1.0e-5f);

    commanded_rate = 0.0f;
    differential = 0.0f;
    assert(select_rate_target(0, &commanded_rate, 45.0f) == 45.0f);
    assert(select_output(0, &differential, 120.0f) == 120.0f);
    assert(commanded_rate == 0.0f);
    assert(differential == 0.0f);

    puts("car LADRC slew model tests passed");
    return 0;
}
