#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define SPEED_VALID 0x01U
#define POSITION_VALID 0x02U
#define TELEMETRY_TIMEOUT_MS 200U

static uint32_t query_period_ms(uint32_t frequency_hz)
{
    uint32_t period_ms = 1000U / frequency_hz;
    return period_ms == 0U ? 1U : period_ms;
}

static uint32_t feedback_timeout_ms(uint32_t frequency_hz)
{
    uint32_t timeout_ms = query_period_ms(frequency_hz) * 4U;
    return timeout_ms < 100U ? 100U : timeout_ms;
}

static uint8_t valid_flags(uint16_t speed_age_ms, uint16_t position_age_ms)
{
    uint8_t flags = 0U;
    if (speed_age_ms <= TELEMETRY_TIMEOUT_MS) flags |= SPEED_VALID;
    if (position_age_ms <= TELEMETRY_TIMEOUT_MS) flags |= POSITION_VALID;
    return flags;
}

static int query_completed(uint32_t issued_ms, uint32_t response_baseline_ms,
                           uint32_t response_ms, uint32_t now_ms, uint32_t timeout_ms)
{
    return response_ms != response_baseline_ms ||
           now_ms - issued_ms >= timeout_ms;
}

int main(void)
{
    assert(query_period_ms(20U) == 50U);
    assert(query_period_ms(100U) == 10U);
    assert(query_period_ms(200U) == 5U);
    assert(feedback_timeout_ms(20U) == 200U);
    assert(feedback_timeout_ms(100U) == 100U);
    assert(feedback_timeout_ms(200U) == 100U);

    assert(valid_flags(0U, 0U) == (SPEED_VALID | POSITION_VALID));
    assert(valid_flags(200U, 200U) == (SPEED_VALID | POSITION_VALID));
    assert(valid_flags(201U, 10U) == POSITION_VALID);
    assert(valid_flags(10U, 201U) == SPEED_VALID);
    assert(valid_flags(UINT16_MAX, UINT16_MAX) == 0U);

    assert(!query_completed(100U, 99U, 99U, 110U, 20U));
    assert(query_completed(100U, 99U, 100U, 101U, 20U));
    assert(query_completed(100U, 0U, 0U, 120U, 20U));
    assert(!query_completed(UINT32_MAX - 5U, 0U, 0U, 5U, 20U));
    assert(query_completed(UINT32_MAX - 5U, 0U, 0U, 14U, 20U));

    puts("arm motor telemetry model tests passed");
    return 0;
}
