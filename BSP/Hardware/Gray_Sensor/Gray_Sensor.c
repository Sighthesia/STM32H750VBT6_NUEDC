#include "Gray_Sensor.h"

#include "main.h"

#define GRAY_SENSOR_SAMPLE_PERIOD_MS 5U
#define GRAY_SENSOR_DEBOUNCE_COUNT   3U

static volatile uint8_t stable_value;
static uint8_t debounce_count[8];
static uint32_t last_sample_ms;
static uint8_t initialized;
static uint8_t active_low = 1U;

void Gray_Sensor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    EXTI->IMR1 &= ~EXTI_IMR1_IM8;

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio);

    stable_value = Gray_Sensor_Read();
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        debounce_count[bit] = (stable_value & (1U << bit)) != 0U ?
                              GRAY_SENSOR_DEBOUNCE_COUNT : 0U;
    }
    last_sample_ms = HAL_GetTick();
    initialized = 1U;
}

void Gray_Sensor_Process(void)
{
    if (initialized == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_sample_ms) < GRAY_SENSOR_SAMPLE_PERIOD_MS) return;
    last_sample_ms = now;

    uint8_t raw = Gray_Sensor_Read();
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        uint8_t mask = (uint8_t)(1U << bit);
        if ((raw & mask) != 0U)
        {
            if (debounce_count[bit] < GRAY_SENSOR_DEBOUNCE_COUNT) debounce_count[bit]++;
            if (debounce_count[bit] == GRAY_SENSOR_DEBOUNCE_COUNT) stable_value |= mask;
        }
        else
        {
            if (debounce_count[bit] > 0U) debounce_count[bit]--;
            if (debounce_count[bit] == 0U) stable_value &= (uint8_t)~mask;
        }
    }
}

uint8_t Gray_Sensor_Read(void)
{
    uint8_t value = 0U;

    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET ? 0x01U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET ? 0x02U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET ? 0x04U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET ? 0x08U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET ? 0x10U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET ? 0x20U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET ? 0x40U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET ? 0x80U : 0U;
    return Gray_Sensor_Normalize(value, active_low);
}

uint8_t Gray_Sensor_Get_Stable(void)
{
    return stable_value;
}

void Gray_Sensor_Set_Active_Low(uint8_t state)
{
    active_low = state != 0U ? 1U : 0U;
    if (initialized != 0U)
    {
        stable_value = Gray_Sensor_Read();
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            debounce_count[bit] = (stable_value & (1U << bit)) != 0U ?
                                  GRAY_SENSOR_DEBOUNCE_COUNT : 0U;
        }
    }
}

uint8_t Gray_Sensor_Get_Active_Low(void)
{
    return active_low;
}
