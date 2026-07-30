#include "Gray_Sensor.h"

#include "main.h"

void Gray_Sensor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    EXTI->IMR1 &= ~EXTI_IMR1_IM8;

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);
}

uint8_t Gray_Sensor_Read(void)
{
    uint8_t value = 0U;

    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET ? 0x01U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET ? 0x02U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET ? 0x04U : 0U;
    value |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET ? 0x08U : 0U;
    return value;
}
