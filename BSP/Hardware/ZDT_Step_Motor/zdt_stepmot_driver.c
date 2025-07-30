#include "zdt_stepmot_driver.h"

// 引脚配置
#define GPIO_EN 1
#define GPIO_STP 2
#define GPIO_DIR 3

// 电机状态
static uint8_t zdt_stepmot_state = 0;

// 引脚初始化
static void gpio_init();
// 引脚设置
static void io(uint16_t pin, uint8_t level);
// 毫秒延迟
static void delay(uint32_t ms);
// 电机初始化
void zdt_stepmot_init();
// 电机使能
void zdt_stepmot_en(uint8_t en);
// 电机设置
void zdt_stepmot_set(uint8_t dir, uint32_t stp, float spd);
// 电机状态检查（仅在电机空闲时可操作）
uint8_t zdt_stepmot_check();

// 引脚初始化
static void gpio_init()
{
    // gpio_config_t config={0};
    // //使能引脚
    // config.intr_type=GPIO_INTR_DISABLE;
    // config.mode=GPIO_MODE_OUTPUT;
    // config.pin_bit_mask=(1ULL<<GPIO_EN);
    // config.pull_down_en=0;
    // config.pull_up_en=0;
    // gpio_config(&config);
    // //使能引脚
    // config.intr_type=GPIO_INTR_DISABLE;
    // config.mode=GPIO_MODE_OUTPUT;
    // config.pin_bit_mask=(1ULL<<GPIO_STP);
    // config.pull_down_en=0;
    // config.pull_up_en=0;
    // gpio_config(&config);
    // //方向引脚
    // config.intr_type=GPIO_INTR_DISABLE;
    // config.mode=GPIO_MODE_OUTPUT;
    // config.pin_bit_mask=(1ULL<<GPIO_DIR);
    // config.pull_down_en=0;
    // config.pull_up_en=0;
    // gpio_config(&config);
}

// 引脚设置
static void io(uint16_t pin, uint8_t level)
{
    switch (pin)
    {
    case GPIO_EN:
        HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, (GPIO_PinState)(level));
        break;
    case GPIO_STP:
        HAL_GPIO_WritePin(MOT1_STP_GPIO_Port, MOT1_STP_Pin, (GPIO_PinState)(level));
        break;
    case GPIO_DIR:
        HAL_GPIO_WritePin(MOT1_DIR_GPIO_Port, MOT1_DIR_Pin, (GPIO_PinState)(level));
        break;
    default:
        break;
    }
}

// 毫秒延迟
static void delay(uint32_t ms)
{
    // printf("dl=%ld\n",pdMS_TO_TICKS(ms));
    // vTaskDelay(pdMS_TO_TICKS(ms));
    HAL_Delay(ms);
    return;
}

// 电机初始化
void zdt_stepmot_init()
{
    gpio_init();
    io(GPIO_EN, 1);
    io(GPIO_STP, 0);
    io(GPIO_DIR, 0);
}

// 电机使能设置
void zdt_stepmot_en(uint8_t en)
{
    // 设置使能
    io(GPIO_EN, en);
}

// 电机步进设置
void zdt_stepmot_set(uint8_t dir, uint32_t stp, float spd)
{
    // 检查状态（防止重入）
    if (!zdt_stepmot_state)
    {
        zdt_stepmot_state = 1;
        // 设置方向
        io(GPIO_DIR, dir);
        // 进行步数
        while (stp)
        {
            io(GPIO_STP, 1);
            delay((uint32_t)(1000 / spd));
            io(GPIO_STP, 0);
            delay((uint32_t)(1000 / spd));
            --stp;
        }
        zdt_stepmot_state = 0;
    }
    else
    {
        // 失能防止意外
        //  io(GPIO_EN,0);
    }
}

// 电机状态检查（仅在电机空闲时可操作）
uint8_t zdt_stepmot_check()
{
    return zdt_stepmot_state;
}

// //电机配置
// typedef struct zdt_stepmot_t
// {
//     uint8_t en;
//     uint8_t dir;
//     float spd;
// }zdt_stepmot_t;
// static zdt_stepmot_t zdt_stepmot={0};