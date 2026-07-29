#include "lcd_driver.h"
#include "lcd_wegui_menu_mlist.h"
#include "lcd_wegui_tip.h"
#include "Encoder_EXTI.h"
#include "gpio.h"
#include "spi.h"

#define WEGUI_LCD_CS_PORT GPIOA
#define WEGUI_LCD_CS_PIN GPIO_PIN_15
#define WEGUI_LCD_DC_PORT GPIOD
#define WEGUI_LCD_DC_PIN GPIO_PIN_2
#define WEGUI_LCD_BL_PORT GPIOD
#define WEGUI_LCD_BL_PIN GPIO_PIN_10
#define WEGUI_LCD_SPI_TIMEOUT 100U

static void wegui_lcd_command(uint8_t value)
{
    HAL_GPIO_WritePin(WEGUI_LCD_DC_PORT, WEGUI_LCD_DC_PIN, GPIO_PIN_RESET);
    (void)HAL_SPI_Transmit(&hspi1, &value, 1U, WEGUI_LCD_SPI_TIMEOUT);
}

static void wegui_lcd_data(const uint8_t *data, uint16_t length)
{
    HAL_GPIO_WritePin(WEGUI_LCD_DC_PORT, WEGUI_LCD_DC_PIN, GPIO_PIN_SET);
    (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, WEGUI_LCD_SPI_TIMEOUT);
}

void wegui_stm32_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t address[4];

    x0 += SCREEN_X_OFFSET;
    x1 += SCREEN_X_OFFSET;
    y0 += SCREEN_Y_OFFSET;
    y1 += SCREEN_Y_OFFSET;
    wegui_lcd_command(0x2AU);
    address[0] = (uint8_t)(x0 >> 8);
    address[1] = (uint8_t)x0;
    address[2] = (uint8_t)(x1 >> 8);
    address[3] = (uint8_t)x1;
    wegui_lcd_data(address, sizeof(address));
    wegui_lcd_command(0x2BU);
    address[0] = (uint8_t)(y0 >> 8);
    address[1] = (uint8_t)y0;
    address[2] = (uint8_t)(y1 >> 8);
    address[3] = (uint8_t)y1;
    wegui_lcd_data(address, sizeof(address));
    wegui_lcd_command(0x2CU);
}

void wegui_stm32_port_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = WEGUI_LCD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(WEGUI_LCD_CS_PORT, &gpio);
    gpio.Pin = WEGUI_LCD_DC_PIN | WEGUI_LCD_BL_PIN;
    HAL_GPIO_Init(WEGUI_LCD_DC_PORT, &gpio);
    HAL_GPIO_WritePin(WEGUI_LCD_CS_PORT, WEGUI_LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WEGUI_LCD_BL_PORT, WEGUI_LCD_BL_PIN, GPIO_PIN_RESET);
    HAL_Delay(120U);
    lcd_ic_init();
    HAL_GPIO_WritePin(WEGUI_LCD_CS_PORT, WEGUI_LCD_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WEGUI_LCD_BL_PORT, WEGUI_LCD_BL_PIN, GPIO_PIN_SET);
}

uint8_t lcd_is_busy(void)
{
    return 0U;
}

void lcd_send_1Cmd(uint8_t value) { wegui_lcd_command(value); }
void lcd_send_nCmd(uint8_t *data, uint16_t length)
{
    if (length == 0U) return;

    wegui_lcd_command(*data++);
    if (--length != 0U) wegui_lcd_data(data, length);
}
void lcd_send_1Dat(uint8_t value) { wegui_lcd_data(&value, 1U); }
void lcd_send_nDat(uint8_t *data, uint16_t length) { wegui_lcd_data(data, length); }
void lcd_bl_on(void) { HAL_GPIO_WritePin(WEGUI_LCD_BL_PORT, WEGUI_LCD_BL_PIN, GPIO_PIN_SET); }
void lcd_bl_off(void) { HAL_GPIO_WritePin(WEGUI_LCD_BL_PORT, WEGUI_LCD_BL_PIN, GPIO_PIN_RESET); }
void lcd_delay_ms(volatile uint32_t ms) { HAL_Delay(ms); }

void lcd_rgb565_port(uint16_t x0, uint16_t x1, uint16_t page, uint8_t *gram)
{
    static uint8_t pixels[(SCREEN_WIDTH * 8U) * 2U];
    uint16_t x;
    uint8_t bit;
    uint16_t length = 0U;

    if (page * 8U >= SCREEN_HIGH) return;
    HAL_GPIO_WritePin(WEGUI_LCD_CS_PORT, WEGUI_LCD_CS_PIN, GPIO_PIN_RESET);
    wegui_stm32_set_window(x0, page * 8U, x1, (uint16_t)(((page + 1U) * 8U - 1U < SCREEN_HIGH) ? ((page + 1U) * 8U - 1U) : (SCREEN_HIGH - 1U)));
    for (bit = 0U; bit < 8U && page * 8U + bit < SCREEN_HIGH; ++bit) {
        uint8_t mask = (uint8_t)(1U << bit);
        uint8_t *source = gram;
        for (x = 0U; x <= x1 - x0; ++x) {
            uint8_t colour = (*source++ & mask) ? 1U : 0U;
            if (*source++ & mask) colour |= 2U;
            uint16_t rgb = lcd_driver.colour[colour];
            pixels[length++] = (uint8_t)(rgb >> 8);
            pixels[length++] = (uint8_t)rgb;
        }
    }
    wegui_lcd_data(pixels, length);
    HAL_GPIO_WritePin(WEGUI_LCD_CS_PORT, WEGUI_LCD_CS_PIN, GPIO_PIN_SET);
}

static void wegui_lcd_fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t colour)
{
    uint8_t pixels[256];
    uint32_t remaining = (uint32_t)(x1 - x0 + 1U) * (uint32_t)(y1 - y0 + 1U);
    uint16_t index;

    for (index = 0U; index < sizeof(pixels); index += 2U) {
        pixels[index] = (uint8_t)(colour >> 8);
        pixels[index + 1U] = (uint8_t)colour;
    }

    HAL_GPIO_WritePin(WEGUI_LCD_CS_PORT, WEGUI_LCD_CS_PIN, GPIO_PIN_RESET);
    wegui_stm32_set_window(x0, y0, x1, y1);
    while (remaining != 0U) {
        uint16_t bytes = remaining > (sizeof(pixels) / 2U) ? sizeof(pixels) : (uint16_t)(remaining * 2U);
        wegui_lcd_data(pixels, bytes);
        remaining -= bytes / 2U;
    }
    HAL_GPIO_WritePin(WEGUI_LCD_CS_PORT, WEGUI_LCD_CS_PIN, GPIO_PIN_SET);
}

void wegui_stm32_panel_self_test(void)
{
    wegui_lcd_fill(0U, 0U, SCREEN_WIDTH - 1U, SCREEN_HIGH - 1U, 0x0000U);
    wegui_lcd_fill(0U, 0U, SCREEN_WIDTH / 3U - 1U, SCREEN_HIGH - 1U, 0xF800U);
    wegui_lcd_fill(SCREEN_WIDTH / 3U, 0U, (SCREEN_WIDTH * 2U) / 3U - 1U, SCREEN_HIGH - 1U, 0x07E0U);
    wegui_lcd_fill((SCREEN_WIDTH * 2U) / 3U, 0U, SCREEN_WIDTH - 1U, SCREEN_HIGH - 1U, 0x001FU);
    wegui_lcd_fill(0U, 0U, SCREEN_WIDTH - 1U, 1U, 0xFFFFU);
    wegui_lcd_fill(0U, SCREEN_HIGH - 2U, SCREEN_WIDTH - 1U, SCREEN_HIGH - 1U, 0xFFFFU);
    wegui_lcd_fill(0U, 0U, 1U, SCREEN_HIGH - 1U, 0xFFFFU);
    wegui_lcd_fill(SCREEN_WIDTH - 2U, 0U, SCREEN_WIDTH - 1U, SCREEN_HIGH - 1U, 0xFFFFU);
}

void wegui_stm32_input_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5U, 0U);
    Encoder_Init();
}

void wegui_stm32_input_process(uint16_t elapsed_ms)
{
    static uint16_t pressed_ms;
    static uint16_t release_ms;
    static uint8_t click_count;
    uint8_t pressed = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET;
    encoder_state_t rotation = Encoder_Get_State_Change();
    if (rotation == ENCODER_INCREASE) {
        if (wegui.tip.state == ENTERING || wegui.tip.state == DISPLAYING) {
            if (wegui.tip.type == slider) wegui_tip_value_add();
        } else {
            wegui_mlist_cursor_Next();
        }
    }
    if (rotation == ENCODER_DECREASE) {
        if (wegui.tip.state == ENTERING || wegui.tip.state == DISPLAYING) {
            if (wegui.tip.type == slider) wegui_tip_value_dec();
        } else {
            wegui_mlist_cursor_Prev();
        }
    }
    if (pressed) {
        pressed_ms = (uint16_t)((pressed_ms + elapsed_ms > 0xFFFFU) ? 0xFFFFU : pressed_ms + elapsed_ms);
        release_ms = 0U;
    } else if (pressed_ms != 0U) {
        if (pressed_ms >= 20U) click_count++;
        pressed_ms = 0U;
    } else if (click_count != 0U) {
        release_ms = (uint16_t)((release_ms + elapsed_ms > 0xFFFFU) ? 0xFFFFU : release_ms + elapsed_ms);
    }
    if (click_count != 0U && release_ms >= 220U) {
        if (click_count == 1U) {
            if (wegui.tip.state == ENTERING || wegui.tip.state == DISPLAYING) wegui_tip_save_and_quit();
            else wegui_mlist_Enter_cursor();
        } else {
            if (wegui.tip.state == ENTERING || wegui.tip.state == DISPLAYING) wegui_tip_quit();
            else wegui_mlist_Back_menu();
        }
        click_count = 0U;
        release_ms = 0U;
    }
}
