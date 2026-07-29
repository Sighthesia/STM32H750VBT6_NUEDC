#include "ips114_lcd.h"

#include "main.h"
#include "spi.h"

#define IPS114_LCD_CS_PORT GPIOA
#define IPS114_LCD_CS_PIN GPIO_PIN_15
#define IPS114_LCD_DC_PORT GPIOD
#define IPS114_LCD_DC_PIN GPIO_PIN_2
#define IPS114_LCD_BL_PORT GPIOD
#define IPS114_LCD_BL_PIN GPIO_PIN_10

#define IPS114_LCD_X_OFFSET 53U
#define IPS114_LCD_Y_OFFSET 40U
#define IPS114_LCD_SPI_TIMEOUT 100U

static void IPS114_LCD_WriteCommand(uint8_t command)
{
    HAL_GPIO_WritePin(IPS114_LCD_DC_PORT, IPS114_LCD_DC_PIN, GPIO_PIN_RESET);
    (void)HAL_SPI_Transmit(&hspi1, &command, 1U, IPS114_LCD_SPI_TIMEOUT);
}

static void IPS114_LCD_WriteData(const uint8_t *data, uint16_t length)
{
    HAL_GPIO_WritePin(IPS114_LCD_DC_PORT, IPS114_LCD_DC_PIN, GPIO_PIN_SET);
    (void)HAL_SPI_Transmit(&hspi1, data, length, IPS114_LCD_SPI_TIMEOUT);
}

static void IPS114_LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    x0 += IPS114_LCD_X_OFFSET;
    x1 += IPS114_LCD_X_OFFSET;
    y0 += IPS114_LCD_Y_OFFSET;
    y1 += IPS114_LCD_Y_OFFSET;

    IPS114_LCD_WriteCommand(0x2AU);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)x0;
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)x1;
    IPS114_LCD_WriteData(data, sizeof(data));

    IPS114_LCD_WriteCommand(0x2BU);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)y0;
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)y1;
    IPS114_LCD_WriteData(data, sizeof(data));
    IPS114_LCD_WriteCommand(0x2CU);
}

static void IPS114_LCD_WriteColor(uint16_t color)
{
    uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)color};

    IPS114_LCD_WriteData(data, sizeof(data));
}

static const uint8_t *IPS114_LCD_GetGlyph(char character)
{
    static const uint8_t glyphs[][5] = {
        {'\0', '\0', '\0', '\0', '\0'}, /* Space */
        {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}, /* A */
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}, /* B */
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, /* C */
        {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}, /* D */
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, /* E */
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}, /* F */
        {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, /* G */
        {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}, /* H */
        {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, /* I */
        {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}, /* J */
        {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, /* K */
        {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}, /* L */
        {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, /* M */
        {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}, /* N */
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, /* O */
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}, /* P */
        {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, /* Q */
        {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}, /* R */
        {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, /* S */
        {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}, /* T */
        {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, /* U */
        {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}, /* V */
        {0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU}, /* W */
        {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}, /* X */
        {0x03U, 0x04U, 0x78U, 0x04U, 0x03U}, /* Y */
        {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}, /* Z */
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, /* 0 */
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}, /* 1 */
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, /* 2 */
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}, /* 3 */
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, /* 4 */
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}, /* 5 */
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, /* 6 */
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}, /* 7 */
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, /* 8 */
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}, /* 9 */
        {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}, /* - */
        {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}, /* . */
        {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}  /* ? */
    };

    if (character >= 'a' && character <= 'z')
    {
        character = (char)(character - ('a' - 'A'));
    }
    if (character == ' ')
    {
        return glyphs[0];
    }
    if (character >= 'A' && character <= 'Z')
    {
        return glyphs[(character - 'A') + 1];
    }
    if (character >= '0' && character <= '9')
    {
        return glyphs[(character - '0') + 27];
    }
    if (character == '-')
    {
        return glyphs[37];
    }
    if (character == '.')
    {
        return glyphs[38];
    }
    return glyphs[39];
}

void IPS114_LCD_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    static const uint8_t porch[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    static const uint8_t power[] = {0xA4U, 0xA1U};
    static const uint8_t gamma_positive[] = {0xD0U, 0x04U, 0x0DU, 0x11U, 0x13U, 0x2BU, 0x3FU, 0x54U, 0x4CU, 0x18U, 0x0DU, 0x0BU, 0x1FU, 0x23U};
    static const uint8_t gamma_negative[] = {0xD0U, 0x04U, 0x0CU, 0x11U, 0x13U, 0x2CU, 0x3FU, 0x44U, 0x51U, 0x2FU, 0x1FU, 0x1FU, 0x20U, 0x23U};
    uint8_t value;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = IPS114_LCD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IPS114_LCD_CS_PORT, &gpio);
    gpio.Pin = IPS114_LCD_DC_PIN | IPS114_LCD_BL_PIN;
    HAL_GPIO_Init(IPS114_LCD_DC_PORT, &gpio);
    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IPS114_LCD_DC_PORT, IPS114_LCD_DC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IPS114_LCD_BL_PORT, IPS114_LCD_BL_PIN, GPIO_PIN_RESET);

    MX_SPI1_Init();
    HAL_Delay(120U); /* LCD reset follows the MCU NRST line. */

    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_RESET);
    IPS114_LCD_WriteCommand(0x11U);
    HAL_Delay(120U);
    IPS114_LCD_WriteCommand(0x36U);
    value = 0xC0U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0x3AU);
    value = 0x05U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xB2U);
    IPS114_LCD_WriteData(porch, sizeof(porch));
    IPS114_LCD_WriteCommand(0xB7U);
    value = 0x35U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xBBU);
    value = 0x37U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xC0U);
    value = 0x2CU;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xC2U);
    value = 0x01U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xC3U);
    value = 0x12U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xC4U);
    value = 0x20U;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xC6U);
    value = 0x0FU;
    IPS114_LCD_WriteData(&value, 1U);
    IPS114_LCD_WriteCommand(0xD0U);
    IPS114_LCD_WriteData(power, sizeof(power));
    IPS114_LCD_WriteCommand(0xE0U);
    IPS114_LCD_WriteData(gamma_positive, sizeof(gamma_positive));
    IPS114_LCD_WriteCommand(0xE1U);
    IPS114_LCD_WriteData(gamma_negative, sizeof(gamma_negative));
    IPS114_LCD_WriteCommand(0x21U);
    IPS114_LCD_WriteCommand(0x29U);
    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IPS114_LCD_BL_PORT, IPS114_LCD_BL_PIN, GPIO_PIN_SET);
}

void IPS114_LCD_Fill(uint16_t color)
{
    uint8_t buffer[256];
    uint32_t pixels = IPS114_LCD_WIDTH * IPS114_LCD_HEIGHT;
    uint32_t index;

    for (index = 0U; index < sizeof(buffer); index += 2U)
    {
        buffer[index] = (uint8_t)(color >> 8);
        buffer[index + 1U] = (uint8_t)color;
    }

    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_RESET);
    IPS114_LCD_SetWindow(0U, 0U, IPS114_LCD_WIDTH - 1U, IPS114_LCD_HEIGHT - 1U);
    while (pixels >= (sizeof(buffer) / 2U))
    {
        IPS114_LCD_WriteData(buffer, sizeof(buffer));
        pixels -= sizeof(buffer) / 2U;
    }
    if (pixels != 0U)
    {
        IPS114_LCD_WriteData(buffer, (uint16_t)(pixels * 2U));
    }
    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_SET);
}

void IPS114_LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= IPS114_LCD_WIDTH || y >= IPS114_LCD_HEIGHT)
    {
        return;
    }
    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_RESET);
    IPS114_LCD_SetWindow(x, y, x, y);
    IPS114_LCD_WriteColor(color);
    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_SET);
}

void IPS114_LCD_WritePixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                            const uint8_t *data, uint16_t length)
{
    if (data == NULL || x0 > x1 || y0 > y1 || x1 >= IPS114_LCD_WIDTH || y1 >= IPS114_LCD_HEIGHT)
    {
        return;
    }

    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_RESET);
    IPS114_LCD_SetWindow(x0, y0, x1, y1);
    IPS114_LCD_WriteData(data, length);
    HAL_GPIO_WritePin(IPS114_LCD_CS_PORT, IPS114_LCD_CS_PIN, GPIO_PIN_SET);
}

void IPS114_LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int32_t dx = (x0 > x1) ? (int32_t)x0 - x1 : (int32_t)x1 - x0;
    int32_t sx = (x0 < x1) ? 1 : -1;
    int32_t dy = (y0 > y1) ? (int32_t)y0 - y1 : (int32_t)y1 - y0;
    int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t error = dx - dy;

    while (1)
    {
        IPS114_LCD_DrawPoint(x0, y0, color);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        if ((error * 2) > -dy)
        {
            error -= dy;
            x0 = (uint16_t)((int32_t)x0 + sx);
        }
        if ((error * 2) < dx)
        {
            error += dx;
            y0 = (uint16_t)((int32_t)y0 + sy);
        }
    }
}

void IPS114_LCD_DrawString(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t background)
{
    uint16_t column;
    uint16_t row;

    while (*text != '\0' && x < (IPS114_LCD_WIDTH - 5U))
    {
        const uint8_t *glyph = IPS114_LCD_GetGlyph(*text++);
        for (column = 0U; column < 5U; ++column)
        {
            for (row = 0U; row < 7U; ++row)
            {
                IPS114_LCD_DrawPoint((uint16_t)(x + column), (uint16_t)(y + row),
                                      (glyph[column] & (1U << row)) ? color : background);
            }
        }
        x += 6U;
    }
}

void IPS114_LCD_ShowSelfTest(void)
{
    IPS114_LCD_Fill(IPS114_COLOR_BLACK);
    IPS114_LCD_DrawLine(0U, 0U, IPS114_LCD_WIDTH - 1U, 0U, IPS114_COLOR_RED);
    IPS114_LCD_DrawLine(0U, IPS114_LCD_HEIGHT - 1U, IPS114_LCD_WIDTH - 1U, IPS114_LCD_HEIGHT - 1U, IPS114_COLOR_BLUE);
    IPS114_LCD_DrawString(16U, 24U, "IPS114", IPS114_COLOR_WHITE, IPS114_COLOR_BLACK);
    IPS114_LCD_DrawString(16U, 40U, "ST7789 READY", IPS114_COLOR_GREEN, IPS114_COLOR_BLACK);
}
