#include "Vision_Link.h"

#include "main.h"
#include "usart.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VISION_LINK_LINE_MAX_LENGTH 48U
#define VISION_LINK_TIMEOUT_MS 100U

static uint8_t vision_rx_byte;
static uint8_t vision_rx_line[VISION_LINK_LINE_MAX_LENGTH];
static uint8_t vision_ready_line[VISION_LINK_LINE_MAX_LENGTH];
static volatile uint8_t vision_rx_length;
static volatile uint8_t vision_ready_length;
static volatile uint8_t vision_discard_until_newline;
static volatile uint8_t vision_link_frame_valid;
static volatile vision_link_sample_t vision_link_sample;

static const char *Vision_Link_Skip_Space(const char *text)
{
    while (*text != '\0' && isspace((unsigned char)*text)) text++;
    return text;
}

static bool Vision_Link_Parse_Line(char *line, float *position_cm, float *velocity_cm_s)
{
    char *end;
    float position;
    float velocity;

    line = (char *)Vision_Link_Skip_Space(line);
    if (line[0] != '0' || !isspace((unsigned char)line[1])) return false;
    line = (char *)Vision_Link_Skip_Space(line + 1);
    position = strtof(line, &end);
    if (end == line || !isspace((unsigned char)*end)) return false;
    line = (char *)Vision_Link_Skip_Space(end);
    velocity = strtof(line, &end);
    if (end == line || *Vision_Link_Skip_Space(end) != '\0' ||
        !isfinite(position) || !isfinite(velocity)) return false;

    *position_cm = position;
    *velocity_cm_s = velocity;
    return true;
}

void Vision_Link_Init(void)
{
    vision_rx_length = 0U;
    vision_ready_length = 0U;
    vision_discard_until_newline = 0U;
    vision_link_frame_valid = 0U;
    (void)HAL_UART_Receive_IT(&huart4, &vision_rx_byte, 1U);
}

void Vision_Link_UART_RxCplt_Callback(void)
{
    const uint8_t byte = vision_rx_byte;

    if (byte == '\n')
    {
        if (vision_discard_until_newline == 0U && vision_rx_length != 0U && vision_ready_length == 0U)
        {
            memcpy(vision_ready_line, vision_rx_line, vision_rx_length);
            vision_ready_line[vision_rx_length] = '\0';
            vision_ready_length = vision_rx_length;
        }
        vision_rx_length = 0U;
        vision_discard_until_newline = 0U;
    }
    else if (byte != '\r' && vision_discard_until_newline == 0U)
    {
        if (vision_rx_length < VISION_LINK_LINE_MAX_LENGTH - 1U)
        {
            vision_rx_line[vision_rx_length++] = byte;
        }
        else
        {
            vision_rx_length = 0U;
            vision_discard_until_newline = 1U;
        }
    }

    (void)HAL_UART_Receive_IT(&huart4, &vision_rx_byte, 1U);
}

void Vision_Link_UART_Error_Callback(void)
{
    vision_rx_length = 0U;
    vision_discard_until_newline = 0U;
    (void)HAL_UART_Receive_IT(&huart4, &vision_rx_byte, 1U);
}

void Vision_Link_Process(void)
{
    char line[VISION_LINK_LINE_MAX_LENGTH];
    uint8_t length;
    uint32_t primask;
    float position_cm;
    float velocity_cm_s;

    primask = __get_PRIMASK();
    __disable_irq();
    length = vision_ready_length;
    if (length != 0U)
    {
        memcpy(line, vision_ready_line, (size_t)length + 1U);
        vision_ready_length = 0U;
    }
    __set_PRIMASK(primask);

    if (length == 0U || !Vision_Link_Parse_Line(line, &position_cm, &velocity_cm_s)) return;

    primask = __get_PRIMASK();
    __disable_irq();
    vision_link_sample.position_cm = position_cm;
    vision_link_sample.velocity_cm_s = velocity_cm_s;
    vision_link_sample.received_ms = HAL_GetTick();
    vision_link_frame_valid = 1U;
    __set_PRIMASK(primask);
}

bool Vision_Link_Get_Sample(vision_link_sample_t *sample)
{
    uint32_t primask;

    if (sample == NULL) return false;
    primask = __get_PRIMASK();
    __disable_irq();
    if (vision_link_frame_valid == 0U ||
        (uint32_t)(HAL_GetTick() - vision_link_sample.received_ms) > VISION_LINK_TIMEOUT_MS)
    {
        __set_PRIMASK(primask);
        return false;
    }
    *sample = vision_link_sample;
    __set_PRIMASK(primask);
    return true;
}

bool Vision_Link_Is_Online(void)
{
    vision_link_sample_t sample;
    return Vision_Link_Get_Sample(&sample);
}
