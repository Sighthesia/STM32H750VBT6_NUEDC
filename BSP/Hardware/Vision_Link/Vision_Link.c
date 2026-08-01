#include "Vision_Link.h"

#include "main.h"
#include "usart.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VISION_LINK_LINE_MAX_LENGTH 48U
#define VISION_LINK_TIMEOUT_MS 100U
#define VISION_LINK_MAX_POSITION_STEP_CM 8.0f

static uint8_t vision_rx_byte;
static uint8_t vision_rx_line[VISION_LINK_LINE_MAX_LENGTH];
static uint8_t vision_ready_line[VISION_LINK_LINE_MAX_LENGTH];
static volatile uint8_t vision_rx_length;
static volatile uint8_t vision_ready_length;
static volatile uint8_t vision_discard_until_newline;
static volatile uint8_t vision_link_frame_valid;
static volatile vision_link_sample_t vision_link_sample;
static volatile vision_link_diagnostic_t vision_link_diagnostic;
static float kalman_position;
static float kalman_velocity;
static float kalman_covariance[2][2];
static uint8_t kalman_initialized;
static uint8_t kalman_enabled = 1U;
static uint32_t kalman_last_update_ms;
static float kalman_measurement_noise = 0.02f;
static float kalman_process_noise_position = 0.02f;
static float kalman_process_noise_velocity = 0.3f;

void Vision_Link_Set_Kalman_Parameters(float measurement_noise, float process_noise_position, float process_noise_velocity)
{
    kalman_measurement_noise = measurement_noise;
    kalman_process_noise_position = process_noise_position;
    kalman_process_noise_velocity = process_noise_velocity;
}

static void Vision_Link_Reset_Kalman_State(float position)
{
    kalman_position = position;
    kalman_velocity = 0.0f;
    kalman_covariance[0][0] = 1.0f;
    kalman_covariance[0][1] = 0.0f;
    kalman_covariance[1][0] = 0.0f;
    kalman_covariance[1][1] = 25.0f;
    kalman_initialized = 1U;
}

static void Vision_Link_Update_Kalman(float measurement, float dt)
{
    float predicted_position;
    float predicted_covariance00;
    float predicted_covariance01;
    float predicted_covariance10;
    float predicted_covariance11;
    float innovation;
    float innovation_covariance;
    float gain_position;
    float gain_velocity;

    if (kalman_initialized == 0U)
    {
        Vision_Link_Reset_Kalman_State(measurement);
        return;
    }
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;
    predicted_position = kalman_position + dt * kalman_velocity;
    predicted_covariance00 = kalman_covariance[0][0] + dt * (kalman_covariance[1][0] + kalman_covariance[0][1]) +
                             dt * dt * kalman_covariance[1][1] + kalman_process_noise_position;
    predicted_covariance01 = kalman_covariance[0][1] + dt * kalman_covariance[1][1];
    predicted_covariance10 = kalman_covariance[1][0] + dt * kalman_covariance[1][1];
    predicted_covariance11 = kalman_covariance[1][1] + kalman_process_noise_velocity * dt;
    innovation = measurement - predicted_position;
    innovation_covariance = predicted_covariance00 + kalman_measurement_noise;
    gain_position = predicted_covariance00 / innovation_covariance;
    gain_velocity = predicted_covariance10 / innovation_covariance;
    kalman_position = predicted_position + gain_position * innovation;
    kalman_velocity += gain_velocity * innovation;
    kalman_covariance[0][0] = (1.0f - gain_position) * predicted_covariance00;
    kalman_covariance[0][1] = (1.0f - gain_position) * predicted_covariance01;
    kalman_covariance[1][0] = predicted_covariance10 - gain_velocity * predicted_covariance00;
    kalman_covariance[1][1] = predicted_covariance11 - gain_velocity * predicted_covariance01;
}

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
    Vision_Link_Reset_Kalman();
    memset((void *)&vision_link_diagnostic, 0, sizeof(vision_link_diagnostic));
    (void)HAL_UART_Receive_IT(&huart3, &vision_rx_byte, 1U);
}

void Vision_Link_Set_Kalman_Enabled(bool enabled)
{
    kalman_enabled = enabled ? 1U : 0U;
    Vision_Link_Reset_Kalman();
}

void Vision_Link_Reset_Kalman(void)
{
    kalman_position = 0.0f;
    kalman_velocity = 0.0f;
    kalman_covariance[0][0] = 1.0f;
    kalman_covariance[0][1] = 0.0f;
    kalman_covariance[1][0] = 0.0f;
    kalman_covariance[1][1] = 25.0f;
    kalman_initialized = 0U;
    kalman_last_update_ms = 0U;
}

void Vision_Link_UART_RxCplt_Callback(void)
{
    const uint8_t byte = vision_rx_byte;
    vision_link_diagnostic.received_bytes++;

    if (byte == '\r' || byte == '\n')
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

    (void)HAL_UART_Receive_IT(&huart3, &vision_rx_byte, 1U);
}

void Vision_Link_UART_Error_Callback(void)
{
    uint32_t error = HAL_UART_GetError(&huart3);

    vision_link_diagnostic.uart_errors++;
    vision_link_diagnostic.last_uart_error = error;
    vision_link_diagnostic.last_uart_isr = huart3.Instance->ISR;
    vision_rx_length = 0U;
    vision_discard_until_newline = 0U;
    if ((error & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO)) != 0U &&
        HAL_UART_Receive_IT(&huart3, &vision_rx_byte, 1U) != HAL_OK)
    {
        vision_link_diagnostic.restart_failures++;
    }
}

void Vision_Link_Process(void)
{
    char line[VISION_LINK_LINE_MAX_LENGTH];
    uint8_t length;
    uint32_t primask;
    float position_cm = 0.0f;
    float velocity_cm_s = 0.0f;
    float filtered_position;
    float filtered_velocity;
    uint32_t received_ms;

    primask = __get_PRIMASK();
    __disable_irq();
    length = vision_ready_length;
    if (length != 0U)
    {
        memcpy(line, vision_ready_line, (size_t)length + 1U);
        vision_ready_length = 0U;
    }
    __set_PRIMASK(primask);

    if (length == 0U || !Vision_Link_Parse_Line(line, &position_cm, &velocity_cm_s))
    {
        if (length != 0U) vision_link_diagnostic.parse_errors++;
        return;
    }

    received_ms = HAL_GetTick();
    if (kalman_enabled != 0U)
    {
        if (kalman_initialized != 0U && fabsf(position_cm - kalman_position) > VISION_LINK_MAX_POSITION_STEP_CM)
            Vision_Link_Reset_Kalman_State(position_cm);
        Vision_Link_Update_Kalman(position_cm, kalman_last_update_ms == 0U ? 0.05f :
                                   (float)(received_ms - kalman_last_update_ms) * 0.001f);
        filtered_position = kalman_position;
        filtered_velocity = kalman_velocity;
        kalman_last_update_ms = received_ms;
    }
    else
    {
        filtered_position = position_cm;
        filtered_velocity = velocity_cm_s;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    vision_link_sample.position_cm = filtered_position;
    vision_link_sample.velocity_cm_s = filtered_velocity;
    vision_link_sample.received_ms = received_ms;
    vision_link_frame_valid = 1U;
    vision_link_diagnostic.valid_frames++;
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

bool Vision_Link_Get_Last_Sample(vision_link_sample_t *sample)
{
    uint32_t primask;

    if (sample == NULL) return false;
    primask = __get_PRIMASK();
    __disable_irq();
    if (vision_link_frame_valid == 0U)
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

void Vision_Link_Get_Diagnostic(vision_link_diagnostic_t *diagnostic)
{
    uint32_t primask;

    if (diagnostic == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *diagnostic = vision_link_diagnostic;
    __set_PRIMASK(primask);
}
