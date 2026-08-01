#ifndef CAR_TRACKING_MODEL_H
#define CAR_TRACKING_MODEL_H

#include <stdint.h>

static inline float Car_Tracking_Target_Offset(uint8_t sensor_state,
                                               float layer1_weight,
                                               float layer2_weight,
                                               float layer3_weight,
                                               float layer4_weight,
                                               float lookahead_time_s)
{
    float heading_rate = 0.0f;

    if ((sensor_state & 0x01U) != 0U) heading_rate += layer4_weight;
    if ((sensor_state & 0x02U) != 0U) heading_rate += layer3_weight;
    if ((sensor_state & 0x04U) != 0U) heading_rate += layer2_weight;
    if ((sensor_state & 0x08U) != 0U) heading_rate += layer1_weight;
    if ((sensor_state & 0x10U) != 0U) heading_rate -= layer1_weight;
    if ((sensor_state & 0x20U) != 0U) heading_rate -= layer2_weight;
    if ((sensor_state & 0x40U) != 0U) heading_rate -= layer3_weight;
    if ((sensor_state & 0x80U) != 0U) heading_rate -= layer4_weight;
    return heading_rate * lookahead_time_s;
}

static inline float Car_Tracking_Method_Target_Offset(uint8_t sensor_state,
                                                      uint8_t method,
                                                      float layer1_weight,
                                                      float layer2_weight,
                                                      float layer3_weight,
                                                      float layer4_weight,
                                                      float lookahead_time_s)
{
    if (method == 1U && (sensor_state & 0x18U) == 0x18U) return 0.0f;
    return Car_Tracking_Target_Offset(sensor_state, layer1_weight, layer2_weight,
                                      layer3_weight, layer4_weight, lookahead_time_s);
}

static inline float Car_Tracking_Direct_Command(uint8_t sensor_state,
                                                float layer1,
                                                float layer2,
                                                float layer3,
                                                float layer4)
{
    return Car_Tracking_Target_Offset(sensor_state, layer1, layer2, layer3, layer4, 1.0f);
}

static inline float Car_Tracking_Lost_Command(float last_command,
                                              uint8_t strategy,
                                              uint32_t elapsed_ms,
                                              uint32_t hold_ms)
{
    if (strategy == 2U) return 0.0f;
    (void)elapsed_ms;
    (void)hold_ms;
    return last_command;
}

static inline float Car_Tracking_Lost_Speed_Scale(uint8_t strategy,
                                                  uint32_t elapsed_ms,
                                                  uint32_t hold_ms,
                                                  float slow_ratio)
{
    if (strategy == 1U) return 1.0f;
    if (strategy == 2U || elapsed_ms >= hold_ms) return slow_ratio;
    return 1.0f;
}

static inline float Car_Tracking_Display_Correction(float target_offset)
{
    return target_offset;
}

static inline float Car_Tracking_Approach(float current, float target, float maximum_step)
{
    if (target > current + maximum_step) return current + maximum_step;
    if (target < current - maximum_step) return current - maximum_step;
    return target;
}

static inline float Car_Tracking_Arc_Target_Rate(float feedforward_rate,
                                                 float optical_rate,
                                                 float optical_limit)
{
    if (optical_rate > optical_limit) optical_rate = optical_limit;
    if (optical_rate < -optical_limit) optical_rate = -optical_limit;
    return feedforward_rate + optical_rate;
}

#endif /* CAR_TRACKING_MODEL_H */
