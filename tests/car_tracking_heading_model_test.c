#include "Car_Tracking_Model.h"
#include "Car_Attitude_Control.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(CAR_TRACKING_METHOD_OPTICAL_IMU == 0);
    assert(CAR_TRACKING_METHOD_CENTER_LOCK == 1);
    assert(CAR_TRACKING_METHOD_GEOMETRY == 2);
    assert(CAR_TRACKING_METHOD_DIRECT_ANGLE == 3);
    assert(CAR_TRACKING_METHOD_DIRECT_RATE == 4);
    assert(CAR_TRACKING_METHOD_OPEN_LOOP == 5);
    assert(CAR_TRACKING_LOST_HOLD_THEN_SLOW == 0);
    assert(CAR_TRACKING_LOST_HOLD == 1);
    assert(CAR_TRACKING_LOST_STRAIGHT_SLOW == 2);

    const float layer1 = 7.0f;
    const float layer2 = 25.0f;
    const float layer3 = 55.0f;
    const float layer4 = 101.0f;

    const float lookahead_s = 0.05f;

    assert(Car_Tracking_Target_Offset(0x01U, layer1, layer2, layer3, layer4, lookahead_s) == 5.05f);
    assert(Car_Tracking_Target_Offset(0x02U, layer1, layer2, layer3, layer4, lookahead_s) == 2.75f);
    assert(Car_Tracking_Target_Offset(0x04U, layer1, layer2, layer3, layer4, lookahead_s) == 1.25f);
    assert(Car_Tracking_Target_Offset(0x08U, layer1, layer2, layer3, layer4, lookahead_s) == 0.35f);
    assert(Car_Tracking_Target_Offset(0x10U, layer1, layer2, layer3, layer4, lookahead_s) == -0.35f);
    assert(Car_Tracking_Target_Offset(0x20U, layer1, layer2, layer3, layer4, lookahead_s) == -1.25f);
    assert(Car_Tracking_Target_Offset(0x40U, layer1, layer2, layer3, layer4, lookahead_s) == -2.75f);
    assert(Car_Tracking_Target_Offset(0x80U, layer1, layer2, layer3, layer4, lookahead_s) == -5.05f);
    assert(Car_Tracking_Target_Offset(0x00U, layer1, layer2, layer3, layer4, lookahead_s) == 0.0f);
    assert(Car_Tracking_Target_Offset(0x18U, layer1, layer2, layer3, layer4, lookahead_s) == 0.0f);
    assert(Car_Tracking_Target_Offset(0x81U, layer1, layer2, layer3, layer4, lookahead_s) == 0.0f);
    assert(Car_Tracking_Target_Offset(0x03U, layer1, layer2, layer3, layer4, lookahead_s) == 7.80f);
    assert(Car_Tracking_Target_Offset(0xC0U, layer1, layer2, layer3, layer4, lookahead_s) == -7.80f);
    assert(Car_Tracking_Method_Target_Offset(0x19U, 0U, layer1, layer2, layer3, layer4,
                                             lookahead_s) == 5.05f);
    assert(Car_Tracking_Method_Target_Offset(0x19U, 1U, layer1, layer2, layer3, layer4,
                                             lookahead_s) == 0.0f);
    assert(Car_Tracking_Method_Target_Offset(0x08U, 1U, layer1, layer2, layer3, layer4,
                                             lookahead_s) == 0.35f);
    assert(Car_Tracking_Direct_Command(0x08U, 7.0f, 25.0f, 60.0f, 101.0f) == 7.0f);
    assert(Car_Tracking_Direct_Command(0x10U, 7.0f, 25.0f, 60.0f, 101.0f) == -7.0f);
    assert(Car_Tracking_Direct_Command(0x04U, 7.0f, 25.0f, 60.0f, 101.0f) == 25.0f);
    assert(Car_Tracking_Direct_Command(0x40U, 7.0f, 25.0f, 60.0f, 101.0f) == -60.0f);
    assert(Car_Tracking_Direct_Command(0x01U, 7.0f, 25.0f, 60.0f, 101.0f) == 101.0f);
    assert(Car_Tracking_Direct_Command(0x80U, 7.0f, 25.0f, 60.0f, 101.0f) == -101.0f);
    assert(Car_Tracking_Direct_Command(0x18U, 7.0f, 25.0f, 60.0f, 101.0f) == 0.0f);
    assert(Car_Tracking_Lost_Command(60.0f, 0U, 200U, 150U) == 60.0f);
    assert(Car_Tracking_Lost_Command(-60.0f, 1U, 1000U, 150U) == -60.0f);
    assert(Car_Tracking_Lost_Command(60.0f, 2U, 0U, 150U) == 0.0f);
    assert(Car_Tracking_Lost_Speed_Scale(0U, 149U, 150U, 0.5f) == 1.0f);
    assert(Car_Tracking_Lost_Speed_Scale(0U, 150U, 150U, 0.5f) == 0.5f);
    assert(Car_Tracking_Lost_Speed_Scale(1U, 1000U, 150U, 0.5f) == 1.0f);
    assert(Car_Tracking_Lost_Speed_Scale(2U, 0U, 150U, 0.5f) == 0.5f);
    assert(Car_Tracking_Display_Correction(0.35f) == 0.35f);
    assert(Car_Tracking_Display_Correction(-0.35f) == -0.35f);
    assert(Car_Tracking_Display_Correction(0.0f) == 0.0f);
    assert(Car_Tracking_Approach(0.0f, 30.0f, 3.0f) == 3.0f);
    assert(Car_Tracking_Approach(30.0f, 0.0f, 3.0f) == 27.0f);
    assert(Car_Tracking_Approach(10.0f, 11.0f, 3.0f) == 11.0f);
    assert(Car_Tracking_Arc_Target_Rate(30.0f, 100.0f, 15.0f) == 45.0f);
    assert(Car_Tracking_Arc_Target_Rate(30.0f, -100.0f, 15.0f) == 15.0f);

    puts("car tracking heading model tests passed");
    return 0;
}
