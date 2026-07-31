#include "Car_Tracking_Model.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
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
    assert(Car_Tracking_Display_Correction(0.35f) == 0.35f);
    assert(Car_Tracking_Display_Correction(-0.35f) == -0.35f);
    assert(Car_Tracking_Display_Correction(0.0f) == 0.0f);

    puts("car tracking heading model tests passed");
    return 0;
}
