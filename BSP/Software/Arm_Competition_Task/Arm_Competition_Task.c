#include "Arm_Competition_Task.h"

#include <math.h>

#include "Arm_Angle_Control.h"
#include "stm32h7xx_hal.h"

#define ARM_COMPETITION_TARGET_CM 5.0f
#define ARM_COMPETITION_TOLERANCE_CM 1.0f
#define ARM_COMPETITION_SETTLE_MS 200U

static arm_competition_task_id_t selected_task;
static arm_competition_state_t state;
static uint8_t arm_ready;
static uint32_t settled_since_ms;

static bool set_ball_target(float target_cm)
{
    arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();

    next.ball_position_loop_enabled = 1.0f;
    next.ball_velocity_loop_enabled = 1.0f;
    next.ball_target_position_cm = target_cm;
    return Arm_Angle_Control_Apply_Config(&next);
}

void Arm_Competition_Task_Init(void)
{
    selected_task = ARM_COMPETITION_TASK_NONE;
    state = ARM_COMPETITION_STATE_IDLE;
    arm_ready = 0U;
    settled_since_ms = 0U;
    Arm_Angle_Control_Set_Authorized(false);
}

void Arm_Competition_Task_Set_Ready(bool ready)
{
    arm_ready = ready ? 1U : 0U;
    if (!ready) Arm_Competition_Task_Cancel();
}

bool Arm_Competition_Task_Select(arm_competition_task_id_t task)
{
    if (task < ARM_COMPETITION_TASK_3 || task > ARM_COMPETITION_TASK_6) return false;
    Arm_Competition_Task_Cancel();
    selected_task = task;
    return true;
}

bool Arm_Competition_Task_Start(void)
{
    const arm_angle_control_debug_t *debug = Arm_Angle_Control_Get_Debug();
    float target_cm;

    if (arm_ready == 0U || selected_task == ARM_COMPETITION_TASK_NONE ||
        debug->vision_valid == 0U) return false;

    target_cm = selected_task == ARM_COMPETITION_TASK_3 ? ARM_COMPETITION_TARGET_CM :
                selected_task == ARM_COMPETITION_TASK_6 ? debug->vision_position_cm : 0.0f;
    if (!set_ball_target(target_cm)) return false;

    settled_since_ms = 0U;
    state = selected_task == ARM_COMPETITION_TASK_3 ? ARM_COMPETITION_STATE_TASK3_POSITIVE :
            ARM_COMPETITION_STATE_HOLD;
    Arm_Angle_Control_Set_Authorized(true);
    return true;
}

void Arm_Competition_Task_Cancel(void)
{
    state = ARM_COMPETITION_STATE_IDLE;
    settled_since_ms = 0U;
    Arm_Angle_Control_Set_Authorized(false);
    (void)Arm_Angle_Control_Return_To_Balance();
}

void Arm_Competition_Task_Process(void)
{
    const arm_angle_control_debug_t *debug;
    uint32_t now;

    if (state == ARM_COMPETITION_STATE_IDLE) return;
    debug = Arm_Angle_Control_Get_Debug();
    if (debug->enabled == 0U || debug->vision_valid == 0U)
    {
        Arm_Competition_Task_Cancel();
        return;
    }
    if (state != ARM_COMPETITION_STATE_TASK3_POSITIVE) return;

    if (fabsf(debug->vision_position_cm - ARM_COMPETITION_TARGET_CM) >
        ARM_COMPETITION_TOLERANCE_CM)
    {
        settled_since_ms = 0U;
        return;
    }

    now = HAL_GetTick();
    if (settled_since_ms == 0U)
    {
        settled_since_ms = now;
        return;
    }
    if ((uint32_t)(now - settled_since_ms) < ARM_COMPETITION_SETTLE_MS) return;

    if (set_ball_target(-ARM_COMPETITION_TARGET_CM))
    {
        state = ARM_COMPETITION_STATE_TASK3_NEGATIVE;
        settled_since_ms = 0U;
    }
    else
    {
        Arm_Competition_Task_Cancel();
    }
}

arm_competition_task_id_t Arm_Competition_Task_Get_Selected(void) { return selected_task; }
arm_competition_state_t Arm_Competition_Task_Get_State(void) { return state; }
