#ifndef ARM_COMPETITION_TASK_H
#define ARM_COMPETITION_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ARM_COMPETITION_TASK_NONE = 0,
    ARM_COMPETITION_TASK_3 = 3,
    ARM_COMPETITION_TASK_4 = 4,
    ARM_COMPETITION_TASK_5 = 5,
    ARM_COMPETITION_TASK_6 = 6
} arm_competition_task_id_t;

typedef enum
{
    ARM_COMPETITION_STATE_IDLE = 0,
    ARM_COMPETITION_STATE_TASK3_POSITIVE,
    ARM_COMPETITION_STATE_TASK3_NEGATIVE,
    ARM_COMPETITION_STATE_HOLD
} arm_competition_state_t;

void Arm_Competition_Task_Init(void);
void Arm_Competition_Task_Process(void);
void Arm_Competition_Task_Set_Ready(bool ready);
bool Arm_Competition_Task_Select(arm_competition_task_id_t task);
bool Arm_Competition_Task_Start(void);
void Arm_Competition_Task_Cancel(void);
arm_competition_task_id_t Arm_Competition_Task_Get_Selected(void);
arm_competition_state_t Arm_Competition_Task_Get_State(void);

#endif /* ARM_COMPETITION_TASK_H */
