/**
 * @file    main.c
 * @brief   Application entry point.
 *
 * Delegates initialization and task logic to the App-layer scheduler
 * (App/task_scheduler.c).
 */

#include "stm32f10x.h"
#include "App/task_scheduler.h"

int main(void)
{
    Scheduler_Init();

    while (1)
    {
        Scheduler_Run();
    }
}
