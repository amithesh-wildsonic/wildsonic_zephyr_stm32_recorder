#include "TaskManager.hpp"
#include "assert.hpp"
#include "shared.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_system.hpp"

#include <string.h>

/**
 * @brief
 * Initialize Task Manager object member variables
 */
void TaskManager::initialize()
{
    num_tasks              = 0;
    registration_completed = false;

    memset(task_weights, 0, sizeof(task_weights));
    memset(pTasks, 0, sizeof(pTasks));
}

/**
 * @brief
 * Add task to TaskManager
 *
 * @param task task to register
 * @param weight number of invocations to be made to task per iteration
 */
void TaskManager::add_task(Task &task, uint32_t weight)
{
    SYSTEM_ASSERT_LT(num_tasks, MAX_TASKS);
    SYSTEM_ASSERT_FALSE(registration_completed);
    SYSTEM_ASSERT_GT(weight, 0);

    task_weights[num_tasks] = weight;
    pTasks[num_tasks]       = &task;
    ++num_tasks;
}

/**
 * @brief
 * Executes tasks indefinitely. This function will never return.
 */
void TaskManager::run_loop()
{
    registration_completed = true;
    time_t last_time       = w_hal_rtc_get_time();

    while (true)
    {
#if __CORE__ == RTC_CORE
        // To apply the new RTC time set by console command.
        wHalErrorCode rtc_error = w_hal_rtc_post_init();
        SYSTEM_ASSERT_EQUAL(rtc_error, wHalErrorCode::NO_ERROR);
#endif

        if (w_hal_rtc_get_time() - last_time >= 1)
        {
            last_time = w_hal_rtc_get_time();
            w_hal_system_check_stack_guard();
        }

        // Counter for number of tasks running
        uint32_t num_awake_tasks = 0;

        for (uint32_t task_iter = 0; task_iter < num_tasks; task_iter++)
        {
            SYSTEM_ASSERT_NOT_EQUAL(pTasks[task_iter],
                                    static_cast<Task *>(nullptr));

            if (pTasks[task_iter]->is_awake())
            {
                pTasks[task_iter]->run_for_count(task_weights[task_iter]);
                num_awake_tasks++;
            }
        }

        // When no tasks are running or all the tasks went to sleep, then
        // num_awake_tasks will be zero and so check for reboot flag, If it is
        // set than exit from this continuous loop, and go to main function to
        // terminate all the HAL components and for re-initialization.
        if (num_awake_tasks == 0)
        {
            if (shared_glbl.reboot_flag)
            {
                break;
            }
        }
    }
}
