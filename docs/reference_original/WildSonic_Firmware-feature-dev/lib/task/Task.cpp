#include "Task.hpp"
#include "assert.hpp"

/**
 * @brief
 * Initialize member variables for Task class
 */
void Task::initialize()
{
    task_state = TaskState::Awake;
}

/**
 * @brief
 * Run task specified number of times
 *
 * @param weight number of times to call task
 */
void Task::run_for_count(uint32_t weight)
{
    SYSTEM_ASSERT_TRUE(is_awake());

    // capture the function pointer once to avoid going through the vtable FIXME
    TaskErrorCode (Task::*task_func)() = &Task::run;

    for (uint32_t iter = 0; iter < weight; iter++)
    {
        TaskErrorCode code = (*this.*task_func)();

        switch (code)
        {
            case TaskErrorCode::NoError:
                break;
            case TaskErrorCode::Sleep:
                task_state = TaskState::Asleep;
                // FALL-THROUGH
            case TaskErrorCode::Yield:
                return;
            default:
                SYSTEM_ASSERT_TRUE(false);
                break;
        }
    }
}
