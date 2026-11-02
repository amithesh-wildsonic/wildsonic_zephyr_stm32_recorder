#ifndef _TASK_HPP
#define _TASK_HPP

#include <stdint.h>

/**
 * @brief
 * Result from running one call of task
 */
enum class TaskErrorCode : uint8_t
{
    NoError, // task run successful
    Yield,   // task has no work and should yield to other tasks for time quota
    Sleep,   // task has no work and should not run at all
};

/**
 * @brief
 * State of task operation
 */
enum class TaskState : uint8_t
{
    Asleep,
    Awake,
};

/**
 * @brief
 * Interface template and base class for Tasks
 */
class Task
{
  public:
    // Default constructor
    Task() = default;

    // Initialize member variables for Task class
    void                  initialize();

    // Run task specified number of times
    void                  run_for_count(uint32_t weight);

    // Run one pass of task
    virtual TaskErrorCode run() = 0;

    // Indicates if task is allowed to run
    bool                  is_awake() const;

    // Allow task to be executable
    void                  wake();

  private:
    TaskState task_state; // Indicates whether task is running
};

/**
 * @brief
 * Indicates if task is allowed to run
 * @return true task can run
 * @return false otherwise
 */
inline bool Task::is_awake() const
{
    return (task_state == TaskState::Awake);
}

/**
 * @brief
 * Allow task to be executable
 */
inline void Task::wake()
{
    task_state = TaskState::Awake;
}

#endif // _TASK_HPP