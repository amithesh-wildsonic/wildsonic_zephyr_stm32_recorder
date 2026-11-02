#ifndef _TASK_MANAGER_HPP
#define _TASK_MANAGER_HPP

#include "Task.hpp"
#include <stdint.h>

class TaskManager
{
  public:
    // Construct a new Task Manager object
    TaskManager() = default;

    // Initialize Task Manager object member variables
    void initialize();

    // Add task to TaskManager
    void add_task(Task &task, uint32_t weight = 1);

    // Executes tasks indefinitely
    void run_loop();

  private:
    static const uint32_t MAX_TASKS = 16; // Maximum supported tasks

    Task *                pTasks[MAX_TASKS];       // Task objects
    uint32_t              task_weights[MAX_TASKS]; // Run weight for each task
    uint32_t              num_tasks; // Number of task objected registered
    bool                  registration_completed; // Indicates whether
                                                  // registration period has
                                                  // ended
};

#endif // _TASK_MANAGER_HPP