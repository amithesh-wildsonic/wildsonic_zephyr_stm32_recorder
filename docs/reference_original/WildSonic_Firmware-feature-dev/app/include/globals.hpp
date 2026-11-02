#ifndef _GLOBALS_HPP
#define _GLOBALS_HPP

#include "AdminTask.hpp"
#if __CORE__ == AUDIO_CORE
#include "AudioTask.hpp"
#endif
#if __CORE__ == COMM_CORE
#include "CommTask.hpp"
#endif
#if __CORE__ == DETECT_CORE
#include "DetectTask.hpp"
#endif
#if __CORE__ == OFF_CHIP_MEMORY_CORE
#include "OffChipMemoryTask.hpp"
#endif
#if __CORE__ == STORAGE_CORE
#include "StorageTask.hpp"
#endif
#include "TaskManager.hpp"

/**
 * @brief
 * Structure containing all global variables
 */
struct globals_t
{
    TaskManager task_manager; // Task manager
    AdminTask   admin_task;   // Admin Task (one on every core)
#if __CORE__ == AUDIO_CORE
    AudioTask audio_task; // Audio Task
#endif
#if __CORE__ == COMM_CORE
    CommTask comm_task; // Communication Task
#endif
#if __CORE__ == DETECT_CORE
    DetectTask detect_task; // Detection Task
#endif
#if __CORE__ == OFF_CHIP_MEMORY_CORE
    OffChipMemoryTask ocm_task; // Off-Chip Memory Task
#endif
#if __CORE__ == STORAGE_CORE
    StorageTask storage_task; // Task for saving to file
#endif
};

extern globals_t glbl;

#endif // _GLOBALS_HPP