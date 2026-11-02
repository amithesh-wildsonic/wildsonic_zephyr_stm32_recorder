#if __CORE__ == DETECT_CORE

#include "DetectTask.hpp"
#include "shared.hpp"

/**
 * @brief
 * Set up and initialization for Detect Task
 *
 * @param queues_info queue information
 */
void DetectTask::initialize(queues_info_t &queues_info)
{
    Task::initialize();

    // Save pointer to message array
    m_p_detect_msgs           = queues_info.detect_msgs;

    // Save queue pointers
    m_p_detect_request_queue  = &queues_info.detect_request_queue;
    m_p_detect_response_queue = &queues_info.detect_response_queue;
}

/**
 * @brief
 * Main routine for Detect Task
 *
 * @return TaskErrorCode NoError
 */
TaskErrorCode DetectTask::run()
{
    const uint32_t      MAX_TRANSFERS_FOR_DETECTION = 4;
    static uint32_t     transfer_count              = 0;
    // TODO: Real Implementation
    // For now, just return response
    detect_request_id_t detect_request_id;
    if (m_p_detect_request_queue->pop(detect_request_id))
    {
        DetectRequestMsg *p_detect_request =
            &m_p_detect_msgs[detect_request_id];

        MsgStatus status;
        if (p_detect_request->is_detect_msg())
        {
            SYSTEM_ASSERT_EQUAL(transfer_count, 0);
            status = MsgStatus::DETECT_SOUND_EVENT_DETECTED;
        }
        else
        {
            if (++transfer_count == MAX_TRANSFERS_FOR_DETECTION)
            {
                status         = MsgStatus::DETECT_BUFFER_FULLY_LOADED;
                transfer_count = 0;
            }
            else
            {
                status = MsgStatus::GOOD;
            }
        }

        p_detect_request->set_status(status);

        m_p_detect_response_queue->push(detect_request_id);
    }

    switch (shared_glbl.shutdown_stage)
    {
        case shutdown_stage_t::Stage_4:
        {
            // Check for Detect request queue to be empty and go to
            // sleep
            SYSTEM_ASSERT_TRUE(m_p_detect_request_queue->isEmpty())
            shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                static_cast<int>(shared_glbl.shutdown_stage) + 1);
            return TaskErrorCode::Sleep;
        }
        default:
        {
            break;
        }
    }

    return TaskErrorCode::NoError;
}

#endif // __CORE__ == DETECT_CORE