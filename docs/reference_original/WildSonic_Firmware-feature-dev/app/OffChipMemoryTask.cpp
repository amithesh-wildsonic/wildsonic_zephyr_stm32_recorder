#if __CORE__ == OFF_CHIP_MEMORY_CORE
#include "OffChipMemoryTask.hpp"
#include "assert.hpp"
#include "shared.hpp"

#include "w_hal_ocm.hpp"

/**
 * @brief
 * Set up and initialization for Off-Chip Memory Task
 */
void OffChipMemoryTask::initialize(queues_info_t &queues_info)
{
    Task::initialize();

    m_read_write                 = true;

    m_request_tracker.state      = State::WAIT_FOR_REQUEST;
    m_request_tracker.request_id = INVALID_OCM_MSG_ID;

    m_p_ocm_msgs                 = queues_info.ocm_msgs;

    m_p_ocm_read_request_queue   = &queues_info.ocm_read_request_queue;
    m_p_ocm_write_request_queue  = &queues_info.ocm_write_request_queue;
    m_p_ocm_response_queue       = &queues_info.ocm_response_queue;
}

/**
 * @brief
 * Main routine for Off-Chip Memory Task
 *
 * @return TaskErrorCode Sleep if idle and not waiting on response,
 * Yield if idle, NoError otherwise
 */
TaskErrorCode OffChipMemoryTask::run()
{
    TaskErrorCode task_error = TaskErrorCode::NoError;
    wHalErrorCode error;

    error                              = w_hal_ocm_handler();

    const wHalErrorCode valid_errors[] = {wHalErrorCode::NO_ERROR,
                                          wHalErrorCode::BUSY};
    SYSTEM_ASSERT_IN_SET(error,
                         valid_errors,
                         sizeof(valid_errors) / sizeof(wHalErrorCode));

    if (error == wHalErrorCode::BUSY)
    {
        return task_error;
    }

    switch (m_request_tracker.state)
    {
        case State::WAIT_FOR_REQUEST:
        {
            SYSTEM_ASSERT_EQUAL(m_request_tracker.request_id,
                                INVALID_OCM_MSG_ID);

            if (m_read_write)
            {
                // Pop from write queue first
                m_p_ocm_write_request_queue->pop(m_request_tracker.request_id);

                if (m_request_tracker.request_id == INVALID_OCM_MSG_ID)
                {
                    // Try read queue
                    m_p_ocm_read_request_queue->pop(
                        m_request_tracker.request_id);
                }
            }
            else
            {
                // Pop from read queue first
                m_p_ocm_read_request_queue->pop(m_request_tracker.request_id);

                if (m_request_tracker.request_id == INVALID_OCM_MSG_ID)
                {
                    // Try write queue
                    m_p_ocm_write_request_queue->pop(
                        m_request_tracker.request_id);
                }
            }

            if (m_request_tracker.request_id == INVALID_OCM_MSG_ID)
            {
                // Nothing new to process
                task_error = TaskErrorCode::Yield;
                break;
            }

            m_request_tracker.state = State::HAL_REQUEST;
        }
        // FALL-THROUGH
        case State::HAL_REQUEST:
        {

            OCMRequestMsg *p_request =
                &m_p_ocm_msgs[m_request_tracker.request_id];

            buffer_id_t buffer_id;
            uint16_t    size;

            p_request->get_buffer(buffer_id, size);

            uint8_t *p_buffer = get_buffer_address(buffer_id);

            if (p_request->is_write())
            {
                error = w_hal_ocm_write_data(p_request->get_external_address(),
                                             p_buffer,
                                             size);
                SYSTEM_ASSERT_EQUAL(error,
                                    wHalErrorCode::OCM_WRITE_IN_PROGRESS);

                // Toggle read
                m_read_write = false;
            }
            else
            {
                error = w_hal_ocm_read_data(p_request->get_external_address(),
                                            p_buffer,
                                            size);
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::OCM_READ_IN_PROGRESS);

                // Toggle write
                m_read_write = true;
            }

            m_request_tracker.state = State::WAIT_FOR_COMPLETION;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_COMPLETION:
        {
            error = w_hal_ocm_check_status();
            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::OCM_COMMAND_DONE);

            error = w_hal_ocm_clear_completion_status();
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            OCMRequestMsg *p_request =
                &m_p_ocm_msgs[m_request_tracker.request_id];
            p_request->set_status(MsgStatus::GOOD);
            m_p_ocm_response_queue->push(m_request_tracker.request_id);

            m_request_tracker.request_id = INVALID_OCM_MSG_ID;
            m_request_tracker.state      = State::WAIT_FOR_REQUEST;

            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    switch (shared_glbl.shutdown_stage)
    {
        case shutdown_stage_t::Stage_3:
        {
            // Check for OCM read and write request queue to be empty and go to
            // sleep
            SYSTEM_ASSERT_TRUE(m_p_ocm_read_request_queue->isEmpty());
            SYSTEM_ASSERT_TRUE(m_p_ocm_write_request_queue->isEmpty());
            SYSTEM_ASSERT_EQUAL(m_request_tracker.state,
                                State::WAIT_FOR_REQUEST);
            task_error                 = TaskErrorCode::Sleep;
            shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                static_cast<int>(shared_glbl.shutdown_stage) + 1);
        }
        default:
        {
            break;
        }
    }

    return task_error;
}

#endif // __CORE__ == OFF_CHIP_MEMORY_CORE