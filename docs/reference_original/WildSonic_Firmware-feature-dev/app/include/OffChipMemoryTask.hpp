#ifndef _OFF_CHIP_MEMORY_TASK
#define _OFF_CHIP_MEMORY_TASK

#include "BitwisePool.hpp"
#include "Messages.hpp"
#include "Task.hpp"
#include "constants.hpp"

/**
 * @brief
 * Task for writing and reading off-chip memory
 */
class OffChipMemoryTask :
    public Task
{
  public:
    // Default constructor
    OffChipMemoryTask() = default;

    // Set up and initialization for Off-Chip Memory Task
    void          initialize(queues_info_t &queues_info);

    // Main routine for Off-Chip Memory Task
    TaskErrorCode run() override;

  private:
    /**
     * @brief State of execution in OCM Task
     */
    enum class State : uint8_t
    {
        WAIT_FOR_REQUEST,    // Initial State - waiting for request
        HAL_REQUEST,         // Make request to HAL to write external memory
        WAIT_FOR_COMPLETION, // Wait for HAL completion
    };

    /**
     * @brief
     * Tracker information for managing incoming requests
     */
    struct request_tracker_st
    {
        State            state;      // state of request
        ocm_request_id_t request_id; // Id for current
                                     // ocm request
    };

    bool               m_read_write; // Indicates whether
                                     // to process read
                                     // or write request next

    request_tracker_st m_request_tracker; // Request tracker

    OCMRequestMsg *    m_p_ocm_msgs; // Array of OCM request messages

    OCMFifoQueue *     m_p_ocm_read_request_queue;  // read request queue
                                                    // from audio task
    OCMFifoQueue *     m_p_ocm_write_request_queue; // write request queue
                                                    // from audio task

    OCMFifoQueue *     m_p_ocm_response_queue; // response queue from
                                               // audio task
};

#endif // _OFF_CHIP_MEMORY_TASK