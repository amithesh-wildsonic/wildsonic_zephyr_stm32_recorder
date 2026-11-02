#ifndef _DETECT_TASK_HPP
#define _DETECT_TASK_HPP

#include "FifoQueue.hpp"
#include "Messages.hpp"
#include "Task.hpp"

class DetectTask :
    public Task
{
  public:
    // Default constructor
    DetectTask() = default;

    // Set up and initialization for Detect Task
    void          initialize(queues_info_t &queues_info);

    // Main routine for Detect Task
    TaskErrorCode run() override;

  private:
    DetectRequestMsg *m_p_detect_msgs; // Array of detect request messages
    DetectFifoQueue * m_p_detect_request_queue; // request queue from audio task
    DetectFifoQueue * m_p_detect_response_queue; // response queue to audio task
};

#endif // _DETECT_TASK_HPP