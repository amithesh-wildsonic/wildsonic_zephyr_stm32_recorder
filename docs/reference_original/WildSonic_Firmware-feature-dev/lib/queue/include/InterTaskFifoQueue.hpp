#ifndef _INTER_TASK_FIFO_QUEUE_HPP
#define _INTER_TASK_FIFO_QUEUE_HPP

#include "FifoQueue.hpp"
#include "Task.hpp"

/**
 * @brief
 * Queue that is used between
 * @tparam D data type
 * @tparam size max number of queue elements
 */
template <typename D, unsigned long size>
class InterTaskFifoQueue :
    public FifoQueue<D, size>
{
  public:
    // Default constructor
    InterTaskFifoQueue() = default;

    // Initialize Fifo Queue
    void initialize(Task &task);

    // Enqueue an entry into the fifo
    bool push(D value);

  private:
    Task *pTask; // Consuming Task
};

/**
 * @brief
 * Initialize queue with task
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param task consuming task of fifo
 */
template <typename D, unsigned long size>
void InterTaskFifoQueue<D, size>::initialize(Task &task)
{
    FifoQueue<D, size>::initialize();
    pTask = &task;
}

/**
 * @brief
 * Enqueue an entry into the fifo and wakes the consuming Task
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param value value to push into the fifo
 * @return true if successful
 * @return false if fifo is too full to enqueue any more entries
 */
template <typename D, unsigned long size>
inline bool InterTaskFifoQueue<D, size>::push(D value)
{
    bool success = FifoQueue<D, size>::push(value);

    if (success)
    {
        pTask->wake();
    }

    return success;
}

#endif // _INTER_TASK_FIFO_QUEUE_HPP