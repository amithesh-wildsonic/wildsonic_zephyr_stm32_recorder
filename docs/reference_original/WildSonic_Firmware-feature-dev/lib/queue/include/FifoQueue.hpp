#ifndef _FIFO_QUEUE_HPP
#define _FIFO_QUEUE_HPP

#include <stdint.h>

/**
 * @brief
 * Circular Buffer queue
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 */
template <typename D, unsigned long size>
class FifoQueue
{
  public:
    // Construct a new Fifo Queue object
    FifoQueue() : head(0), tail(0){};

    // Initialize fifo pointers
    void initialize();

    // Check if fifo is empty
    bool isEmpty() const;

    // Check if fifo is full
    bool isFull() const;

    // Enqueue an entry into the fifo
    bool push(D value);

    // Enqueue an entry into the fifo
    bool push_front(D value);

    // Dequeue an entry from the fifo
    bool pop(D &value);

  private:
    // Decrement fifo pointer value
    uint32_t decrementPointer(uint32_t ptr) const;

    // Increment fifo pointer value
    uint32_t incrementPointer(uint32_t ptr) const;

    uint32_t head;             // consumer pointer
    uint32_t tail;             // producer pointer
    D        memory[size + 1]; // circular buffer memory
};

/**
 * @brief
 * Initialize fifo pointers
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 */
template <typename D, unsigned long size>
inline void FifoQueue<D, size>::initialize()
{
    head = tail = 0;
}

/**
 * @brief
 * Decrement fifo pointer value
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param ptr pointer value
 * @return uint32_t next pointer value accounting for wrap-around conditions
 */
template <typename D, unsigned long size>
inline uint32_t FifoQueue<D, size>::decrementPointer(uint32_t ptr) const
{
    if (ptr == 0)
    {
        ptr = size;
    }
    else
    {
        ptr--;
    }

    return ptr;
}

/**
 * @brief
 * Increment fifo pointer value
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param ptr pointer value
 * @return uint32_t next pointer value accounting for wrap-around conditions
 */
template <typename D, unsigned long size>
inline uint32_t FifoQueue<D, size>::incrementPointer(uint32_t ptr) const
{
    if (++ptr >= size + 1)
    {
        ptr = 0;
    }

    return ptr;
}

/**
 * @brief
 * Check if fifo is empty
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @return true if fifo contains no entries
 * @return false otherwise
 */
template <typename D, unsigned long size>
inline bool FifoQueue<D, size>::isEmpty() const
{
    return (head == tail);
}

/**
 * @brief
 * Check if fifo is full
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @return true if fifo is too full to enqueue any more entries
 * @return false otherwise
 */
template <typename D, unsigned long size>
inline bool FifoQueue<D, size>::isFull() const
{
    uint32_t nextTail = incrementPointer(tail);

    return (nextTail == head);
}

/**
 * @brief
 * Enqueue an entry into the fifo
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param value value to push into the fifo
 * @return true if successful
 * @return false if fifo is too full to enqueue any more entries
 */
template <typename D, unsigned long size>
inline bool FifoQueue<D, size>::push(D value)
{
    if (isFull())
    {
        return false;
    }

    memory[tail] = value;
    tail         = incrementPointer(tail);

    return true;
}

/**
 * @brief
 * Enqueue an entry into the fifo
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param value value to push into the fifo
 * @return true if successful
 * @return false if fifo is too full to enqueue any more entries
 */
template <typename D, unsigned long size>
inline bool FifoQueue<D, size>::push_front(D value)
{
    if (isFull())
    {
        return false;
    }

    head         = decrementPointer(head);
    memory[head] = value;

    return true;
}

/**
 * @brief
 * Dequeue an entry from the fifo
 *
 * @tparam D data type
 * @tparam size max number of queue elements
 * @param value result from the pop operation
 * @return true if successful
 * @return false if fifo contains no entries
 */
template <typename D, unsigned long size>
inline bool FifoQueue<D, size>::pop(D &value)
{
    if (isEmpty())
    {
        return false;
    }

    value = memory[head];
    head  = incrementPointer(head);

    return true;
}

#endif // _FIFO_QUEUE_HPP
