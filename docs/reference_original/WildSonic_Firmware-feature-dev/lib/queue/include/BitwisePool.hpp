#ifndef _BITWISE_POOL_HPP
#define _BITWISE_POOL_HPP

#include "assert.hpp"
#include "constants.hpp"

#include <stdint.h>
#include <string.h>

/**
 * @brief
 * Use for allocating numeric types
 *
 * @tparam D data type
 * @tparam size max number of elements
 */
template <typename D, unsigned long size>
class BitwisePool
{
  public:
    // Construct a new Bitwise pool object
    BitwisePool() : head(0), count(size){};

    // Initialize pool tracking
    void initialize();

    // Check if pool is empty
    bool isEmpty() const;

    // Check if pool is full
    bool isFull() const;

    // Returns an entry back to the pool
    bool free(D value);

    // Allocate an entry from the pool
    bool allocate(D &value);

  private:
    static constexpr uint32_t MEM_SIZE_DWORDS =
        (size >> BITS_PER_DWORD_LOG2) + ((size & (BITS_PER_DWORD - 1)) != 0);

    // Increment pool pointer value
    uint32_t incrementPointer(uint32_t ptr) const;

    uint32_t head;                    // consumer pointer
    uint32_t count;                   // count of free numbers
    uint32_t memory[MEM_SIZE_DWORDS]; // pool memory
};

/**
 * @brief
 * Initialize pool tracking
 *
 * @tparam D data type
 * @tparam size max number of elements
 */
template <typename D, unsigned long size>
inline void BitwisePool<D, size>::initialize()
{
    head  = 0;
    count = size;
    memset(memory, 0xFF, sizeof(uint32_t) * MEM_SIZE_DWORDS);
}

/**
 * @brief
 * Increment pool pointer value
 *
 * @tparam D data type
 * @tparam size max number of elements
 * @param ptr pointer value
 * @return uint32_t next pointer value accounting for wrap-around conditions
 */
template <typename D, unsigned long size>
inline uint32_t BitwisePool<D, size>::incrementPointer(uint32_t ptr) const
{
    if (++ptr >= size)
    {
        ptr = 0;
    }

    return ptr;
}

/**
 * @brief
 * Check if pool is empty
 *
 * @tparam D data type
 * @tparam size max number of elements
 * @return true if pool contains no entries
 * @return false otherwise
 */
template <typename D, unsigned long size>
inline bool BitwisePool<D, size>::isEmpty() const
{
    return (count == 0);
}

/**
 * @brief
 * Check if pool is full
 *
 * @tparam D data type
 * @tparam size max number of elements
 * @return true if pool is full
 * @return false otherwise
 */
template <typename D, unsigned long size>
inline bool BitwisePool<D, size>::isFull() const
{
    return (count == size);
}

/**
 * @brief
 * Returns an entry back to the pool
 *
 * @tparam D data type
 * @tparam size max number of elements
 * @param value value to push into the pool
 * @return true if successful
 * @return false if pool is too full to add any more entries
 */
template <typename D, unsigned long size>
inline bool BitwisePool<D, size>::free(D value)
{
    if (isFull())
    {
        return false;
    }

    uint32_t bitMask = 1UL << (value & (BITS_PER_DWORD - 1));

    // Check if entry has already been freed
    if (memory[value >> BITS_PER_DWORD_LOG2] & bitMask)
    {
        return false;
    }

    memory[value >> BITS_PER_DWORD_LOG2] |= bitMask;
    ++count;

    return true;
}

/**
 * @brief
 * Allocate an entry from the pool
 *
 * @tparam D data type
 * @tparam size max number of elements
 * @param value result from the pop operation
 * @return true if successful
 * @return false if pool contains no entries
 */
template <typename D, unsigned long size>
inline bool BitwisePool<D, size>::allocate(D &value)
{
    if (isEmpty())
    {
        return false;
    }

    uint32_t iter;
    for (iter = 0; iter < size; iter++)
    {
        uint32_t index   = head >> BITS_PER_DWORD_LOG2;
        uint32_t bitMask = 1UL << (head & (BITS_PER_DWORD - 1));
        if (memory[index] & bitMask)
        {
            memory[index] &= ~bitMask;
            --count;

            value = head;
            break;
        }
        head = incrementPointer(head);
    }

    // A value should have been acquired from the pool
    SYSTEM_ASSERT_NOT_EQUAL(iter, size);

    return true;
}

#endif // _BITWISE_POOL_HPP
