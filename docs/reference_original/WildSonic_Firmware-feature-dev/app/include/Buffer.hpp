#ifndef _BUFFER_HPP
#define _BUFFER_HPP

#include <climits>
#include <stdint.h>

using buffer_id_t                 = uint8_t;

// Number of buffers and buffer id's available
constexpr uint32_t    NUM_BUFFERS = 10;

// Invalid buffer id
constexpr buffer_id_t INVALID_BUFFER_ID =
    (1 << sizeof(buffer_id_t) * CHAR_BIT) - 1;

// Number of bits to describe size of buffer
constexpr uint32_t BUFFER_SIZE_LOG2 = 12;
// Size of buffer in bytes
constexpr uint32_t BUFFER_SIZE      = (1 << BUFFER_SIZE_LOG2);

/**
 * @brief
 * Indicates whether buffer id is not in range of valid id's
 *
 * @param buffer_id buffer id value
 * @return true if buffer is invalid
 * @return false if buffer is valid
 */
inline bool        is_invalid_buffer(buffer_id_t buffer_id)
{
    return (buffer_id == INVALID_BUFFER_ID) || (buffer_id >= NUM_BUFFERS);
}

#endif // _BUFFER_HPP