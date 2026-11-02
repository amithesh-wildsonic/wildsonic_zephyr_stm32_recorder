#ifndef _CRC32_HPP
#define _CRC32_HPP

#include <climits>
#include <stdint.h>

/**
 * @brief
 * Calculate CRC32 checksum of data
 *
 * @param data pointer to data to calculate CRC32
 * @param length length of data
 * @param crc initial CRC value
 */
uint32_t
compute_crc32(const void *data, uint32_t length, uint32_t crc = 0xFFFFFFFF);

#endif // _CRC32_HPP