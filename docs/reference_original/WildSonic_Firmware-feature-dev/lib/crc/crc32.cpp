#include "crc32.hpp"

/**
 * @brief
 * Calculate CRC32 checksum of data
 *
 * @param data pointer to data to calculate CRC32
 * @param length length of data
 * @param crc initial CRC value
 */
uint32_t compute_crc32(const void *data, uint32_t length, uint32_t crc)
{
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    for (uint32_t i = 0; i < length; i++)
    {
        crc = crc ^ bytes[i];
        for (uint32_t j = 0; j < 8; j++)
        {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return crc;
}