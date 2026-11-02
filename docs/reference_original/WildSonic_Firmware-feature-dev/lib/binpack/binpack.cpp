#include "binpack.hpp"
#include <stdio.h>

using namespace binpack;

/**
 * @brief Print binary data contents
 *
 * @param p_buffer pointer to binary packed data
 */
void binpack::unpack_print(uint8_t *p_buffer)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    main_header_t *   p_header = reinterpret_cast<main_header_t *>(p_buffer);
    section_header_t *p_section_headers = reinterpret_cast<section_header_t *>(
        p_buffer + p_header->section_header_table_offset);
#pragma GCC diagnostic pop

    uint32_t num_sections = p_header->num_sections;

    for (uint32_t section_index = 0; section_index < num_sections;
         section_index++)
    {
        uint32_t name_offset =
            p_section_headers[section_index].section_name_offset;
        uint32_t data_offset = p_section_headers[section_index].data_offset;
        uint32_t data_size   = p_section_headers[section_index].data_size;

        printf("SECTION NAME: %s, SIZE: 0x%x\n",
               reinterpret_cast<char *>(p_buffer + name_offset),
               data_size);

        for (uint32_t index = 0; index < data_size; index++)
        {
            printf("%01x ", *(p_buffer + data_offset + index));
        }
        printf("\n");
    }
}

/**
 * @brief Print binary data contents
 *
 * @param p_buffer pointer to binary packed data
 *
 * @return NO_ERROR if nothing, else error code
 */
error_t binpack::verify_header(uint8_t *p_buffer, uint32_t size)
{
    if (size < sizeof(main_header_t))
    {
        printf("Not enough data provided to form header\n");
        return error_t::ERROR;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    main_header_t *p_header = reinterpret_cast<main_header_t *>(p_buffer);
#pragma GCC diagnostic pop

    // Data signature is not correct?
    if (memcmp(p_header->signature, DATA_SIGNATURE, HEADER_SIGNATURE_LENGTH))
    {
        printf("Invalid data signature\n");
        return error_t::ERROR;
    }

    // Unsupported header version?
    if (p_header->version != BIN_PACK_HEADER_VERSION)
    {
        printf("Incompatible header version\n");
        return error_t::ERROR;
    }

    uint32_t header_size  = p_header->header_size;
    uint32_t num_sections = p_header->num_sections;

    if (header_size > size)
    {
        printf("Not enough data provided to form header\n");
        return error_t::ERROR;
    }

    // Invalid header size?
    if (header_size <
        sizeof(main_header_t) + (num_sections * sizeof(section_header_t)))
    {
        printf("Invalid header size\n");
        return error_t::ERROR;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    section_header_t *p_section_headers = reinterpret_cast<section_header_t *>(
        p_buffer + p_header->section_header_table_offset);
#pragma GCC diagnostic pop

    for (uint32_t section_index = 0; section_index < num_sections;
         section_index++)
    {
        // Offset is beyond length of buffer?
        if (p_section_headers[section_index].data_offset >= size)
        {
            printf("Section %d data offset %d exceeds file size %d\n",
                   section_index,
                   p_section_headers[section_index].data_offset,
                   size);
            return error_t::ERROR;
        }

        // Size extends beyond length of buffer?
        if (size - p_section_headers[section_index].data_offset <
            p_section_headers[section_index].data_size)
        {
            printf(
                "Section %d data offset %d data size %d exceeds file size %d\n",
                section_index,
                p_section_headers[section_index].data_offset,
                p_section_headers[section_index].data_size,
                size);

            return error_t::ERROR;
        }

        uint8_t *p_string_buffer =
            p_buffer + p_section_headers[section_index].section_name_offset;
        while (p_string_buffer <= p_buffer + size)
        {
            if (p_string_buffer == 0)
            {
                break;
            }
            ++p_string_buffer;
        }

        // String is not null-terminated?
        if (p_string_buffer == p_buffer + size)
        {
            printf("Could not find null-terminated string for section %d\n",
                   section_index);
            return error_t::ERROR;
        }
    }

    return error_t::NO_ERROR;
}