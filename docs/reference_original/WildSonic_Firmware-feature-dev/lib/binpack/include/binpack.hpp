#ifndef LIB_BINPACK_INCLUDE_BINPACK_HPP
#define LIB_BINPACK_INCLUDE_BINPACK_HPP

#include <stdint.h>
#include <string.h>

namespace binpack
{
    static constexpr uint32_t BIN_PACK_HEADER_VERSION                 = 1;

    static constexpr uint32_t HEADER_SIGNATURE_LENGTH                 = 4;

    static constexpr char DATA_SIGNATURE[HEADER_SIGNATURE_LENGTH + 1] = "ABRA";

    enum class error_t : uint8_t
    {
        NO_ERROR,
        ERROR,
    };

    /**
     * @brief Top header describing all of data. No section specific information
     * is here.
     */
    struct main_header_t
    {
        unsigned char signature[HEADER_SIGNATURE_LENGTH];
        uint32_t      version;                     // Data header
                                                   // format version
        uint16_t      header_size;                 // Header size.
                                                   // Includes section headers.
        uint16_t      section_header_table_offset; // Offset of the section
                                                   // header table from
                                                   // beginning of binary data
        uint16_t      section_header_entry_size;   // Size of section
                                                   // header entry
        uint16_t      rsvd0;                       // Reserved
        uint32_t      num_sections;                // Number of sections
        uint32_t      crc;                         // Header crc. Size is
                                                   // specified by header_size.
    };

    /**
     * @brief Header describing section data
     */
    struct section_header_t
    {
        uint16_t section_name_offset; // Offset of NULL-terminated section name
                                      // string from beginning of binary data
        uint32_t data_offset;         // Offset of the data from
                                      // beginning of binary data
        uint32_t data_size;           // Size of the section data
    };

    /**
     * @brief Total header including main header and section headers and section
     * names
     *
     * @tparam NUM_DATA_SECTIONS number of sections
     * @tparam SECTION_NAMES_SIZE size of all section names data
     */
    template <int NUM_DATA_SECTIONS, int SECTION_NAMES_SIZE>
    struct total_header_t
    {
        binpack::main_header_t main_header;
        binpack::section_header_t
             section_header[static_cast<uint32_t>(NUM_DATA_SECTIONS)];
        char section_names[static_cast<uint32_t>(SECTION_NAMES_SIZE)];
    };

    /**
     * @brief Set up the header object
     *
     * @tparam NUM_DATA_SECTIONS number of sections
     * @tparam SECTION_NAMES_SIZE size of all section names data
     * @param header
     * @param section_names
     *
     * @return error code if set up of header failed, NO_ERROR is successful
     */
    template <int NUM_DATA_SECTIONS, int SECTION_NAMES_SIZE>
    inline error_t
    setup_header(total_header_t<NUM_DATA_SECTIONS, SECTION_NAMES_SIZE> &header,
                 const char **section_names)
    {
        memcpy(header.main_header.signature,
               binpack::DATA_SIGNATURE,
               binpack::HEADER_SIGNATURE_LENGTH);
        header.main_header.version = BIN_PACK_HEADER_VERSION;
        header.main_header.header_size =
            sizeof(main_header_t) +
            (NUM_DATA_SECTIONS * sizeof(section_header_t)) + SECTION_NAMES_SIZE;
        header.main_header.section_header_table_offset = sizeof(main_header_t);
        header.main_header.section_header_entry_size = sizeof(section_header_t);
        header.main_header.num_sections              = NUM_DATA_SECTIONS;

        uint32_t dest_index                          = 0;
        for (uint32_t section_index = 0; section_index < NUM_DATA_SECTIONS;
             section_index++)
        {
            uint32_t length = strlen(section_names[section_index]);

            if (length + 1 > SECTION_NAMES_SIZE - dest_index)
            {
                return error_t::ERROR;
            }

            // copy section names into header
            strcpy(&header.section_names[dest_index],
                   section_names[section_index]);

            // save offset of section name
            header.section_header[section_index].section_name_offset =
                reinterpret_cast<uint8_t *>(&header.section_names[dest_index]) -
                reinterpret_cast<uint8_t *>(&header);

            dest_index += length + 1;
        }

        return error_t::NO_ERROR;
    }

    /**
     * @brief Set up the section headers object
     *
     * @tparam NUM_DATA_SECTIONS number of sections
     * @tparam SECTION_NAMES_SIZE size of all section names data
     * @param header
     * @param p_section_sizes
     */
    template <int NUM_DATA_SECTIONS, int SECTION_NAMES_SIZE>
    void setup_section_headers(
        total_header_t<NUM_DATA_SECTIONS, SECTION_NAMES_SIZE> &header,
        const uint32_t *                                       p_section_sizes)
    {
        uint32_t offset = header.main_header.header_size;
        for (uint32_t section_index = 0; section_index < NUM_DATA_SECTIONS;
             section_index++)
        {
            uint32_t section_size = p_section_sizes[section_index];

            header.section_header[section_index].data_offset = offset;
            header.section_header[section_index].data_size   = section_size;

            offset += section_size;
        }
    }

    /**
     * @brief Get the header object from data buffer
     *
     * @param p_buffer data buffer
     * @return main_header_t* header object
     */
    inline main_header_t *get_header(uint8_t *p_buffer)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
        main_header_t *p_header = reinterpret_cast<main_header_t *>(p_buffer);
#pragma GCC diagnostic pop

        return p_header;
    }

    /**
     * @brief Get the section headers array
     *
     * @param p_buffer data buffer
     * @return section_header_t* array of section headers
     */
    inline section_header_t *get_section_headers(uint8_t *p_buffer)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
        main_header_t *   p_header = get_header(p_buffer);

        section_header_t *p_section_headers =
            reinterpret_cast<section_header_t *>(
                p_buffer + p_header->section_header_table_offset);
#pragma GCC diagnostic pop

        return p_section_headers;
    }

    /**
     * @brief Get the section name for section from buffer
     *
     * @param p_buffer data buffer
     * @param section_index section index
     * @return char* section name string
     */
    inline char *get_section_name(uint8_t *p_buffer, uint32_t section_index)
    {
        section_header_t *p_section_headers = get_section_headers(p_buffer);

        return reinterpret_cast<char *>(p_buffer) +
               p_section_headers[section_index].section_name_offset;
    }

    // Print binary data contents
    error_t verify_header(uint8_t *p_buffer, uint32_t size);

    // Print binary data contents
    void    unpack_print(uint8_t *p_buffer);

} // namespace binpack

#endif // LIB_BINPACK_INCLUDE_BINPACK_HPP