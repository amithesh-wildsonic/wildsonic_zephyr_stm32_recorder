#include "constants.hpp"
#include "trace.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <new>

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Invalid number of arguments. Expected 1. Got:%d\n", argc - 1);
        return 1;
    }

    try
    {
        std::ifstream infile(argv[1], std::ios::in | std::ios::binary);
        Trace::trace_log_entry_hdr_st entry_header;
        uint8_t *                     p_buffer = new uint8_t[1024];

        while (true)
        {
            if (!infile.read(reinterpret_cast<char *>(&entry_header),
                             sizeof(entry_header.entry_size)))
            {
                break;
            }

            // Not a valid entry. This is very likely padding.
            if (entry_header.entry_size == 0)
            {
                continue;
            }

            if (!infile.read(reinterpret_cast<char *>(&entry_header) +
                                 sizeof(entry_header.entry_size),
                             sizeof(entry_header) -
                                 sizeof(entry_header.entry_size)))
            {
                break;
            }

            if (entry_header.entry_size < sizeof(entry_header))
            {
                printf("Invalid Entry detected.\n");
                return 1;
            }

            uint32_t remaining_bytes =
                entry_header.entry_size - sizeof(entry_header);

            tm tm_struct;
            gmtime_r(&entry_header.time, &tm_struct);

            printf("%04d/%02d/%02d %02d:%02d:%02d C:%d [F:(%d), L:%d]",
                   static_cast<int>(TIME_BASE_YEAR) + tm_struct.tm_year,
                   static_cast<int>(TIME_BASE_MONTH) + tm_struct.tm_mon,
                   tm_struct.tm_mday,
                   tm_struct.tm_hour,
                   tm_struct.tm_min,
                   tm_struct.tm_sec,
                   entry_header.core,
                   entry_header.trace_file_number,
                   entry_header.line_number);

            if (remaining_bytes == 0)
            {
                printf("\n");
            }
            else
            {
                printf(" WITH ARG DATA (%d bytes):\n", remaining_bytes);

                while (remaining_bytes >= sizeof(uint16_t))
                {
                    // Get 2-byte value size
                    uint32_t size       = sizeof(uint16_t);
                    uint32_t value_size = 0;
                    for (uint32_t index = 0; index < sizeof(uint16_t); index++)
                    {
                        char c;
                        if (!infile.read(&c, 1))
                        {
                            size = index;
                            break;
                        }

                        value_size |= static_cast<uint32_t>(c)
                                      << (__CHAR_BIT__ * index);
                    }

                    if (size != sizeof(uint16_t))
                    {
                        printf("Not enough file data for Value Size...Only "
                               "found %d bytes\n",
                               size);
                        return 1;
                    }

                    remaining_bytes -= sizeof(uint16_t);

                    if (value_size == 0)
                    {
                        break;
                    }

                    if (value_size > remaining_bytes)
                    {
                        printf("Value size %d greater than %d remaining bytes "
                               "after header",
                               value_size,
                               remaining_bytes);
                        return 1;
                    }

                    // Read remaining bytes (includes padding) for entry
                    size = value_size;
                    for (uint32_t index = 0; index < value_size; index++)
                    {
                        if (!infile.read(
                                reinterpret_cast<char *>(&p_buffer[index]),
                                1))
                        {
                            size = index;
                            break;
                        }
                    }

                    if (size != value_size)
                    {
                        printf("Not enough file data for Value...Only found %d "
                               "of %d bytes!\n",
                               size,
                               value_size);
                        return 1;
                    }

                    printf("    HEX: ");
                    for (uint32_t index = 0; index < value_size; index++)
                    {
                        printf("%02x ", p_buffer[index]);
                    }
                    printf("\n");

                    printf("    ASCII: ");
                    for (uint32_t index = 0; index < value_size; index++)
                    {
                        printf("%c", static_cast<char>(p_buffer[index]));
                    }
                    printf("\n");

                    remaining_bytes -= value_size;
                }

                // Read remaining bytes (includes padding) for entry
                for (uint32_t index = 0; index < remaining_bytes; index++)
                {
                    if (!infile.read(reinterpret_cast<char *>(&p_buffer[index]),
                                     1))
                    {
                        printf("Reached EOF before end of entry.\n");
                        return 1;
                    }
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cout << e.what();
    }

    return 0;
}