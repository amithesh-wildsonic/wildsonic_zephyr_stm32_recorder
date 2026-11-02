#include "binpack.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <new>

constexpr uint32_t MAX_INPUT_FILE_SIZE = 1024 * 1024 * 32;

int                main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Invalid number of arguments. Expected 1. Got:%d\n", argc - 1);
        return 1;
    }

    try
    {
        std::ifstream infile(argv[1], std::ios::in | std::ios::binary);
        uint8_t *     p_buffer =
            reinterpret_cast<uint8_t *>(new uint32_t[MAX_INPUT_FILE_SIZE / 4]);

        uint32_t size;
        for (size = 0; size < MAX_INPUT_FILE_SIZE; size++)
        {
            if (!infile.read(reinterpret_cast<char *>(&p_buffer[size]), 1))
            {
                break;
            }
        }

        if (!infile.eof())
        {
            printf("File too large.\n");
            return 1;
        }

        if (size == 0)
        {
            printf("Not enough file data!\n");
            return 1;
        }

        binpack::error_t error;
        if ((error = binpack::verify_header(p_buffer, size)) !=
            binpack::error_t::NO_ERROR)
        {
            printf("Verify failed with error %d!", static_cast<int>(error));
            return 1;
        }

        binpack::main_header_t *   p_header = binpack::get_header(p_buffer);
        binpack::section_header_t *p_section_headers =
            binpack::get_section_headers(p_buffer);

        boost::filesystem::path dir(std::string(argv[1]) + "_out");
        if (!boost::filesystem::create_directory(dir))
        {
            printf("Failed to create directory %s!\n", dir.c_str());
            return 1;
        }

        for (uint32_t section_index = 0; section_index < p_header->num_sections;
             section_index++)
        {
            char *p_section_name =
                binpack::get_section_name(p_buffer, section_index);

            uint32_t data_offset = p_section_headers[section_index].data_offset;
            uint32_t data_size   = p_section_headers[section_index].data_size;

            std::string s =
                std::string(dir.c_str()) + "/" + std::string(p_section_name);

            std::ofstream outfile(s, std::ios::out | std::ios::binary);
            if (!outfile)
            {
                printf("Cannot open file %s for writing!\n", s.c_str());
                return 1;
            }

            outfile.write(reinterpret_cast<char *>(p_buffer) + data_offset,
                          data_size);
            outfile.close();
        }

        printf("Success! Output in folder %s\n", dir.c_str());
    }
    catch (const std::exception &e)
    {
        std::cout << e.what();
    }

    return 0;
}