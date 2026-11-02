#include "binpack.hpp"

#include <fstream>
#include <iostream>
#include <new>

constexpr uint32_t MAX_INPUT_FILE_SIZE              = 1024 * 1024 * 32;
constexpr uint32_t NUM_DATA_SECTIONS                = 5;
constexpr uint32_t SECTION_NAMES_SIZE               = 128;

const char *       SECTION_NAMES[NUM_DATA_SECTIONS] = {"ALPHA",
                                                "BETA",
                                                "CHARLIE",
                                                "DELTA",
                                                "EPSILON"};
const uint32_t     SECTION_SIZES[NUM_DATA_SECTIONS] = {32, 64, 128, 256, 512};

binpack::total_header_t<NUM_DATA_SECTIONS, SECTION_NAMES_SIZE> g_binary_header;

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Invalid number of arguments. Expected 1. Got:%d\n", argc - 1);
        return 1;
    }

    binpack::error_t error =
        binpack::setup_header(g_binary_header, SECTION_NAMES);
    if (error != binpack::error_t::NO_ERROR)
    {
        return 1;
    }

    binpack::setup_section_headers(g_binary_header, SECTION_SIZES);

    printf("Opening file %s\n", argv[1]);
    std::ofstream outfile(argv[1], std::ios::out | std::ios::binary);

    if (!outfile)
    {
        printf("Cannot open file!\n");
        return 1;
    }
    outfile.write(reinterpret_cast<char *>(&g_binary_header),
                  sizeof(g_binary_header));

    char byte = 0;
    for (uint32_t section_index = 0; section_index < NUM_DATA_SECTIONS;
         section_index++)
    {
        for (uint32_t byte_index = 0; byte_index < SECTION_SIZES[section_index];
             byte_index++)
        {
            outfile.write(&byte, 1);
            byte++;
        }
    }

    outfile.close();

    return 0;
}