#include "bson_encode.hpp"

#include <cassert>
#include <ctime>
#include <fstream>
#include <iostream>
#include <new>

constexpr uint32_t MAX_FILE_SIZE = 1024 * 1024 * 32;
constexpr uint32_t NUM_FIELDS    = 7;

const char *       FIELD_KEYS[NUM_FIELDS] =
    {"ALPHA", "BETA", "GAMMA", "DELTA", "EPSILON", "ZETA", "TIMESTAMP"};

const bson_encode::type_t FIELD_TYPES[NUM_FIELDS] = {
    bson_encode::type_t::INT32,
    bson_encode::type_t::INT64,
    bson_encode::type_t::DOUBLE,
    bson_encode::type_t::STRING,
    bson_encode::type_t::BINARY,
    bson_encode::type_t::BOOLEAN,
    bson_encode::type_t::UTC_DATETIME};

const char *   STRING_DATA              = "This is a string";

const uint32_t BINARY_SIZE              = 18;
const uint8_t  BINARY_DATA[BINARY_SIZE] = {0x00,
                                          0x01,
                                          0x02,
                                          0x03,
                                          0x04,
                                          0x05,
                                          0x06,
                                          0x07,
                                          0x08,
                                          0x09,
                                          0x0A,
                                          0x0B,
                                          0x0C,
                                          0x0D,
                                          0x0E,
                                          0x0F,
                                          0x10,
                                          0x11};

uint32_t       FIELD_SIZES[NUM_FIELDS]  = {0};
uint32_t       KEY_LENGTHS[NUM_FIELDS]  = {0};

uint8_t        BUFFER[MAX_FILE_SIZE];

int            main(int argc, char *argv[])
{

    if (argc != 2)
    {
        printf("Invalid number of arguments. Expected 1. Got:%d\n", argc - 1);
        return 1;
    }

    uint32_t document_size = bson_encode::get_header_size();

    for (uint32_t field_index = 0; field_index < NUM_FIELDS; field_index++)
    {
        KEY_LENGTHS[field_index] = strlen(FIELD_KEYS[field_index]);

        switch (FIELD_TYPES[field_index])
        {
            case bson_encode::type_t::INT32:
            {
                FIELD_SIZES[field_index] =
                    bson_encode::get_int32_field_size(KEY_LENGTHS[field_index]);
                break;
            }
            case bson_encode::type_t::INT64:
            {
                FIELD_SIZES[field_index] =
                    bson_encode::get_int64_field_size(KEY_LENGTHS[field_index]);
                break;
            }
            case bson_encode::type_t::DOUBLE:
            {
                FIELD_SIZES[field_index] = bson_encode::get_double_field_size(
                    KEY_LENGTHS[field_index]);
                break;
            }
            case bson_encode::type_t::STRING:
            {
                FIELD_SIZES[field_index] =
                    bson_encode::get_string_field_size(KEY_LENGTHS[field_index],
                                                       strlen(STRING_DATA));
                break;
            }
            case bson_encode::type_t::BINARY:
            {
                FIELD_SIZES[field_index] =
                    bson_encode::get_binary_field_size(KEY_LENGTHS[field_index],
                                                       BINARY_SIZE);
                break;
            }
            case bson_encode::type_t::BOOLEAN:
            {
                FIELD_SIZES[field_index] = bson_encode::get_boolean_field_size(
                    KEY_LENGTHS[field_index]);
                break;
            }
            case bson_encode::type_t::UTC_DATETIME:
            {
                FIELD_SIZES[field_index] = bson_encode::get_datetime_field_size(
                    KEY_LENGTHS[field_index]);
                break;
            }
            default:
            {
                printf("Invalid field type!\n");
                return 1;
            }
        }

        document_size += FIELD_SIZES[field_index];
    }

    // Add the end of document field
    document_size += bson_encode::get_end_of_document_field_size();

    bson_encode::error_t error =
        bson_encode::setup_header(BUFFER, MAX_FILE_SIZE, document_size);
    if (error != bson_encode::error_t::NO_ERROR)
    {
        return 1;
    }

    uint32_t offset = bson_encode::get_header_size();
    for (uint32_t field_index = 0; field_index < NUM_FIELDS; field_index++)
    {
        switch (FIELD_TYPES[field_index])
        {
            case bson_encode::type_t::INT32:
            {
                error = bson_encode::setup_int32_field(&BUFFER[offset],
                                                       MAX_FILE_SIZE - offset,
                                                       FIELD_KEYS[field_index],
                                                       KEY_LENGTHS[field_index],
                                                       0x12345678);
                break;
            }
            case bson_encode::type_t::INT64:
            {
                error = bson_encode::setup_int64_field(&BUFFER[offset],
                                                       MAX_FILE_SIZE - offset,
                                                       FIELD_KEYS[field_index],
                                                       KEY_LENGTHS[field_index],
                                                       0x789ABCDEF0123456ull);
                break;
            }
            case bson_encode::type_t::DOUBLE:
            {
                error = bson_encode::setup_double_field(
                    &BUFFER[offset],
                    MAX_FILE_SIZE - offset,
                    FIELD_KEYS[field_index],
                    KEY_LENGTHS[field_index],
                    3.1415926535897932384626433832795);
                break;
            }
            case bson_encode::type_t::STRING:
            {
                error =
                    bson_encode::setup_string_field(&BUFFER[offset],
                                                    MAX_FILE_SIZE - offset,
                                                    FIELD_KEYS[field_index],
                                                    KEY_LENGTHS[field_index],
                                                    STRING_DATA,
                                                    strlen(STRING_DATA));
                break;
            }
            case bson_encode::type_t::BINARY:
            {
                error = bson_encode::setup_binary_field_header(
                    &BUFFER[offset],
                    MAX_FILE_SIZE - offset,
                    FIELD_KEYS[field_index],
                    KEY_LENGTHS[field_index],
                    BINARY_SIZE,
                    bson_encode::binary_subtype_t::GENERIC);

                memcpy(
                    &BUFFER[offset + bson_encode::get_binary_field_header_size(
                                         KEY_LENGTHS[field_index])],
                    BINARY_DATA,
                    BINARY_SIZE);
                break;
            }
            case bson_encode::type_t::BOOLEAN:
            {
                error =
                    bson_encode::setup_boolean_field(&BUFFER[offset],
                                                     MAX_FILE_SIZE - offset,
                                                     FIELD_KEYS[field_index],
                                                     KEY_LENGTHS[field_index],
                                                     true);
                break;
            }
            case bson_encode::type_t::UTC_DATETIME:
            {

                time_t time = std::time(nullptr);
                error       = bson_encode::setup_datetime_field(
                    &BUFFER[offset],
                    MAX_FILE_SIZE - offset,
                    FIELD_KEYS[field_index],
                    KEY_LENGTHS[field_index],
                    static_cast<uint64_t>(time) * 1000);
                break;
            }
            default:
            {
                printf("Invalid field type!\n");
                return 1;
            }
        }

        assert(error == bson_encode::error_t::NO_ERROR);

        offset += FIELD_SIZES[field_index];
    }

    // Add end of document field
    error = bson_encode::setup_end_of_document_field(&BUFFER[offset],
                                                     MAX_FILE_SIZE - offset);
    assert(error == bson_encode::error_t::NO_ERROR);
    offset += bson_encode::get_end_of_document_field_size();
    assert(offset == document_size);

    printf("Opening file %s\n", argv[1]);
    std::ofstream outfile(argv[1], std::ios::out | std::ios::binary);

    if (!outfile)
    {
        printf("Cannot open file!\n");
        return 1;
    }
    outfile.write(reinterpret_cast<char *>(BUFFER), document_size);
    outfile.close();

    return 0;
}