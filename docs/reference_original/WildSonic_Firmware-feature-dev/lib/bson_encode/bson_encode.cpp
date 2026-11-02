#include "bson_encode.hpp"

using namespace bson_encode;

/**
 * @brief Set up the end of document field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_end_of_document_field(uint8_t *p_buffer,
                                                 uint32_t buffer_size)
{
    end_of_document_field_t *p_field =
        reinterpret_cast<end_of_document_field_t *>(p_buffer);

    if (buffer_size < get_end_of_document_field_size())
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    p_field->type = type_t::END_OF_DOCUMENT;

    return error_t::NO_ERROR;
}

/**
 * @brief Set up the key field common object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param type type of data
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_key_field_common(uint8_t *   p_buffer,
                                            uint32_t    buffer_size,
                                            type_t      type,
                                            const char *p_key,
                                            uint32_t    key_length)
{
    key_field_common_t *p_field =
        reinterpret_cast<key_field_common_t *>(p_buffer);

    size_t field_size = sizeof(key_field_common_t) + key_length;

    if (buffer_size < field_size)
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    p_field->type = type;
    memcpy(p_field->key, p_key, key_length);
    p_field->key[key_length] = '\0';

    return error_t::NO_ERROR;
}

/**
 * @brief Set up the primitive field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param type type of data
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param p_data field data
 * @param data_size size of field data
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_primitive_field(uint8_t *   p_buffer,
                                           uint32_t    buffer_size,
                                           type_t      type,
                                           const char *p_key,
                                           uint32_t    key_length,
                                           uint8_t *   p_data,
                                           uint32_t    data_size)
{
    key_field_common_t *p_field =
        reinterpret_cast<key_field_common_t *>(p_buffer);

    const size_t field_size = get_primitive_field_size(key_length, data_size);

    if (buffer_size < field_size)
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    error_t error =
        setup_key_field_common(p_buffer, buffer_size, type, p_key, key_length);

    if (error == error_t::NO_ERROR)
    {
        memcpy(&p_field->key[key_length + 1], p_data, data_size);
    }

    return error;
}

/**
 * @brief Set up the double field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param data field data
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_double_field(uint8_t *   p_buffer,
                                        uint32_t    buffer_size,
                                        const char *p_key,
                                        uint32_t    key_length,
                                        double      data)
{
    return setup_primitive_field(p_buffer,
                                 buffer_size,
                                 type_t::DOUBLE,
                                 p_key,
                                 key_length,
                                 reinterpret_cast<uint8_t *>(&data),
                                 sizeof(double));
}

/**
 * @brief Set up the string field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param string_length the length of the string (not including null terminator)
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_string_field_header(uint8_t *   p_buffer,
                                               uint32_t    buffer_size,
                                               const char *p_key,
                                               uint32_t    key_length,
                                               uint32_t    string_length)
{
    const size_t field_size = get_string_field_header_size(key_length);

    if (buffer_size < field_size)
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    error_t error = setup_key_field_common(p_buffer,
                                           buffer_size,
                                           type_t::STRING,
                                           p_key,
                                           key_length);

    if (error != error_t::NO_ERROR)
    {
        return error;
    }

    string_field_subheader_t *p_subheader =
        reinterpret_cast<string_field_subheader_t *>(
            &p_buffer[sizeof(key_field_common_t) + key_length]);

    p_subheader->length = string_length + 1;

    return error_t::NO_ERROR;
}

/**
 * @brief Set up the string field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param string_data field data
 * @param string_length the length of the string (not including null terminator)
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_string_field(uint8_t *   p_buffer,
                                        uint32_t    buffer_size,
                                        const char *p_key,
                                        uint32_t    key_length,
                                        const char *string_data,
                                        uint32_t    string_length)
{
    const size_t header_size = get_string_field_header_size(key_length);
    const size_t field_size  = get_string_field_size(key_length, string_length);

    if (buffer_size < field_size)
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    error_t error = setup_string_field_header(p_buffer,
                                              buffer_size,
                                              p_key,
                                              key_length,
                                              string_length);

    if (error != error_t::NO_ERROR)
    {
        return error;
    }

    // Copy string data right after header
    char *p_string_data = reinterpret_cast<char *>(&p_buffer[header_size]);
    memcpy(p_string_data, string_data, string_length);
    p_string_data[string_length] = '\0';

    return error_t::NO_ERROR;
}

/**
 * @brief Set up the binary field header object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param binary_length the length of the binary data
 * @param subtype binary subtype
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_binary_field_header(uint8_t *        p_buffer,
                                               uint32_t         buffer_size,
                                               const char *     p_key,
                                               uint32_t         key_length,
                                               uint32_t         binary_length,
                                               binary_subtype_t subtype)
{
    const size_t header_size = get_binary_field_header_size(key_length);

    if (buffer_size < header_size)
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    error_t error = setup_key_field_common(p_buffer,
                                           buffer_size,
                                           type_t::BINARY,
                                           p_key,
                                           key_length);

    if (error != error_t::NO_ERROR)
    {
        return error;
    }

    binary_field_subheader_t *p_subheader =
        reinterpret_cast<binary_field_subheader_t *>(
            &p_buffer[sizeof(key_field_common_t) + key_length]);

    p_subheader->length  = binary_length;
    p_subheader->subtype = subtype;

    return error_t::NO_ERROR;
}

/**
 * @brief Set up the binary field header object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param p_binary_data pointer to binary data
 * @param binary_length the length of the binary data
 * @param subtype binary subtype
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_binary_field(uint8_t *        p_buffer,
                                        uint32_t         buffer_size,
                                        const char *     p_key,
                                        uint32_t         key_length,
                                        const uint8_t *  p_binary_data,
                                        uint32_t         binary_length,
                                        binary_subtype_t subtype)
{
    const size_t header_size = get_binary_field_header_size(key_length);
    const size_t field_size  = get_binary_field_size(key_length, binary_length);

    if (buffer_size < field_size)
    {
        return error_t::BUFFER_TOO_SMALL;
    }

    error_t error = setup_binary_field_header(p_buffer,
                                              buffer_size,
                                              p_key,
                                              key_length,
                                              binary_length,
                                              subtype);

    if (error != error_t::NO_ERROR)
    {
        return error;
    }

    // Copy binary data right after header
    uint8_t *p_dest_binary_data = &p_buffer[header_size];
    memcpy(p_dest_binary_data, p_binary_data, binary_length);

    return error_t::NO_ERROR;
}

/**
 * @brief Set up the boolean field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param key string key
 * @param key_length the length of the key (not including null terminator)
 * @param data field data
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_boolean_field(uint8_t *   p_buffer,
                                         uint32_t    buffer_size,
                                         const char *key,
                                         uint32_t    key_length,
                                         bool        data)
{
    return setup_primitive_field(p_buffer,
                                 buffer_size,
                                 type_t::BOOLEAN,
                                 key,
                                 key_length,
                                 reinterpret_cast<uint8_t *>(&data),
                                 sizeof(bool));
}

/**
 * @brief Set up the int32 field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param data field data
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_int32_field(uint8_t *   p_buffer,
                                       uint32_t    buffer_size,
                                       const char *p_key,
                                       uint32_t    key_length,
                                       int32_t     data)
{
    return setup_primitive_field(p_buffer,
                                 buffer_size,
                                 type_t::INT32,
                                 p_key,
                                 key_length,
                                 reinterpret_cast<uint8_t *>(&data),
                                 sizeof(int32_t));
}

/**
 * @brief Set up the datetime field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param key string key
 * @param key_length the length of the key (not including null terminator)
 * @param time_ms the number of milliseconds since the UNIX epoch
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_datetime_field(uint8_t *   p_buffer,
                                          uint32_t    buffer_size,
                                          const char *p_key,
                                          uint32_t    key_length,
                                          uint64_t    time_ms)
{
    return setup_primitive_field(p_buffer,
                                 buffer_size,
                                 type_t::UTC_DATETIME,
                                 p_key,
                                 key_length,
                                 reinterpret_cast<uint8_t *>(&time_ms),
                                 sizeof(uint64_t));
}

/**
 * @brief Set up the int64 field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param data field data
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_int64_field(uint8_t *   p_buffer,
                                       uint32_t    buffer_size,
                                       const char *p_key,
                                       uint32_t    key_length,
                                       int64_t     data)
{
    return setup_primitive_field(p_buffer,
                                 buffer_size,
                                 type_t::INT64,
                                 p_key,
                                 key_length,
                                 reinterpret_cast<uint8_t *>(&data),
                                 sizeof(int64_t));
}

/**
 * @brief Set up the decimal128 field object
 *
 * @param p_buffer pointer to buffer
 * @param buffer_size size of buffer
 * @param p_key string key
 * @param key_length the length of the key (not including null terminator)
 * @param p_data pointer to field data. Must be 16 bytes long
 * @return error_t error code if set up of field failed, NO_ERROR otherwise
 */
error_t bson_encode::setup_decimal128_field(uint8_t *   p_buffer,
                                            uint32_t    buffer_size,
                                            const char *p_key,
                                            uint32_t    key_length,
                                            uint8_t *   p_data)
{
    return setup_primitive_field(p_buffer,
                                 buffer_size,
                                 type_t::DECIMAL128,
                                 p_key,
                                 key_length,
                                 p_data,
                                 16);
}