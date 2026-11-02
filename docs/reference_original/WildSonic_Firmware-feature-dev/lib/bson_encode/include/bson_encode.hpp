#ifndef LIB_BSON_ENCODE_INCLUDE_BSON_ENCODE_HPP
#define LIB_BSON_ENCODE_INCLUDE_BSON_ENCODE_HPP

#include <stdint.h>
#include <string.h>
#include <time.h>

/**
 * @brief BSON Encode library namespace
 */
namespace bson_encode
{
    /**
     * @brief error code definitions
     *
     */
    enum class error_t : uint8_t
    {
        NO_ERROR,
        BUFFER_TOO_SMALL,
        ERROR,
    };

    /**
     * @brief BSON type definitions
     */
    enum class type_t : uint8_t
    {
        END_OF_DOCUMENT = 0x00,
        DOUBLE          = 0x01,
        STRING          = 0x02,
        DOCUMENT        = 0x03,
        ARRAY           = 0x04,
        BINARY          = 0x05,
        UNDEFINED       = 0x06,
        OBJECT_ID       = 0x07,
        BOOLEAN         = 0x08,
        UTC_DATETIME    = 0x09,
        NULL_VALUE      = 0x0A,
        REGEX           = 0x0B,
        DBPOINTER       = 0x0C,
        JAVASCRIPT      = 0x0D,
        SYMBOL          = 0x0E,
        JAVASCRIPT_W_S  = 0x0F,
        INT32           = 0x10,
        TIMESTAMP       = 0x11,
        INT64           = 0x12,
        DECIMAL128      = 0x13,
        MIN_KEY         = 0xFF,
        MAX_KEY         = 0x7F,
    };

    /**
     * @brief Binary subtype definitions
     */
    enum class binary_subtype_t : uint8_t
    {
        GENERIC = 0x00,
        FUNCTION,
        BINARY_DEPRECATED,
        UUID_DEPRECATED,
        UUID,
        MD5,
        USER_DEFINED = 0x80,
    };

    /**
     * @brief Top header describing all of data.
     * No section specific information is here.
     */
    struct main_header_t
    {
        uint32_t size;
    };

    /**
     * @brief Description of end of document field
     */
    struct __attribute((packed)) end_of_document_field_t
    {
        type_t type; // type_t::END_OF_DOCUMENT
    };

    struct __attribute((packed)) key_field_common_t
    {
        type_t type;   // type of the field
        char   key[1]; // key string with null terminator. Size of this field
                       // is variable
    };

    /**
     * @brief Description of string field with data
     */
    struct __attribute((packed)) string_field_subheader_t
    {
        uint32_t length; // length of string data
                         // with null terminator
    };

    /**
     * @brief Description of binary field with data
     */
    struct __attribute((packed)) binary_field_subheader_t
    {
        uint32_t         length;  // length of binary data
        binary_subtype_t subtype; // binary_subtype_t
    };

    /**
     * @brief Set up the end of document field object
     *
     * @param p_buffer pointer to buffer
     * @param size size of buffer
     * @return error_t error code if set up of field failed, NO_ERROR otherwise
     */
    error_t setup_end_of_document_field(uint8_t *p_buffer, uint32_t size);

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
    error_t setup_key_field_common(uint8_t *   p_buffer,
                                   uint32_t    buffer_size,
                                   type_t      type,
                                   const char *p_key,
                                   uint32_t    key_length);

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
    error_t setup_primitive_field(uint8_t *   p_buffer,
                                  uint32_t    buffer_size,
                                  type_t      type,
                                  const char *p_key,
                                  uint32_t    key_length,
                                  uint8_t *   p_data,
                                  uint32_t    data_size);

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
    error_t setup_double_field(uint8_t *   p_buffer,
                               uint32_t    buffer_size,
                               const char *p_key,
                               uint32_t    key_length,
                               double      data);

    /**
     * @brief Set up the string field object
     *
     * @param p_buffer pointer to buffer
     * @param buffer_size size of buffer
     * @param p_key string key
     * @param key_length the length of the key (not including null terminator)
     * @param string_length the length of the string (not including null
     * terminator)
     * @return error_t error code if set up of field failed, NO_ERROR otherwise
     */
    error_t setup_string_field_header(uint8_t *   p_buffer,
                                      uint32_t    buffer_size,
                                      const char *p_key,
                                      uint32_t    key_length,
                                      uint32_t    string_length);

    /**
     * @brief Set up the string field object
     *
     * @param p_buffer pointer to buffer
     * @param buffer_size size of buffer
     * @param p_key string key
     * @param key_length the length of the key (not including null terminator)
     * @param string_data field data
     * @param string_length the length of the string (not including null
     * terminator)
     * @return error_t error code if set up of field failed, NO_ERROR otherwise
     */
    error_t setup_string_field(uint8_t *   p_buffer,
                               uint32_t    buffer_size,
                               const char *p_key,
                               uint32_t    key_length,
                               const char *string_data,
                               uint32_t    string_length);

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
    error_t setup_binary_field_header(uint8_t *        p_buffer,
                                      uint32_t         buffer_size,
                                      const char *     p_key,
                                      uint32_t         key_length,
                                      uint32_t         binary_length,
                                      binary_subtype_t subtype);

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
    error_t setup_binary_field(uint8_t *        p_buffer,
                               uint32_t         buffer_size,
                               const char *     p_key,
                               uint32_t         key_length,
                               const uint8_t *  p_binary_data,
                               uint32_t         binary_length,
                               binary_subtype_t subtype);

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
    error_t setup_boolean_field(uint8_t *   p_buffer,
                                uint32_t    buffer_size,
                                const char *key,
                                uint32_t    key_length,
                                bool        data);

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
    error_t setup_int32_field(uint8_t *   p_buffer,
                              uint32_t    buffer_size,
                              const char *p_key,
                              uint32_t    key_length,
                              int32_t     data);

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
    error_t setup_datetime_field(uint8_t *   p_buffer,
                                 uint32_t    buffer_size,
                                 const char *p_key,
                                 uint32_t    key_length,
                                 uint64_t    time_ms);

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
    error_t setup_int64_field(uint8_t *   p_buffer,
                              uint32_t    buffer_size,
                              const char *p_key,
                              uint32_t    key_length,
                              int64_t     data);

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
    error_t setup_decimal128_field(uint8_t *   p_buffer,
                                   uint32_t    buffer_size,
                                   const char *p_key,
                                   uint32_t    key_length,
                                   uint8_t *   p_data);

    /**
     * @brief Get the field size for end of document field
     *
     * @return uint32_t size of field
     */
    inline uint32_t get_end_of_document_field_size()
    {
        return sizeof(end_of_document_field_t);
    }

    /**
     * @brief Get the field size for a primitive field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @param data_size the size of the primitive data type (int32, int64,
     * double, bool, etc.)
     * @return uint32_t size of field
     */
    inline uint32_t get_primitive_field_size(uint32_t key_length,
                                             uint32_t data_size)
    {
        return sizeof(key_field_common_t) + key_length + data_size;
    }

    /**
     * @brief Get the field size for a double field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_double_field_size(uint32_t key_length)
    {
        return get_primitive_field_size(key_length, sizeof(double));
    }

    /**
     * @brief Get the size of the string field header
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_string_field_header_size(uint32_t key_length)
    {
        return sizeof(key_field_common_t) + key_length +
               sizeof(string_field_subheader_t);
    }

    /**
     * @brief Get the field size for a string field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @param string_length the length of the string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_string_field_size(uint32_t key_length,
                                          uint32_t string_length)
    {
        return get_string_field_header_size(key_length) + string_length + 1;
    }

    /**
     * @brief Get the size for a binary field header
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_binary_field_header_size(uint32_t key_length)
    {
        return sizeof(key_field_common_t) + key_length +
               sizeof(binary_field_subheader_t);
    }

    /**
     * @brief Get the field size for a binary field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @param binary_length the length of the binary data
     * @return uint32_t size of field
     */
    inline uint32_t get_binary_field_size(uint32_t key_length,
                                          uint32_t binary_length)
    {
        return get_binary_field_header_size(key_length) + binary_length;
    }

    /**
     * @brief Get the field size for a boolean field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_boolean_field_size(uint32_t key_length)
    {
        return get_primitive_field_size(key_length, sizeof(bool));
    }

    /**
     * @brief Get the field size for a int32 field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_int32_field_size(uint32_t key_length)
    {
        return get_primitive_field_size(key_length, sizeof(int32_t));
    }

    /**
     * @brief Get the datetime field size object
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_datetime_field_size(uint32_t key_length)
    {
        return get_primitive_field_size(key_length, sizeof(uint64_t));
    }

    /**
     * @brief Get the field size for a int64 field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_int64_field_size(uint32_t key_length)
    {
        return get_primitive_field_size(key_length, sizeof(int64_t));
    }

    /**
     * @brief Get the field size for a decimal128 field
     *
     * @param key_length the length of the key string (not including null
     * terminator)
     * @return uint32_t size of field
     */
    inline uint32_t get_decimal128_field_size(uint32_t key_length)
    {
        return get_primitive_field_size(key_length, 16);
    }

    /**
     * @brief Get the header size object
     *
     * @return uint32_t size of header
     */
    inline uint32_t get_header_size()
    {
        return sizeof(main_header_t);
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
     * @brief Set up the header object
     *
     * @param p_buffer pointer to buffer
     * @param buffer_size size of buffer
     * @param size size of the document
     * @return error_t error code if set up of header failed, NO_ERROR otherwise
     */
    inline error_t
    setup_header(uint8_t *p_buffer, uint32_t buffer_size, uint32_t size)
    {
        if (buffer_size < sizeof(main_header_t))
        {
            return error_t::BUFFER_TOO_SMALL;
        }

        main_header_t *p_header = get_header(p_buffer);

        p_header->size          = size;

        return error_t::NO_ERROR;
    }

    // Print binary data contents
    error_t verify_header(uint8_t *p_buffer, uint32_t size);

    // Print binary data contents
    void    unpack_print(uint8_t *p_buffer);

} // namespace bson_encode

#endif // LIB_BSON_ENCODE_INCLUDE_BSON_ENCODE_HPP