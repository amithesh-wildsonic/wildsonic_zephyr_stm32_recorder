#ifndef _TRACE_HPP
#define _TRACE_HPP

#include <algorithm>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/**
 * @brief Trace source file type
 */
enum class TraceFile
{
    MAIN_CPP,
    ADMIN_TASK_CPP,
    AUDIO_TASK_CPP,
    COMM_TASK_CPP,
    DETECT_TASK_CPP,
    STORAGE_TASK_CPP,
    BLE_COMM_CPP,
    W_HAL_GSM_CPP,
    W_HAL_GSM_MAXIM_CPP,
    W_HAL_I2C_CPP,
    W_HAL_I2S_CPP,
    W_HAL_RTC_CPP,
    W_HAL_SD_CARD_CPP,
    W_HAL_SPI_CPP,
    W_HAL_SYSTEM_CPP,
    W_HAL_UART_CPP,
    W_HAL_I2S_SGTL_CPP,
    W_HAL_I2S_TLV_CPP,
    W_HAL_I2S_MXC_CPP,
    W_HAL_I2S_SPH_CPP,
    W_HAL_FLASH_CPP,
    W_HAL_PM_CPP,
};

#define LOG_MSG_WITH_ARGS(trace_file, ...)                                     \
    Trace::trace_log_with_args(true,                                           \
                               __FILE__,                                       \
                               trace_file,                                     \
                               __LINE__,                                       \
                               __VA_ARGS__)
#define LOG_MSG_WITH_ARGS_NO_PRINT(trace_file, ...)                            \
    Trace::trace_log_with_args(false,                                          \
                               __FILE__,                                       \
                               trace_file,                                     \
                               __LINE__,                                       \
                               __VA_ARGS__)
#define LOG_MSG(trace_file, msg)                                               \
    Trace::trace_log(true, __FILE__, trace_file, __LINE__, msg)
#define LOG_MSG_NO_PRINT(trace_file, msg)                                      \
    Trace::trace_log(false, __FILE__, trace_file, __LINE__, msg);

class Trace
{

  public:
    static const uint32_t LOG_BUFFER_SIZE = 1024;

    /**
     * @brief Trace buffer and information structure
     */
    struct info_st
    {
        uint8_t           log_buffer[LOG_BUFFER_SIZE]; // log buffer data

        volatile uint16_t saved_log_offset; // pointer to data that
                                            // has not been saved
        volatile uint16_t log_offset;       // producer pointer
                                            // pointer within log
        volatile uint16_t log_bytes;        // indicates number
                                            // of bytes populated in log
    };

    /**
     * @brief Trace log entry header format
     */
    struct trace_log_entry_hdr_st
    {
        uint16_t entry_size;        // Size of the entry
                                    // header, args, and padding
        uint8_t  core;              // Core number which
                                    // logged trace entry
        uint8_t  rsvd0;             // Reserved
        uint16_t trace_file_number; // File enum
        uint16_t line_number;       // File line number
        time_t   time;              // Time in file
    };

    // Initialize trace module
    static void        initialize();

    // Add to trace log and print to console
    static inline void trace_log(bool        print,
                                 const char *file_name,
                                 TraceFile   trace_file,
                                 uint16_t    line_number,
                                 const char *msg);

    // Add to trace log and print to console
    template <typename... Args>
    static inline void trace_log_with_args(bool        print,
                                           const char *file_name,
                                           TraceFile   trace_file,
                                           uint16_t    line_number,
                                           const char *format,
                                           Args... args);

    // Get the write trace data buffer
    static uint8_t *   get_write_trace_buffer(uint32_t &buffer_size);

    // Move the write trace buffer pointer to indicate data has been written
    static void        move_write_trace_buffer_pointer(uint32_t buffer_size);

  private:
    Trace() = default;

    // Formatted print on console
    static inline void     console_print(const char *file_name,
                                         TraceFile   trace_file,
                                         uint16_t    line_number,
                                         const char *msg);

    // Add trace log variables to buffer
    static inline uint32_t trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                        uint16_t space);

    // Add trace log variables to buffer
    template <typename T>
    static inline uint32_t
    trace_add_args_to_log_buffer(uint8_t *p_buffer, uint16_t space, T value);

    // Add trace log variables to buffer
    static inline uint32_t trace_add_args_to_log_buffer(uint8_t *   p_buffer,
                                                        uint16_t    space,
                                                        const char *p_value);

    // Add trace log variables to buffer
    static inline uint32_t trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                        uint16_t space,
                                                        char *   p_value);

    // Add trace log variables to buffer
    template <typename T, typename... Args>
    static inline uint32_t trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                        uint16_t space,
                                                        T        value,
                                                        Args... args);

    // Add trace log variables to buffer
    template <typename... Args>
    static inline uint32_t trace_add_args_to_log_buffer(uint8_t *   p_buffer,
                                                        uint16_t    space,
                                                        const char *p_value,
                                                        Args... args);

    // Add trace log variables to buffer
    template <typename... Args>
    static inline uint32_t trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                        uint16_t space,
                                                        char *   p_value,
                                                        Args... args);

    // Add an entry to trace log
    template <typename... Args>
    static inline void    trace_add_entry_to_log(TraceFile trace_file,
                                                 uint16_t  line_number,
                                                 Args... args);

    // Lock the trace log for exclusive access
    // for this core
    static void           lock();
    // Release lock for the trace log
    static void           unlock();

    // Set up header for trace log entry
    static void           populate_header(void *    p_header,
                                          uint16_t  bytes_written,
                                          TraceFile trace_file,
                                          uint16_t  line_number);

    static const uint32_t SEMAPHORE_INDEX = 0;

    static info_st        info; // Global trace info
};

/**
 * @brief Formatted print on console
 *
 * @param file_name caller file name
 * @param trace_file caller file
 * @param line_number line number
 * @param msg console message
 */
inline void Trace::console_print(const char *file_name,
                                 TraceFile   trace_file,
                                 uint16_t    line_number,
                                 const char *msg)
{
    printf("C:%d [F:(%s), L:%d] %s", __CORE__, file_name, line_number, msg);
    fflush(stdout);
}

/**
 * @brief Add to trace log and print to console
 *
 * @param print flag to check whether to print on console or not
 * @param file_name caller file name
 * @param trace_file caller file
 * @param line_number line number
 * @param msg console message
 */
inline void Trace::trace_log(bool        print,
                             const char *file_name,
                             TraceFile   trace_file,
                             uint16_t    line_number,
                             const char *msg)
{
    lock();
    if (print)
    {
        console_print(file_name, trace_file, line_number, msg);
    }
    trace_add_entry_to_log(trace_file, line_number);

    unlock();
}

/**
 * @brief Add to trace log and print to console
 *
 * @tparam Args variable arguments type
 * @param print flag to check whether to print on console or not
 * @param file_name caller file name
 * @param trace_file caller file
 * @param line_number line number
 * @param format print format
 * @param args variable number of arguments
 */
template <typename... Args>
inline void Trace::trace_log_with_args(bool        print,
                                       const char *file_name,
                                       TraceFile   trace_file,
                                       uint16_t    line_number,
                                       const char *format,
                                       Args... args)
{
    lock();
    if (print)
    {
        console_print(file_name, trace_file, line_number, "");
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    if (print)
    {
        printf(format, args...);
    }
#pragma GCC diagnostic pop

    trace_add_entry_to_log(trace_file, line_number, args...);

    unlock();
}

/**
 * @brief Add trace log variables to buffer
 * This is the recursion base case.
 *
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @return uint32_t 0 always
 */
inline uint32_t Trace::trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                    uint16_t space)
{
    return 0;
}

/**
 * @brief Add trace log variables to buffer
 *
 * @tparam T value type
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @param value value
 * @return uint32_t size of data added to buffer
 */
template <typename T>
inline uint32_t
Trace::trace_add_args_to_log_buffer(uint8_t *p_buffer, uint16_t space, T value)
{
    uint16_t value_size = sizeof(T);
    uint16_t size       = sizeof(value_size) + value_size;

    if (space >= size)
    {
        // Add size of value to buffer
        memcpy(p_buffer, &value_size, sizeof(value_size));
        p_buffer += sizeof(value_size);

        memcpy(p_buffer, &value, value_size);
    }

    return size;
}

/**
 * @brief Add trace log variables to buffer
 *
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @param p_value string value
 * @return uint32_t size of data added to buffer
 */
inline uint32_t Trace::trace_add_args_to_log_buffer(uint8_t *   p_buffer,
                                                    uint16_t    space,
                                                    const char *p_value)
{
    uint16_t value_size = strlen(p_value) + 1;
    uint16_t size       = sizeof(value_size) + value_size;

    if (space >= size)
    {
        // Add size of value to buffer
        memcpy(p_buffer, &value_size, sizeof(value_size));
        p_buffer += sizeof(value_size);

        memcpy(p_buffer, p_value, value_size);
    }

    return size;
}

/**
 * @brief Add trace log variables to buffer
 *
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @param p_value string value
 * @return uint32_t size of data added to buffer
 */
inline uint32_t Trace::trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                    uint16_t space,
                                                    char *   p_value)
{
    return trace_add_args_to_log_buffer(p_buffer,
                                        space,
                                        static_cast<const char *>(p_value));
}

/**
 * @brief Add trace log variables to buffer
 *
 * @tparam T value type
 * @tparam Args variable arguments type
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @param value value
 * @param args variable number of arguments
 * @return uint32_t size of data added to buffer
 */
template <typename T, typename... Args>
inline uint32_t Trace::trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                    uint16_t space,
                                                    T        value,
                                                    Args... args)
{
    uint32_t size = trace_add_args_to_log_buffer(p_buffer, space, value);
    uint16_t remaining_space = (space > size) ? space - size : 0;

    return size + trace_add_args_to_log_buffer(p_buffer + size,
                                               remaining_space,
                                               args...);
}

/**
 * @brief Add trace log variables to buffer
 *
 * @tparam Args variable arguments type
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @param p_value string value
 * @param args variable number of arguments
 * @return uint32_t size of data added to buffer
 */
template <typename... Args>
inline uint32_t Trace::trace_add_args_to_log_buffer(uint8_t *   p_buffer,
                                                    uint16_t    space,
                                                    const char *p_value,
                                                    Args... args)
{
    uint32_t size = trace_add_args_to_log_buffer(p_buffer, space, p_value);
    uint16_t remaining_space = (space > size) ? space - size : 0;

    return size + trace_add_args_to_log_buffer(p_buffer + size,
                                               remaining_space,
                                               args...);
}

/**
 * @brief Add trace log variables to buffer
 *
 * @tparam Args variable arguments type
 * @param p_buffer buffer pointer
 * @param space amount of space available
 * @param p_value string value
 * @param args variable number of arguments
 * @return uint32_t size of data added to buffer
 */
template <typename... Args>
inline uint32_t Trace::trace_add_args_to_log_buffer(uint8_t *p_buffer,
                                                    uint16_t space,
                                                    char *   p_value,
                                                    Args... args)
{
    return trace_add_args_to_log_buffer(p_buffer,
                                        space,
                                        static_cast<const char *>(p_value),
                                        args...);
}

/**
 * @brief Add an entry to trace log
 *
 * @tparam Args variable arguments type
 * @param trace_file caller file
 * @param line_number line number
 * @param args variable number of arguments
 */
template <typename... Args>
inline void Trace::trace_add_entry_to_log(TraceFile trace_file,
                                          uint16_t  line_number,
                                          Args... args)
{
    uint8_t *      p_buffer          = &info.log_buffer[info.log_offset];
    uint16_t       spaceToWrapAround = LOG_BUFFER_SIZE - info.log_offset;
    uint16_t       spaceToLogFull    = LOG_BUFFER_SIZE - info.log_bytes;
    uint16_t       space       = std::min(spaceToWrapAround, spaceToLogFull);

    const uint16_t header_size = sizeof(trace_log_entry_hdr_st);
    uint16_t       bytes_written =
        header_size + trace_add_args_to_log_buffer(
                          p_buffer + header_size,
                          (space > header_size) ? space - header_size : 0,
                          args...);

    if (bytes_written > space)
    {
        // If the required contiguous space exceeds the end boundary of the
        // log buffer but there is free space from the beginning of the
        // buffer, then use that
        if ((bytes_written > spaceToWrapAround) &&
            (bytes_written <= (spaceToLogFull - spaceToWrapAround)))
        {
            // Invalidate rest of log buffer
            memset(p_buffer, 0, spaceToWrapAround);

            info.log_offset = 0;
            info.log_bytes += spaceToWrapAround;

            p_buffer          = &info.log_buffer[info.log_offset];
            spaceToWrapAround = LOG_BUFFER_SIZE - info.log_offset;
            spaceToLogFull    = LOG_BUFFER_SIZE - info.log_bytes;
            space             = std::min(spaceToWrapAround, spaceToLogFull);

            // Try again
            bytes_written     = header_size +
                            trace_add_args_to_log_buffer(
                                p_buffer + header_size,
                                (space > header_size) ? space - header_size : 0,
                                args...);
        }
    }

    if (bytes_written <= space)
    {
        // Pad to align to word boundary
        uint32_t pad_align =
            sizeof(uint32_t) - (bytes_written & (sizeof(uint32_t) - 1));
        if (pad_align < sizeof(uint32_t))
        {
            memset(p_buffer + bytes_written, 0, pad_align);
            bytes_written += pad_align;
        }

        populate_header(p_buffer, bytes_written, trace_file, line_number);

        info.log_offset += bytes_written;
        if (info.log_offset >= LOG_BUFFER_SIZE)
        {
            info.log_offset = 0;
        }
        info.log_bytes += bytes_written;
    }
    else
    {
        printf("Unable to save last log entry, so skipping...\n");
    }
}

#endif // _TRACE_HPP
