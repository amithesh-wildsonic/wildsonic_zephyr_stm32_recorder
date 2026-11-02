#include "trace.hpp"
#ifndef TRACE_TEST
#include "w_hal_rtc.hpp"
#endif
#include "w_hal_system.hpp"
#include <stdint.h>
#include <string.h>
#include <time.h>

TRACE_SHARED Trace::info_st Trace::info;

/**
 * @brief Initialize trace module
 */
void                        Trace::initialize()
{
    info.saved_log_offset = 0;
    info.log_offset       = 0;
    info.log_bytes        = 0;
}

/**
 * @brief Set up header for trace log entry
 *
 * @param p_header header to be set up
 * @param bytes_written total size of trace log entry
 * @param trace_file caller file
 * @param line_number line number
 */
void Trace::populate_header(void *    p_header,
                            uint16_t  bytes_written,
                            TraceFile trace_file,
                            uint16_t  line_number)
{
    trace_log_entry_hdr_st header;
#ifdef TRACE_TEST
    static time_t time_count = 0;
#endif

    // Populate header
    header.entry_size = bytes_written;
#ifdef TRACE_TEST
    header.time = ++time_count;
#else
    header.time = w_hal_rtc_get_time();
#endif
    header.core              = __CORE__;
    header.trace_file_number = static_cast<uint16_t>(trace_file);
    header.line_number       = line_number;
    header.rsvd0             = 0;

    memcpy(p_header, &header, sizeof(header));
}

/**
 * @brief Lock the trace log for exclusive access for this core
 */
void Trace::lock()
{
#ifndef TRACE_TEST
    while (w_hal_system_get_semaphore(SEMAPHORE_INDEX) == wHalErrorCode::BUSY)
    {
    }
#endif
}

/**
 * @brief Release lock for the trace log
 */
void Trace::unlock()
{
#ifndef TRACE_TEST
    w_hal_system_free_semaphore(SEMAPHORE_INDEX);
#endif
}

/**
 * @brief Get the write trace data buffer
 *
 * @param buffer_size [out] : the size of buffer data
 * @return uint8_t* start of buffer data pointer
 */
uint8_t *Trace::get_write_trace_buffer(uint32_t &buffer_size)
{
    uint8_t *p_buffer;

    uint16_t log_bytes = info.log_bytes;

    if (log_bytes == 0)
    {
        p_buffer    = nullptr;
        buffer_size = 0;
    }
    else
    {
        lock();

        uint16_t saved_log_offset = info.saved_log_offset;

        log_bytes                 = info.log_bytes;

        p_buffer                  = &info.log_buffer[saved_log_offset];

        if (saved_log_offset + log_bytes >= LOG_BUFFER_SIZE)
        {
            buffer_size = LOG_BUFFER_SIZE - saved_log_offset;
        }
        else
        {
            buffer_size = log_bytes;
        }

        unlock();
    }

    return p_buffer;
}

/**
 * @brief Move the write trace buffer pointer to indicate data has been written
 *
 * @param buffer_size the amount of buffer data written
 */
void Trace::move_write_trace_buffer_pointer(uint32_t buffer_size)
{
    lock();

    info.saved_log_offset += buffer_size;
    if (info.saved_log_offset >= LOG_BUFFER_SIZE)
    {
        info.saved_log_offset -= LOG_BUFFER_SIZE;
    }
    info.log_bytes -= buffer_size;

    unlock();
}