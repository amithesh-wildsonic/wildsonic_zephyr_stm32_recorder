#ifndef _STORAGE_TASK_HPP
#define _STORAGE_TASK_HPP

#include "FifoQueue.hpp"
#include "Messages.hpp"
#include "OnBootConfig.hpp"
#include "Task.hpp"
#include "WAVFile.hpp"
#include "constants.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_sd_card.hpp"

#include <chrono>
#include <stdint.h>
#include <time.h>

class StorageTask :
    public Task
{
  public:
    // Default constructor
    StorageTask() = default;

    // Set up and initialization for Storage Task
    void          initialize(queues_info_t &queues_info);

    // Main routine for Storage Task
    TaskErrorCode run() override;

  private:
    static const uint32_t YEAR_STRING_LENGTH   = 4; // Length of year string
    static const uint32_t MONTH_STRING_LENGTH  = 2; // Length of month string
    static const uint32_t DAY_STRING_LENGTH    = 2; // Length of day string
    static const uint32_t HOUR_STRING_LENGTH   = 2; // Length of hour string
    static const uint32_t MINUTE_STRING_LENGTH = 2; // Length of minute string
    static const uint32_t SECOND_STRING_LENGTH = 2; // Length of second string
    static const uint32_t MILLISECOND_STRING_LENGTH      = 3; // Length of
                                                              // millisecond
                                                              // string
    static const uint32_t TIME_ZONE_DIGITS_STRING_LENGTH = 4; // Length of
                                                              // time zone
                                                              // digits
                                                              // string
                                                              // (does not
                                                              // include sign)
    static const uint32_t TIME_ZONE_STRING_LENGTH =
        TIME_ZONE_DIGITS_STRING_LENGTH + 1; // Length of
                                            // time zone
                                            // string
                                            // (includes
                                            // sign)

    static const uint8_t  FILE_FLUSH_WAIT_PERIOD = 1;    // Periodic time for
                                                         // flushing a WAV file
                                                         // to file system
    static const uint32_t CAPACITY_CHECK_PERIOD  = 3600; // Periodic time for
                                                         // checking storage
                                                         // capacity
    static const uint32_t CAPACITY_LOG_PERIOD    = 300;  // Periodic time
                                                         // for logging
                                                         // storage
                                                         // capacity

    static const uint32_t WRITE_CHUNK_SIZE =
        SECTOR_SIZE_512; // Assumed to be
                         // multiple of
                         // sector size used
                         // by FileSystem

    static const uint32_t FOLDER_NAME_MAX_LENGTH              = 40;
    static const uint32_t FILE_NAME_MAX_LENGTH                = 40;
    static const uint32_t MIN_YEARMONTHDAY_FOLDER_NAME_LENGTH = 8;
    static const uint32_t MAX_YEARMONTHDAY_FOLDER_NAME_LENGTH =
        MIN_YEARMONTHDAY_FOLDER_NAME_LENGTH + 1 +
        TIME_ZONE_STRING_LENGTH; // 1 for whitespace
    static const uint32_t MIN_YEARMONTHDAYHOUR_FOLDER_NAME_LENGTH = 11;
    static const uint32_t MAX_YEARMONTHDAYHOUR_FOLDER_NAME_LENGTH =
        MIN_YEARMONTHDAYHOUR_FOLDER_NAME_LENGTH + 1 +
        TIME_ZONE_STRING_LENGTH; // 1 for whitespace

    static const uint32_t MAX_DEQUEUED_REQUESTS = 4;

    static const uint32_t NS_PRECISION_BITS     = 20; // Number of bits of
                                                      // precision for
                                                      // nanoseconds

    static const char     ROOT_DIR[];           // Path to root
                                                // directory of sd card
    static const char     TRACE_FILE_STRING[];  // Trace file format string
    static const char     FILE_FORMAT_STRING[]; // File name format string
    static const char     FILE_NAME_REGEX_FORMAT_STRING[]; // File name regex
                                                           // format string
    static const char     FILE_NAME_NO_TZ_REGEX_FORMAT_STRING[]; // File name
                                                                 // regex format
                                                                 // string

    static const char     YEARMONTHDAY_FOLDER_FORMAT_STRING[]; // Year-month-day
                                                               // folder
                                                               // name format
                                                               // string.
    static const char
        YEARMONTHDAY_FOLDER_REGEX_FORMAT_STRING[]; // Year-month-day
                                                   // folder
                                                   // name regex
                                                   // format
                                                   // string

    static const char
        YEARMONTHDAY_FOLDER_NO_TZ_REGEX_FORMAT_STRING[]; // Year-month-day
                                                         // folder
                                                         // name
                                                         // regex
                                                         // format
                                                         // string

    static const char YEARMONTHDAYHOUR_FOLDER_FORMAT_STRING[];       // Hourly
                                                                     // folder
                                                                     // name
                                                                     // format
                                                                     // string
    static const char YEARMONTHDAYHOUR_FOLDER_REGEX_FORMAT_STRING[]; // Hourly
                                                                     // folder
                                                                     // name
                                                                     // regex
                                                                     // format
                                                                     // string
    static const char
        YEARMONTHDAYHOUR_FOLDER_NO_TZ_REGEX_FORMAT_STRING[]; // Hourly
                                                             // folder
                                                             // name
                                                             // regex
                                                             // format
                                                             // string

    /**
     * @brief State of execution in Storage Task
     */
    enum class State : uint8_t
    {
        FIRST,
        WAIT_FOR_REQUEST = FIRST, // Initial State - waiting for request
        SETUP_TIMESTAMP,          // Setup new timestamp for next file
        CHECK_FOLDER_CREATION,    // Check if folders need to be created, if so,
                                  // create them
        CREATE_FILE,              // Create File
        WAIT_FOR_CREATE_FILE,     // Wait for file to be created
        WAIT_FOR_NEXT_REQUEST,    // Check for another request in
                                  // order to see if a two buffers from
                                  // different requests can be written at
                                  // the same time since they are
                                  // adjacent. This improves performance
                                  // by avoid internal low-level memcpy
        WRITE_FILE,               // Write data to file
        WAIT_FOR_WRITE_FILE,      // Wait for data to finish writing
        SEND_RESPONSE,            // Send response for request
        SYNC_FILE,                // Sync file system
        WAIT_FOR_SYNC_FILE,       // Wait for sync file
        CLOSE_FILE,               // Close file
        WAIT_FOR_CLOSE_FILE,      // Wait for file to be closed
    };

    /**
     * @brief State of execution in Storage Task
     */
    enum class TraceState : uint8_t
    {
        FIRST,
        CHECK_FOR_SPACE = FIRST,
        CHECK_NEW_TRACE_DATA, // Initial State - waiting for new data to write
        WRITE_FILE,           // Write data to file
        WAIT_FOR_WRITE_FILE,  // Wait for data to finish writing
        SYNC_FILE,            // Sync file system
        WAIT_FOR_SYNC_FILE,   // Wait for sync file
    };

    /**
     * @brief
     * Folder level starting with deepest folder
     */
    enum class folder_level_t : uint8_t
    {
        FIRST,
        YEAR_MONTH_DAY_HOUR = FIRST,
        YEAR_MONTH_DAY,
        NUM_LEVELS,
    };

    /**
     * @brief Folder check state
     */
    enum class folder_check_state_t : uint8_t
    {
        FIRST,
        CHECK_GO_UP_ONE_FOLDER = FIRST, // Check if traversing up to
                                        // next parent folder is needed
        GO_UP_ONE_FOLDER,               // Traverse up to next parent folder
        CREATE_FOLDER,                  // Create new folder
        GO_TO_NEW_FOLDER,               // Go to newly created folder
    };

    /**
     * @brief
     * Tracker information for managing filling of audio wav file
     */
    struct file_tracker_st
    {
        wFileHandle file_handle;                  // File handle to
                                                  // write wav file
        uint32_t    bytes_written_in_write_chunk; // Number of bytes written to
                                                  // write chunk/sector
        uint32_t    bytes_written_in_file;        // Number of bytes
                                                  // written to file
    };

    /**
     * @brief
     * Tracker information for managing incoming requests
     */
    struct request_tracker_st
    {
        State                state;                             // state of
                                                                // request
        storage_request_id_t request_id[MAX_DEQUEUED_REQUESTS]; // Id's
                                                                // for
                                                                // storage
                                                                // request
        uint8_t              num_requests;                      // Number of
                                                                // sequential
                                                                // concatenated
                                                                // requests
        uint32_t             current_offset_in_buffer;          // Offset into
                                                                // buffer of
                                                                // current
                                                                // request
    };

    // Check if year-month-day folder on file system a
    // valid folder created by device.
    static bool is_year_month_day_folder_valid(const char *p_folder_name,
                                               uint32_t    folder_name_length);

    // Check if year-month-day folder on file system a
    // valid folder created by device.
    static bool
    is_year_month_day_folder_valid(const w_hal_file_info_t *p_folder_info);

    // Check if year-month-day-hour folder on file system a
    // valid folder created by device.
    static bool
    is_year_month_day_hour_folder_valid(const char *p_folder_name,
                                        uint32_t    folder_name_length);

    // Check if year-month-day-hour folder on file system a
    // valid folder created by device.
    static bool
    is_year_month_day_hour_folder_valid(const w_hal_file_info_t *p_folder_info);

    // Check if file on file system a valid wav file written by device
    static bool    is_wav_file_valid(const char *p_file_name,
                                     uint32_t    file_name_length);

    // Check if file on file system a valid wav file written by device
    static bool    is_wav_file_valid(const w_hal_file_info_t *p_file_info);

    // Compare wav files to see which one is newer
    static bool    compare_wav_files(const w_hal_file_info_t *p_file_info_1,
                                     const w_hal_file_info_t *p_file_info_2);

    // Compare folders to see which one is newer
    static bool    compare_folders(const w_hal_file_info_t *p_folder_info_1,
                                   const w_hal_file_info_t *p_folder_info_2);

    // Setup current trace log file
    void           setup_trace_log_file();

    // Parse time zone from string
    bool           parse_time_zone(char *   p_string,
                                   uint32_t length,
                                   int &    time_zone_hour,
                                   int &    time_zone_minute);

    // Parse folder name
    bool           parse_folder_name(char *   p_folder_name,
                                     uint32_t folder_name_length,
                                     bool     hourly_or_daily,
                                     tm &     tm_struct);

    // Parse time from filename
    bool           parse_time_from_filename(char *    p_filename,
                                            uint32_t  filename_length,
                                            tm &      time_struct,
                                            uint32_t &nanoseconds);

    // Get the duration of a file in seconds
    void           get_file_duration(const char *             p_file_name,
                                     uint32_t                 file_name_length,
                                     uint32_t                 file_size,
                                     w_hal_rtc_time_delta_st &time_delta);

    // Setup current timestamp
    void           setup_current_time();

    // Check if need to go up one folder to create next new folder
    bool           check_go_up_one_folder() const;

    // Check if new folder is needed and create one.
    bool           check_new_folder();

    // Update the bookkeeping for the SD card usage.
    void           update_usage_stats_after_flush(uint32_t bytes);

    // Check if a buffer has been fully processed.
    // If so, send response for request.
    void           check_send_request_response();

    // Check if any buffers are completed so that their respective
    // requests can be sent back.
    void           check_current_write_buffers_and_send_response();

    // Handle saving of trace log
    bool           handle_save_trace_log();

    // check for available SD card space
    void           check_for_available_space();

    // Aborting storage requests
    void           abort_storage_request();

    // Get the deepest folder level
    folder_level_t get_deepest_folder_level();

    bool           m_abort_requests; // Flag to abort requests

    int            m_time_zone_offset; // Time zone offset
    char           m_time_zone_sign;   // Time zone sign
    uint16_t       m_time_zone_hour;   // Time zone hour
                                       // (equivalent to
                                       // offset / 3600)
    uint16_t       m_time_zone_minute; // Time zone minute
                                       // (equivalent to
                                       // (offset % 3600) / 60)

    DeviceConfig::folder_structure_t m_folder_structure;    // Folder structure
    uint32_t                         m_recording_length;    // Recording length
                                                            // in seconds
    uint32_t                         m_total_wav_file_size; // Total size
                                                            // of wav file
                                                            // to be written
                                                            // to file system
    uint32_t                         m_sd_card_slack_space; // The slack
                                                            // space to
                                                            // check before
                                                            // the SD card
                                                            // gets to full
    uint32_t                         m_slack_space_for_trace_log; // The slack
                                                                  // space to
                                                                  // check
                                                                  // before the
                                                                  // SD card
                                                                  // gets to
                                                                  // full to
                                                                  // load
                                                                  // trace
                                                                  // log data
    file_tracker_st                  m_file_tracker;    // File tracker
    request_tracker_st               m_request_tracker; // Request tracker

    TraceState                       m_trace_state; // Trace save flow state
    file_tracker_st m_trace_file_tracker;     // File tracker for trace file
    uint8_t *       m_p_current_trace_buffer; // Trace buffer
                                              // currently being written.
    uint32_t        m_trace_buffer_size;      // Trace buffer size

    wav_header_t    m_wav_header; // Header for WAV file
    bool            m_wav_header_written;

    char m_folder_name[FOLDER_NAME_MAX_LENGTH + 1]; // Folder name string
    char m_file_name[FILE_NAME_MAX_LENGTH + 1];     // File name string

    w_hal_rtc_time_st    m_current_time;       // Current time to use to
                                               // populate file and folder name
    uint8_t              m_current_hour;       // Last seen hour of time.
                                               // Used to compare against
                                               // hour of current time
                                               // after update.
    uint8_t              m_current_dayInMonth; // Last seen day of time.
                                               // Used to compare against
                                               // hour of current time
                                               // after update.
    uint8_t              m_current_month;      // Last seen month of time.
                                               // Used to compare against
                                               // hour of current time
                                               // after update.
    uint16_t             m_current_year;       // Last seen year of time.
                                               // Used to compare against
                                               // hour of current time
                                               // after update.

    tm                   m_time_struct;          // Current time structure
    folder_level_t       m_current_folder_level; // Current folder level
    folder_check_state_t m_folder_check_state;   // Action to perform
                                                 // at current folder level

    bool                 m_file_flush_close;        // Flag to close
                                                    // file to file system
    uint32_t             m_write_bytes_in_progress; // Number of bytes
                                                    // currently being
                                                    // written to file
    uint32_t             m_unflushed_write_bytes;   // Number of bytes
                                                    // written to file
                                                    // that have not been
                                                    // flushed to file
                                                    // system
    uint32_t             m_num_bytes_captured;      // The number of
                                                    // bytes captured
    uint32_t             m_data_byte_rate;          // data byte rate
                                                    // for system
    uint32_t             m_unflushed_trace_bytes;   // Number of bytes
                                                    // written to file
                                                    // that have not been
                                                    // flushed to file
                                                    // system

    w_hal_rtc_time_st    m_last_capacity_check_time;  // Last time storage
                                                      // was queried for
                                                      // amount of
                                                      // used/free space
    w_hal_rtc_time_st    m_last_logged_capacity_time; // Last time storage
                                                      // used space
                                                      // was logged
    w_hal_rtc_time_st    m_last_flush_time;           // Last time WAV file
                                                      // flushed to
                                                      // file system
    uint64_t             m_nanoseconds_per_byte;      // Nanoseconds per
                                                      // byte shifted up
                                                      // to preserve
                                                      // precision

    // Array of storage request messages
    StorageRequestMsg *  m_p_storage_msgs;
    // request queue from audio task
    StorageFifoQueue *   m_p_storage_request_queue;
    // response queue to audio task
    StorageFifoQueue *   m_p_storage_response_queue;
};

#endif // _STORAGE_TASK_HPP
