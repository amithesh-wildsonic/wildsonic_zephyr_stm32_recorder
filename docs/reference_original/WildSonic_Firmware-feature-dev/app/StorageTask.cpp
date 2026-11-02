#if __CORE__ == STORAGE_CORE

#include "StorageTask.hpp"
#include "WAVFile.hpp"
#include "assert.hpp"
#include "constants.hpp"
#include "device_config.hpp"
#include "shared.hpp"
#include "trace.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_sd_card.hpp"

#include "re.h"

#include <algorithm>
#include <chrono>
#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

const char StorageTask::ROOT_DIR[]          = "/";

const char StorageTask::TRACE_FILE_STRING[] = "trace_log.bin";
const char StorageTask::FILE_FORMAT_STRING[] =
    "%04d%02d%02d_%02d%02d%02d_%03d.wav";
const char StorageTask::FILE_NAME_REGEX_FORMAT_STRING[] =
    "\\d\\d\\d\\d\\d\\d\\d\\d_\\d\\d\\d\\d\\d\\d_\\d\\d\\d [\\+\\-]"
    "\\d\\d\\d\\d\\.wav";
const char StorageTask::FILE_NAME_NO_TZ_REGEX_FORMAT_STRING[] =
    "\\d\\d\\d\\d\\d\\d\\d\\d_\\d\\d\\d\\d\\d\\d_\\d\\d\\d\\.wav";

const char StorageTask::YEARMONTHDAY_FOLDER_FORMAT_STRING[] = "%04d%02d%02d";
const char StorageTask::YEARMONTHDAY_FOLDER_REGEX_FORMAT_STRING[] =
    "\\d\\d\\d\\d\\d\\d\\d\\d [\\+\\-]\\d\\d\\d\\d";
const char StorageTask::YEARMONTHDAY_FOLDER_NO_TZ_REGEX_FORMAT_STRING[] =
    "\\d\\d\\d\\d\\d\\d\\d\\d";

const char StorageTask::YEARMONTHDAYHOUR_FOLDER_FORMAT_STRING[] =
    "%04d%02d%02d_%02d";
const char StorageTask::YEARMONTHDAYHOUR_FOLDER_REGEX_FORMAT_STRING[] =
    "\\d\\d\\d\\d\\d\\d\\d\\d_\\d\\d [\\+\\-]\\d\\d\\d\\d";
const char StorageTask::YEARMONTHDAYHOUR_FOLDER_NO_TZ_REGEX_FORMAT_STRING[] =
    "\\d\\d\\d\\d\\d\\d\\d\\d_\\d\\d";

/**
 * @brief
 * Check if file on file system a valid wav file written by device
 *
 * @param p_file_name file name
 * @param file_name_length length of file name
 *
 * @return true if file is an expected wav file
 * @return false otherwise
 */
bool StorageTask::is_wav_file_valid(const char *p_file_name,
                                    uint32_t    file_name_length)
{
    // Filename should contain file name format and file extension
    int match_length = 0;
    int match_index  = re_match(FILE_NAME_REGEX_FORMAT_STRING,
                               p_file_name,
                               static_cast<int>(file_name_length),
                               &match_length);

    if ((match_index != 0) ||
        (static_cast<uint32_t>(match_length) != file_name_length))
    {
        match_index = re_match(FILE_NAME_NO_TZ_REGEX_FORMAT_STRING,
                               p_file_name,
                               static_cast<int>(file_name_length),
                               &match_length);
    }

    return ((match_index == 0) &&
            (static_cast<uint32_t>(match_length) == file_name_length));
}

/**
 * @brief
 * Check if file on file system a valid wav file written by device
 *
 * @param p_file_info file information from HAL
 * @return true if file is an expected wav file
 * @return false otherwise
 */
bool StorageTask::is_wav_file_valid(const w_hal_file_info_t *p_file_info)
{
    return is_wav_file_valid(p_file_info->p_file_name,
                             p_file_info->file_name_length);
}

/**
 * @brief
 * Check if year-month-day folder on file system a
 * valid folder created by device.
 *
 * @param p_folder_name folder name
 * @param folder_name_length length of folder name
 * @return true if folder is a valid folder created by device
 * @return false otherwise
 */
bool StorageTask::is_year_month_day_folder_valid(const char *p_folder_name,
                                                 uint32_t    folder_name_length)
{
    int match_length = 0;

    if ((folder_name_length < MIN_YEARMONTHDAY_FOLDER_NAME_LENGTH) ||
        (folder_name_length > MAX_YEARMONTHDAY_FOLDER_NAME_LENGTH))
    {
        return false;
    }

    int match_index = re_match(YEARMONTHDAY_FOLDER_REGEX_FORMAT_STRING,
                               p_folder_name,
                               static_cast<int>(folder_name_length),
                               &match_length);

    if ((match_index != 0) ||
        (static_cast<uint32_t>(match_length) != folder_name_length))
    {
        match_index = re_match(YEARMONTHDAY_FOLDER_NO_TZ_REGEX_FORMAT_STRING,
                               p_folder_name,
                               static_cast<int>(folder_name_length),
                               &match_length);
    }

    return ((match_index == 0) &&
            (static_cast<uint32_t>(match_length) == folder_name_length));
}

/**
 * @brief
 * Check if year-month-day folder on file system a
 * valid folder created by device.
 *
 * @param p_folder_info folder information from HAL
 * @return true if folder is a valid folder created by device
 * @return false otherwise
 */
bool StorageTask::is_year_month_day_folder_valid(
    const w_hal_file_info_t *p_folder_info)
{
    return is_year_month_day_folder_valid(p_folder_info->p_file_name,
                                          p_folder_info->file_name_length);
}

/**
 * @brief
 * Check if year-month-day-hour folder on file system a
 * valid folder created by device.
 *
 * @param p_folder_name folder name
 * @param folder_name_length length of folder name
 * @return true if folder is a valid folder created by device
 * @return false otherwise
 */
bool StorageTask::is_year_month_day_hour_folder_valid(
    const char *p_folder_name,
    uint32_t    folder_name_length)
{
    int match_length = 0;

    if ((folder_name_length < MIN_YEARMONTHDAYHOUR_FOLDER_NAME_LENGTH) ||
        (folder_name_length > MAX_YEARMONTHDAYHOUR_FOLDER_NAME_LENGTH))
    {
        return false;
    }

    int match_index = re_match(YEARMONTHDAYHOUR_FOLDER_REGEX_FORMAT_STRING,
                               p_folder_name,
                               static_cast<int>(folder_name_length),
                               &match_length);

    if ((match_index != 0) ||
        (static_cast<uint32_t>(match_length) != folder_name_length))
    {
        match_index =
            re_match(YEARMONTHDAYHOUR_FOLDER_NO_TZ_REGEX_FORMAT_STRING,
                     p_folder_name,
                     static_cast<int>(folder_name_length),
                     &match_length);
    }

    return ((match_index == 0) &&
            (static_cast<uint32_t>(match_length) == folder_name_length));
}

/**
 * @brief
 * Check if year-month-day-hour folder on file system a
 * valid folder created by device.
 *
 * @param p_folder_info folder information from HAL
 * @return true if folder is a valid folder created by device
 * @return false otherwise
 */
bool StorageTask::is_year_month_day_hour_folder_valid(
    const w_hal_file_info_t *p_folder_info)
{
    return is_year_month_day_hour_folder_valid(p_folder_info->p_file_name,
                                               p_folder_info->file_name_length);
}

/**
 * @brief
 * Compare wav files to see which one is newer
 *
 * @param p_file_info_1 first file information from HAL
 * @param p_file_info_2 second file information from HAL
 * @return true if first file is newer than second one
 * @return false if second file is newer than first one
 */
bool StorageTask::compare_wav_files(const w_hal_file_info_t *p_file_info_1,
                                    const w_hal_file_info_t *p_file_info_2)
{
    return (strcmp(p_file_info_1->p_file_name, p_file_info_2->p_file_name) > 0);
}

/**
 * @brief
 * Compare folders to see which one is newer
 *
 * @param p_folder_info_1 first folder information from HAL
 * @param p_folder_info_2 second folder information from HAL
 * @return true if first folder is newer than second one
 * @return false if second folder is newer than first one
 */
bool StorageTask::compare_folders(const w_hal_file_info_t *p_folder_info_1,
                                  const w_hal_file_info_t *p_folder_info_2)
{
    return (strcmp(p_folder_info_1->p_file_name, p_folder_info_2->p_file_name) >
            0);
}

/**
 * @brief Extract first N digits from string buffer and return value
 *
 * @param p_buffer string buffer
 * @param num_digits number of digits to look up to
 * @return int value from first N digits
 */
static int extract_first_digits(char *p_buffer, uint32_t num_digits)
{
    char prev_char;

    prev_char            = p_buffer[num_digits];
    p_buffer[num_digits] = 0;

    int num              = atoi(p_buffer);

    // Restore previous character
    p_buffer[num_digits] = prev_char;

    return num;
}

/**
 * @brief Get the deepest folder level
 *
 * @return folder_level_t the deepest folder level
 */
StorageTask::folder_level_t StorageTask::get_deepest_folder_level()
{
    folder_level_t folder_level = static_cast<folder_level_t>(
        static_cast<int>(folder_level_t::NUM_LEVELS) - 1);

    if (m_folder_structure == DeviceConfig::folder_structure_t::DAY)
    {
        folder_level = folder_level_t::YEAR_MONTH_DAY;
    }
    else if (m_folder_structure == DeviceConfig::folder_structure_t::HOUR)
    {
        folder_level = folder_level_t::YEAR_MONTH_DAY_HOUR;
    }

    return folder_level;
}

/**
 * @brief
 * Setup current trace log file
 */
void StorageTask::setup_trace_log_file()
{
    // Set up file name and create file
    strncpy(m_file_name, TRACE_FILE_STRING, FILE_NAME_MAX_LENGTH);

    wHalErrorCode error_code =
        w_hal_sd_card_open_file(m_file_name,
                                FILE_NAME_MAX_LENGTH,
                                m_trace_file_tracker.file_handle);

    wHalErrorCode error_set[] = {wHalErrorCode::NO_ERROR,
                                 wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST};
    SYSTEM_ASSERT_IN_SET(error_code,
                         error_set,
                         sizeof(error_set) / sizeof(error_set[0]));

    if (error_code == wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST)
    {
        error_code =
            w_hal_sd_card_create_file(m_file_name,
                                      FILE_NAME_MAX_LENGTH,
                                      m_trace_file_tracker.file_handle);
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
    }
}

/**
 * @brief Parse time from filename
 *
 * @param p_filename file name to parse. Assumption is that the file name is in
 * the format of "YYYYMMDD_HHMMSS_mmm" or "YYYYMMDD_HHMMSS_mmm_[+-]ZZZZ".
 * @param file_name_length length of the file name
 * @param time_struct[out] time structure to store parsed time
 * @param nanoseconds[out] nanoseconds to store parsed time
 *
 * @return true if time is parsed successfully, false otherwise
 */
bool StorageTask::parse_time_from_filename(char *    p_filename,
                                           uint32_t  file_name_length,
                                           tm &      time_struct,
                                           uint32_t &nanoseconds)
{

    if (!is_wav_file_valid(p_filename, file_name_length))
    {
        return false;
    }

    memset(&time_struct, 0, sizeof(tm));

    int match_index = 0;

    time_struct.tm_year =
        extract_first_digits(&p_filename[match_index], YEAR_STRING_LENGTH);
    // Skip over year
    match_index += static_cast<int>(YEAR_STRING_LENGTH);

    time_struct.tm_mon =
        extract_first_digits(&p_filename[match_index], MONTH_STRING_LENGTH);
    // Skip over month
    match_index += static_cast<int>(MONTH_STRING_LENGTH);

    time_struct.tm_mday =
        extract_first_digits(&p_filename[match_index], DAY_STRING_LENGTH);
    // Skip over day in month and underscore
    match_index += static_cast<int>(DAY_STRING_LENGTH) + 1;

    time_struct.tm_hour =
        extract_first_digits(&p_filename[match_index], HOUR_STRING_LENGTH);
    // Skip over hour
    match_index += static_cast<int>(HOUR_STRING_LENGTH);

    time_struct.tm_min =
        extract_first_digits(&p_filename[match_index], MINUTE_STRING_LENGTH);
    // Skip over minute
    match_index += static_cast<int>(MINUTE_STRING_LENGTH);

    time_struct.tm_sec =
        extract_first_digits(&p_filename[match_index], SECOND_STRING_LENGTH);
    // Skip over second and underscore
    match_index += static_cast<int>(SECOND_STRING_LENGTH) + 1;

    nanoseconds =
        static_cast<uint32_t>(extract_first_digits(&p_filename[match_index],
                                                   MILLISECOND_STRING_LENGTH)) *
        (NSEC_PER_SEC / 1000);
    // Skip over millisecond
    match_index += static_cast<int>(MILLISECOND_STRING_LENGTH);

    // If there is a space after the millisecond, then there is a time
    // zone offset
    if ((static_cast<uint32_t>(match_index) < file_name_length) &&
        isspace(p_filename[match_index]))
    {
        // This function always returns the UTC time. If the timezone is present
        // in the filename, then the time is in the local timezone. Adjust the
        // time to UTC.
        match_index++;
        int  time_zone_hour;
        int  time_zone_minute;
        bool success = parse_time_zone(&p_filename[match_index],
                                       file_name_length -
                                           static_cast<uint32_t>(match_index),
                                       time_zone_hour,
                                       time_zone_minute);
        SYSTEM_ASSERT_TRUE(success);

        // Adjust the time to UTC
        time_struct.tm_hour -= time_zone_hour;
        time_struct.tm_min -= time_zone_minute;
    }
    else
    {
        // No explicit timezone is present in the filename; interpret the
        // timestamp as local time in the configured timezone and convert to
        // UTC.
        int tz_hour = static_cast<int>(m_time_zone_hour);
        int tz_min  = static_cast<int>(m_time_zone_minute);

        // Convert local time to UTC
        time_struct.tm_hour -= tz_hour;
        time_struct.tm_min -= tz_min;
    }

    // Subtract base year and month
    time_struct.tm_year -= static_cast<int>(TIME_BASE_YEAR);
    time_struct.tm_mon -= static_cast<int>(TIME_BASE_MONTH);

    return true;
}

/**
 * @brief Get the duration of a file in seconds
 *
 * @param p_file_name Pointer to the file name string
 * @param file_name_length Length of the file name string
 * @param file_size Size of the file in bytes
 * @param time_delta [out] : Duration of the file elapsed since the start of the
 * file
 */
void StorageTask::get_file_duration(const char *             p_file_name,
                                    uint32_t                 file_name_length,
                                    uint32_t                 file_size,
                                    w_hal_rtc_time_delta_st &time_delta)
{
    time_delta.seconds     = 0;
    time_delta.nanoseconds = 0;

    // If no more than the file header, then the file has no data
    if (file_size <= sizeof(wav_header_t))
    {
        return;
    }
    // Read the file header to get the sample rate of the file
    // and get the time elapsed since the start of the file.
    // This will be used to calculate the time of the next file.
    wFileHandle   file_handle;
    wHalErrorCode error_code =
        w_hal_sd_card_open_file(p_file_name, file_name_length, file_handle);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        return;
    }

    wav_header_t wav_header;
    uint32_t     bytes_read = 0;
    error_code =
        w_hal_sd_card_read_from_file(reinterpret_cast<uint8_t *>(&wav_header),
                                     sizeof(wav_header_t),
                                     bytes_read,
                                     file_handle);

    error_code = w_hal_sd_card_close_file(file_handle);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    if ((error_code == wHalErrorCode::NO_ERROR) &&
        (bytes_read == sizeof(wav_header_t)))
    {
        // Get the byte rate from the file header and the number of samples
        // from the file size. Don't use the number of samples from the file
        // header as it may not be accurate due to the file being truncated
        // when the device was powered off. From the byte rate and the file
        // size, calculate the duration of the file in nanoseconds.
        const uint32_t byte_rate = wav_header.format_subchunk.byte_rate;

        if (byte_rate > 0)
        {
            // Get the number of samples in the file. Left and right channels
            // are counted as 1 sample.
            uint64_t data_size = file_size - sizeof(wav_header_t);
            time_delta.seconds = static_cast<int64_t>(data_size / byte_rate);
            time_delta.nanoseconds =
                ((data_size % byte_rate) * NSEC_PER_SEC) / byte_rate;
        }
    }
}

/**
 * @brief Parse time zone from string
 *
 * @param p_string the string with time zone to parse (format: [+-]HHMM)
 * @param length length of the string
 * @param time_zone_hour [out] time zone hour
 * @param time_zone_minute [out] time zone minute
 * @return true if time zone is parsed successfully
 * @return false otherwise
 */
bool StorageTask::parse_time_zone(char *   p_string,
                                  uint32_t length,
                                  int &    time_zone_hour,
                                  int &    time_zone_minute)
{
    bool is_negative = false;

    switch (*p_string)
    {
        case '-':
        {
            is_negative = true;
        }
        // FALL-THROUGH
        case '+':
        {
            if (length < HOUR_STRING_LENGTH + MINUTE_STRING_LENGTH + 1)
            {
                return false;
            }

            p_string++;
            length--;
        }
        default:
        {
            break;
        }
    }

    if (length < HOUR_STRING_LENGTH + MINUTE_STRING_LENGTH)
    {
        return false;
    }

    time_zone_hour = extract_first_digits(p_string, HOUR_STRING_LENGTH);
    p_string += HOUR_STRING_LENGTH;
    time_zone_minute = extract_first_digits(p_string, MINUTE_STRING_LENGTH);

    if (is_negative)
    {
        time_zone_hour   = -time_zone_hour;
        time_zone_minute = -time_zone_minute;
    }

    return true;
}

/**
 * @brief Parse folder name and extract time from it
 *
 * @param p_folder_name folder name
 * @param folder_name_length length of folder name
 * @param hourly_or_daily hourly or daily folder
 * @param tm_struct time structure to store parsed time
 * @return true if folder name is parsed successfully
 * @return false otherwise
 */
bool StorageTask::parse_folder_name(char *   p_folder_name,
                                    uint32_t folder_name_length,
                                    bool     hourly_or_daily,
                                    tm &     tm_struct)
{
    bool is_valid =
        hourly_or_daily ?
            is_year_month_day_hour_folder_valid(p_folder_name,
                                                folder_name_length) :
            is_year_month_day_folder_valid(p_folder_name, folder_name_length);

    SYSTEM_ASSERT_TRUE(is_valid);

    // Capture the time from the folder name
    memset(&tm_struct, 0, sizeof(tm_struct));
    tm_struct.tm_year =
        extract_first_digits(p_folder_name, YEAR_STRING_LENGTH) -
        static_cast<int>(TIME_BASE_YEAR);
    p_folder_name += YEAR_STRING_LENGTH;
    tm_struct.tm_mon =
        extract_first_digits(p_folder_name, MONTH_STRING_LENGTH) -
        static_cast<int>(TIME_BASE_MONTH);
    p_folder_name += MONTH_STRING_LENGTH;
    tm_struct.tm_mday = extract_first_digits(p_folder_name, DAY_STRING_LENGTH);
    // Skip over day in month
    p_folder_name += DAY_STRING_LENGTH;
    if (hourly_or_daily)
    {
        // Skip over underscore
        p_folder_name++;

        tm_struct.tm_hour =
            extract_first_digits(p_folder_name, HOUR_STRING_LENGTH);
        // Skip over hour
        p_folder_name += HOUR_STRING_LENGTH;
    }

    if ((p_folder_name < &p_folder_name[folder_name_length]) &&
        isspace(*p_folder_name))
    {
        // Increment pointer to skip over space
        p_folder_name++;

        int  time_zone_hour;
        int  time_zone_minute;
        bool success = parse_time_zone(
            p_folder_name,
            static_cast<uint32_t>(&p_folder_name[folder_name_length] -
                                  p_folder_name),
            time_zone_hour,
            time_zone_minute);

        if (!success)
        {
            return false;
        }

        // Adjust the time to UTC
        tm_struct.tm_hour -= time_zone_hour;
        tm_struct.tm_min -= time_zone_minute;
    }
    else
    {
        // No explicit timezone is present in the folder name; interpret the
        // timestamp as local time in the configured timezone and convert to
        // UTC.
        int tz_hour = static_cast<int>(m_time_zone_hour);
        int tz_min  = static_cast<int>(m_time_zone_minute);

        tm_struct.tm_hour -= tz_hour;
        tm_struct.tm_min -= tz_min;
    }

    return true;
}

/**
 * @brief
 * Setup current timestamp
 */
void StorageTask::setup_current_time()
{
    static tm      last_tm;

    folder_level_t folder_level = static_cast<folder_level_t>(
        static_cast<int>(folder_level_t::NUM_LEVELS) - 1);
    w_hal_rtc_time_st       latest_file_time_in_root       = {.time_seconds     = 0,
                                                  .time_nanoseconds = 0};
    w_hal_rtc_time_st       latest_file_time_in_folder     = {.time_seconds     = 0,
                                                    .time_nanoseconds = 0};
    w_hal_rtc_time_st       latest_folder_time             = {.time_seconds     = 0,
                                            .time_nanoseconds = 0};
    w_hal_rtc_time_delta_st latest_file_duration_in_root   = {.seconds     = 0,
                                                            .nanoseconds = 0};
    w_hal_rtc_time_delta_st latest_file_duration_in_folder = {.seconds     = 0,
                                                              .nanoseconds = 0};
    uint32_t                file_size                      = 0;

    wHalErrorCode           error_code =
        w_hal_sd_card_goto_folder(ROOT_DIR, FOLDER_NAME_MAX_LENGTH);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Get the latest timestamp in root folder
    error_code =
        w_hal_sd_card_last_file_created(m_file_name,
                                        FILE_NAME_MAX_LENGTH,
                                        file_size,
                                        StorageTask::is_wav_file_valid,
                                        StorageTask::compare_wav_files);

    // Get time from last created file or folder name
    if (error_code != wHalErrorCode::SD_CARD_FOLDER_EMPTY)
    {
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

        LOG_MSG_WITH_ARGS(TraceFile::STORAGE_TASK_CPP,
                          "Last file detected in root folder: %s\n",
                          m_file_name);

        const uint32_t file_name_length =
            std::min(strlen(m_file_name),
                     static_cast<size_t>(FILE_NAME_MAX_LENGTH));

        uint32_t nanoseconds = 0;
        bool     success     = parse_time_from_filename(m_file_name,
                                                file_name_length,
                                                last_tm,
                                                nanoseconds);
        if (success)
        {
            latest_file_time_in_root.time_seconds = w_hal_rtc_timegm(&last_tm);
            latest_file_time_in_root.time_nanoseconds = nanoseconds;

            get_file_duration(m_file_name,
                              file_name_length,
                              file_size,
                              latest_file_duration_in_root);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::STORAGE_TASK_CPP,
                              "Failed to parse time from file name: %s\n",
                              m_file_name);
        }
    }

    // Navigate to latest folder
    int index;
    for (index = 0; index < static_cast<int>(folder_level_t::NUM_LEVELS);
         index++)
    {
        sd_is_file_valid_func is_folder_valid_func;

        switch (folder_level)
        {
            case folder_level_t::YEAR_MONTH_DAY:
            {
                is_folder_valid_func =
                    StorageTask::is_year_month_day_folder_valid;
                break;
            }
            case folder_level_t::YEAR_MONTH_DAY_HOUR:
            {
                is_folder_valid_func =
                    StorageTask::is_year_month_day_hour_folder_valid;
                break;
            }
            default:
            {
                // Unexpected condition here
                SYSTEM_ASSERT_TRUE(false);
                break;
            }
        }

        // Navigate to latest folder
        error_code =
            w_hal_sd_card_last_folder_created(m_folder_name,
                                              FOLDER_NAME_MAX_LENGTH,
                                              is_folder_valid_func,
                                              StorageTask::compare_folders);

        if (error_code == wHalErrorCode::SD_CARD_FOLDER_EMPTY)
        {
            break;
        }

        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

        switch (folder_level)
        {
            case folder_level_t::YEAR_MONTH_DAY:
            {
                bool success = parse_folder_name(m_folder_name,
                                                 strlen(m_folder_name),
                                                 false,
                                                 last_tm);
                SYSTEM_ASSERT_TRUE(success);

                break;
            }
            case folder_level_t::YEAR_MONTH_DAY_HOUR:
            {
                bool success = parse_folder_name(m_folder_name,
                                                 strlen(m_folder_name),
                                                 true,
                                                 last_tm);
                SYSTEM_ASSERT_TRUE(success);

                break;
            }
            default:
            {
                // Unexpected condition here
                SYSTEM_ASSERT_TRUE(false);
                break;
            }
        }

        w_hal_rtc_time_st time;
        time.time_seconds     = w_hal_rtc_timegm(&last_tm);
        time.time_nanoseconds = 0;
        if (time > latest_folder_time)
        {
            latest_folder_time = time;
        }

        error_code =
            w_hal_sd_card_goto_folder(m_folder_name, FOLDER_NAME_MAX_LENGTH);
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

        error_code =
            w_hal_sd_card_last_file_created(m_file_name,
                                            FILE_NAME_MAX_LENGTH,
                                            file_size,
                                            StorageTask::is_wav_file_valid,
                                            StorageTask::compare_wav_files);

        // Get time from last created file or folder name
        if (error_code != wHalErrorCode::SD_CARD_FOLDER_EMPTY)
        {
            SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

            LOG_MSG_WITH_ARGS(TraceFile::STORAGE_TASK_CPP,
                              "Last file detected in folder: %s is %s\n",
                              m_folder_name,
                              m_file_name);

            const uint32_t file_name_length =
                std::min(strlen(m_file_name),
                         static_cast<size_t>(FILE_NAME_MAX_LENGTH));

            uint32_t nanoseconds = 0;
            bool     success     = parse_time_from_filename(m_file_name,
                                                    file_name_length,
                                                    last_tm,
                                                    nanoseconds);

            if (success)
            {
                time.time_seconds     = w_hal_rtc_timegm(&last_tm);
                time.time_nanoseconds = nanoseconds;
                if (time > latest_file_time_in_folder)
                {
                    latest_file_time_in_folder = time;
                }

                get_file_duration(m_file_name,
                                  file_name_length,
                                  file_size,
                                  latest_file_duration_in_folder);
            }
            else
            {
                LOG_MSG_WITH_ARGS(TraceFile::STORAGE_TASK_CPP,
                                  "Failed to parse time from file name: %s\n",
                                  m_file_name);
            }
        }

        folder_level =
            static_cast<folder_level_t>(static_cast<int>(folder_level) - 1);
    }

    LOG_MSG_WITH_ARGS(TraceFile::STORAGE_TASK_CPP,
                      "Last folder detected: %s\n",
                      m_folder_name);

    // Compare latest file time in root and folder
    w_hal_rtc_time_delta_st latest_file_duration     = {.seconds     = 0,
                                                    .nanoseconds = 0};
    bool                    is_latest_time_from_file = false;
    m_current_time.time_nanoseconds                  = 0;
    if ((latest_file_time_in_root > latest_file_time_in_folder) &&
        (latest_file_time_in_root > latest_folder_time))
    {
        m_current_time.time_seconds = latest_file_time_in_root;
        latest_file_duration        = latest_file_duration_in_root;
        is_latest_time_from_file    = true;
    }
    else if (latest_folder_time > 0)
    {
        if (latest_file_time_in_folder >= latest_folder_time)
        {
            m_current_time.time_seconds = latest_file_time_in_folder;
            latest_file_duration        = latest_file_duration_in_folder;
            is_latest_time_from_file    = true;
        }
        else
        {
            m_current_time.time_seconds = latest_folder_time;
        }
    }
    else
    {
        w_hal_rtc_get_time(m_current_time);
    }

    if ((latest_file_time_in_root == 0) && (latest_folder_time == 0) &&
        (latest_file_time_in_folder == 0))
    {
        shared_glbl.storage_info.sd_card_is_empty = true;
    }
    else
    {
        shared_glbl.storage_info.sd_card_is_empty = false;
    }

    // Create folder directory structure from latest timestamp
    error_code = w_hal_sd_card_goto_folder(ROOT_DIR, FOLDER_NAME_MAX_LENGTH);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Create new folder and switch to that. Before creating new folder, convert
    // the time to local time zone.
    w_hal_rtc_time_st local_time;
    local_time.time_seconds     = m_current_time.time_seconds;
    local_time.time_nanoseconds = m_current_time.time_nanoseconds;
    local_time.time_seconds += m_time_zone_offset;
    gmtime_r(&local_time.time_seconds, &last_tm);

    if ((m_folder_structure == DeviceConfig::folder_structure_t::DAY) ||
        (m_folder_structure == DeviceConfig::folder_structure_t::HOUR))
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
        int retval =
            snprintf(m_folder_name,
                     FOLDER_NAME_MAX_LENGTH,
                     YEARMONTHDAY_FOLDER_FORMAT_STRING,
                     last_tm.tm_year + static_cast<int>(TIME_BASE_YEAR),
                     last_tm.tm_mon + static_cast<int>(TIME_BASE_MONTH),
                     last_tm.tm_mday);
#pragma GCC diagnostic pop
        SYSTEM_ASSERT_GT(retval, 0);
        SYSTEM_ASSERT_LT(static_cast<uint32_t>(retval), FOLDER_NAME_MAX_LENGTH);

        error_code =
            w_hal_sd_card_check_exists(m_folder_name, FOLDER_NAME_MAX_LENGTH);

        wHalErrorCode error_set[] = {
            wHalErrorCode::NO_ERROR,
            wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST};
        SYSTEM_ASSERT_IN_SET(error_code,
                             error_set,
                             sizeof(error_set) / sizeof(error_set[0]));

        if (error_code == wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST)
        {
            error_code = w_hal_sd_card_create_folder(m_folder_name,
                                                     FOLDER_NAME_MAX_LENGTH);
            SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
        }

        error_code =
            w_hal_sd_card_goto_folder(m_folder_name, FOLDER_NAME_MAX_LENGTH);
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

        m_current_folder_level = folder_level_t::YEAR_MONTH_DAY;
    }

    if (m_folder_structure == DeviceConfig::folder_structure_t::HOUR)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
        int retval =
            snprintf(m_folder_name,
                     FOLDER_NAME_MAX_LENGTH,
                     YEARMONTHDAYHOUR_FOLDER_FORMAT_STRING,
                     last_tm.tm_year + static_cast<int>(TIME_BASE_YEAR),
                     last_tm.tm_mon + static_cast<int>(TIME_BASE_MONTH),
                     last_tm.tm_mday,
                     last_tm.tm_hour);
#pragma GCC diagnostic pop
        SYSTEM_ASSERT_GT(retval, 0);
        SYSTEM_ASSERT_LT(static_cast<uint32_t>(retval), FOLDER_NAME_MAX_LENGTH);

        error_code =
            w_hal_sd_card_check_exists(m_folder_name, FOLDER_NAME_MAX_LENGTH);

        wHalErrorCode error_set[] = {
            wHalErrorCode::NO_ERROR,
            wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST};
        SYSTEM_ASSERT_IN_SET(error_code,
                             error_set,
                             sizeof(error_set) / sizeof(error_set[0]));

        if (error_code == wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST)
        {
            error_code = w_hal_sd_card_create_folder(m_folder_name,
                                                     FOLDER_NAME_MAX_LENGTH);
            SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
        }

        error_code =
            w_hal_sd_card_goto_folder(m_folder_name, FOLDER_NAME_MAX_LENGTH);
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

        m_current_folder_level = folder_level_t::YEAR_MONTH_DAY_HOUR;
    }

    m_current_hour       = last_tm.tm_hour;
    m_current_dayInMonth = last_tm.tm_mday;
    m_current_month      = last_tm.tm_mon;
    m_current_year       = last_tm.tm_year;

    // First file of folder should match the start time of the folder
    // Otherwise, time should be for the next file
    if (is_latest_time_from_file)
    {
        // Increment time by file duration and only 1 extra second.
        // This is to ensure that the next file created will have a different
        // name than the previous one.
        m_current_time += latest_file_duration;
        m_current_time.time_seconds++;
    }

    // Filename should be based on max time between gprs and last file
    // created
    w_hal_rtc_time_st rtc_time;
    w_hal_rtc_get_time(rtc_time);
    if (rtc_time > m_current_time)
    {
        m_current_time = rtc_time;
    }
    else
    {
        error_code = w_hal_rtc_set_time(m_current_time);
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
    }
}

/**
 * @brief
 * Set up and initialization for Storage Task
 *
 * @param queues_info queue information
 */
void StorageTask::initialize(queues_info_t &queues_info)
{
    Task::initialize();

    const uint32_t wav_file_data_size =
        DeviceConfig::get_wav_file_sample_data_size();

    m_folder_structure          = DeviceConfig::p_page->folder_structure;
    m_recording_length          = DeviceConfig::p_page->recording_length;

    // WAV file size in bytes to be written
    m_total_wav_file_size       = sizeof(wav_header_t) + wav_file_data_size;
    // Slack space of 10 MB
    m_sd_card_slack_space       = 10000000;
    // Slack space for trace log data before the sd card gets full
    m_slack_space_for_trace_log = 1024 * 1024; // 1MB
    m_file_tracker.file_handle  = nullptr;
    m_file_tracker.bytes_written_in_write_chunk       = 0;
    m_file_tracker.bytes_written_in_file              = 0;

    m_trace_file_tracker.file_handle                  = nullptr;
    m_trace_file_tracker.bytes_written_in_write_chunk = 0;
    m_trace_file_tracker.bytes_written_in_file        = 0;
    m_p_current_trace_buffer                          = nullptr;

    for (uint32_t index = 0; index < MAX_DEQUEUED_REQUESTS; index++)
    {
        m_request_tracker.request_id[index] = INVALID_STORAGE_MSG_ID;
    }
    m_request_tracker.num_requests             = 0;
    m_request_tracker.current_offset_in_buffer = 0;
    m_request_tracker.state                    = State::FIRST;
    m_trace_state                              = TraceState::FIRST;

    // Save pointer to message array
    m_p_storage_msgs                           = queues_info.storage_msgs;

    // Save queue pointers
    m_p_storage_request_queue  = &queues_info.storage_request_queue;
    m_p_storage_response_queue = &queues_info.storage_response_queue;

    wav_file_init_header(
        m_wav_header,
        DeviceConfig::p_page->num_audio_channels,
        DeviceConfig::p_page->sampling_rate,
        DeviceConfig::p_page->audio_sampling_width,
        static_cast<uint16_t>(shared_glbl.environment_info.temperature_celsius),
        static_cast<uint16_t>(shared_glbl.environment_info.relative_humidity),
        shared_glbl.power_mgmt_info.voltage,
        shared_glbl.power_mgmt_info.current,
        shared_glbl.power_mgmt_info.remaining_capacity,
        shared_glbl.power_mgmt_info.average_current,
        shared_glbl.power_mgmt_info.time_to_empty,
        DeviceConfig::p_page->latitude_e7,
        DeviceConfig::p_page->longitude_e7);

    wav_file_set_data_size(m_wav_header, wav_file_data_size);

    // File and folder name initialized to empty string
    m_folder_name[0] = '\0';
    m_file_name[0]   = '\0';

    // Default current time and hour values
    m_current_time.invalidate();
    m_current_hour         = static_cast<uint8_t>(-1);
    m_current_dayInMonth   = static_cast<uint8_t>(-1);
    m_current_month        = static_cast<uint8_t>(-1);
    m_current_year         = static_cast<uint16_t>(-1);

    m_current_folder_level = get_deepest_folder_level();
    m_folder_check_state   = folder_check_state_t::FIRST;

    // setup trace file first since current working directory is root now
    setup_trace_log_file();

    m_time_zone_offset = DeviceConfig::p_page->time_zone_offset;

    w_hal_rtc_get_time_zone_components(m_time_zone_offset,
                                       m_time_zone_sign,
                                       m_time_zone_hour,
                                       m_time_zone_minute);

    setup_current_time();

    uint64_t total_size                             = w_hal_sd_card_get_size();
    shared_glbl.storage_info.total_sd_card_capacity = total_size;
    shared_glbl.storage_info.total_bytes_written =
        (total_size - w_hal_sd_card_get_free_space());

    w_hal_rtc_time_st rtc_time;
    w_hal_rtc_get_time(rtc_time);
    m_last_capacity_check_time  = rtc_time;
    m_last_flush_time           = rtc_time;
    m_last_logged_capacity_time = rtc_time;

    m_nanoseconds_per_byte      = (NSEC_PER_SEC << NS_PRECISION_BITS) /
                             (DeviceConfig::get_data_bit_rate() / CHAR_BIT);

    m_abort_requests          = false;

    m_file_flush_close        = false;
    m_write_bytes_in_progress = 0;
    m_unflushed_write_bytes   = 0;
    m_unflushed_trace_bytes   = 0;

    m_num_bytes_captured      = 0;
    m_data_byte_rate          = DeviceConfig::get_data_bit_rate() / CHAR_BIT;

    // Get format of the file system
    w_hal_sd_card_get_file_system_format(
        shared_glbl.storage_info.sd_card_file_format,
        MAX_FILE_SYSTEM_FORMAT_LENGTH);
}

/**
 * @brief Check if need to go up one folder to create next new folder
 *
 * @return true if need to go up one folder
 * @return false otherwise
 */
bool StorageTask::check_go_up_one_folder() const
{
    bool folder_change = false;

    switch (m_current_folder_level)
    {
        case folder_level_t::YEAR_MONTH_DAY_HOUR:
        {
            folder_change =
                folder_change || (m_time_struct.tm_hour != m_current_hour);
        }
        // FALL-THROUGH
        case folder_level_t::YEAR_MONTH_DAY:
        {
            folder_change = folder_change ||
                            (m_time_struct.tm_mday != m_current_dayInMonth);
            folder_change =
                folder_change || (m_time_struct.tm_mon != m_current_month);
            folder_change =
                folder_change || (m_time_struct.tm_year != m_current_year);
            break;
        }
        default:
        {
            folder_change = false;
            break;
        }
    }

    return folder_change;
}

/**
 * @brief Check if new folder is needed and create one.
 *
 * @return true if done
 * @return false otherwise
 */
bool StorageTask::check_new_folder()
{
    wHalErrorCode  error;
    bool           done                 = false;
    folder_level_t deepest_folder_level = get_deepest_folder_level();

    // If flat folder structure, then no need to create new folder. Stay in
    // root.
    if (m_folder_structure == DeviceConfig::folder_structure_t::FLAT)
    {
        return true;
    }

    switch (m_folder_check_state)
    {
        case folder_check_state_t::CHECK_GO_UP_ONE_FOLDER:
        {
            if (check_go_up_one_folder())
            {
                m_folder_check_state = folder_check_state_t::GO_UP_ONE_FOLDER;
            }
            else
            {
                if (m_current_folder_level == deepest_folder_level)
                {
                    done = true;
                }
                else
                {
                    // Go to creating the next subfolder
                    SYSTEM_ASSERT_NOT_EQUAL(m_current_folder_level,
                                            folder_level_t::FIRST);
                    m_current_folder_level = static_cast<folder_level_t>(
                        static_cast<int>(m_current_folder_level) - 1);
                    m_folder_check_state = folder_check_state_t::CREATE_FOLDER;
                }
                break;
            }
        }
        // FALL-THROUGH
        case folder_check_state_t::GO_UP_ONE_FOLDER:
        {
            // Go up one folder level
            error = w_hal_sd_card_goto_folder("..", FOLDER_NAME_MAX_LENGTH);
            if (error != wHalErrorCode::BUSY)
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                // Check if going up another folder level is needed
                folder_level_t next_folder_level = static_cast<folder_level_t>(
                    static_cast<int>(m_current_folder_level) + 1);
                if (next_folder_level == folder_level_t::NUM_LEVELS)
                {
                    m_folder_check_state = folder_check_state_t::CREATE_FOLDER;
                }
                else
                {
                    m_current_folder_level = next_folder_level;
                    m_folder_check_state =
                        folder_check_state_t::CHECK_GO_UP_ONE_FOLDER;
                }
            }

            break;
        }
        case folder_check_state_t::CREATE_FOLDER:
        {
            // Create new folder
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
            int retval;
            if (m_current_folder_level == folder_level_t::YEAR_MONTH_DAY)
            {
                retval = snprintf(
                    m_folder_name,
                    FOLDER_NAME_MAX_LENGTH,
                    YEARMONTHDAY_FOLDER_FORMAT_STRING,
                    m_time_struct.tm_year + static_cast<int>(TIME_BASE_YEAR),
                    m_time_struct.tm_mon + static_cast<int>(TIME_BASE_MONTH),
                    m_time_struct.tm_mday);
            }
            else
            {
                SYSTEM_ASSERT_EQUAL(m_current_folder_level,
                                    folder_level_t::YEAR_MONTH_DAY_HOUR);

                retval = snprintf(
                    m_folder_name,
                    FOLDER_NAME_MAX_LENGTH,
                    YEARMONTHDAYHOUR_FOLDER_FORMAT_STRING,
                    m_time_struct.tm_year + static_cast<int>(TIME_BASE_YEAR),
                    m_time_struct.tm_mon + static_cast<int>(TIME_BASE_MONTH),
                    m_time_struct.tm_mday,
                    m_time_struct.tm_hour);
            }
#pragma GCC diagnostic pop
            SYSTEM_ASSERT_GT(retval, 0);
            SYSTEM_ASSERT_LT(static_cast<uint32_t>(retval),
                             FOLDER_NAME_MAX_LENGTH);

            error = w_hal_sd_card_create_folder(m_folder_name,
                                                FOLDER_NAME_MAX_LENGTH);
            if (error != wHalErrorCode::BUSY)
            {
                wHalErrorCode error_set[] = {
                    wHalErrorCode::NO_ERROR,
                    wHalErrorCode::SD_CARD_FOLDER_EXIST};
                SYSTEM_ASSERT_IN_SET(error,
                                     error_set,
                                     sizeof(error_set) / sizeof(error_set[0]));

                m_folder_check_state = folder_check_state_t::GO_TO_NEW_FOLDER;
            }

            break;
        }
        case folder_check_state_t::GO_TO_NEW_FOLDER:
        {
            error = w_hal_sd_card_goto_folder(m_folder_name,
                                              FOLDER_NAME_MAX_LENGTH);
            if (error != wHalErrorCode::BUSY)
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                // If in the deepest folder, then done
                // Otherwise, go back to creating the next subfolder
                if (m_current_folder_level == deepest_folder_level)
                {
                    // Reset the state after done
                    m_folder_check_state = folder_check_state_t::FIRST;
                    done                 = true;
                }
                else
                {
                    // Since folder was just created, there is no subfolder, so
                    // jump straight to creation.
                    SYSTEM_ASSERT_NOT_EQUAL(m_current_folder_level,
                                            folder_level_t::FIRST);
                    m_current_folder_level = static_cast<folder_level_t>(
                        static_cast<int>(m_current_folder_level) - 1);
                    m_folder_check_state = folder_check_state_t::CREATE_FOLDER;
                }
            }

            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    return done;
}

/**
 * @brief
 * Check if a buffer has been fully processed.
 * If so, send response for request.
 */
void StorageTask::check_send_request_response()
{
    if (m_request_tracker.num_requests == 0)
    {
        SYSTEM_ASSERT_EQUAL(m_request_tracker.current_offset_in_buffer, 0);
        return;
    }

    storage_request_id_t storage_request_id = m_request_tracker.request_id[0];
    SYSTEM_ASSERT_NOT_EQUAL(storage_request_id, INVALID_STORAGE_MSG_ID);

    StorageRequestMsg *p_storage_request =
        &m_p_storage_msgs[storage_request_id];
    uint16_t current_request_length = p_storage_request->get_buffer_length();

    if (m_request_tracker.current_offset_in_buffer >= current_request_length)
    {
        m_request_tracker.current_offset_in_buffer -= current_request_length;

        p_storage_request->set_status(MsgStatus::GOOD);

        if (p_storage_request->is_flush())
        {
            m_file_flush_close = true;
        }

        m_p_storage_response_queue->push(m_request_tracker.request_id[0]);

        // Move to the next request id if it is valid
        for (uint32_t index = 0; index < MAX_DEQUEUED_REQUESTS - 1; index++)
        {
            m_request_tracker.request_id[index] =
                m_request_tracker.request_id[index + 1];
        }
        m_request_tracker.request_id[MAX_DEQUEUED_REQUESTS - 1] =
            INVALID_STORAGE_MSG_ID;

        m_request_tracker.num_requests--;
    }
}

/**
 * @brief
 * Check if any buffers are completed so that their respective
 * requests can be sent back
 */
void StorageTask::check_current_write_buffers_and_send_response()
{
    // Check if any buffers are done
    for (uint32_t index = 0; index < MAX_DEQUEUED_REQUESTS; index++)
    {
        if (m_request_tracker.num_requests == 0)
        {
            break;
        }

        // Check if the first buffer is done
        storage_request_id_t storage_request_id =
            m_request_tracker.request_id[0];
        SYSTEM_ASSERT_NOT_EQUAL(storage_request_id, INVALID_STORAGE_MSG_ID);

        const StorageRequestMsg *p_storage_request =
            &m_p_storage_msgs[storage_request_id];

        buffer_id_t buffer_id;
        uint16_t    buffer_length;
        p_storage_request->get_buffer(buffer_id, buffer_length);

        // If not all the bytes in the buffer were issued to the sd card, then
        // the buffer cannot be done as we have more requests to send to the
        // sd card.
        if (m_request_tracker.current_offset_in_buffer < buffer_length)
        {
            break;
        }

        const uint8_t *p_buffer = get_buffer_address(buffer_id);
        wHalErrorCode  buffer_error =
            w_hal_sd_card_get_write_buffer_status(p_buffer, buffer_length);

        if (buffer_error == wHalErrorCode::BUSY)
        {
            break;
        }

        // Buffer can be reused in the system
        SYSTEM_ASSERT_EQUAL(buffer_error, wHalErrorCode::NO_ERROR);

        check_send_request_response();
    }
}

/**
 * @brief Handle saving of trace log
 *
 * @return true if trace log saving is busy
 * @return false otherwise
 */
bool StorageTask::handle_save_trace_log()
{
    bool          busy = true;
    wHalErrorCode error;

    switch (m_trace_state)
    {
        case TraceState::CHECK_FOR_SPACE:
        {
            SYSTEM_ASSERT_EQUAL(m_p_current_trace_buffer,
                                static_cast<uint8_t *>(nullptr));
            if ((shared_glbl.storage_info.total_sd_card_capacity -
                 shared_glbl.storage_info.total_bytes_written) <
                m_slack_space_for_trace_log)
            {
                busy = true;
                break;
            }
            m_trace_state = TraceState::CHECK_NEW_TRACE_DATA;
        }
        // FALL-THROUGH
        case TraceState::CHECK_NEW_TRACE_DATA:
        {
            SYSTEM_ASSERT_EQUAL(m_p_current_trace_buffer,
                                static_cast<uint8_t *>(nullptr));

            m_p_current_trace_buffer =
                Trace::get_write_trace_buffer(m_trace_buffer_size);
            if (m_p_current_trace_buffer == nullptr)
            {
                busy = false;
                break;
            }

            m_trace_state = TraceState::WRITE_FILE;
        }
        // FALL-THROUGH
        case TraceState::WRITE_FILE:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_trace_file_tracker.file_handle,
                                    static_cast<wFileHandle>(nullptr));

            // Write to sd card
            error =
                w_hal_sd_card_write_to_file(m_p_current_trace_buffer,
                                            m_trace_buffer_size,
                                            m_trace_file_tracker.file_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            m_trace_state = TraceState::WAIT_FOR_WRITE_FILE;
        }
        // FALL-THROUGH
        case TraceState::WAIT_FOR_WRITE_FILE:
        {
            // Check if busy
            error = w_hal_sd_card_handler();
            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_NOT_EQUAL(m_trace_buffer_size, 0);

            m_unflushed_trace_bytes += m_trace_buffer_size;

            // Move on to next buffer
            Trace::move_write_trace_buffer_pointer(m_trace_buffer_size);
            m_p_current_trace_buffer = nullptr;
            m_trace_buffer_size      = 0;

            m_p_current_trace_buffer =
                Trace::get_write_trace_buffer(m_trace_buffer_size);
            if (m_p_current_trace_buffer != nullptr)
            {
                m_trace_state = TraceState::WRITE_FILE;
                break;
            }

            m_trace_state = TraceState::SYNC_FILE;
        }
        // FALL-THROUGH
        case TraceState::SYNC_FILE:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_trace_file_tracker.file_handle,
                                    static_cast<wFileHandle>(nullptr));

            // Sync file to filesystem
            error = w_hal_sd_card_sync_file(m_trace_file_tracker.file_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            m_trace_state = TraceState::WAIT_FOR_SYNC_FILE;
        }
        // FALL-THROUGH
        case TraceState::WAIT_FOR_SYNC_FILE:
        {
            // Check if busy
            error = w_hal_sd_card_handler();
            if (error != wHalErrorCode::BUSY)
            {
                update_usage_stats_after_flush(m_unflushed_trace_bytes);
                m_unflushed_trace_bytes = 0;

                m_trace_state           = TraceState::FIRST;
                busy                    = false;
            }
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
        }
    }

    return busy;
}

/**
 * @brief Update the bookkeeping for the SD card usage.
 *
 * @param bytes Number of unflushed bytes written to the SD card.
 */
void StorageTask::update_usage_stats_after_flush(uint32_t bytes)
{
    // Update the bytes written for precision here.
    // To make this code performance friendly, don't query the total
    // bytes written for every file. Check it every hour.
    w_hal_rtc_time_st current_time;
    w_hal_rtc_get_time(current_time);

    w_hal_rtc_time_delta_st delta_time;
    current_time.subtract(m_last_capacity_check_time, delta_time);
    if (delta_time.seconds > CAPACITY_CHECK_PERIOD)
    {
        shared_glbl.storage_info.total_bytes_written =
            w_hal_sd_card_get_size() - w_hal_sd_card_get_free_space();
        m_last_capacity_check_time = current_time;

        m_unflushed_write_bytes    = 0;
        m_unflushed_trace_bytes    = 0;
    }
    else
    {
        shared_glbl.storage_info.total_bytes_written += bytes;
    }

    current_time.subtract(m_last_logged_capacity_time, delta_time);
    if (delta_time.seconds > CAPACITY_LOG_PERIOD)
    {
        LOG_MSG_WITH_ARGS_NO_PRINT(
            TraceFile::STORAGE_TASK_CPP,
            "SD Card Bytes Written: %lld, Capacity: %lld\n",
            shared_glbl.storage_info.total_bytes_written,
            shared_glbl.storage_info.total_sd_card_capacity);
        m_last_logged_capacity_time = current_time;
    }

    // Updating the number of seconds recorded
    m_num_bytes_captured += m_unflushed_write_bytes;
    uint32_t seconds = m_num_bytes_captured / m_data_byte_rate;
    shared_glbl.storage_info.seconds_recorded_since_boot += seconds;
    m_num_bytes_captured = (m_num_bytes_captured % m_data_byte_rate);
}

/**
 * @brief This will check for SD card space against the slack space available,
 * If it is less than slack space it will abort storage request id's and stop
 * allocating buffers
 */
void StorageTask::check_for_available_space()
{
    // Get free space. If it is less than slack space,
    // we need to start aborting requests.
    if (!m_abort_requests)
    {
        if ((shared_glbl.storage_info.total_sd_card_capacity -
             shared_glbl.storage_info.total_bytes_written) <
            m_sd_card_slack_space)
        {
            LOG_MSG(TraceFile::STORAGE_TASK_CPP, "The SD card is Full\n");
            m_abort_requests = true;
        }
    }

    if (m_abort_requests)
    {
        abort_storage_request();
    }
}

/**
 * @brief Aborting storage requests
 *
 */
void StorageTask::abort_storage_request()
{
    // Abort storage request id and next storage request id
    for (uint32_t index = 0; index < MAX_DEQUEUED_REQUESTS; index++)
    {
        if (m_request_tracker.request_id[index] != INVALID_STORAGE_MSG_ID)
        {
            StorageRequestMsg *p_storage_request =
                &m_p_storage_msgs[m_request_tracker.request_id[index]];

            p_storage_request->set_status(MsgStatus::ABORT);
            m_p_storage_response_queue->push(
                m_request_tracker.request_id[index]);
            m_request_tracker.request_id[index] = INVALID_STORAGE_MSG_ID;
            m_request_tracker.num_requests--;
        }
    }

    // Do the same with the buffers in the queue
    while (true)
    {
        storage_request_id_t request_id;
        if (m_p_storage_request_queue->pop(request_id) == false)
        {
            break;
        }

        StorageRequestMsg *p_storage_request = &m_p_storage_msgs[request_id];
        // Abort the request and send it back
        p_storage_request->set_status(MsgStatus::ABORT);
        m_p_storage_response_queue->push(request_id);
    }
}

/**
 * @brief
 * Main routine for Storage Task
 *
 * @return TaskErrorCode NoError
 */
TaskErrorCode StorageTask::run()
{
    TaskErrorCode task_error = TaskErrorCode::NoError;
    wHalErrorCode error;

    error = w_hal_sd_card_handler();
    if (error == wHalErrorCode::BUSY)
    {
        // Check if any buffers are done
        if (m_request_tracker.state == State::WAIT_FOR_WRITE_FILE)
        {
            check_current_write_buffers_and_send_response();
        }

        // Trace log is busy. Don't do anything else until it is clear.
        return task_error;
    }

    // Handle saving of trace log
    if (handle_save_trace_log())
    {
        // Trace log is busy. Don't do anything else until it is clear.
        return task_error;
    }

    switch (m_request_tracker.state)
    {
        case State::WAIT_FOR_REQUEST:
        {
            // Check for available space
            check_for_available_space();

            // If low battery flag is set abort the request and send it back
            if (shared_glbl.power_mgmt_info.low_battery_flag ||
                m_abort_requests)
            {
                abort_storage_request();
                if (m_file_tracker.file_handle != nullptr)
                {
                    // Close the file
                    m_request_tracker.state = State::CLOSE_FILE;
                }
                break;
            }

            for (uint32_t index = 0; index < MAX_DEQUEUED_REQUESTS; index++)
            {
                SYSTEM_ASSERT_EQUAL(m_request_tracker.request_id[index],
                                    INVALID_STORAGE_MSG_ID);
            }
            SYSTEM_ASSERT_EQUAL(m_request_tracker.num_requests, 0);

            m_p_storage_request_queue->pop(m_request_tracker.request_id[0]);

            if (m_request_tracker.request_id[0] == INVALID_STORAGE_MSG_ID)
            {
                // Only update config status when there is no audio storage work
                // pending.
                if (shared_glbl.update_config_status_flag &&
                    (m_file_tracker.file_handle == nullptr))
                {
                    shared_glbl.update_config_status_flag = false;
                    create_config_status_file();
                }

                // Nothing new to process
                task_error = TaskErrorCode::Yield;
                break;
            }

            m_request_tracker.num_requests             = 1;

            m_request_tracker.current_offset_in_buffer = 0;
            if (m_file_tracker.file_handle == nullptr)
            {
                m_request_tracker.state = State::SETUP_TIMESTAMP;
            }
            else
            {
                m_request_tracker.state = State::WRITE_FILE;
                break;
            }
        }
        // FALL-THROUGH
        case State::SETUP_TIMESTAMP:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_request_tracker.request_id[0],
                                    INVALID_STORAGE_MSG_ID);
            StorageRequestMsg *p_storage_request =
                &m_p_storage_msgs[m_request_tracker.request_id[0]];

            p_storage_request->get_time(m_current_time);
            // Convert current buffer offset to nanoseconds
            const uint64_t nanoseconds =
                (m_request_tracker.current_offset_in_buffer *
                 m_nanoseconds_per_byte) >>
                NS_PRECISION_BITS;

            m_current_time.time_nanoseconds += nanoseconds;
            while (m_current_time.time_nanoseconds >= NSEC_PER_SEC)
            {
                m_current_time.time_seconds++;
                m_current_time.time_nanoseconds -= NSEC_PER_SEC;
            }

            // Convert using the timezone offset included
            w_hal_rtc_time_st local_time;
            local_time.time_seconds     = m_current_time.time_seconds;
            local_time.time_nanoseconds = m_current_time.time_nanoseconds;
            local_time.time_seconds += m_time_zone_offset;
            gmtime_r(&local_time.time_seconds, &m_time_struct);

            // Set up file name and create file
            int retval = snprintf(
                m_file_name,
                FILE_NAME_MAX_LENGTH,
                FILE_FORMAT_STRING,
                TIME_BASE_YEAR + static_cast<uint32_t>(m_time_struct.tm_year),
                TIME_BASE_MONTH + static_cast<uint32_t>(m_time_struct.tm_mon),
                m_time_struct.tm_mday,
                m_time_struct.tm_hour,
                m_time_struct.tm_min,
                m_time_struct.tm_sec,
                static_cast<uint32_t>(m_current_time.time_nanoseconds /
                                      NSEC_PER_MSEC));

            SYSTEM_ASSERT_GT(retval, 0);
            SYSTEM_ASSERT_LT(static_cast<uint32_t>(retval),
                             FILE_NAME_MAX_LENGTH);

            // Setup start states for checking and possibly doing folder
            // creation
            m_current_folder_level  = get_deepest_folder_level();
            m_folder_check_state    = folder_check_state_t::FIRST;

            m_request_tracker.state = State::CHECK_FOLDER_CREATION;
        }
        // FALL-THROUGH
        case State::CHECK_FOLDER_CREATION:
        {
            if (!check_new_folder())
            {
                break;
            }

            // Move time ahead
            m_current_time.time_seconds += m_recording_length;
            m_current_hour          = m_time_struct.tm_hour;
            m_current_dayInMonth    = m_time_struct.tm_mday;
            m_current_month         = m_time_struct.tm_mon;
            m_current_year          = m_time_struct.tm_year;

            m_request_tracker.state = State::CREATE_FILE;
        }
        // FALL-THROUGH
        case State::CREATE_FILE:
        {
            wHalErrorCode dht_error = w_hal_dht_init();
            if (dht_error != wHalErrorCode::NO_ERROR)
            {
                LOG_MSG_WITH_ARGS(
                    TraceFile::ADMIN_TASK_CPP,
                    "DHT initialization failed with error code: %d\n",
                    dht_error);
            }

            w_hal_dht_get_env_info(
                shared_glbl.environment_info.temperature_celsius,
                shared_glbl.environment_info.relative_humidity);

            m_wav_header.env_subchunk.temperature = static_cast<uint16_t>(
                shared_glbl.environment_info.temperature_celsius);
            m_wav_header.env_subchunk.humidity = static_cast<uint16_t>(
                shared_glbl.environment_info.relative_humidity);

            m_wav_header.batt_subchunk.voltage =
                static_cast<int32_t>(shared_glbl.power_mgmt_info.voltage);
            m_wav_header.batt_subchunk.current =
                static_cast<int32_t>(shared_glbl.power_mgmt_info.current);
            m_wav_header.batt_subchunk.remaining_capacity =
                static_cast<int32_t>(
                    shared_glbl.power_mgmt_info.remaining_capacity);
            m_wav_header.batt_subchunk.average_current = static_cast<int32_t>(
                shared_glbl.power_mgmt_info.average_current);
            m_wav_header.batt_subchunk.time_to_empty =
                static_cast<int32_t>(shared_glbl.power_mgmt_info.time_to_empty);

            m_wav_header.loc_subchunk.latitude_e7 =
                DeviceConfig::p_page->latitude_e7;
            m_wav_header.loc_subchunk.longitude_e7 =
                DeviceConfig::p_page->longitude_e7;

            error = w_hal_sd_card_create_file(m_file_name,
                                              FILE_NAME_MAX_LENGTH,
                                              m_file_tracker.file_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            SYSTEM_ASSERT_NOT_EQUAL(m_file_tracker.file_handle,
                                    static_cast<wFileHandle>(nullptr));

            m_request_tracker.state = State::WAIT_FOR_CREATE_FILE;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_CREATE_FILE:
        {
            error = w_hal_sd_card_handler();

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            m_wav_header_written                        = false;
            m_file_tracker.bytes_written_in_write_chunk = 0;
            m_file_tracker.bytes_written_in_file        = 0;

            m_request_tracker.state                     = State::WRITE_FILE;
        }
        // FALL-THROUGH
        case State::WRITE_FILE:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_request_tracker.request_id[0],
                                    INVALID_STORAGE_MSG_ID);
            StorageRequestMsg *p_storage_request =
                &m_p_storage_msgs[m_request_tracker.request_id[0]];

            buffer_id_t current_buffer_id;
            uint16_t    current_request_length;

            p_storage_request->get_buffer(current_buffer_id,
                                          current_request_length);
            const uint8_t *p_current_buffer =
                get_buffer_address(current_buffer_id) +
                m_request_tracker.current_offset_in_buffer;

            SYSTEM_ASSERT_LTEQ(m_request_tracker.current_offset_in_buffer,
                               current_request_length);
            const uint32_t remaining_bytes_in_current_buffer =
                current_request_length -
                m_request_tracker.current_offset_in_buffer;

            SYSTEM_ASSERT_LTEQ(m_file_tracker.bytes_written_in_file,
                               m_total_wav_file_size);
            const uint32_t remaining_bytes_in_file =
                m_total_wav_file_size - m_file_tracker.bytes_written_in_file;

            // Compute number of bytes in all requests
            uint32_t total_bytes_in_all_requests =
                remaining_bytes_in_current_buffer;
            for (uint32_t index = 1; index < m_request_tracker.num_requests;
                 index++)
            {
                SYSTEM_ASSERT_NOT_EQUAL(m_request_tracker.request_id[index],
                                        INVALID_STORAGE_MSG_ID);

                total_bytes_in_all_requests += BUFFER_SIZE;
            }

            const storage_request_id_t last_request_id =
                m_request_tracker
                    .request_id[m_request_tracker.num_requests - 1];

            if ((m_request_tracker.num_requests < MAX_DEQUEUED_REQUESTS) &&
                (total_bytes_in_all_requests < remaining_bytes_in_file) &&
                !m_p_storage_msgs[last_request_id].is_flush() &&
                !shared_glbl.reboot_flag)
            {
                // Wait for the next buffer and try to do sequential
                // detection
                m_request_tracker.state = State::WAIT_FOR_NEXT_REQUEST;
            }
            else
            {
                uint32_t bytes_to_write = total_bytes_in_all_requests;

                if (!m_wav_header_written)
                {
                    bytes_to_write += sizeof(m_wav_header);
                }

                uint32_t remaining_bytes_in_write_chunk =
                    WRITE_CHUNK_SIZE -
                    m_file_tracker.bytes_written_in_write_chunk;

                if (bytes_to_write < remaining_bytes_in_file)
                {
                    if (bytes_to_write >= remaining_bytes_in_write_chunk)
                    {
                        // After writing remaining bytes in write chunk,
                        // align the rest to the write chunk size
                        bytes_to_write = remaining_bytes_in_write_chunk +
                                         (((bytes_to_write -
                                            remaining_bytes_in_write_chunk) /
                                           WRITE_CHUNK_SIZE) *
                                          WRITE_CHUNK_SIZE);
                    }
                }
                else
                {
                    // Write up till end of file
                    bytes_to_write = remaining_bytes_in_file;
                }

                const void * p_buffers[MAX_DEQUEUED_REQUESTS + 1];
                unsigned int buffer_sizes[MAX_DEQUEUED_REQUESTS + 1];
                unsigned int number_of_buffers    = 0;
                unsigned int byte_counter         = 0;
                unsigned int buffer_bytes_counter = 0;

                if (!m_wav_header_written)
                {
                    SYSTEM_ASSERT_EQUAL(remaining_bytes_in_file,
                                        m_total_wav_file_size);
                    p_buffers[number_of_buffers]    = &m_wav_header;
                    buffer_sizes[number_of_buffers] = sizeof(m_wav_header);
                    byte_counter += sizeof(m_wav_header);
                    number_of_buffers++;
                }

                // At least one buffer data should be written
                SYSTEM_ASSERT_GT(bytes_to_write, byte_counter);

                p_buffers[number_of_buffers] = p_current_buffer;
                buffer_sizes[number_of_buffers] =
                    std::min(bytes_to_write - byte_counter,
                             remaining_bytes_in_current_buffer);
                byte_counter += buffer_sizes[number_of_buffers];
                buffer_bytes_counter += buffer_sizes[number_of_buffers];
                number_of_buffers++;

                for (uint32_t index = 1; index < m_request_tracker.num_requests;
                     index++)
                {
                    // Can't write more than the total bytes to write
                    if (byte_counter >= bytes_to_write)
                    {
                        break;
                    }

                    SYSTEM_ASSERT_NOT_EQUAL(m_request_tracker.request_id[index],
                                            INVALID_STORAGE_MSG_ID);

                    const StorageRequestMsg *p_next_storage_request =
                        &m_p_storage_msgs[m_request_tracker.request_id[index]];
                    buffer_id_t next_buffer_id;
                    uint16_t    next_request_length;

                    p_next_storage_request->get_buffer(next_buffer_id,
                                                       next_request_length);

                    const uint8_t *p_next_buffer =
                        get_buffer_address(next_buffer_id);

                    p_buffers[number_of_buffers] = p_next_buffer;
                    buffer_sizes[number_of_buffers] =
                        std::min(bytes_to_write - byte_counter,
                                 static_cast<uint32_t>(next_request_length));
                    byte_counter += buffer_sizes[number_of_buffers];
                    buffer_bytes_counter += buffer_sizes[number_of_buffers];
                    number_of_buffers++;
                }

                // Fill in the rest of the buffers with nullptr
                for (uint32_t index = number_of_buffers;
                     index < MAX_DEQUEUED_REQUESTS + 1;
                     index++)
                {
                    p_buffers[index]    = nullptr;
                    buffer_sizes[index] = 0;
                }

                // Write to sd card
                error = w_hal_sd_card_write_to_file(p_buffers,
                                                    buffer_sizes,
                                                    number_of_buffers,
                                                    m_file_tracker.file_handle);

                if (error == wHalErrorCode::BUSY)
                {
                    break;
                }

                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                // No need to write wav header again for this file
                if (!m_wav_header_written)
                {
                    SYSTEM_ASSERT_EQUAL(
                        m_file_tracker.bytes_written_in_write_chunk,
                        0);
                    SYSTEM_ASSERT_EQUAL(m_file_tracker.bytes_written_in_file,
                                        0);

                    m_wav_header_written = true;
                }

                // Update ALL pointers for file, buffer, and write chunk
                // tracking
                m_file_tracker.bytes_written_in_write_chunk += bytes_to_write;
                m_file_tracker.bytes_written_in_write_chunk %= WRITE_CHUNK_SIZE;

                m_file_tracker.bytes_written_in_file += bytes_to_write;
                SYSTEM_ASSERT_LTEQ(m_file_tracker.bytes_written_in_file,
                                   m_total_wav_file_size);

                // Current offset in buffer is temporarily set to included the
                // size of all buffers written. It will be decremented later on
                // write completion.
                m_request_tracker.current_offset_in_buffer +=
                    buffer_bytes_counter;

                SYSTEM_ASSERT_EQUAL(m_write_bytes_in_progress, 0);
                m_write_bytes_in_progress = buffer_bytes_counter;

                m_request_tracker.state   = State::WAIT_FOR_WRITE_FILE;
                break;
            }
        }
        // FALL-THROUGH
        case State::WAIT_FOR_NEXT_REQUEST:
        {
            // Check for available space to write
            check_for_available_space();

            // If low battery flag is set abort the request and send it back
            if (shared_glbl.power_mgmt_info.low_battery_flag ||
                m_abort_requests)
            {
                abort_storage_request();
                if (m_file_tracker.file_handle != nullptr)
                {
                    // Close the file
                    m_request_tracker.state = State::CLOSE_FILE;
                }
                else
                {
                    // No file to close, so go back to first state
                    m_request_tracker.state = State::FIRST;
                }
                break;
            }

            SYSTEM_ASSERT_LT(m_request_tracker.num_requests,
                             MAX_DEQUEUED_REQUESTS);
            for (uint32_t index = 0; index < m_request_tracker.num_requests;
                 index++)
            {
                SYSTEM_ASSERT_NOT_EQUAL(m_request_tracker.request_id[index],
                                        INVALID_STORAGE_MSG_ID);
            }
            SYSTEM_ASSERT_EQUAL(
                m_request_tracker.request_id[m_request_tracker.num_requests],
                INVALID_STORAGE_MSG_ID);

            m_p_storage_request_queue->pop(
                m_request_tracker.request_id[m_request_tracker.num_requests]);

            if (m_request_tracker.request_id[m_request_tracker.num_requests] ==
                INVALID_STORAGE_MSG_ID)
            {
                if (!shared_glbl.reboot_flag)
                {
                    // Still need to wait for next request
                    break;
                }
            }
            else
            {
                m_request_tracker.num_requests++;
            }

            m_request_tracker.state = State::WRITE_FILE;

            break;
        }
        case State::WAIT_FOR_WRITE_FILE:
        {
            error = w_hal_sd_card_handler();

            if (error == wHalErrorCode::BUSY)
            {
                check_current_write_buffers_and_send_response();
                break;
            }

            m_request_tracker.state = State::SEND_RESPONSE;
        }
        // FALL-THROUGH
        case State::SEND_RESPONSE:
        {
            // Send response up to many times, once for current and
            // next request
            for (uint32_t index = 0; index < MAX_DEQUEUED_REQUESTS; index++)
            {
                check_send_request_response();
            }

            // Update the bytes written for precision here.
            m_unflushed_write_bytes += m_write_bytes_in_progress;
            m_write_bytes_in_progress = 0;

            SYSTEM_ASSERT_LT(m_request_tracker.current_offset_in_buffer,
                             BUFFER_SIZE);

            w_hal_rtc_time_st current_flush_time;
            w_hal_rtc_get_time(current_flush_time);

            w_hal_rtc_time_delta_st time_since_last_flush;
            current_flush_time.subtract(m_last_flush_time,
                                        time_since_last_flush);

            if ((m_file_tracker.bytes_written_in_file ==
                 m_total_wav_file_size) ||
                m_file_flush_close)
            {
                m_request_tracker.state = State::CLOSE_FILE;
                m_last_flush_time       = current_flush_time;
                m_file_flush_close      = false;
                break;
            }
            else if (((m_file_tracker.bytes_written_in_file %
                       WRITE_CHUNK_SIZE) == 0) &&
                     (time_since_last_flush.seconds >= FILE_FLUSH_WAIT_PERIOD))
            {
                m_request_tracker.state = State::SYNC_FILE;
                m_last_flush_time       = current_flush_time;
            }
            else
            {
                if (m_request_tracker.request_id[0] == INVALID_STORAGE_MSG_ID)
                {
                    m_request_tracker.state = State::FIRST;
                }
                else
                {
                    m_request_tracker.state = State::WRITE_FILE;
                }

                break;
            }
        }
        // FALL-THROUGH
        case State::SYNC_FILE:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_file_tracker.file_handle,
                                    static_cast<wFileHandle>(nullptr));

            // Sync file to filesystem
            error = w_hal_sd_card_sync_file(m_file_tracker.file_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            m_request_tracker.state = State::WAIT_FOR_SYNC_FILE;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_SYNC_FILE:
        {
            // Check if busy
            error = w_hal_sd_card_handler();
            if (error != wHalErrorCode::BUSY)
            {
                update_usage_stats_after_flush(m_unflushed_write_bytes);
                m_unflushed_write_bytes = 0;

                // Check for available space to write
                check_for_available_space();

                if ((m_request_tracker.request_id[0] ==
                     INVALID_STORAGE_MSG_ID) ||
                    m_abort_requests)
                {
                    m_request_tracker.state = State::FIRST;
                }
                else
                {
                    m_request_tracker.state = State::WRITE_FILE;
                }
            }
            break;
        }
        case State::CLOSE_FILE:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_file_tracker.file_handle,
                                    static_cast<wFileHandle>(nullptr));
            error = w_hal_sd_card_close_file(m_file_tracker.file_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            m_file_tracker.file_handle = nullptr;

            m_request_tracker.state    = State::WAIT_FOR_CLOSE_FILE;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_CLOSE_FILE:
        {
            error = w_hal_sd_card_handler();

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            update_usage_stats_after_flush(m_unflushed_write_bytes);
            m_unflushed_write_bytes = 0;

            // Check for available space to write
            check_for_available_space();

            if ((m_request_tracker.request_id[0] == INVALID_STORAGE_MSG_ID) ||
                m_abort_requests)
            {
                m_request_tracker.state = State::FIRST;
            }
            else
            {
                m_request_tracker.state = State::SETUP_TIMESTAMP;
            }

            break;
        }
        default:
        {
            break;
        }
    }

    switch (shared_glbl.shutdown_stage)
    {
        case shutdown_stage_t::Stage_5:
        {
            // Check for Storage task request queue to be empty, close
            // all the files and go to sleep
            if ((m_request_tracker.state == State::WAIT_FOR_REQUEST) &&
                (m_p_storage_request_queue->isEmpty()))
            {
                if (m_file_tracker.file_handle != nullptr)
                {
                    // To close file tracker
                    error =
                        w_hal_sd_card_close_file(m_file_tracker.file_handle);

                    if (error == wHalErrorCode::BUSY)
                    {
                        break;
                    }

                    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
                    m_file_tracker.file_handle = nullptr;
                }

                if (m_trace_file_tracker.file_handle != nullptr)
                {
                    // To close trace file tracker
                    error = w_hal_sd_card_close_file(
                        m_trace_file_tracker.file_handle);

                    if (error == wHalErrorCode::BUSY)
                    {
                        break;
                    }

                    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
                    m_trace_file_tracker.file_handle = nullptr;
                }

                // If SD_card format flag is set, Will format the SD_card
                if (shared_glbl.format_sd_card_flag)
                {
                    printf("\nFormatting SD card\n");
                    error = w_hal_sd_card_format();
                    if (error == wHalErrorCode::BUSY)
                    {
                        break;
                    }
                    printf("\nSD card format successful\n");
                }
            }
            else
            {
                break;
            }

            task_error                 = TaskErrorCode::Sleep;
            shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                static_cast<int>(shared_glbl.shutdown_stage) + 1);
        }
        default:
        {
            break;
        }
    }

    return task_error;
}
#endif // __CORE__ == STORAGE_CORE