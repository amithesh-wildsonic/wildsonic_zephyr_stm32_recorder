#if __CORE__ == STORAGE_CORE

#include "OnBootConfig.hpp"
#include "Buffer.hpp"
#include "assert.hpp"
#include "console.hpp"
#include "device_config.hpp"
#include "shared.hpp"
#include "version.hpp"

#include "w_hal_sd_card.hpp"

#include "re.h"

#include <algorithm>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief SD card device config commands
 *
 */
enum class sd_device_config_cmd_t : uint32_t
{
    SYSTEMTIME,              // Date and time
    TIMEZONE,                // Timezone
    GSM,                     // GSM Transmission mode
    SAMF,                    // Sampling frequency
    RECT,                    // Recording time
    SLEEP,                   // Sleep time
    DATERANGE,               // Recording date range
    TIMEOFDAY,               // Recording time of day
    TIMEOFDAY_0,             // Recording time of day
    TIMEOFDAY_1,             // Recording time of day
    TIMEOFDAY_2,             // Recording time of day
    TIMEOFDAY_3,             // Recording time of day
    TIMEOFDAY_4,             // Recording time of day
    TIMEOFDAY_5,             // Recording time of day
    TIMEOFDAY_6,             // Recording time of day
    TIMEOFDAY_7,             // Recording time of day
    TIMEOFDAY_8,             // Recording time of day
    TIMEOFDAY_9,             // Recording time of day
    DAYSOFWEEK,              // Recording days of week
    BTPS,                    // Bits per sample
    NUMCH,                   // Number of channels
    GAIN,                    // Audio gain
    LCUTOFF,                 // Lower cutoff frequency
    UCUTOFF,                 // Upper cutoff frequency
    FLDR,                    // folder structure
    LOCATION,                // Device location (latitude longitude)
    SMSFREQ,                 // SMS upload frequency
    LBAT,                    // Low battery_indicating voltage
    BUILD_VERSION,           // FW Build version
    BUILD_DATETIME,          // FW Build date and time
    BUILD_SHA,               // FW Build SHA
    SUN_RECORDING_MODE,      // Sun recording mode
    SUNRISE_INTERVAL_BEFORE, // Sunrise interval before
    SUNRISE_INTERVAL_AFTER,  // Sunrise interval after
    SUNSET_INTERVAL_BEFORE,  // Sunset interval before
    SUNSET_INTERVAL_AFTER,   // Sunset interval after
    SUNRISE_SUNSET_ROUNDING, // Sunrise sunset rounding
    NUM_CMDS                 // Number of Commands
};

/**
 *  Device config commands lookup / translation table
 */
const char *config_cmd_lookup_table[] = {
    "system_time",             // SYSTEMTIME
    "time_zone",               // TIMEZONE
    "transmission_mode",       // GSM
    "sampling_frequency",      // SAMF
    "record_length",           // RECT
    "sleep_interval",          // SLEEP
    "date_range",              // DATERANGE
    "time_of_day",             // TIMEOFDAY
    "time_of_day_0",           // TIMEOFDAY_0
    "time_of_day_1",           // TIMEOFDAY_1
    "time_of_day_2",           // TIMEOFDAY_2
    "time_of_day_3",           // TIMEOFDAY_3
    "time_of_day_4",           // TIMEOFDAY_4
    "time_of_day_5",           // TIMEOFDAY_5
    "time_of_day_6",           // TIMEOFDAY_6
    "time_of_day_7",           // TIMEOFDAY_7
    "time_of_day_8",           // TIMEOFDAY_8
    "time_of_day_9",           // TIMEOFDAY_9
    "days_of_week",            // DAYSOFWEEK
    "bits_per_sample",         // BTPS
    "number_of_channels",      // NUMCH
    "mic_gain",                // GAIN
    "lower_cutoff_frequency",  // LCUTOFF
    "upper_cutoff_frequency",  // UCUTOFF
    "folder_structure",        // FLDR
    "location",                // LOCATION
    "sms_frequency",           // SMSFREQ
    "low_battery_threshold",   // LBAT
    "build_version",           // BUILD_VERSION
    "build_datetime",          // BUILD_DATETIME
    "build_sha",               // BUILD_SHA
    "sun_recording_mode",      // SUN_RECORDING_MODE
    "sunrise_interval_before", // SUNRISE_INTERVAL_BEFORE
    "sunrise_interval_after",  // SUNRISE_INTERVAL_AFTER
    "sunset_interval_before",  // SUNSET_INTERVAL_BEFORE
    "sunset_interval_after",   // SUNSET_INTERVAL_AFTER
    "sunrise_sunset_rounding"  // SUNRISE_SUNSET_ROUNDING
};

static_assert((sizeof(config_cmd_lookup_table) /
               sizeof(config_cmd_lookup_table[0])) ==
                  static_cast<int>(sd_device_config_cmd_t::NUM_CMDS),
              "config_cmd_lookup_table size mismatch");

/**
 * @brief Func to lookup console command
 *
 * @param cmd buffer containing characters from terminal
 * @param length length of the command string
 * @return sd_device_config_cmd_t command ID
 */
static sd_device_config_cmd_t cmd_lookup_func(const char *cmd, uint32_t length)
{
    for (int cmd_id = 0;
         cmd_id < static_cast<int>(sd_device_config_cmd_t::NUM_CMDS);
         cmd_id++)
    {
        size_t lenpre            = strlen(config_cmd_lookup_table[cmd_id]);
        // Check if the lookup table entry is a substring of the command
        // and the next character is a space or the end of the string
        bool   equal_length      = (length == lenpre);
        bool   next_char_is_null = ((length > lenpre) && (cmd[lenpre] == '\0'));

        if ((equal_length || next_char_is_null) &&
            (memcmp(config_cmd_lookup_table[cmd_id], cmd, lenpre) == 0))
        {
            return static_cast<sd_device_config_cmd_t>(cmd_id);
        }
    }

    char temp_buffer[30];
    memset(temp_buffer, 0, sizeof(temp_buffer));
    strncpy(temp_buffer, cmd, length);
    printf("\nUnknown or BAD config option : %s\n", temp_buffer);

    return sd_device_config_cmd_t::NUM_CMDS;
}

/**
 * @brief Create configuration status file
 */
void create_config_status_file()
{
    const char    STATUS_FILE_PATH[] = "/config_status.txt";

    wHalErrorCode error_code =
        w_hal_sd_card_check_exists(STATUS_FILE_PATH, strlen(STATUS_FILE_PATH));

    wHalErrorCode error_set[] = {
        wHalErrorCode::NO_ERROR,
        wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST};
    SYSTEM_ASSERT_IN_SET(error_code,
                         error_set,
                         sizeof(error_set) / sizeof(error_set[0]));

    if (error_code == wHalErrorCode::NO_ERROR)
    {
        // Delete the file
        error_code = w_hal_sd_card_delete_file(STATUS_FILE_PATH,
                                               strlen(STATUS_FILE_PATH));

        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
    }

    wFileHandle config_status_handle = nullptr;
    error_code = w_hal_sd_card_create_file(STATUS_FILE_PATH,
                                           strlen(STATUS_FILE_PATH),
                                           config_status_handle);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    SYSTEM_ASSERT_NOT_EQUAL(config_status_handle,
                            static_cast<wFileHandle>(nullptr));

    // Write config status to file

    // Set up buffer to write to file
    static uint8_t
        config_status_buffer[BUFFER_SIZE]; // static local to avoid write
                                           // issues during midnight in sun
                                           // mode as config status previously
                                           // used shared audio buffer.
    const uint32_t buffer_size = sizeof(config_status_buffer);
    uint8_t *const p_buffer    = config_status_buffer;
    memset(p_buffer, 0, buffer_size);

    // Populate the buffer with the config status
    uint32_t buffer_offset = 0;

    buffer_offset += static_cast<uint32_t>(
        snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                 buffer_size - buffer_offset,
                 "Below are the parameters that are configured.\n"));

    // Populate all the parameters in the buffer
    for (int cmd_id = 0;
         cmd_id < static_cast<int>(sd_device_config_cmd_t::NUM_CMDS);
         cmd_id++)
    {
        SYSTEM_ASSERT_LT(buffer_offset, buffer_size);

        switch (static_cast<sd_device_config_cmd_t>(cmd_id))
        {
            case sd_device_config_cmd_t::SYSTEMTIME:
            {
                // Write date and time
                struct tm tm_struct;
                int time_zone_offset = DeviceConfig::p_page->time_zone_offset;
                time_t current_time =
                    DeviceConfig::p_page->program_time + time_zone_offset;
                gmtime_r(&current_time, &tm_struct);

                char     time_zone_sign;
                uint16_t tz_hour;
                uint16_t tz_min;
                w_hal_rtc_get_time_zone_components(time_zone_offset,
                                                   time_zone_sign,
                                                   tz_hour,
                                                   tz_min);

                buffer_offset += static_cast<uint32_t>(snprintf(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset,
                    "%s=%04d-%02d-%02d %02d:%02d:%02d %c%02d:%02d\n",

                    config_cmd_lookup_table[cmd_id],
                    TIME_BASE_YEAR + static_cast<uint32_t>(tm_struct.tm_year),
                    TIME_BASE_MONTH + static_cast<uint32_t>(tm_struct.tm_mon),
                    tm_struct.tm_mday,
                    tm_struct.tm_hour,
                    tm_struct.tm_min,
                    tm_struct.tm_sec,
                    time_zone_sign,
                    tz_hour,
                    tz_min));
                break;
            }
            case sd_device_config_cmd_t::TIMEZONE:
            {
                // Write time zone
                char     time_zone_sign;
                uint16_t tz_hour, tz_min;
                w_hal_rtc_get_time_zone_components(
                    DeviceConfig::p_page->time_zone_offset,
                    time_zone_sign,
                    tz_hour,
                    tz_min);

                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%c%02d:%02d\n",
                             config_cmd_lookup_table[cmd_id],
                             time_zone_sign,
                             tz_hour,
                             tz_min));
                break;
            }
#if COMM_CORE == INVALID_CORE_NUM
            case sd_device_config_cmd_t::GSM:
            case sd_device_config_cmd_t::SMSFREQ:
            {
                // Do nothing
                break;
            }
#else
            case sd_device_config_cmd_t::GSM:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=",
                             config_cmd_lookup_table[cmd_id]));

                DeviceConfig::get_transmission_mode(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset);
                buffer_offset +=
                    strlen(reinterpret_cast<char *>(p_buffer) + buffer_offset);

                p_buffer[buffer_offset++] = '\n';

                break;
            }
            case sd_device_config_cmd_t::SMSFREQ:
            {
                buffer_offset += static_cast<uint32_t>(snprintf(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset,
                    "%s=%d\n",
                    config_cmd_lookup_table[cmd_id],
                    DeviceConfig::p_page->sms_send_upload_frequency_seconds));

                break;
            }
#endif // COMM_CORE != INVALID_CORE_NUM
#if AUDIO_CORE != INVALID_CORE_NUM
            case sd_device_config_cmd_t::SAMF:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->sampling_rate));
                break;
            }
            case sd_device_config_cmd_t::BTPS:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->audio_sampling_width));
                break;
            }
            case sd_device_config_cmd_t::NUMCH:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->num_audio_channels));
                break;
            }
            case sd_device_config_cmd_t::GAIN:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%f\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->mic_gain_times_ten / 10.0));
                break;
            }
            case sd_device_config_cmd_t::LCUTOFF:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->lower_cutoff_frequency));
                break;
            }
            case sd_device_config_cmd_t::UCUTOFF:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->upper_cutoff_frequency));
                break;
            }
#endif // AUDIO_CORE != INVALID_CORE_NUM

#if STORAGE_CORE != INVALID_CORE_NUM
            case sd_device_config_cmd_t::RECT:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->recording_length));
                break;
            }
            case sd_device_config_cmd_t::SLEEP:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->sleep_pause_interval));
                break;
            }
            case sd_device_config_cmd_t::DATERANGE:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=",
                             config_cmd_lookup_table[cmd_id]));

                DeviceConfig::get_start_end_date(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset);
                buffer_offset +=
                    strlen(reinterpret_cast<char *>(p_buffer) + buffer_offset);

                buffer_offset += static_cast<uint32_t>(snprintf(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset,
                    ", Note: The date range is inclusive of the start and end "
                    "dates. 9999/99/99 means no date is selected.\n"));

                break;
            }
            case sd_device_config_cmd_t::TIMEOFDAY:
            {
                // TIMEOFDAY and TIMEOFDAY_0 are the same. No need to print
                // TIMEOFDAY again
                break;
            }
            case sd_device_config_cmd_t::TIMEOFDAY_0:
            case sd_device_config_cmd_t::TIMEOFDAY_1:
            case sd_device_config_cmd_t::TIMEOFDAY_2:
            case sd_device_config_cmd_t::TIMEOFDAY_3:
            case sd_device_config_cmd_t::TIMEOFDAY_4:
            case sd_device_config_cmd_t::TIMEOFDAY_5:
            case sd_device_config_cmd_t::TIMEOFDAY_6:
            case sd_device_config_cmd_t::TIMEOFDAY_7:
            case sd_device_config_cmd_t::TIMEOFDAY_8:
            case sd_device_config_cmd_t::TIMEOFDAY_9:
            {
                // TIMEOFDAY and TIMEOFDAY_0 are the same. No need to print
                // TIMEOFDAY again
                const int index =
                    cmd_id -
                    static_cast<int>(sd_device_config_cmd_t::TIMEOFDAY_0);

                if (!DeviceConfig::is_valid_time_of_day_range(
                        DeviceConfig::p_page->time_of_day_range[index]))
                {
                    continue;
                }

                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=",
                             config_cmd_lookup_table[cmd_id]));

                DeviceConfig::get_start_end_time_of_day(
                    static_cast<uint32_t>(index),
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset);
                buffer_offset +=
                    strlen(reinterpret_cast<char *>(p_buffer) + buffer_offset);

                p_buffer[buffer_offset++] = '\n';

                break;
            }
            case sd_device_config_cmd_t::DAYSOFWEEK:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=",
                             config_cmd_lookup_table[cmd_id]));

                DeviceConfig::get_days_of_week(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset);
                buffer_offset +=
                    strlen(reinterpret_cast<char *>(p_buffer) + buffer_offset);

                p_buffer[buffer_offset++] = '\n';

                break;
            }
            case sd_device_config_cmd_t::FLDR:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=",
                             config_cmd_lookup_table[cmd_id]));

                DeviceConfig::get_folder_structure(
                    reinterpret_cast<char *>(p_buffer) + buffer_offset,
                    buffer_size - buffer_offset);
                buffer_offset +=
                    strlen(reinterpret_cast<char *>(p_buffer) + buffer_offset);

                p_buffer[buffer_offset++] = '\n';

                break;
            }
#endif // STORAGE_CORE != INVALID_CORE_NUM

            case sd_device_config_cmd_t::LOCATION:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=",
                             config_cmd_lookup_table[cmd_id]));

                DeviceConfig::get_location(reinterpret_cast<char *>(p_buffer) +
                                               buffer_offset,
                                           buffer_size - buffer_offset);
                buffer_offset +=
                    strlen(reinterpret_cast<char *>(p_buffer) + buffer_offset);

                p_buffer[buffer_offset++] = '\n';
                break;
            }

            case sd_device_config_cmd_t::LBAT:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             DeviceConfig::p_page->low_battery_voltage_cutoff));
                break;
            }

            case sd_device_config_cmd_t::BUILD_VERSION:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%s\n",
                             config_cmd_lookup_table[cmd_id],
                             fw_version));
                break;
            }
            case sd_device_config_cmd_t::BUILD_DATETIME:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%s\n",
                             config_cmd_lookup_table[cmd_id],
                             build_datetime));
                break;
            }
            case sd_device_config_cmd_t::BUILD_SHA:
            {
                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%s\n",
                             config_cmd_lookup_table[cmd_id],
                             git_sha));
                break;
            }
            case sd_device_config_cmd_t::SUN_RECORDING_MODE:
            {
                const char *mode_str = "Disabled";
                switch (DeviceConfig::p_page->sun_recording_mode)
                {
                    case DeviceConfig::SUN_MODE_SUNRISE:
                        mode_str = "Sunrise";
                        break;
                    case DeviceConfig::SUN_MODE_SUNSET:
                        mode_str = "Sunset";
                        break;
                    case DeviceConfig::SUN_MODE_SUNRISE_AND_SUNSET:
                        mode_str = "Sunrise and Sunset";
                        break;
                    case DeviceConfig::SUN_MODE_SUNRISE_TO_SUNSET:
                        mode_str = "Sunrise to Sunset";
                        break;
                    case DeviceConfig::SUN_MODE_SUNSET_TO_SUNRISE:
                        mode_str = "Sunset to Sunrise";
                        break;
                    default:
                        mode_str = "Disabled";
                        break;
                }

                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%s\n",
                             config_cmd_lookup_table[cmd_id],
                             mode_str));
                break;
            }
            case sd_device_config_cmd_t::SUNRISE_INTERVAL_BEFORE:
            case sd_device_config_cmd_t::SUNRISE_INTERVAL_AFTER:
            case sd_device_config_cmd_t::SUNSET_INTERVAL_BEFORE:
            case sd_device_config_cmd_t::SUNSET_INTERVAL_AFTER:
            case sd_device_config_cmd_t::SUNRISE_SUNSET_ROUNDING:
            {
                // Do not print sun interval params when sun mode is disabled.
                // Stale values from a previous mode would otherwise be
                // reapplied on next boot via OnBootConfig.
                if (DeviceConfig::p_page->sun_recording_mode ==
                    DeviceConfig::SUN_MODE_DISABLED)
                {
                    break;
                }

                uint32_t value = 0;
                switch (static_cast<sd_device_config_cmd_t>(cmd_id))
                {
                    case sd_device_config_cmd_t::SUNRISE_INTERVAL_BEFORE:
                        value = DeviceConfig::p_page->sunrise_interval_before;
                        break;
                    case sd_device_config_cmd_t::SUNRISE_INTERVAL_AFTER:
                        value = DeviceConfig::p_page->sunrise_interval_after;
                        break;
                    case sd_device_config_cmd_t::SUNSET_INTERVAL_BEFORE:
                        value = DeviceConfig::p_page->sunset_interval_before;
                        break;
                    case sd_device_config_cmd_t::SUNSET_INTERVAL_AFTER:
                        value = DeviceConfig::p_page->sunset_interval_after;
                        break;
                    case sd_device_config_cmd_t::SUNRISE_SUNSET_ROUNDING:
                        value = DeviceConfig::p_page->sunrise_sunset_rounding;
                        break;
                    default:
                        break;
                }

                buffer_offset += static_cast<uint32_t>(
                    snprintf(reinterpret_cast<char *>(p_buffer) + buffer_offset,
                             buffer_size - buffer_offset,
                             "%s=%d\n",
                             config_cmd_lookup_table[cmd_id],
                             value));
                break;
            }

            default:
            {
                printf("Invalid command entered %d in config status file\n",
                       static_cast<int>(cmd_id));
                break;
            }
        }
    }

    // Write to sd card
    error_code = w_hal_sd_card_write_to_file(p_buffer,
                                             buffer_offset,
                                             config_status_handle);

    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    do
    {
        // Check if busy
        error_code = w_hal_sd_card_handler();
    } while (error_code == wHalErrorCode::BUSY);

    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Close the outstanding file
    error_code = w_hal_sd_card_close_file(config_status_handle);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    printf("Configuration status file has been created\n");
}

/**
 *@brief Convert sd card config parameter to console command
 *
 * @param p_command command to be converted
 * @param length length of the command string
 * @param p_out_buffer[out]: output command string
 * @param buffer_length: size of the output buffer
 * @return uint32_t returns the string length of the command
 */
static uint32_t extract_command(const char *p_command,
                                uint32_t    length,
                                char *      p_out_buffer,
                                uint32_t    buffer_length)
{
    int match_length = 0;
    int match_index;

    match_index = re_match("[a-zA-Z0-9_]+\\s*=",
                           p_command,
                           static_cast<int>(length),
                           &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        p_command += match_index;

        const char *value_buff = p_command + match_length;
        uint32_t    value_length =
            length - static_cast<uint32_t>(match_length + match_index);

        // Skip spaces and tabs
        while (isspace(*value_buff))
        {
            value_buff++;
        }

        // Skip spaces and equal sign
        while (match_length > 0)
        {
            const char *p_last_char = p_command + match_length - 1;
            if (!isspace(*p_last_char) && (*p_last_char != '='))
            {
                break;
            }

            match_length--;
        }

        sd_device_config_cmd_t cmd_id =
            cmd_lookup_func(p_command, static_cast<uint32_t>(match_length));

        switch (cmd_id)
        {
            // Command to set date and time
            case sd_device_config_cmd_t::SYSTEMTIME:
            {
                snprintf(p_out_buffer, buffer_length, "DATETIME ");
                break;
            }

            case sd_device_config_cmd_t::TIMEZONE:
            {
                snprintf(p_out_buffer, buffer_length, "TIMEZONE ");
                break;
            }

#if COMM_CORE != INVALID_CORE_NUM
            // Command to set GSM transmission mode
            case sd_device_config_cmd_t::GSM:
            {
                snprintf(p_out_buffer, buffer_length, "GSM ");
                break;
            }

            // Command to set SMS upload frequency
            case sd_device_config_cmd_t::SMSFREQ:
            {
                snprintf(p_out_buffer, buffer_length, "SMSFREQ ");
                break;
            }
#endif // COMM_CORE != INVALID_CORE_NUM

#if AUDIO_CORE != INVALID_CORE_NUM
            // Command to set recording sampling frequency
            case sd_device_config_cmd_t::SAMF:
            {
                snprintf(p_out_buffer, buffer_length, "SAMF ");
                break;
            }

            // Command to set bits per sample
            case sd_device_config_cmd_t::BTPS:
            {
                snprintf(p_out_buffer, buffer_length, "BTPS ");
                break;
            }

            // Command to set number of audio channels
            case sd_device_config_cmd_t::NUMCH:
            {
                snprintf(p_out_buffer, buffer_length, "NUMCH ");
                break;
            }

            // Command to set digital gain
            case sd_device_config_cmd_t::GAIN:
            {
                snprintf(p_out_buffer, buffer_length, "GAIN ");
                break;
            }

            // Command to set lower cutoff frequency
            case sd_device_config_cmd_t::LCUTOFF:
            {
                snprintf(p_out_buffer, buffer_length, "LCUTOFF ");
                break;
            }

            // Command to set upper cutoff frequency
            case sd_device_config_cmd_t::UCUTOFF:
            {
                snprintf(p_out_buffer, buffer_length, "UCUTOFF ");
                break;
            }
#endif // AUDIO_CORE != INVALID_CORE_NUM

#if STORAGE_CORE != INVALID_CORE_NUM
            // Command to set recording length
            case sd_device_config_cmd_t::RECT:
            {
                snprintf(p_out_buffer, buffer_length, "RECT ");
                break;
            }

            // Command to set sleep pause interval
            case sd_device_config_cmd_t::SLEEP:
            {
                snprintf(p_out_buffer, buffer_length, "SLEEP ");
                break;
            }

            // Command to set operating date range
            case sd_device_config_cmd_t::DATERANGE:
            {
                snprintf(p_out_buffer, buffer_length, "DATERANGE ");
                break;
            }

            // Command to set operating time of the day
            case sd_device_config_cmd_t::TIMEOFDAY:
            {
                snprintf(p_out_buffer, buffer_length, "TIMEOFDAY ");
                break;
            }

            case sd_device_config_cmd_t::TIMEOFDAY_0:
            case sd_device_config_cmd_t::TIMEOFDAY_1:
            case sd_device_config_cmd_t::TIMEOFDAY_2:
            case sd_device_config_cmd_t::TIMEOFDAY_3:
            case sd_device_config_cmd_t::TIMEOFDAY_4:
            case sd_device_config_cmd_t::TIMEOFDAY_5:
            case sd_device_config_cmd_t::TIMEOFDAY_6:
            case sd_device_config_cmd_t::TIMEOFDAY_7:
            case sd_device_config_cmd_t::TIMEOFDAY_8:
            case sd_device_config_cmd_t::TIMEOFDAY_9:
            {
                snprintf(
                    p_out_buffer,
                    buffer_length,
                    "TIMEOFDAY %d ",
                    static_cast<int>(cmd_id) -
                        static_cast<int>(sd_device_config_cmd_t::TIMEOFDAY_0));
                break;
            }

            // Command to set operating days of the week
            case sd_device_config_cmd_t::DAYSOFWEEK:
            {
                snprintf(p_out_buffer, buffer_length, "DAYSOFWEEK ");
                break;
            }

            // Command to set folder structure
            case sd_device_config_cmd_t::FLDR:
            {
                snprintf(p_out_buffer, buffer_length, "FLDR ");
                break;
            }
#endif // STORAGE_CORE != INVALID_CORE_NUM

            case sd_device_config_cmd_t::LOCATION:
            {
                snprintf(p_out_buffer, buffer_length, "LOC ");
                break;
            }

            // Command to set low voltage threshold
            case sd_device_config_cmd_t::LBAT:
            {
                snprintf(p_out_buffer, buffer_length, "LBAT ");
                break;
            }

            // Command to set sun recording mode and related parameters
            case sd_device_config_cmd_t::SUN_RECORDING_MODE:
            {
                snprintf(p_out_buffer, buffer_length, "SUNMODE ");
                break;
            }
            case sd_device_config_cmd_t::SUNRISE_INTERVAL_BEFORE:
            {
                snprintf(p_out_buffer, buffer_length, "SUNRISEBEF ");
                break;
            }
            case sd_device_config_cmd_t::SUNRISE_INTERVAL_AFTER:
            {
                snprintf(p_out_buffer, buffer_length, "SUNRISEAFT ");
                break;
            }
            case sd_device_config_cmd_t::SUNSET_INTERVAL_BEFORE:
            {
                snprintf(p_out_buffer, buffer_length, "SUNSETBEF ");
                break;
            }
            case sd_device_config_cmd_t::SUNSET_INTERVAL_AFTER:
            {
                snprintf(p_out_buffer, buffer_length, "SUNSETAFT ");
                break;
            }
            case sd_device_config_cmd_t::SUNRISE_SUNSET_ROUNDING:
            {
                snprintf(p_out_buffer, buffer_length, "SUNROUND ");
                break;
            }

            default:
            {
                printf("Invalid command entered %d\n",
                       static_cast<int>(cmd_id));
                return 0;
            }
        }

        uint32_t str_len = strlen(p_out_buffer);
        strncpy(p_out_buffer + str_len, value_buff, value_length);

        return strlen(p_out_buffer);
    }
    return 0;
}

/**
 * @brief To process the config file
 *
 * @param p_line command string
 * @param length length of the command string
 */
static void process_config_line(char *p_line, uint32_t length)
{
    // Trim leading spaces, tabs, and newlines
    while (isspace(*p_line))
    {
        p_line++;
        length--;
    }

    // Ignore fully commented lines and parts of lines
    int match_length = 0;
    int match_index =
        re_match("#", p_line, static_cast<int>(length), &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        // Only process the part of the line before the comment
        length = static_cast<uint32_t>(match_index);
    }

    // Ignore empty lines
    if (length == 0)
    {
        return;
    }

    // Trim leading spaces, tabs, and newlines
    while (isspace(*p_line))
    {
        p_line++;
        length--;
    }

    // Trim trailing spaces, tabs, and newlines
    while (length > 0)
    {
        char *p_last_char = p_line + length - 1;
        if (!isspace(*p_last_char))
        {
            break;
        }

        length--;
    }

    match_length = 0;
    match_index  = re_match("[a-zA-Z0-9_]+\\s*=\\s*[a-zA-Z0-9_\\-:/\\.]+",
                           p_line,
                           static_cast<int>(length),
                           &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        char console_command_buffer[100];
        memset(console_command_buffer, 0, sizeof(console_command_buffer));

        uint32_t console_command_length =
            extract_command(p_line,
                            length,
                            console_command_buffer,
                            sizeof(console_command_buffer));

        if (console_command_length > 0)
        {
            console_command_buffer[console_command_length] = '\0';

            printf("OnBootConfig - Sending command to console: %s\n",
                   console_command_buffer);

            new_console_command_handler(console_command_buffer,
                                        console_command_length);
        }
    }
}

/**
 * @brief Read config file from SD card
 *
 * @return true if successfully read and applied the configuration
 * settings
 * @return false if failed to read config file
 */
bool read_config_from_sd_card()
{
    // Check if config.ini file exists. It is always located in the root
    // of the SD card If it does, read it and parse it
    const char *  config_file_path = "/config.ini";

    wHalErrorCode error =
        w_hal_sd_card_check_exists(config_file_path, strlen(config_file_path));

    if (error != wHalErrorCode::NO_ERROR)
    {
        printf("Configuration file is missing. Using earlier programmed "
               "configuration.\n");

        // Create the configuration status file with the current settings
        create_config_status_file();

        return false;
    }

    wFileHandle config_ini_file = nullptr;
    error                       = w_hal_sd_card_open_file(config_file_path,
                                    strlen(config_file_path),
                                    config_ini_file);

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    uint8_t *const p_buffer      = shared_buffers_glbl.audio_buffer;
    const uint32_t buffer_size   = BUFFER_SIZE * NUM_BUFFERS;

    uint32_t       buffer_offset = 0;

    // Outer loop to read the config_ini_file in chunks
    // Inner loop to parse the chunk line by line
    while (true)
    {
        SYSTEM_ASSERT_LT(buffer_offset, buffer_size);

        // Read the config_ini_file
        uint32_t bytes_read = 0;
        error = w_hal_sd_card_read_from_file(p_buffer + buffer_offset,
                                             buffer_size - buffer_offset,
                                             bytes_read,
                                             config_ini_file);

        if (error != wHalErrorCode::NO_ERROR)
        {
            break;
        }

        // Nothing more to read?
        if (bytes_read == 0)
        {
            break;
        }

        bool     end_of_file     = (bytes_read < buffer_size - buffer_offset);

        // Parse the chunk line by line from the beginning of the
        // buffer
        uint32_t bytes_in_buffer = buffer_offset + bytes_read;
        for (buffer_offset = 0; buffer_offset < bytes_in_buffer;)
        {
            int match_length = 0;
            int match_index =
                re_match("[\r\n]",
                         reinterpret_cast<char *>(p_buffer + buffer_offset),
                         static_cast<int>(bytes_in_buffer - buffer_offset),
                         &match_length);

            if (match_index != RE_NO_MATCH_INDEX)
            {
                process_config_line(
                    reinterpret_cast<char *>(p_buffer + buffer_offset),
                    static_cast<uint32_t>(match_index));
            }
            buffer_offset += static_cast<uint32_t>(match_index) + 1;
        }

        if (buffer_offset < bytes_in_buffer)
        {
            if (end_of_file)
            {
                process_config_line(
                    reinterpret_cast<char *>(p_buffer + buffer_offset),
                    bytes_in_buffer - buffer_offset);

                buffer_offset = bytes_in_buffer;
            }
            else
            {
                // Move the remaining bytes to the beginning of the
                // buffer
                memmove(p_buffer,
                        p_buffer + buffer_offset,
                        bytes_in_buffer - buffer_offset);

                buffer_offset = bytes_in_buffer - buffer_offset;
            }
        }

        if (end_of_file)
        {
            break;
        }
    }

    // Close the outstanding file
    error = w_hal_sd_card_close_file(config_ini_file);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Create the configuration status file
    create_config_status_file();

    // Delete the configuration file from the sd card
    error =
        w_hal_sd_card_delete_file(config_file_path, strlen(config_file_path));
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    printf("Configuration file has been deleted\n");

    return true;
}

#endif // __CORE__ == STORAGE_CORE