#include "device_config.hpp"
#include "assert.hpp"
#include "constants.hpp"
#include "device_config_version.hpp"

#include "w_hal_error_code.hpp"
#include "w_hal_flash.hpp"
#include "w_hal_gsm.hpp"
#include "w_hal_i2s.hpp"
#include "w_hal_ocm.hpp"
#include "w_hal_sd_card.hpp"

#include <stdio.h>
#include <string.h>

DeviceConfig::config_page_st *DeviceConfig::p_page; // Global config info
DeviceConfig::transmission_mode_t
    DeviceConfig::max_transmission_mode; // Global declaration of
                                         // max_transmission_mode

static const uint32_t NUM_CONFIG_PAGES = 2;
static const uint32_t MAGIC_NUMBER     = 0x13245768;

/**
 * @brief Generate checksum for config page.
 *
 * @return uint32_t checksum
 */
uint32_t              DeviceConfig::generate_checksum()
{
    uint32_t checksum = 0;
    for (uint32_t index = 0;
         index < sizeof(config_page_st) - sizeof(p_page->checksum);
         index++)
    {
        checksum += reinterpret_cast<uint8_t *>(p_page)[index];
    }

    return checksum;
}

/**
 * @brief Get minimum start time for RTC
 *
 * @return time_t the minimum start time
 */
time_t DeviceConfig::get_minimum_start_time()
{
    tm time_struct;

    memset(&time_struct, 0, sizeof(time_struct));
    time_struct.tm_year = MIN_CONFIGURATION_YEAR - TIME_BASE_YEAR;
    time_struct.tm_mon  = 1 - TIME_BASE_MONTH;
    time_struct.tm_mday = 1;

    return w_hal_rtc_timegm(&time_struct);
}

/**
 * @brief Validate config page.
 *
 * @return true if valid
 * @return false otherwise
 */
bool DeviceConfig::validate()
{
    // Check the validity of the config page.
    if (p_page->magic_number != MAGIC_NUMBER)
    {
        return false;
    }

    // Check the checksum of the config page.
    uint32_t checksum = generate_checksum();

    if (checksum != p_page->checksum)
    {
        return false;
    }

    return param_validate(false);
}

/**
 * @brief Get start and end date in string format
 *
 * @param p_string string buffer
 * @param size size of the string buffer
 */
void DeviceConfig::get_start_end_date(char *p_string, uint32_t size)
{
    // Format of the string is "YYYY/MM/DD-YYYY/MM/DD"
    // Example: "2018/01/01-2018/12/31"

    uint32_t start_year, start_month, start_day;
    uint32_t end_year, end_month, end_day;

    if ((p_page->start_date.year == INVALID_YEAR) ||
        (p_page->start_date.month == INVALID_MONTH) ||
        (p_page->start_date.day == INVALID_DAY_IN_MONTH))
    {
        start_year  = 9999;
        start_month = 99;
        start_day   = 99;
    }
    else
    {
        start_year  = p_page->start_date.year + TIME_BASE_YEAR;
        start_month = p_page->start_date.month + TIME_BASE_MONTH;
        start_day   = p_page->start_date.day;
    }

    if ((p_page->end_date.year == INVALID_YEAR) ||
        (p_page->end_date.month == INVALID_MONTH) ||
        (p_page->end_date.day == INVALID_DAY_IN_MONTH))
    {
        end_year  = 9999;
        end_month = 99;
        end_day   = 99;
    }
    else
    {
        end_year  = p_page->end_date.year + TIME_BASE_YEAR;
        end_month = p_page->end_date.month + TIME_BASE_MONTH;
        end_day   = p_page->end_date.day;
    }

    int ret_val = snprintf(p_string,
                           size,
                           "%04d/%02d/%02d-%04d/%02d/%02d",
                           start_year,
                           start_month,
                           start_day,
                           end_year,
                           end_month,
                           end_day);

    SYSTEM_ASSERT_GT(ret_val, 0);
}

/**
 * @brief Get location in string format
 *
 * @param p_string string buffer

 * * @param size size of the string buffer
 */
void DeviceConfig::get_location(char *p_string, uint32_t size)
{
    int ret_val = -1;

    if ((p_page->latitude_e7 == INVALID_LOCATION_COORDINATE) ||
        (p_page->longitude_e7 == INVALID_LOCATION_COORDINATE))
    {
        ret_val = snprintf(p_string, size, "Not set");
    }
    else
    {
        ret_val = snprintf(p_string,
                           size,
                           "%.7f %.7f",
                           static_cast<double>(p_page->latitude_e7) /
                               LOCATION_COORDINATE_SCALE,
                           static_cast<double>(p_page->longitude_e7) /
                               LOCATION_COORDINATE_SCALE);
    }

    SYSTEM_ASSERT_GT(ret_val, 0);
}

/**
 * @brief Clear all start and end time of day ranges
 */
void DeviceConfig::reset_time_of_day_ranges()
{
    for (uint32_t index = 0; index < MAX_SCHEDULE_TIME_OF_DAY_RANGES; index++)
    {
        p_page->time_of_day_range[index].start_second = 0;
        p_page->time_of_day_range[index].end_second   = NUM_SECONDS_IN_DAY;
    }
}

/**
 * @brief Check if time of day range is valid
 *
 * @param time_of_day_range time of day range
 */
bool DeviceConfig::is_valid_time_of_day_range(
    const DeviceConfig::time_of_day_range_st &time_of_day_range)
{
    return ((time_of_day_range.start_second > 0) ||
            (time_of_day_range.end_second < NUM_SECONDS_IN_DAY)) &&
           (time_of_day_range.start_second < time_of_day_range.end_second);
}

/**
 * @brief Sort time of day ranges ascending by start_second and merge overlaps.
 *
 * @param p_ranges pointer to the range array
 * @param num_ranges total number of slots in the array
 */
void DeviceConfig::sort_time_of_day_ranges(
    DeviceConfig::time_of_day_range_st *p_ranges,
    uint32_t                            num_ranges)
{
    for (uint32_t i = 0; i < num_ranges; i++)
    {
        for (uint32_t j = i + 1; j < num_ranges; j++)
        {
            bool valid_i = is_valid_time_of_day_range(p_ranges[i]);
            bool valid_j = is_valid_time_of_day_range(p_ranges[j]);
            bool swap    = false;

            if (valid_i && valid_j)
            {
                // Overlapping ranges: merge into p_ranges[i], invalidate j.
                if ((p_ranges[i].start_second <= p_ranges[j].end_second) &&
                    (p_ranges[i].end_second >= p_ranges[j].start_second))
                {
                    if (p_ranges[j].start_second < p_ranges[i].start_second)
                        p_ranges[i].start_second = p_ranges[j].start_second;
                    if (p_ranges[j].end_second > p_ranges[i].end_second)
                        p_ranges[i].end_second = p_ranges[j].end_second;

                    // Reset j to the "all-day" sentinel (treated as invalid)
                    p_ranges[j].start_second = 0;
                    p_ranges[j].end_second   = NUM_SECONDS_IN_DAY;
                }
                else if (p_ranges[i].start_second > p_ranges[j].start_second)
                {
                    // Out of order: bring the earlier-starting range forward
                    swap = true;
                }
            }
            else if (valid_j)
            {
                // Push invalid entries toward the back
                swap = true;
            }

            if (swap)
            {
                DeviceConfig::time_of_day_range_st tmp = p_ranges[i];
                p_ranges[i]                            = p_ranges[j];
                p_ranges[j]                            = tmp;
            }
        }
    }
}

/**
 * @brief Get start and end time of day in string format
 *
 * @param index index of the time of day range
 * @param p_string string buffer
 * @param size size of the string buffer
 */
void DeviceConfig::get_start_end_time_of_day(uint32_t index,
                                             char *   p_string,
                                             uint32_t size)
{
    // Format of the string is "HH:MM:SS-HH:MM:SS"
    // Example: "00:00:00-23:59:59"

    if (index >= MAX_SCHEDULE_TIME_OF_DAY_RANGES)
    {
        index = 0;
    }

    time_of_day_range_st *p_time_of_day = &p_page->time_of_day_range[index];

    uint32_t start_minute = p_time_of_day->start_second / SECS_PER_MIN;
    uint32_t end_minute   = p_time_of_day->end_second / SECS_PER_MIN;

    int      ret_val      = snprintf(p_string,
                           size,
                           "%02d:%02d:%02d-%02d:%02d:%02d",
                           start_minute / MINS_PER_HR,
                           start_minute % MINS_PER_HR,
                           p_time_of_day->start_second % SECS_PER_MIN,
                           end_minute / MINS_PER_HR,
                           end_minute % MINS_PER_HR,
                           p_time_of_day->end_second % SECS_PER_MIN);

    SYSTEM_ASSERT_GT(ret_val, 0);
}

/**
 * @brief Get days of week in string format
 *
 * @param p_string string buffer
 * @param size size of the string buffer
 */
void DeviceConfig::get_days_of_week(char *p_string, uint32_t size)
{
    // Format of the string is "MTWRFSU"
    SYSTEM_ASSERT_GTEQ(size, 8);

    for (int i = 0; i < 7; i++)
    {
        if (p_page->day_of_week_mask & (1 << i))
        {
            *p_string++ = "MTWRFSU"[i];
        }
    }
    p_string[0] = '\0';
}

/**
 * @brief Get detector in string format
 *
 * @param p_string string buffer
 * @param size size of the string buffer
 */
void DeviceConfig::get_detector(char *p_string, uint32_t size)
{
    int ret_val = -1;

    switch (p_page->detector)
    {
        case detector_select_t::NONE:
            ret_val = snprintf(p_string, size, "None");
            break;
        case detector_select_t::AI:
            ret_val = snprintf(p_string, size, "AI Detector");
            break;
        case detector_select_t::SILENCE_AMPLITUDE:
            ret_val = snprintf(p_string, size, "Silence Amplitude Detector");
            break;
        default:
            break;
    }

    SYSTEM_ASSERT_GT(ret_val, 0);
}

/**
 * @brief Get folder structure configuration in string format
 *
 * @param p_string string buffer
 * @param size size of the string buffer
 */
void DeviceConfig::get_folder_structure(char *p_string, uint32_t size)
{
    int ret_val = -1;

    switch (p_page->folder_structure)
    {
        case folder_structure_t::HOUR:
            ret_val = snprintf(p_string, size, "hourly");
            break;
        case folder_structure_t::DAY:
            ret_val = snprintf(p_string, size, "daily");
            break;
        case folder_structure_t::FLAT:
            ret_val = snprintf(p_string, size, "flat");
            break;
        default:
            break;
    }

    SYSTEM_ASSERT_GT(ret_val, 0);
}

/**
 * @brief Get transmission mode configuration in string format
 *
 * @param p_string string buffer
 * @param size size of the string buffer
 */
void DeviceConfig::get_transmission_mode(char *p_string, uint32_t size)
{
    int ret_val = -1;

    switch (p_page->transmission_mode)
    {
        case transmission_mode_t::MODE_2G:
            ret_val = snprintf(p_string, size, "2G");
            break;
        case transmission_mode_t::MODE_3G:
            ret_val = snprintf(p_string, size, "3G");
            break;
        case transmission_mode_t::MODE_4G:
            ret_val = snprintf(p_string, size, "4G");
            break;
        default:
            break;
    }

    SYSTEM_ASSERT_GT(ret_val, 0);
}

/**
 * @brief Initialize test server setting as device configuration.
 */
void DeviceConfig::initialize_test_server_setting()
{
    const char                server_url[]        = "34.221.132.131";
    const char                server_post_route[] = "/api/upload";
    const uint16_t            server_port         = 80;
    const http_content_type_t server_content_type =
        http_content_type_t::MULTIPART_FORM_DATA;

    strncpy(p_page->server_url, server_url, sizeof(p_page->server_url));
    p_page->server_port = server_port;
    strncpy(p_page->server_post_route,
            server_post_route,
            sizeof(p_page->server_post_route));
    p_page->http_content_type = server_content_type;
}

/**
 * @brief Initialize wildsonic server setting as device configuration.
 */
void DeviceConfig::initialize_wildsonic_server_setting()
{
    const char                server_url[]        = "dwsai.gleeze.com";
    const char                server_post_route[] = "/api/upload-device-params";
    const uint16_t            server_port         = 80;
    const http_content_type_t server_content_type =
        http_content_type_t::OCTET_STREAM;

    strncpy(p_page->server_url, server_url, sizeof(p_page->server_url));
    p_page->server_port = server_port;
    strncpy(p_page->server_post_route,
            server_post_route,
            sizeof(p_page->server_post_route));
    p_page->http_content_type = server_content_type;
}

/**
 * @brief Initialize configuration defaults.
 */
void DeviceConfig::initialize_config_defaults()
{
    memset(p_page, 0, sizeof(config_page_st));
    p_page->magic_number    = MAGIC_NUMBER;
    p_page->version         = DEVICE_CONFIG_VERSION_NUMBER;
    p_page->sequence_number = 1;

    // No serial number, password, or device key until it is assigned during
    // manufacturing.
    strncpy(p_page->serial_number, "0", sizeof(p_page->serial_number));

    strncpy(p_page->device_password, "", sizeof(p_page->device_password));
    strncpy(p_page->sms_key, "", sizeof(p_page->sms_key));

    p_page->a2p_phone_number[0] = '\0';
    p_page->p2a_phone_number[0] = '\0';

    strncpy(p_page->a2p_phone_number,
            "WLDTHD",
            sizeof(p_page->a2p_phone_number));

    // Test server is the default server for now.
    initialize_test_server_setting();

    p_page->sms_send_upload_frequency_seconds = DEFAULT_SMS_UPLOAD_FREQUENCY;
    p_page->mic_gain_times_ten                = DEFAULT_MIC_GAIN_TIMES_TEN;
    p_page->audio_sampling_width              = DEFAULT_BITS_PER_AUDIO_SAMPLE;
    p_page->num_audio_channels                = DEFAULT_NUM_AUDIO_CHANNELS;
    p_page->sampling_rate                     = DEFAULT_AUDIO_SAMPLE_RATE;
    p_page->recording_length                  = DEFAULT_SECONDS_PER_AUDIO_FILE;
    p_page->sleep_pause_interval              = DEFAULT_SLEEP_PAUSE_INTERVAL;

    p_page->folder_structure                  = folder_structure_t::HOUR;

    p_page->program_time                      = get_minimum_start_time();
    p_page->time_zone_offset                  = 0;

    p_page->start_date.year                   = INVALID_YEAR;
    p_page->start_date.month                  = INVALID_MONTH;
    p_page->start_date.day                    = INVALID_DAY_IN_MONTH;

    p_page->end_date.year                     = INVALID_YEAR;
    p_page->end_date.month                    = INVALID_MONTH;
    p_page->end_date.day                      = INVALID_DAY_IN_MONTH;

    reset_time_of_day_ranges();
    p_page->day_of_week_mask           = FULL_WEEK_MASK; // All days of the week

    p_page->detector                   = detector_select_t::NONE;
    p_page->ai_sensitivity             = 50; // Balanced sensitivity
    p_page->transmission_mode          = transmission_mode_t::MODE_4G;

    p_page->lower_cutoff_frequency     = 0;
    p_page->upper_cutoff_frequency     = p_page->sampling_rate / 2;

    p_page->low_battery_voltage_cutoff = DEFAULT_LOW_BATTERY_VOLTAGE;
    p_page->num_battery_cells_used     = DEFAULT_NUMBER_OF_BATTERY_CELLS;
    p_page->latitude_e7                = INT32_MIN;
    p_page->longitude_e7               = INT32_MIN;

    p_page->flags.dw                   = 0;
}

/**
 * @brief Validate HAL configuration.
 *
 * @param fix indicates whether to fix the config
 * @return true if valid
 * @return false otherwise
 */
bool DeviceConfig::param_validate(bool fix)
{
#if (AUDIO_CORE != INVALID_CORE_NUM) ||                                        \
    (OFF_CHIP_MEMORY_CORE != INVALID_CORE_NUM) ||                              \
    (STORAGE_CORE != INVALID_CORE_NUM) || (COMM_CORE != INVALID_CORE_NUM)
    wHalErrorCode error;
#endif

#if AUDIO_CORE != INVALID_CORE_NUM
    error = w_hal_i2s_validate_config(*DeviceConfig::p_page, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return false;
    }

    // Only need to compare the start and end date if the start and end date
    // are valid.
    if ((p_page->start_date.year != INVALID_YEAR) &&
        (p_page->start_date.month != INVALID_MONTH) &&
        (p_page->start_date.day != INVALID_DAY_IN_MONTH) &&
        (p_page->end_date.year != INVALID_YEAR) &&
        (p_page->end_date.month != INVALID_MONTH) &&
        (p_page->end_date.day != INVALID_DAY_IN_MONTH))
    {
        // Compare the start and end date
        if ((p_page->start_date.year > p_page->end_date.year) ||
            ((p_page->start_date.year == p_page->end_date.year) &&
             (p_page->start_date.month > p_page->end_date.month)) ||
            ((p_page->start_date.year == p_page->end_date.year) &&
             (p_page->start_date.month == p_page->end_date.month) &&
             (p_page->start_date.day > p_page->end_date.day)))
        {
            // Start date is greater than end date.
            if (fix)
            {
                // Swap the start and end date.
                date_st temp_date  = p_page->start_date;
                p_page->start_date = p_page->end_date;
                p_page->end_date   = temp_date;
            }
            else
            {
                return false;
            }
        }
    }

#endif

#if OFF_CHIP_MEMORY_CORE != INVALID_CORE_NUM
    error = w_hal_ocm_validate_config(*DeviceConfig::p_page, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return false;
    }
#endif

#if STORAGE_CORE != INVALID_CORE_NUM
    error = w_hal_sd_card_validate_config(*DeviceConfig::p_page, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return false;
    }
#endif

#if COMM_CORE != INVALID_CORE_NUM
    if (DeviceConfig::p_page->sms_send_upload_frequency_seconds <
        MIN_SMS_UPLOAD_FREQUENCY)
    {
        if (fix)
        {
            DeviceConfig::p_page->sms_send_upload_frequency_seconds =
                MIN_SMS_UPLOAD_FREQUENCY;
        }
        else
        {
            return false;
        }
    }

    error = w_hal_gsm_validate_config(*DeviceConfig::p_page, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return false;
    }
#endif

    return true;
}

/**
 * @brief Read and initialize device config.
 *
 * @param device_config device config
 * @param read_only read only from flash, do not write or modify
 */
void DeviceConfig::initialize(DeviceConfig::config_page_st &device_config,
                              bool                          read_only)
{
    p_page                = &device_config;
    max_transmission_mode = DeviceConfig::transmission_mode_t::MODE_4G;

    wHalErrorCode error   = w_hal_flash_init();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    uint32_t memory_size;
    uint32_t page_size;
    bool     valid[2]                          = {false, false};
    uint32_t sequence_number[NUM_CONFIG_PAGES] = {0, 0};
    uint32_t best_page                         = NUM_CONFIG_PAGES;

    error = w_hal_flash_get_parameters(page_size, memory_size);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    if ((memory_size == 0) || (page_size == 0) || (page_size > memory_size))
    {
        // No flash memory.
        initialize_config_defaults();
        return;
    }

    uint32_t config_page_address = memory_size - NUM_CONFIG_PAGES * page_size;

    // The last two flash pages are reserved for two copies of the config
    // page. The config page is a fixed size and is written to flash in its
    // entirety. Read each config page and determine which one is the most
    // recent.

    // Read first config page.
    error                        = w_hal_flash_read(config_page_address,
                             reinterpret_cast<uint8_t *>(p_page),
                             sizeof(config_page_st));
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Check the validity of the config page.
    if (validate())
    {
        valid[0]           = true;
        sequence_number[0] = p_page->sequence_number;
    }

    // Read second config page.
    error = w_hal_flash_read(config_page_address + page_size,
                             reinterpret_cast<uint8_t *>(p_page),
                             sizeof(config_page_st));

    // Check the validity of the config page.
    if (validate())
    {
        valid[1]           = true;
        sequence_number[1] = p_page->sequence_number;
    }

    // Determine which config page is the most recent.
    if (valid[0] && valid[1])
    {
        if (sequence_number[0] >= sequence_number[1])
        {
            // First config page is the most recent.
            best_page = 0;
        }
        else
        {
            // Second config page is the most recent.
            best_page = 1;
        }
    }
    else if (valid[0])
    {
        best_page = 0;
    }
    else if (valid[1])
    {
        best_page = 1;
    }
    else
    {
        // No valid config page.
        initialize_config_defaults();
        return;
    }

    // Read the best config page.
    error = w_hal_flash_read(config_page_address + best_page * page_size,
                             reinterpret_cast<uint8_t *>(p_page),
                             sizeof(config_page_st));
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    if (!read_only)
    {
        // Write the best config page to the other config page region in flash
        if (sequence_number[0] != sequence_number[1])
        {
            // Erase config page.
            error = w_hal_flash_page_erase(config_page_address +
                                           (1 - best_page) * page_size);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            // Write config page.
            error = w_hal_flash_program(config_page_address +
                                            (1 - best_page) * page_size,
                                        reinterpret_cast<uint8_t *>(p_page),
                                        sizeof(config_page_st));
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
        }
    }
}

/**
 * @brief Set config page.
 *
 * @param config_page config page
 */
void DeviceConfig::set_config_page(config_page_st &config_page)
{
    p_page = &config_page;
}

/**
 * @brief Write config page to flash.
 *
 */
void DeviceConfig::write_config_page_to_flash()
{
    uint32_t      memory_size;
    uint32_t      page_size;
    wHalErrorCode error = w_hal_flash_get_parameters(page_size, memory_size);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    if ((memory_size == 0) || (page_size == 0) || (page_size > memory_size))
    {
        // No flash memory.
        return;
    }

    // Update sequence number in config page.
    p_page->sequence_number++;

    uint32_t checksum            = generate_checksum();

    // Update checksum in config page.
    p_page->checksum             = checksum;

    uint32_t config_page_address = memory_size - NUM_CONFIG_PAGES * page_size;

    // Erase config page.
    error                        = w_hal_flash_page_erase(config_page_address);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Write config page.
    error = w_hal_flash_program(config_page_address,
                                reinterpret_cast<uint8_t *>(p_page),
                                sizeof(config_page_st));
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
}
