#ifndef LIB_CONFIG_DEVICE_CONFIG_HPP
#define LIB_CONFIG_DEVICE_CONFIG_HPP

#include "WAVFile.hpp"
#include "constants.hpp"

#include <stdint.h>
#include <time.h>

/**
 * @brief HTTP content type
 */
enum http_content_type_t : uint8_t
{
    OCTET_STREAM,
    MULTIPART_FORM_DATA,
};

/**
 * @brief Device configuration management class
 */
class DeviceConfig
{
  public:
    /**
     * @brief Folder structure
     */
    enum folder_structure_t : uint8_t
    {
        FLAT = 0,
        DAY,
        HOUR,
    };
    /**
     * @brief Sun recording mode
     */
    enum sun_recording_mode_t : uint8_t
    {
        SUN_MODE_DISABLED = 0,
        SUN_MODE_SUNRISE,
        SUN_MODE_SUNSET,
        SUN_MODE_SUNRISE_AND_SUNSET,
        SUN_MODE_SUNRISE_TO_SUNSET,
        SUN_MODE_SUNSET_TO_SUNRISE,
    };

    /**
     * @brief Transmission mode
     */
    enum transmission_mode_t : uint8_t
    {
        MODE_2G = 0,
        MODE_3G,
        MODE_4G,
    };

    /**
     * @brief Detector select
     */
    enum detector_select_t : uint8_t
    {
        NONE,
        AI,
        SILENCE_AMPLITUDE,
    };

    /**
     * @brief Config flag mask
     */
    union config_flags_t
    {
        uint32_t dw;
        struct
        {
            uint32_t save_only_detected_sounds     : 1;
            uint32_t transmit_only_detected_sounds : 1;
            uint32_t reserved                      : 30;
        } flags;
    };

    /**
     * @brief Date structure
     */
    struct date_st
    {
        uint8_t day;   // day of the month
        uint8_t month; // months since January
        uint8_t year;  // years since 1900
        uint8_t rsvd;  // reserved
    };

    /**
     * @brief Time of day range structure
     */
    struct time_of_day_range_st
    {
        uint32_t start_second; // Any time from 00:00 to 23:59 (in seconds)
        uint32_t end_second;   // Any time from 00:00 to 23:59 (in seconds)
    };

    /**
     * @brief Config page structure
     */
    struct config_page_st
    {
        uint32_t             magic_number;
        uint32_t             version;         // Incremented on
                                              // every config
                                              // layout format
                                              // change
        uint32_t             sequence_number; // Incremented on
                                              // every config
                                              // change

        char                 sms_keyword[MAX_SMS_KEYWORD_LENGTH];
        char                 serial_number[MAX_SERIAL_NUMBER_LENGTH];
        char                 device_name[MAX_DEVICE_NAME_LENGTH];
        char                 device_password[MAX_PASSWORD_LENGTH];
        char                 sms_key[MAX_SMS_KEY_LENGTH];
        char                 server_post_route[MAX_SERVER_POST_ROUTE_LENGTH];
        char                 a2p_phone_number[MAX_PHONE_NUMBER_LENGTH];
        char                 p2a_phone_number[MAX_PHONE_NUMBER_LENGTH];
        char                 server_url[MAX_SERVER_URL_LENGTH];
        uint16_t             server_port;
        http_content_type_t  http_content_type;
        folder_structure_t   folder_structure;

        int16_t              mic_gain_times_ten;
        uint8_t              audio_sampling_width; // 8, 16, 24, 32
        uint8_t              num_audio_channels;   // 1, 2
        uint32_t             sampling_rate;
        uint32_t             recording_length;
        uint32_t             sleep_pause_interval;

        time_t               program_time;     // Time in seconds since
                                               // 00:00:00 UTC, January
                                               // 1, 1970
        int                  time_zone_offset; // Time zone offset in seconds

        date_st              start_date;
        date_st              end_date;
        time_of_day_range_st time_of_day_range[MAX_SCHEDULE_TIME_OF_DAY_RANGES];
        uint8_t              day_of_week_mask; // 0x7F for all days
                                               // of the week

        detector_select_t    detector;
        uint8_t              ai_sensitivity;    // From 0 to 100
        transmission_mode_t  transmission_mode; // 2G, 3G, 4G

        uint32_t             lower_cutoff_frequency;
        uint32_t             upper_cutoff_frequency;

        config_flags_t       flags;

        uint32_t             sms_send_upload_frequency_seconds;
        uint16_t             low_battery_voltage_cutoff;
        uint16_t             num_battery_cells_used;

        // Latitude/longitude in signed degrees scaled by 1e7.
        // Example: 12.3456789 -> 123456789
        int32_t              latitude_e7;
        int32_t              longitude_e7;

        sun_recording_mode_t sun_recording_mode;
        uint8_t              sunrise_sunset_rounding;
        uint8_t              sunrise_interval_before;
        uint8_t              sunset_interval_after;
        uint8_t              sunset_interval_before;
        uint8_t              sunrise_interval_after;

        uint8_t              sun_rsvd[2];
        uint32_t             reserved1[4];

        uint32_t             checksum;
    };

    static constexpr uint8_t   INVALID_YEAR         = 0xFF;
    static constexpr uint8_t   INVALID_MONTH        = 0xFF;
    static constexpr uint8_t   INVALID_DAY_IN_MONTH = 0xFF;
    static constexpr uint32_t  NUM_SECONDS_IN_DAY   = 24 * 60 * 60;
    static constexpr uint32_t  NUM_DAYS_IN_WEEK     = 7;
    static constexpr uint32_t  FULL_WEEK_MASK = (1 << NUM_DAYS_IN_WEEK) - 1;

    // Maximum transmission mode supported by the modem
    static transmission_mode_t max_transmission_mode;

    // Validate HAL configuration
    static bool                param_validate(bool fix);

    // Read and initialize device config.
    static void     initialize(config_page_st &device_config, bool read_only);

    // Set config page
    static void     set_config_page(config_page_st &config_page);

    // Write config page to flash.
    static void     write_config_page_to_flash();

    // Get minimum start time for RTC
    static time_t   get_minimum_start_time();

    // Get the data bit rate
    static uint32_t get_data_bit_rate(config_page_st &config_page);

    // Get the data bit rate
    static uint32_t get_data_bit_rate();

    // Get wav file sample data size.
    static uint64_t get_wav_file_sample_data_size(config_page_st &config_page);

    // Get wav file sample data size.
    static uint64_t get_wav_file_sample_data_size();

    // Get file size.
    static uint64_t get_file_size(DeviceConfig::config_page_st &config_page);

    // Get file size.
    static uint64_t get_file_size();

    // Get max file duration
    static uint64_t
    get_max_file_duration(DeviceConfig::config_page_st &config_page);

    // Get max file duration
    static uint64_t get_max_file_duration();

    // Get start and end date in string format
    static void     get_start_end_date(char *p_string, uint32_t size);

    // Get location in string format
    static void     get_location(char *p_string, uint32_t size);

    // Clear start and end time of day ranges
    static void     reset_time_of_day_ranges();

    // Check if time of day range is valid
    static bool
    is_valid_time_of_day_range(const time_of_day_range_st &time_of_day_range);

    // Sort time of day ranges in order and push invalid ranges to the back
    static void sort_time_of_day_ranges(time_of_day_range_st *p_ranges,
                                        uint32_t              num_ranges);

    // Get start and end time of day in string format
    static void
    get_start_end_time_of_day(uint32_t index, char *p_string, uint32_t size);

    // Get days of week in string format
    static void            get_days_of_week(char *p_string, uint32_t size);

    // Get detector in string format
    static void            get_detector(char *p_string, uint32_t size);

    // Get folder structure configuration in string format
    static void            get_folder_structure(char *p_string, uint32_t size);

    // Get transmission mode configuration in string format
    static void            get_transmission_mode(char *p_string, uint32_t size);

    // Initialize test server setting as device configuration.
    static void            initialize_test_server_setting();

    // Initialize wildsonic server setting as device configuration.
    static void            initialize_wildsonic_server_setting();

    static config_page_st *p_page; // Global device config info

  private:
    DeviceConfig() = default;

    // Generate checksum for config page.
    static uint32_t generate_checksum();

    // Validate config page.
    static bool     validate();

    // Initialize config defaults.
    static void     initialize_config_defaults();
};

static_assert(sizeof(DeviceConfig::config_flags_t) == sizeof(uint32_t),
              "config_flags_t size mismatch");
static_assert(sizeof(DeviceConfig::config_page_st) == 512,
              "config_page_st size mismatch");

/**
 * @brief Get data bit rate
 *
 * @return uint32_t the data bit rate
 */
inline uint32_t
DeviceConfig::get_data_bit_rate(DeviceConfig::config_page_st &config_page)
{
    return (config_page.sampling_rate * config_page.num_audio_channels *
            config_page.audio_sampling_width);
}

/**
 * @brief Get data bit rate
 *
 * @return uint32_t the data bit rate
 */
inline uint32_t DeviceConfig::get_data_bit_rate()
{
    return get_data_bit_rate(*p_page);
}

/**
 * @brief Get wav file sample data size
 *
 * @param config_page device configuration page
 *
 * @return uint64_t number of bytes of wav file sample data
 */
inline uint64_t DeviceConfig::get_wav_file_sample_data_size(
    DeviceConfig::config_page_st &config_page)
{
    const uint64_t data_byte_rate = get_data_bit_rate(config_page) / CHAR_BIT;

    return data_byte_rate * config_page.recording_length;
}

/**
 * @brief Get wav file sample data size
 *
 * @return uint64_t number of bytes of wav file sample data
 */
inline uint64_t DeviceConfig::get_wav_file_sample_data_size()
{
    return get_wav_file_sample_data_size(*p_page);
}

/**
 * @brief Get file size to be written to storage
 *
 * @param config_page device configuration page
 * @return uint64_t number of bytes of the file
 */
inline uint64_t
DeviceConfig::get_file_size(DeviceConfig::config_page_st &config_page)
{
    return get_wav_file_sample_data_size(config_page) + sizeof(wav_header_t);
}

/**
 * @brief Get file size to be written to storage
 *
 * @return uint64_t number of bytes of the file
 */
inline uint64_t DeviceConfig::get_file_size()
{
    return get_file_size(*p_page);
}

/**
 * @brief Get the MAX duration of file can be written to storage
 *
 * @param config_page device configuration page
 * @return uint64_t number of seconds of data
 */
inline uint64_t
DeviceConfig::get_max_file_duration(DeviceConfig::config_page_st &config_page)
{
    return (((0xFFFFFFFFULL - sizeof(wav_header_t)) * CHAR_BIT) /
            get_data_bit_rate(config_page));
}

/**
 * @brief Get the MAX duration of file can be written to storage
 *
 * @return uint64_t number of seconds of data
 */
inline uint64_t DeviceConfig::get_max_file_duration()
{
    return get_max_file_duration(*p_page);
}

#endif // LIB_CONFIG_DEVICE_CONFIG_HPP