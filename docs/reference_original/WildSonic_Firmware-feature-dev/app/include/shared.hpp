#ifndef _SHARED_HPP
#define _SHARED_HPP

#include "Messages.hpp"
#include "device_config.hpp"

#include "w_hal_dht.hpp"
#include "w_hal_rtc.hpp"
#include <time.h>

/**
 * @brief Structure containing global variables related to Storage task
 *
 */
struct shared_storage_info_st
{
    uint64_t total_sd_card_capacity; // Bytes
    uint64_t total_bytes_written;    // Bytes
    char
        sd_card_file_format[MAX_FILE_SYSTEM_FORMAT_LENGTH]; // String containing
                                                            // format of the
                                                            // file
    uint32_t seconds_recorded_since_boot;                   // Number of
                                                            // seconds recorded
    bool     sd_card_is_empty; // Indicates when new
                               // or empty sd card is
                               // detected
};

/**
 * @brief Structure containing global variables related to admin task
 *
 */
struct shared_power_mgmt_info_st
{
    int32_t voltage;              // mV
    int32_t current;              // mA
    int32_t remaining_capacity;   // mAH
    int32_t full_charge_capacity; // mAH
    int32_t average_current;      // mA
    int32_t time_to_empty;        // seconds
    int32_t average_power;        // mW
    int32_t temperature;          // Centigrade
    uint8_t low_battery_flag;     // Flag to set for low battery indication
};

/**
 * @brief Structure containing GSM signal quality info
 *
 */
struct shared_gsm_signal_info_st
{
    int32_t rssi;   // received signal strength indication
    int32_t rxqual; // bit error rate
    int32_t rscp;   // received signal code power
    int32_t ecno;   // Ratio of received energy per
                    // PN chip to the total received
                    // power spectral density
    int32_t rsrp;   // reference signal received power
    int32_t rsrq;   // reference signal received quality
};

/**
 * @brief different shutdown stages
 *
 */
enum class shutdown_stage_t : uint8_t
{
    NONE,
    Stage_1, // Transfer buffer from HAL to OCM
    Stage_2, // Transfer buffers from OCM task to Storage task and Comm task
#if OFF_CHIP_MEMORY_CORE != INVALID_CORE_NUM
    Stage_3, // OCM task to sleep
#endif
#if DETECT_CORE != INVALID_CORE_NUM
    Stage_4, // Detect task to sleep
#endif
#if STORAGE_CORE != INVALID_CORE_NUM
    Stage_5, // Storage task to sleep
#endif
#if COMM_CORE != INVALID_CORE_NUM
    Stage_6, // Comm task to sleep
#endif
    Stage_7, // Audio task to sleep
    Stage_8, // Admin task to sleep
};

/**
 * @brief Structure containing all the information of assert
 *
 */
struct shared_assert_info_st
{
    int                    core;             // The variable to store
                                             // core number in
                                             // which assert is occurred
    const char *           comparator;       // compare operator
                                             // expressed as a string
    const char *           expr1;            // first operand of test
                                             // condition expressed as string
    const char *           expr2;            // second operand of test
                                             // condition expressed as string
    unsigned long long int value1;           // value after evaluating
                                             // first expression
    unsigned long long int value2;           // value after evaluating
                                             // second expression
    unsigned long long int number_of_values; // length of values set
    const char *           file;             // file in which assert occurred
    int                    line;             // line number in which
                                             // assert occurred
    bool                   asserted;         // to check whether
                                             // assert occurred or not
    bool                   assert_in_set;    // Flag to check whether
                                             // assert is in set or not
};

/**
 * @brief Structure containing all the environmental information
 *
 */
struct shared_environment_info_st
{
    float temperature_celsius;
    float relative_humidity;
};

/**
 * @brief 8-bit custom system flags
 */
union custom_flags_st
{
    struct
    {
        uint8_t ble_connection_flag : 1;
        uint8_t rtc_mfp_wake_flag   : 1;
        uint8_t mcu_wake_flag       : 1;
        uint8_t flag_3              : 1;
        uint8_t flag_4              : 1;
        uint8_t flag_5              : 1;
        uint8_t flag_6              : 1;
        uint8_t flag_7              : 1;
    };
    uint8_t
        reg; // Allows reading or clearing all 8 flags at once (e.g. reg = 0)
};

/**
 * @brief
 * Structure containing all global variables
 */
struct shared_t
{
    queues_info_t                queues_info;          // contains all the
                                                       // message arrays
                                                       // and queues
    w_hal_rtc_time_st            time;                 // current time
    uint32_t                     time_updated;         // flag to indicate time
                                                       // has been updated in
                                                       // this structure and
                                                       // needs to be updated
                                                       // in the RTC.
    uint32_t                     external_memory_size; // external memory size
    int                          core_init_done[NUM_CORES];
    int                          core_tasks_done[NUM_CORES];
    int                          core_shut_done[NUM_CORES];
    DeviceConfig::config_page_st device_config;          // device configuration
    shared_storage_info_st       storage_info;           // structure containing
                                                         // Storage info
    shared_power_mgmt_info_st    power_mgmt_info;        // structure containing
                                                         // power mgmt info
    shared_gsm_signal_info_st    shared_gsm_signal_info; // Structure containing
                                                         // gsm signal quality
                                                         // information
    shared_assert_info_st shared_assert_info;    // Structure containing assert
                                                 // info
    shared_environment_info_st environment_info; // Structure containing
                                                 // environmental info
    shutdown_stage_t           shutdown_stage;   // enum containing different
                                                 //  shutdown stages
    int                        reboot_flag;      // flag to check
                                                 // for tasks shutdown
    int                        save_config_flag; // flag is set
                                                 // when apply and
                                                 // reboot is entered
                                                 // to flash the
                                                 // device config
    int                        format_sd_card_flag;  // needs to be
                                                     // formatted on reboot
    int                        send_status_now;      // Flag to set when
                                                     // need to send status
    bool                       power_down_flag;      // Flag for scheduled
                                                     // sleep powerdown
    w_hal_rtc_time_st          next_wakeup_time_utc; // structure to hold
                                                     // next wakeup time
                                                     // after deep sleep
                                                     // shutdown
    volatile bool update_config_status_flag; // Flag to set when config status
                                             // file needs to be updated with
                                             // sun-based ranges
    int           ble_pairing_window_request_pending; // Flag to request opening
                                                      // BLE connectable window
    uint32_t      ble_pairing_window_request_seconds; // Requested window length
                                                      // in seconds
    custom_flags_st custom_flags; // Custom flags for user-defined purposes
};

struct shared_buffers_t
{
    uint8_t audio_buffer[BUFFER_SIZE * NUM_BUFFERS];
};

extern shared_t         shared_glbl;
extern shared_buffers_t shared_buffers_glbl;

/**
 * @brief
 * Get the buffer address
 *
 * @param buffer_id buffer id value
 * @return uint8_t* pointer to beginning of buffer
 */
inline uint8_t *        get_buffer_address(buffer_id_t buffer_id)
{
    return shared_buffers_glbl.audio_buffer + (buffer_id << BUFFER_SIZE_LOG2);
}

/**
 * @brief
 * Get the buffer id
 *
 * @param p_buffer buffer address
 * @return buffer_id_t buffer id value
 */
inline buffer_id_t get_buffer_id(uint8_t *p_buffer)
{
    return (p_buffer - shared_buffers_glbl.audio_buffer) >> BUFFER_SIZE_LOG2;
}

#endif // _SHARED_HPP
