#if __CORE__ == CONSOLE_CORE

#include "console.hpp"
#include "constants.hpp"
#include "device_config.hpp"
#include "shared.hpp"
#include "solar_calculator.hpp"
#include "version.hpp"
#if PM_CORE == CONSOLE_CORE
#include "w_hal_pm.hpp"
#endif

#if AUDIO_CORE != INVALID_CORE_NUM
#include "w_hal_i2s.hpp"
#endif
#include "w_hal_rtc.hpp"
#if STORAGE_CORE != INVALID_CORE_NUM
#include "w_hal_sd_card.hpp"
#endif
#include "w_hal_system.hpp"
#include "w_hal_uart.hpp"

#include "gpio.h"
#include "mxc_delay.h"
#include "w_hal_i2c.hpp"
#include "w_hal_smbus.hpp"

#include "re.h"
#include "w_hal_common.hpp"

#include <algorithm>
#include <cmath>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint32_t   MAX_UART_CMD_LEN            = 80;

// Geographic validation limits
static constexpr double LATITUDE_LIMIT              = 90.0;
static constexpr double LONGITUDE_LIMIT             = 180.0;
// Coordinate scaling
static constexpr double ZERO_OFFSET                 = 0.0;
static constexpr double ROUND_OFFSET                = 0.5;
static const uint32_t   BLE_PAIR_WINDOW_MAX_SECONDS = 600;

using date_st                                       = DeviceConfig::date_st;

enum class console_config_cmd_t : uint32_t
{
    HELP, // Help function to understand console configuration
    SERIAL_NUMBER,
    DEVICE_NAME,
    PASSWORD,
    SMS_KEYWORD,
    SMS_KEY,
    A2P_PHONE_NUMBER,
    P2A_PHONE_NUMBER,
    SERVER_URL,
    SERVER_PORT,
    SERVER_POST_ROUTE,
    SERVER_POST_CONTENT_TYPE,
    SERVER_TEST_SETTING,
    SERVER_WILDSONIC_SETTING,
    GSMMODE,            // GSM Transmission mode
    SAMF,               // Sampling frequency
    RECT,               // Recording time
    SLEEP,              // Sleep time
    DATE_RANGE,         // Recording date range
    TIME_OF_DAY,        // Recording time of day
    DAYS_OF_WEEK,       // Recording days of week
    DATETIME,           // System Time
    TIMEZONE,           // Timezone
    BTPS,               // Bits per sample
    NUMCH,              // Number of channels
    GAIN,               // Audio gain
    LOWER_CUTOFF,       // Lower cutoff frequency
    UPPER_CUTOFF,       // Upper cutoff frequency
    FOLDER_STRUCTURE,   // folder structure
    APPLY,              // Apply settings to flash
    REBOOT,             // Reboot the device
    APPLY_REBOOT,       // Apply settings to flash and reboot
    SD_FORMAT,          // Format the SD card
    SMS_FREQ,           // SMS upload frequency
    ASSERT,             // Provides info assert
    BATTERY_STATUS,     // Provides battery status
    SIGNAL_QUALITY,     // Provides signal quality
    STORAGE_FILLED,     // Provides number of bytes filled in SD card
    LOW_BAT,            // Low battery_indicating voltage
    NUM_BAT,            // Setting number of battery cells used
    FULL_CAP,           // Program fuel gauge full charge capacity
    SEND_STATUS,        // Sends status now
    AUDIO,              // Audio recording parameters
    BLEPAIR,            // Open BLE pairing/connect window
    WILDLY,             // Ignore this command
    VERSION,            // Provides version info
    ENVIRONMENT_STATUS, // Provides current environment status like temperature
                        // and humidity
    LOCATION,           // Set/Get latitude and longitude
    SUN,                // Compute Sunrise and Sunset
                        // time based on location and date
    SUN_RECORDING_MODE, // Set sun recording mode
    SUNRISE_INTERVAL_BEFORE, // Set sunrise interval before
    SUNRISE_INTERVAL_AFTER,  // Set sunrise interval after
    SUNSET_INTERVAL_BEFORE,  // Set sunset interval before
    SUNSET_INTERVAL_AFTER,   // Set sunset interval after
    SUNRISE_SUNSET_ROUNDING, // Set sunrise sunset rounding
    DEFAULT,                 // Set default configuration
    I2C,                     // I2C development command
    RTC_RESET,               // Reset RTC time
    TEST_ALARM,              // Test alarm sequence command
    NUM_CMDS                 // Number of Commands
};

/**
 *  Console commands lookup / translation table
 *
 */
const char *console_cmd_lookup_table[] = {
    "HELP",       // HELP
    "SN",         // SERIAL_NUMBER
    "DNAME",      // DEVICE_NAME
    "PW",         // PASSWORD
    "SMSKW",      // SMS_KEYWORD
    "SMSKEY",     // SMS_KEY
    "A2PPHONE",   // A2P_PHONE_NUMBER
    "P2APHONE",   // P2A_PHONE_NUMBER
    "SURL",       // SERVER_URL
    "SPORT",      // SERVER_PORT
    "POSTROUTE",  // SERVER_POST_ROUTE
    "POSTCTNT",   // SERVER_POST_CONTENT_TYPE
    "STEST",      // SERVER_TEST_SETTING
    "SWS",        // SERVER_WILDSONIC_SETTING
    "GSM",        // GSMMODE
    "SAMF",       // SAMF
    "RECT",       // RECT
    "SLEEP",      // SLEEP
    "DATERANGE",  // DATE_RANGE
    "TIMEOFDAY",  // TIME_OF_DAY
    "DAYSOFWEEK", // DAYS_OF_WEEK
    "DATETIME",   // DATETIME
    "TIMEZONE",   // TIMEZONE
    "BTPS",       // BTPS
    "NUMCH",      // NUMCH
    "GAIN",       // GAIN
    "LCUTOFF",    // LOWER_CUTOFF
    "UCUTOFF",    // UPPER_CUTOFF
    "FLDR",       // FOLDER_STRUCTURE
    "APPLY",      // APPLY
    "RBT",        // REBOOT
    "ARBT",       // APPLY_REBOOT
    "SDFMT",      // SD_FORMAT
    "SMSFREQ",    // SMS_FREQ
    "ASSERT",     // ASSERT
    "BATT",       // BATTERY_STATUS
    "SIG",        // SIGNAL_QUALITY
    "STOR",       // STORAGE_FILLED
    "LBAT",       // LOW_BAT
    "NBAT",       // NUM_BAT
    "FULLCAP",    // FULL_CAP
    "SENDSTS",    // SEND_STATUS
    "AUDIO",      // AUDIO
    "BLEPAIR",    // BLEPAIR
    "Wildly",     // WILDLY
    "VERSION",    // VERSION
    "ENVSTS",     // ENVIRONMENT_STATUS
    "LOC",        // LOCATION
    "SUN",        // SUN
    "SUNMODE",    // SUN_RECORDING_MODE
    "SUNRISEBEF", // SUNRISE_INTERVAL_BEFORE
    "SUNRISEAFT", // SUNRISE_INTERVAL_AFTER
    "SUNSETBEF",  // SUNSET_INTERVAL_BEFORE
    "SUNSETAFT",  // SUNSET_INTERVAL_AFTER
    "SUNROUND",   // SUNRISE_SUNSET_ROUNDING
    "DEFAULT",    // DEFAULT
    "I2C",        // I2C
    "RTCRST",     // RTC_RESET
    "TEST_ALARM"  // TEST_ALARM
};

const char *console_cmd_description_table[] = {
    "Display list of commands",                      // HELP
    "Set serial number",                             // SERIAL_NUMBER
    "Set device name for BLE advertising",           // DEVICE_NAME
    "Set device password for server authentication", // PASSWORD
    "Set sms keyword for manufacturing",             // SMS_KEYWORD
    "Set sms key for manufacturing",                 // SMS_KEY
    "Set server phone number to receive "
    "the messages(format +XXXXXXXXXXXX or XXXXXX)", // A2P_PHONE_NUMBER
    "Set server phone number to send "
    "the messages(format +XXXXXXXXXXXX or XXXXXX)", // P2A_PHONE_NUMBER
    "Set server URL (without http:// or https://)", // SERVER_URL
    "Set server port",                              // SERVER_PORT
    "Set server post request api route",            // SERVER_POST_ROUTE
    "Set server post request content-type",         // SERVER_POST_CONTENT_TYPE
    "Quick command to set up url, "
    "port, api route for test server", // SERVER_TEST_SETTING
    "Quick command to set up url, "
    "port, api route for demo server",             // SERVER_WILDSONIC_SETTING
    "Set GSM transmission mode (e.g. 2G, 3G, 4G)", // GSMMODE
    "Set sampling frequency in Hz",                // SAMF
    "Set recording time in seconds",               // RECT
    "Set sleep time in seconds",                   // SLEEP
    "Set recording date range (format YYYY/MM/DD-YYYY/MM/DD)", // DATE_RANGE
    "Set recording time of day (format [index] HH:MM:SS-HH:MM:SS)", // TIME_OF_DAY
    "Set recording days of the week (\"MTWRFSU\") within quotation "
    "mark",                                               // DAYS_OF_WEEK
    "Set sys time in fmt YYYY-MM-DD HH:MM:SS [[+-]ZZZZ]", // DATETIME
    "Set the time zone in format [[+-]ZZZZ]",             // TIMEZONE
    "Set sample bit width",                               // BTPS
    "Set number of channels",                             // NUMCH
    "Set audio gain in dB",                               // GAIN
    "Set lower cutoff frequency in Hz",                   // LOWER_CUTOFF
    "Set upper cutoff frequency in Hz",                   // UPPER_CUTOFF
    "Enable folder structure: hourly, daily, or flat",    // FOLDER_STRUCTURE
    "Apply settings to flash",                            // APPLY
    "Reboot the device",                                  // REBOOT
    "Apply settings to flash and reboot",                 // APPLY_REBOOT
    "SD card format and reboot",                          // SD_FORMAT
    "SMS upload frequency in seconds",                    // SMS_FREQ
    "Provides the information of assert",                 // ASSERT
    "Provides battery status",                            // BATTERY_STATUS
    "Provides signal quality",                            // SIGNAL_QUALITY
    "Provides number of bytes filled in SD card",         // STORAGE_FILLED
    "Low battery indicating voltage (in mV)",             // LOW_BAT
    "Setting number of battery cells used",               // NUM_BAT
    "Program fuel gauge full charge capacity in mAh",     // FULL_CAP
    "Sends Status to the server now",                     // SEND_STATUS
    "Audio recording parameters",                         // AUDIO
    "Open BLE connect window in seconds (default 30)",    // BLEPAIR
    "Ignore this command",                                // WILDLY
    "Provides the version information",                   // VERSION
    "Provides current environment status like temperature and humidity", // ENVIRONMENT_STATUS
    "Set/Get location (format \"LOC <latitude> <longitude>\")", // LOCATION
    "View Sunrise/Sunset times for a specific date",            // SUN
    "Set sun recording mode (format SUNMODE <mode>)", // SUN_RECORDING_MODE
    "Set sunrise interval before (format SUNRISEBEF <minutes>)", // SUNRISE_INTERVAL_BEFORE
    "Set sunrise interval after (format SUNRISEAFT <minutes>)", // SUNRISE_INTERVAL_AFTER
    "Set sunset interval before (format SUNSETBEF <minutes>)", // SUNSET_INTERVAL_BEFORE
    "Set sunset interval after (format SUNSETAFT <minutes>)", // SUNSET_INTERVAL_AFTER
    "Set sunrise sunset rounding (format SUNROUND <minutes>)", // SUNRISE_SUNSET_ROUNDING
    "Sets Default configurations",                             // DEFAULT
    "I2C debugging scan, read/write command",                  // I2C
    "Reset RTC time to 2025-01-01 00:00:00 UTC",               // RTC_RESET
    "Configure custom/default alarm sequence (GPIO 1.7 + RTC)" // TEST_ALARM
};

// Ensure the length of the string table matches the enum
static_assert((sizeof(console_cmd_lookup_table) /
               sizeof(console_cmd_lookup_table[0])) ==
                  static_cast<int>(console_config_cmd_t::NUM_CMDS),
              "console_cmd_lookup_table size mismatch");

// Ensure the length of the string table matches the enum
static_assert((sizeof(console_cmd_description_table) /
               sizeof(console_cmd_description_table[0])) ==
                  static_cast<int>(console_config_cmd_t::NUM_CMDS),
              "console_cmd_description_table size mismatch");

/**
 * @brief Func to handle incoming character
 *
 * @param c is the character returned from terminal
 */
static void     new_character_handler(char c);

/**
 * @brief Variable to form console command from character input
 *
 */
static char     uart_console_cmd[MAX_UART_CMD_LEN] = "";

/**
 * @brief Counter for tracking command length
 *
 */
static uint16_t uart_char_cnt                      = 0;

/**
 * @brief Variable to track command length
 */
static uint16_t uart_cmd_len                       = 0;

/**
 * @brief Func to lookup command table
 *
 * @param cmd buffer containing characters from terminal
 * @param length length of the command passed
 * @param arg_offset is the offset to the arguments in the command
 *
 * @return console_config_cmd_t command ID
 */
static console_config_cmd_t
console_cmd_lookup_func(const char *cmd, uint32_t length, uint32_t &arg_offset)
{
    for (int cmd_id = 0;
         cmd_id < static_cast<int>(console_config_cmd_t::NUM_CMDS);
         cmd_id++)
    {
        size_t lenpre             = strlen(console_cmd_lookup_table[cmd_id]);
        // Check if the lookup table entry is a substring of the command
        // and the next character is a space or the end of the string
        bool   equal_length       = (length == lenpre);
        bool   next_char_is_space = ((length > lenpre) && (cmd[lenpre] == ' '));

        if ((equal_length || next_char_is_space) &&
            (memcmp(console_cmd_lookup_table[cmd_id], cmd, lenpre) == 0))
        {
            if (next_char_is_space)
            {
                arg_offset = lenpre + 1;

                // Skip any additional spaces
                while ((cmd[arg_offset] == ' ') && (arg_offset < length))
                {
                    arg_offset++;
                }
            }
            else
            {
                arg_offset = lenpre;
            }

            return static_cast<console_config_cmd_t>(cmd_id);
        }
    }
    printf("\nUnknown or BAD command : %s", uart_console_cmd);
    return console_config_cmd_t::NUM_CMDS;
}

/**
 * @brief Reset procedure for console commands: clear counter and command buffer
 *
 */
static void reset_console_cmd()
{
    uart_char_cnt = 0;
    uart_cmd_len  = 0;
    memset(uart_console_cmd, 0, (sizeof(char) * MAX_UART_CMD_LEN));
}

/**
 * @brief Initialization for console commands
 */
void console_init()
{
    reset_console_cmd();
    w_hal_uart_console_init(new_character_handler);
}

/**
 * @brief HELP command console display
 *
 */
static void display_console_commands()
{
    printf("\nList of commands\n");
    for (int cmd_id = 0;
         cmd_id < static_cast<int>(console_config_cmd_t::NUM_CMDS);
         cmd_id++)
    {
        // Print the list of commands available, TBD: print the command usage
        printf(" %s \n", console_cmd_lookup_table[cmd_id]);
    }
    printf("\n");
}

/**
 * @brief HELP command console display
 *
 */
static void display_console_help()
{
    printf("\nList of commands\n");
    for (int cmd_id = 0;
         cmd_id < static_cast<int>(console_config_cmd_t::NUM_CMDS);
         cmd_id++)
    {
        // Print the list of commands available
        printf(" %s : %s\n",
               console_cmd_lookup_table[cmd_id],
               console_cmd_description_table[cmd_id]);
    }
    printf("\n");
}

/**
 * @brief Set the value object
 *
 * @tparam T int, float, double, etc
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 * @param value value to set
 */
template <typename T>
static void set_value(const char *p_args, const size_t args_length, T &value)
{
    int  match_length = 0;
    int  match_index;
    bool updated = false;

    match_index  = re_match("\\d+", p_args, args_length, &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        // Parse the value from the string
        T new_value = static_cast<T>(atoi(p_args));

        // Keep the old value in case the new value is invalid
        T old_value = value;
        value       = new_value;

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid value %d\n", new_value);
            value = old_value;
        }
        else
        {
            updated = true;
        }
    }

    printf("\nValue is set to %d\n", value);

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Set unsigned value - rejects negative inputs
 * identical to set_value but rejects negative numbers
 *
 * @tparam T int, float, double, etc
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 * @param value value to set
 */
template <typename T>
static void
set_unsigned_value(const char *p_args, const size_t args_length, T &value)
{
    if ((args_length > 0) && (p_args[0] == '-'))
    {
        printf("\nInvalid value: negative values are not allowed\n");
        printf("\nValue is set to %d\n", value);
        return;
    }
    set_value(p_args, args_length, value);
}

/**
 * @brief Set the string value object
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 * @param p_string value to set
 * @param string_length length of the string container
 */
static void set_value(const char * p_args,
                      const size_t args_length,
                      char *       p_string,
                      const size_t string_length,
                      const char * p_match_regex)
{
    int  match_length = 0;
    int  match_index;
    bool updated = false;

    match_index  = re_match(p_match_regex, p_args, args_length, &match_length);

    if ((match_index == 0) &&
        (static_cast<size_t>(match_length) == args_length))
    {
        if (static_cast<size_t>(match_length) + 1 > string_length)
        {
            printf("\nInvalid value %s\n", p_args);
            return;
        }

        // Parse the value from the string
        strncpy(p_string,
                &p_args[match_index],
                std::min(string_length - 1, static_cast<size_t>(match_length)));
        p_string[match_length] = '\0';

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid value %s\n", p_string);
            p_string[0] = '\0';
        }
        else
        {
            updated = true;
        }
    }

    printf("\nValue is set to %s\n", p_string);

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Serial number console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_serial_number(const char *p_args, const size_t args_length)
{
    if (args_length == 0)
    {
        printf("\nValue is set to %s\n", DeviceConfig::p_page->serial_number);
        return;
    }

    if (args_length > MAX_SERIAL_NUMBER_LENGTH)
    {
        printf("\nInvalid value %s\n", p_args);
        printf("\nValue is set to %s\n", DeviceConfig::p_page->serial_number);
        return;
    }

    for (size_t i = 0; i < args_length; i++)
    {
        if (!isalnum(static_cast<unsigned char>(p_args[i])))
        {
            printf("\nInvalid value %s\n", p_args);
            printf("\nValue is set to %s\n",
                   DeviceConfig::p_page->serial_number);
            return;
        }
    }

    set_value(p_args,
              args_length,
              DeviceConfig::p_page->serial_number,
              sizeof(DeviceConfig::p_page->serial_number),
              "[0-9]+");
}

/**@brief Set the Device Name for BLE advertising
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_device_name(const char *p_args, const size_t args_length)
{
    if (args_length == 0)
    {
        printf("\nValue is set to %s\n", DeviceConfig::p_page->device_name);
        return;
    }

    if (args_length > MAX_DEVICE_NAME_LENGTH)
    {
        printf("\nInvalid value %s\n", p_args);
        printf("\nValue is set to %s\n", DeviceConfig::p_page->device_name);
        return;
    }

    for (size_t i = 0; i < args_length; i++)
    {
        if (!isalnum(static_cast<unsigned char>(p_args[i])))
        {
            printf("\nInvalid value %s\n", p_args);
            printf("\nValue is set to %s\n", DeviceConfig::p_page->device_name);
            return;
        }
    }

    set_value(p_args,
              args_length,
              DeviceConfig::p_page->device_name,
              sizeof(DeviceConfig::p_page->device_name),
              "[A-Za-z]+");
}

/**
 * @brief Set the device password object
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_device_password(const char *p_args, const size_t args_length)
{
    // regex for device password
    const char *p_regex = ".+";

    set_value(p_args,
              args_length,
              DeviceConfig::p_page->device_password,
              sizeof(DeviceConfig::p_page->device_password),
              p_regex);
}

/**
 * @brief Set the sms keyword object
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_sms_keyword(const char *p_args, const size_t args_length)
{
    // regex for basic character set in SMS, including special characters
    const char *p_regex = "[a-zA-Z0-9!@#$%&*();:./<>?]+";

    set_value(p_args,
              args_length,
              DeviceConfig::p_page->sms_keyword,
              sizeof(DeviceConfig::p_page->sms_keyword),
              p_regex);
}

/**
 * @brief Set the sms key object
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_sms_key(const char *p_args, const size_t args_length)
{
    // regex for basic character set in SMS, including special characters
    const char *p_regex = "[a-zA-Z0-9!@#$%&*();:./<>?]+";

    set_value(p_args,
              args_length,
              DeviceConfig::p_page->sms_key,
              sizeof(DeviceConfig::p_page->sms_key),
              p_regex);
}

#if COMM_CORE != INVALID_CORE_NUM

/**
 * @brief Server phone number console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 * @param p_phone_number phone number to set
 * @param phone_number_size size of the phone number container
 */
static void set_phone_number(const char * p_args,
                             const size_t args_length,
                             char *       p_phone_number,
                             const size_t phone_number_size)
{
    int         match_length = 0;
    int         match_index;
    const char *p_pattern = "\\+\\d+";

    match_index = re_match(p_pattern, p_args, args_length, &match_length);

    if ((match_index != 0) ||
        (static_cast<size_t>(match_length) != args_length))
    {
        // Long code format check failed, try short code format
        if ((args_length >= MIN_SHORT_CODE_PHONE_NUMBER_LENGTH) &&
            (args_length <= MAX_SHORT_CODE_PHONE_NUMBER_LENGTH))
        {
            p_pattern = "[A-Za-z0-9]+";
        }
    }

    set_value(p_args,
              args_length,
              p_phone_number,
              phone_number_size,
              p_pattern);
}

/**
 * @brief A2P Server phone number console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_a2p_phone_number(const char *p_args, const size_t args_length)
{
    set_phone_number(p_args,
                     args_length,
                     DeviceConfig::p_page->a2p_phone_number,
                     sizeof(DeviceConfig::p_page->a2p_phone_number));
}

/**
 * @brief P2A Server phone number console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_p2a_phone_number(const char *p_args, const size_t args_length)
{
    set_phone_number(p_args,
                     args_length,
                     DeviceConfig::p_page->p2a_phone_number,
                     sizeof(DeviceConfig::p_page->p2a_phone_number));
}

/**
 * @brief Server URL console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_server_url(const char *p_args, const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->server_url,
              sizeof(DeviceConfig::p_page->server_url),
              "[a-zA-Z0-9_\\.\\-]+");
}

/**
 * @brief Server port console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_server_port(const char *p_args, const size_t args_length)
{
    set_value(p_args, args_length, DeviceConfig::p_page->server_port);
}

/**
 * @brief Server post route console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_server_post_route(const char *p_args, const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->server_post_route,
              sizeof(DeviceConfig::p_page->server_post_route),
              "\\/[a-zA-Z0-9_\\.\\-\\/]+");
}

/**
 * @brief Server post route content type console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_post_route_content_type(const char * p_args,
                                        const size_t args_length)

{
    const char *p_octet_stream = "application/octet-stream";
    const char *p_form_data    = "multipart/form-data";
    bool        update         = false;

    // Check if the value if octet-stream or form-data and set the appropriate
    // enum in device config
    if ((args_length == strlen(p_octet_stream)) &&
        strncmp(p_args, p_octet_stream, args_length) == 0)
    {
        DeviceConfig::p_page->http_content_type =
            http_content_type_t::OCTET_STREAM;
        update = true;
    }
    else if ((args_length == strlen(p_form_data)) &&
             strncmp(p_args, p_form_data, args_length) == 0)
    {
        DeviceConfig::p_page->http_content_type =
            http_content_type_t::MULTIPART_FORM_DATA;
        update = true;
    }

    if (update)
    {
        printf("\nValue is updated to %s\n", p_args);
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
    else
    {
        if (args_length > 0)
        {
            printf("\nInvalid value %s\n", p_args);
        }
        else
        {
            switch (DeviceConfig::p_page->http_content_type)
            {
                case http_content_type_t::OCTET_STREAM:
                {
                    printf("\nValue is set to %s\n", p_octet_stream);
                    break;
                }
                case http_content_type_t::MULTIPART_FORM_DATA:
                {
                    printf("\nValue is set to %s\n", p_form_data);
                    break;
                }
                default:
                {
                    printf("\nValue is set to unknown\n");
                    break;
                }
            }
        }
    }
}

/**
 * @brief Use the test server for uploading files
 */
static void set_server_test_settings()
{
    DeviceConfig::initialize_test_server_setting();

    printf("\nTest server is set up. Send APPLY command and reboot for changes "
           "to take effect.\n");
}

/**
 * @brief Use the wildsonic server for uploading files
 *
 */
static void set_server_wildsonic_settings()
{
    DeviceConfig::initialize_wildsonic_server_setting();

    printf("\nWildsonic server is set up. Send APPLY command and reboot for "
           "changes to take effect.\n");
}

/**
 * @brief Set the transmission mode object
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_gsm_transmission_mode(const char * p_args,
                                      const size_t args_length)
{
    const char *                      p_two_g   = "2G";
    const char *                      p_three_g = "3G";
    const char *                      p_four_g  = "4G";
    bool                              update    = false;

    char                              gsm_transmission_mode[10];
    DeviceConfig::transmission_mode_t old_value;

    // Temporarily storing old value for reference
    old_value = DeviceConfig::p_page->transmission_mode;

    // Check if the value if octet-stream or form-data and set the appropriate
    // enum in device config
    if ((args_length == strlen(p_two_g)) &&
        strncmp(p_args, p_two_g, args_length) == 0)
    {
        DeviceConfig::p_page->transmission_mode =
            DeviceConfig::transmission_mode_t::MODE_2G;
        update = true;
    }
    else if ((args_length == strlen(p_three_g)) &&
             strncmp(p_args, p_three_g, args_length) == 0)
    {
        DeviceConfig::p_page->transmission_mode =
            DeviceConfig::transmission_mode_t::MODE_3G;
        update = true;
    }
    else if ((args_length == strlen(p_four_g)) &&
             strncmp(p_args, p_four_g, args_length) == 0)
    {
        DeviceConfig::p_page->transmission_mode =
            DeviceConfig::transmission_mode_t::MODE_4G;
        update = true;
    }

    // Validate the GSM mode
    if (!DeviceConfig::param_validate(true))
    {
        printf("\nInvalid value %s\n", p_args);
        DeviceConfig::p_page->transmission_mode = old_value;
    }

    if (update)
    {
        DeviceConfig::get_transmission_mode(gsm_transmission_mode,
                                            sizeof(gsm_transmission_mode));
        printf("\nValue is updated to %s\n", gsm_transmission_mode);
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
    else
    {
        if (args_length > 0)
        {
            printf("\nInvalid value %s\n", p_args);
        }
        else
        {
            DeviceConfig::get_transmission_mode(gsm_transmission_mode,
                                                sizeof(gsm_transmission_mode));
            printf("\nValue is set to %s\n", gsm_transmission_mode);
        }
    }
}

/**
 * @brief Set the sms upload frequency object
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_sms_upload_frequency(const char * p_args,
                                     const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->sms_send_upload_frequency_seconds);
}

/**
 * @brief Get the gsm signal quality object
 *
 */
static void get_gsm_signal_quality()
{
    printf(" RSSI index: %d, BER: %d, RSCP: %d, ECNO :%d, RSRQ: %d, RSRP: "
           "%d\n",
           shared_glbl.shared_gsm_signal_info.rssi,
           shared_glbl.shared_gsm_signal_info.rxqual,
           shared_glbl.shared_gsm_signal_info.rscp,
           shared_glbl.shared_gsm_signal_info.ecno,
           shared_glbl.shared_gsm_signal_info.rsrq,
           shared_glbl.shared_gsm_signal_info.rsrp);
}

/**
 * @brief Send the status now
 */
static void send_status_now()
{
    if (shared_glbl.send_status_now == false)
    {
        shared_glbl.send_status_now = true;
        printf("Now the device will send status to the server\n");
    }
}

#endif // COMM_CORE != INVALID_CORE_NUM

#if AUDIO_CORE != INVALID_CORE_NUM

/**
 * @brief Sampling frequency console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_sampling_frequency(const char *p_args, const size_t args_length)
{
    uint32_t old = DeviceConfig::p_page->sampling_rate;

    set_value(p_args, args_length, DeviceConfig::p_page->sampling_rate);

    if ((old != DeviceConfig::p_page->sampling_rate) &&
        (DeviceConfig::p_page->upper_cutoff_frequency == (old / 2)))
    {
        DeviceConfig::p_page->upper_cutoff_frequency =
            DeviceConfig::p_page->sampling_rate / 2;
        printf("\nUpper cutoff frequency adjusted to %d\n",
               DeviceConfig::p_page->upper_cutoff_frequency);
    }
}

/**
 * @brief Sample width console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_sample_width(const char *p_args, const size_t args_length)
{
    set_value(p_args, args_length, DeviceConfig::p_page->audio_sampling_width);
}

/**
 * @brief Number of audio channels console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_num_audio_channels(const char *p_args, const size_t args_length)
{
    set_value(p_args, args_length, DeviceConfig::p_page->num_audio_channels);
}

/**
 * @brief Audio parameters console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_audio_parameters(const char *p_args, const size_t args_length)
{
    int  match_length = 0;
    int  match_index;
    bool updated = false;

    match_index  = re_match("\\d\\d\\d\\d\\d\\d\\d\\d\\d\\d",
                           p_args,
                           args_length,
                           &match_length);

    if ((match_index == 0) &&
        (static_cast<size_t>(match_length) == args_length))
    {
        uint32_t  old_numch, old_samf, old_btps;
        uint32_t  new_numch, new_samf, new_btps;

        const int numch_length = 1;
        const int samf_length  = 7;
        const int btps_length  = 2;

        // Temporary variable to extract specific bits from a string
        char      numch[numch_length + 1];
        char      samf[samf_length + 1];
        char      btps[btps_length + 1];

        strncpy(numch, &p_args[0], numch_length);
        strncpy(samf, &p_args[numch_length], samf_length);
        strncpy(btps, &p_args[numch_length + samf_length], btps_length);

        numch[numch_length] = '\0';
        samf[samf_length]   = '\0';
        btps[btps_length]   = '\0';

        // Parse the value from the string
        new_numch           = static_cast<uint32_t>(atoi(numch));
        new_samf            = static_cast<uint32_t>(atoi(samf));
        new_btps            = static_cast<uint32_t>(atoi(btps));

        // Keep the old value in case the new value is invalid
        old_numch           = DeviceConfig::p_page->num_audio_channels;
        old_samf            = DeviceConfig::p_page->sampling_rate;
        old_btps            = DeviceConfig::p_page->audio_sampling_width;

        // Set new set of values
        DeviceConfig::p_page->num_audio_channels   = new_numch;
        DeviceConfig::p_page->sampling_rate        = new_samf;
        DeviceConfig::p_page->audio_sampling_width = new_btps;

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid values\n");
            DeviceConfig::p_page->num_audio_channels   = old_numch;
            DeviceConfig::p_page->sampling_rate        = old_samf;
            DeviceConfig::p_page->audio_sampling_width = old_btps;
        }
        else
        {
            updated = true;
            if ((old_samf != DeviceConfig::p_page->sampling_rate) &&
                (DeviceConfig::p_page->upper_cutoff_frequency ==
                 (old_samf / 2)))
            {
                DeviceConfig::p_page->upper_cutoff_frequency =
                    DeviceConfig::p_page->sampling_rate / 2;
                printf("\nUpper cutoff frequency adjusted to %d\n",
                       DeviceConfig::p_page->upper_cutoff_frequency);
            }
        }
    }

    printf("\nNumber of channels set to %d\n",
           DeviceConfig::p_page->num_audio_channels);
    printf("Sampling frequency set to %d\n",
           DeviceConfig::p_page->sampling_rate);
    printf("Bits per sample set to %d\n",
           DeviceConfig::p_page->audio_sampling_width);

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Audio gain console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_audio_gain(const char *p_args, const size_t args_length)
{
    int  match_length = 0;
    int  match_index;
    int  audio_gain_times_ten = 0;
    bool updated              = false;

    match_index =
        re_match("[N-]?\\d+\\.?\\d?", p_args, args_length, &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        if (p_args[0] == 'N')
        {
            audio_gain_times_ten = -10 * atoi(&p_args[1]);
        }
        else
        {
            audio_gain_times_ten = 10 * atoi(p_args);
        }

        // See if there are any digits after the decimal point
        match_length = 0;
        match_index  = re_match("\\.\\d", p_args, args_length, &match_length);

        if (match_index != RE_NO_MATCH_INDEX)
        {
            int tenth = p_args[match_index + 1] - '0';

            // convert char to int
            if (audio_gain_times_ten > 0)
            {
                audio_gain_times_ten += tenth;
            }
            else
            {
                audio_gain_times_ten -= tenth;
            }
        }

        int old_value = DeviceConfig::p_page->mic_gain_times_ten;
        DeviceConfig::p_page->mic_gain_times_ten = audio_gain_times_ten;

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid audio gain 0.1*%d\n", audio_gain_times_ten);
            audio_gain_times_ten                     = 0;
            DeviceConfig::p_page->mic_gain_times_ten = old_value;
        }
        else
        {
            updated = true;
        }
    }

    printf("\nAudio gain set to 0.1*%d\n",
           DeviceConfig::p_page->mic_gain_times_ten);

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Set the lower cutoff frequency
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_lower_cutoff_frequency(const char * p_args,
                                       const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->lower_cutoff_frequency);
}

/**
 * @brief Set the upper cutoff frequency
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_upper_cutoff_frequency(const char * p_args,
                                       const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->upper_cutoff_frequency);
}

#endif // AUDIO_CORE != INVALID_CORE_NUM

#if STORAGE_CORE != INVALID_CORE_NUM
/**
 * @brief Recording time length console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_recording_length(const char *p_args, const size_t args_length)
{
    set_value(p_args, args_length, DeviceConfig::p_page->recording_length);
}

/**
 * @brief Recording sleep time console command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_recording_sleep_time(const char * p_args,
                                     const size_t args_length)
{
    set_value(p_args, args_length, DeviceConfig::p_page->sleep_pause_interval);
}

/**
 * @brief Set the folder structure option
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_folder_structure(const char *p_args, const size_t args_length)
{
    const char *p_flat   = "flat";
    const char *p_daily  = "daily";
    const char *p_hourly = "hourly";

    bool        update   = false;

    if ((args_length == strlen(p_flat)) &&
        strncmp(p_args, p_flat, args_length) == 0)
    {
        DeviceConfig::p_page->folder_structure =
            DeviceConfig::folder_structure_t::FLAT;
        update = true;
    }
    else if ((args_length == strlen(p_daily)) &&
             strncmp(p_args, p_daily, args_length) == 0)
    {
        DeviceConfig::p_page->folder_structure =
            DeviceConfig::folder_structure_t::DAY;
        update = true;
    }
    else if ((args_length == strlen(p_hourly)) &&
             strncmp(p_args, p_hourly, args_length) == 0)
    {
        DeviceConfig::p_page->folder_structure =
            DeviceConfig::folder_structure_t::HOUR;
        update = true;
    }

    if (update)
    {
        printf("\nValue is updated to %s\n", p_args);
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
    else
    {
        if (args_length > 0)
        {
            printf("\nInvalid value %s\n", p_args);
        }
        else
        {
            char folder_structure[10];
            DeviceConfig::get_folder_structure(folder_structure,
                                               sizeof(folder_structure));

            printf("\nValue is set to %s\n", folder_structure);
        }
    }
}

#endif // STORAGE_CORE != INVALID_CORE_NUM

/**
 * @brief Set the low battery voltage cutoff
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_low_battery_voltage_cutoff(const char * p_args,
                                           const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->low_battery_voltage_cutoff);
}

/**
 * @brief Set the number of battery cells used
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_number_of_cells_used(const char * p_args,
                                     const size_t args_length)
{
    set_value(p_args,
              args_length,
              DeviceConfig::p_page->num_battery_cells_used);
}

#if PM_CORE != INVALID_CORE_NUM
/**
 * @brief Program the fuel gauge full charge capacity.
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_fuel_gauge_full_charge_capacity(const char * p_args,
                                                const size_t args_length)
{
    int match_length = 0;
    int match_index  = re_match("\\d+", p_args, args_length, &match_length);
    const uint32_t MAXIMUM_PROGRAMMABLE_FULL_CHARGE_CAPACITY_MAH = 32767;

    if ((match_index != 0) ||
        (static_cast<size_t>(match_length) != args_length))
    {
        printf("\nUsage: FULLCAP <mAh>\n");
        return;
    }

    uint32_t full_charge_capacity_mah = static_cast<uint32_t>(atoi(p_args));
    if ((full_charge_capacity_mah == 0) ||
        (full_charge_capacity_mah >
         MAXIMUM_PROGRAMMABLE_FULL_CHARGE_CAPACITY_MAH))
    {
        printf("\nInvalid full charge capacity %s\n", p_args);
        return;
    }

    wHalErrorCode error = w_hal_pm_set_fuel_gauge_full_charge_capacity(
        static_cast<uint16_t>(full_charge_capacity_mah));
    if (error == wHalErrorCode::NO_ERROR)
    {
        printf("\nFuel gauge full charge capacity set to %u mAh\n",
               static_cast<unsigned int>(full_charge_capacity_mah));
    }
    else
    {
        printf("\nFuel gauge full charge capacity programming failed: %d\n",
               static_cast<int>(error));
    }
}
#endif // PM_CORE != INVALID_CORE_NUM

/**
 * @brief Convert a string of length n to an integer
 *
 * @param p_number string buffer containing the number
 * @param n length of the number
 * @return int the number
 */
static int natoi(const char *p_number, const size_t n)
{
    char digit_buffer[5];

    if (n > 4)
    {
        return 0;
    }

    strncpy(digit_buffer, p_number, n);
    digit_buffer[n] = '\0';

    return atoi(digit_buffer);
}

/**
 * @brief Find the index of the next digit in the string
 *
 * @param p_string the string buffer
 * @param n length of the string
 * @return int the index of the next available digit, otherwise n
 */
static uint32_t find_index_of_next_digit(const char *p_string, size_t n)
{
    uint32_t index = 0;

    while ((p_string[index] < '0' || p_string[index] > '9') && index < n)
    {
        index++;
    }

    return index;
}

/**
 * @brief Find the number of string characters in the number
 *
 * @param p_string the string buffer
 * @param n length of the string
 * @return uint32_t the number of string characters in the number
 */
static uint32_t find_length_of_number(const char *p_string, size_t n)
{
    uint32_t index = 0;

    while ((index < n) && (p_string[index] >= '0') && (p_string[index] <= '9'))
    {
        index++;
    }

    return index;
}

/**
 * @brief Extract time from datetime string
 *
 * @param p_datetime_string string buffer containing the datetime string
 * @param datetime_string_length length of the datetime string
 * @param time[out] time extracted from the datetime string
 * @param time_zone_offset[out] time zone offset extracted from the datetime
 * @return true if the time was extracted successfully
 * @return false if the time was not extracted successfully
 */
static bool extract_time_from_datetime_string(const char *p_datetime_string,
                                              uint32_t datetime_string_length,
                                              time_t & utc_time,
                                              int &    time_zone_offset)
{
    const int year_length     = 4;
    const int month_length    = 2;
    const int day_length      = 2;
    const int hour_length     = 2;
    const int minute_length   = 2;
    const int second_length   = 2;

    bool      valid_time_zone = false;

    // Handle the case where the time is one the formats:
    // YYYY-MM-DD HH:MM:SS [[+-]ZZZZ]
    // YYYY/MM/DD HH:MM:SS [[+-]ZZZZ]
    // YYYYMMDD HH:MM:SS [[+-]ZZZZ]
    // YYYYMMDD HHMMSS [[+-]ZZZZ]
    int       match_length    = 0;
    int       match_index = re_match("\\d\\d\\d\\d[\\-/]?\\d\\d[\\-/]?\\d\\d "
                               "\\d\\d:?\\d\\d:?\\d\\d",
                               p_datetime_string,
                               static_cast<int>(datetime_string_length),
                               &match_length);

    if (match_index == 0)
    {
        // Parse the value from the string
        uint32_t args_buffer_index = 0;

        tm       tm_struct;
        memset(&tm_struct, 0, sizeof(tm_struct));

        tm_struct.tm_year =
            natoi(&p_datetime_string[args_buffer_index], year_length) -
            static_cast<int>(TIME_BASE_YEAR);
        args_buffer_index += year_length;
        args_buffer_index += find_index_of_next_digit(
            &p_datetime_string[args_buffer_index],
            datetime_string_length - args_buffer_index);

        tm_struct.tm_mon =
            natoi(&p_datetime_string[args_buffer_index], month_length) -
            static_cast<int>(TIME_BASE_MONTH);
        args_buffer_index += month_length;
        args_buffer_index += find_index_of_next_digit(
            &p_datetime_string[args_buffer_index],
            datetime_string_length - args_buffer_index);

        tm_struct.tm_mday =
            natoi(&p_datetime_string[args_buffer_index], day_length);
        args_buffer_index += day_length;
        args_buffer_index += find_index_of_next_digit(
            &p_datetime_string[args_buffer_index],
            datetime_string_length - args_buffer_index);

        tm_struct.tm_hour =
            natoi(&p_datetime_string[args_buffer_index], hour_length);
        args_buffer_index += hour_length;
        args_buffer_index += find_index_of_next_digit(
            &p_datetime_string[args_buffer_index],
            datetime_string_length - args_buffer_index);

        tm_struct.tm_min =
            natoi(&p_datetime_string[args_buffer_index], minute_length);
        args_buffer_index += minute_length;
        args_buffer_index += find_index_of_next_digit(
            &p_datetime_string[args_buffer_index],
            datetime_string_length - args_buffer_index);

        tm_struct.tm_sec =
            natoi(&p_datetime_string[args_buffer_index], second_length);
        args_buffer_index += second_length;

        if (static_cast<size_t>(match_index + match_length) ==
            datetime_string_length)
        {
            // No timezone, assume UTC only datetime string
            valid_time_zone  = true;
            time_zone_offset = 0;
        }
        else
        {
            int tz_match_length = 0;

            // See if timezone is present. If not, then do not update the time
            // Parse for valid timezone in remaining string
            int tz_match_index  = re_match(
                " [\\+-]\\d\\d:?\\d\\d",
                &p_datetime_string[match_index + match_length],
                static_cast<int>(datetime_string_length) - match_length,
                &tz_match_length);

            if ((tz_match_index == 0) &&
                (static_cast<size_t>(match_index + match_length +
                                     tz_match_index + tz_match_length) ==
                 datetime_string_length))
            {
                // String is a valid datetime string with timezone
                valid_time_zone = true;

                // Skip 2 characters (space and sign) to get the hour
                int tz_hour_offset =
                    match_index + match_length + tz_match_index + 2;
                int tz_hour =
                    natoi(&p_datetime_string[tz_hour_offset], hour_length);

                int tz_min_offset = tz_hour_offset + hour_length;
                // Check if the timezone has a colon
                if (p_datetime_string[tz_min_offset] == ':')
                {
                    tz_min_offset++;
                }
                int tz_min =
                    natoi(&p_datetime_string[tz_min_offset], minute_length);

                // Check the sign of the timezone
                int tz_sign_offset =
                    match_index + match_length + tz_match_index + 1;
                if (p_datetime_string[tz_sign_offset] == '-')
                {
                    tz_hour = -tz_hour;
                    tz_min  = -tz_min;
                }

                time_zone_offset =
                    (tz_hour * SECS_PER_HR) + (tz_min * SECS_PER_MIN);
            }
        }

        if (valid_time_zone)
        {
            utc_time = w_hal_rtc_timegm(&tm_struct);
            if (utc_time < time_zone_offset)
            {
                // Time is before the epoch, so add the offset
                utc_time = 0;
            }
            else
            {
                // Convert to UTC
                utc_time -= time_zone_offset;
            }
        }
    }

    return valid_time_zone;
}

/**
 * @brief Print time in human readable format to console
 *
 * @param p_prefix string to print before printable time
 * @param time time to print
 * @param time_zone_offset time zone offset
 * @param p_suffix string to print after printable time
 */
inline void console_printf_time(const char *             p_prefix,
                                const w_hal_rtc_time_st &time,
                                int                      time_zone_offset,
                                const char *             p_suffix)
{
    struct tm tm_struct;
    time_t    current_time = time.time_seconds + time_zone_offset;
    gmtime_r(&current_time, &tm_struct);

    char     time_zone_sign;
    uint16_t tz_hour;
    uint16_t tz_min;
    w_hal_rtc_get_time_zone_components(time_zone_offset,
                                       time_zone_sign,
                                       tz_hour,
                                       tz_min);

    uint32_t milliseconds = time.time_nanoseconds / 1000000;

    printf("%s%04d-%02d-%02d %02d:%02d:%02d.%03d %c%02d:%02d%s",
           p_prefix,
           TIME_BASE_YEAR + static_cast<uint32_t>(tm_struct.tm_year),
           TIME_BASE_MONTH + static_cast<uint32_t>(tm_struct.tm_mon),
           tm_struct.tm_mday,
           tm_struct.tm_hour,
           tm_struct.tm_min,
           tm_struct.tm_sec,
           milliseconds,
           time_zone_sign,
           tz_hour,
           tz_min,
           p_suffix);
}

/**
 * @brief Set the system time command
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_start_datetime(const char *p_args, const size_t args_length)
{

    bool   updated = false;
    time_t new_time_utc;
    int    new_time_zone_offset;

    bool   success = extract_time_from_datetime_string(p_args,
                                                     args_length,
                                                     new_time_utc,
                                                     new_time_zone_offset);

    if (success)
    {
        // Keep the old value in case the new value is invalid
        time_t old_time_utc         = DeviceConfig::p_page->program_time;
        int    old_time_zone_offset = DeviceConfig::p_page->time_zone_offset;

        // Set the new time
        DeviceConfig::p_page->program_time     = new_time_utc;
        DeviceConfig::p_page->time_zone_offset = new_time_zone_offset;

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid date time %s\n", p_args);
            DeviceConfig::p_page->program_time     = old_time_utc;
            DeviceConfig::p_page->time_zone_offset = old_time_zone_offset;
        }
        else
        {
            updated = true;
        }
    }
    else if (args_length > 0)
    {
        printf("\nInvalid date time %s\n"
               "UTC Time Format should be YYYY-MM-DD HH:MM:SS\n",
               p_args);
    }

    w_hal_rtc_time_st current_time;
    current_time.time_seconds     = DeviceConfig::p_page->program_time;
    current_time.time_nanoseconds = 0;
    int time_zone_offset          = DeviceConfig::p_page->time_zone_offset;
    console_printf_time("\nBoot time UTC set to ", current_time, 0, "\n");
    console_printf_time("\nBoot time Local set to ",
                        current_time,
                        time_zone_offset,
                        "\n");

    w_hal_rtc_get_time(current_time);
    console_printf_time("\nCurrent time UTC time is ", current_time, 0, "\n");
    console_printf_time("\nCurrent time Local is ",
                        current_time,
                        time_zone_offset,
                        "\n");

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Set the time zone
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_time_zone(const char *p_args, const size_t args_length)
{
    int  match_length = 0;
    int  match_index;
    bool updated = false;

    match_index =
        re_match("[\\+-]\\d\\d:?\\d\\d", p_args, args_length, &match_length);

    if ((match_index == 0) &&
        (static_cast<size_t>(match_length) == args_length))
    {
        // Parse the value from the string
        int new_time_zone_offset = natoi(&p_args[1], 2) * SECS_PER_HR;

        int minute_pos           = 3;
        if (p_args[3] == ':')
        {
            minute_pos++;
        }
        new_time_zone_offset += natoi(&p_args[minute_pos], 2) * SECS_PER_MIN;

        if (p_args[0] == '-')
        {
            new_time_zone_offset = -new_time_zone_offset;
        }

        // Keep the old value in case the new value is invalid
        int old_time_zone_offset = DeviceConfig::p_page->time_zone_offset;

        // Set the new time zone
        DeviceConfig::p_page->time_zone_offset = new_time_zone_offset;

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid time zone %s\n", p_args);
            DeviceConfig::p_page->time_zone_offset = old_time_zone_offset;
        }
        else
        {
            updated = true;
        }
    }

    if (updated)
    {
        printf("\nTime zone set to %s\n", p_args);
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
    else
    {
        if (args_length > 0)
        {
            printf("\nInvalid time zone argument %s\n", p_args);
        }
        else
        {
            char     time_zone_sign;
            uint16_t tz_hour;
            uint16_t tz_min;
            w_hal_rtc_get_time_zone_components(
                DeviceConfig::p_page->time_zone_offset,
                time_zone_sign,
                tz_hour,
                tz_min);

            printf("\nTime zone is set to %c%02d:%02d\n",
                   time_zone_sign,
                   tz_hour,
                   tz_min);
        }
    }
}

#if STORAGE_CORE != INVALID_CORE_NUM
/**
 * @brief Set the recording start and end date
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_begin_end_date(const char *p_args, const size_t args_length)
{
    // Format is YYYY/MM/DD-YYYY/MM/DD or YYYYMMDD-YYYYMMDD

    int       match_length1 = 0, match_length2 = 0;
    int       match_index1, match_index2;
    bool      matched;
    bool      updated      = false;

    const int year_length  = 4;
    const int month_length = 2;
    const int day_length   = 2;

    char      start_year[year_length + 1];
    char      start_month[month_length + 1];
    char      start_day[day_length + 1];

    char      end_year[year_length + 1];
    char      end_month[month_length + 1];
    char      end_day[day_length + 1];

    // match_index1 for YYYYMMDD-YYYYMMDD
    match_index1 = re_match("\\d\\d\\d\\d\\d\\d\\d\\d-\\d\\d\\d\\d\\d\\d\\d\\d",
                            p_args,
                            args_length,
                            &match_length1);

    // match_index2 for YYYY/MM/DD-YYYY/MM/DD
    match_index2 =
        re_match("\\d\\d\\d\\d/\\d\\d/\\d\\d-\\d\\d\\d\\d/\\d\\d/\\d\\d",
                 p_args,
                 args_length,
                 &match_length2);

    if ((match_index1 == 0) &&
        (static_cast<size_t>(match_length1) == args_length))
    {
        const int date_length = year_length + month_length + day_length;

        strncpy(start_year, &p_args[0], year_length);
        strncpy(start_month, &p_args[year_length], month_length);
        strncpy(start_day, &p_args[year_length + month_length], day_length);

        strncpy(end_year, &p_args[date_length + 1], year_length);
        strncpy(end_month,
                &p_args[date_length + 1 + year_length],
                month_length);
        strncpy(end_day,
                &p_args[date_length + 1 + year_length + month_length],
                day_length);

        matched = true;
    }
    else if ((match_index2 == 0) &&
             (static_cast<size_t>(match_length2) == args_length))
    {
        const int date_length = year_length + month_length + day_length + 2;

        strncpy(start_year, &p_args[0], year_length);
        strncpy(start_month, &p_args[year_length + 1], month_length);
        strncpy(start_day, &p_args[year_length + month_length + 2], day_length);

        strncpy(end_year, &p_args[date_length + 1], year_length);
        strncpy(end_month,
                &p_args[date_length + year_length + 2],
                month_length);
        strncpy(end_day,
                &p_args[date_length + year_length + month_length + 3],
                day_length);

        matched = true;
    }
    else
    {
        matched = false;
    }

    // null terminating
    start_year[4]  = '\0';
    start_month[2] = '\0';
    start_day[2]   = '\0';
    end_year[4]    = '\0';
    end_month[2]   = '\0';
    end_day[2]     = '\0';

    if (matched)
    {
        date_st  old_start_date     = DeviceConfig::p_page->start_date;
        date_st  old_end_date       = DeviceConfig::p_page->end_date;

        uint32_t parsed_start_year  = static_cast<uint32_t>(atoi(start_year));
        uint32_t parsed_start_month = static_cast<uint32_t>(atoi(start_month));
        uint32_t parsed_start_day   = static_cast<uint32_t>(atoi(start_day));

        date_st  new_start_date;
        // Checking for a valid start date
        if ((parsed_start_year == 0) || (parsed_start_month == 0) ||
            (parsed_start_day == 0) ||
            (parsed_start_year >=
             DeviceConfig::INVALID_YEAR + static_cast<int>(TIME_BASE_YEAR)) ||
            (parsed_start_month > MONTHS_PER_YEAR) ||
            (parsed_start_day > MAX_DAYS_PER_MONTH))
        {
            // Make start date invalid
            new_start_date.year  = DeviceConfig::INVALID_YEAR;
            new_start_date.month = DeviceConfig::INVALID_MONTH;
            new_start_date.day   = DeviceConfig::INVALID_DAY_IN_MONTH;
        }
        else
        {
            // Parse the value from the string
            new_start_date.year =
                atoi(start_year) - static_cast<int>(TIME_BASE_YEAR);
            new_start_date.month =
                atoi(start_month) - static_cast<int>(TIME_BASE_MONTH);
            new_start_date.day = atoi(start_day);
        }

        uint32_t parsed_end_year  = static_cast<uint32_t>(atoi(end_year));
        uint32_t parsed_end_month = static_cast<uint32_t>(atoi(end_month));
        uint32_t parsed_end_day   = static_cast<uint32_t>(atoi(end_day));

        date_st  new_end_date;
        // Checking for a valid end date
        if ((parsed_end_year == 0) || (parsed_end_month == 0) ||
            (parsed_end_day == 0) ||
            (parsed_end_year >=
             DeviceConfig::INVALID_YEAR + static_cast<int>(TIME_BASE_YEAR)) ||
            (parsed_end_month > MONTHS_PER_YEAR) ||
            (parsed_end_day > MAX_DAYS_PER_MONTH))
        {
            // Make end date invalid
            new_end_date.year  = DeviceConfig::INVALID_YEAR;
            new_end_date.month = DeviceConfig::INVALID_MONTH;
            new_end_date.day   = DeviceConfig::INVALID_DAY_IN_MONTH;
        }
        else
        {
            // Parse the value from the string
            new_end_date.year =
                atoi(end_year) - static_cast<int>(TIME_BASE_YEAR);
            new_end_date.month =
                atoi(end_month) - static_cast<int>(TIME_BASE_MONTH);
            new_end_date.day = atoi(end_day);
        }

        DeviceConfig::p_page->start_date = new_start_date;
        DeviceConfig::p_page->end_date   = new_end_date;

        if (!DeviceConfig::param_validate(false))
        {
            printf("\nInvalid begin end date %s\n", p_args);
            DeviceConfig::p_page->start_date = old_start_date;
            DeviceConfig::p_page->end_date   = old_end_date;
        }
        else
        {
            updated = true;
        }
    }
    else if (args_length > 0)
    {
        printf("\nInvalid begin end date %s\n", p_args);
    }

    printf("\nBegin end date set to %04d/%02d/%02d-%04d/%02d/%02d\n",
           static_cast<uint32_t>(DeviceConfig::p_page->start_date.year) +
               TIME_BASE_YEAR,
           static_cast<uint32_t>(DeviceConfig::p_page->start_date.month) +
               TIME_BASE_MONTH,
           DeviceConfig::p_page->start_date.day,
           static_cast<uint32_t>(DeviceConfig::p_page->end_date.year) +
               TIME_BASE_YEAR,
           static_cast<uint32_t>(DeviceConfig::p_page->end_date.month) +
               TIME_BASE_MONTH,
           DeviceConfig::p_page->end_date.day);

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Set the recording time of day
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_begin_end_time(const char *p_args, const size_t args_length)
{
    // Format is HH:MM:SS-HH:MM:SS or HHMMSS-HHMMSS
    int       match_length_1a = 0, match_length_1b = 0;
    int       match_length_2a = 0, match_length_2b = 0;
    int       match_index_1a, match_index_1b, match_index_2a, match_index_2b;
    bool      exact_match_1a, exact_match_1b, exact_match_2a, exact_match_2b;
    bool      matched       = false;
    bool      updated       = false;

    const int hour_length   = 2;
    const int minute_length = 2;
    const int second_length = 2;

    int       start_hour;
    int       start_minute;
    int       start_seconds;

    int       end_hour;
    int       end_minute;
    int       end_seconds;

    match_index_1a = re_match("\\d\\d:\\d\\d:\\d\\d-\\d\\d:\\d\\d:\\d\\d",
                              p_args,
                              args_length,
                              &match_length_1a);
    exact_match_1a = (match_index_1a == 0) &&
                     (static_cast<size_t>(match_length_1a) == args_length);

    match_index_1b = re_match("\\d+ \\d\\d:\\d\\d:\\d\\d-\\d\\d:\\d\\d:\\d\\d",
                              p_args,
                              args_length,
                              &match_length_1b);
    exact_match_1b = (match_index_1b == 0) &&
                     (static_cast<size_t>(match_length_1b) == args_length);

    match_index_2a = re_match("\\d\\d\\d\\d\\d\\d-\\d\\d\\d\\d\\d\\d",
                              p_args,
                              args_length,
                              &match_length_2a);
    exact_match_2a = (match_index_2a == 0) &&
                     (static_cast<size_t>(match_length_2a) == args_length);

    match_index_2b = re_match("\\d+ \\d\\d\\d\\d\\d\\d-\\d\\d\\d\\d\\d\\d",
                              p_args,
                              args_length,
                              &match_length_2b);
    exact_match_2b = (match_index_2b == 0) &&
                     (static_cast<size_t>(match_length_2b) == args_length);

    int time_of_day_index = 0;

    if (exact_match_1a || exact_match_1b || exact_match_2a || exact_match_2b)
    {
        uint32_t parse_index = 0;

        if (exact_match_1b || exact_match_2b)
        {
            uint32_t number_length =
                find_length_of_number(&p_args[parse_index],
                                      args_length - parse_index);

            time_of_day_index = atoi(&p_args[parse_index]);

            // Skip the number and the space
            parse_index += number_length + 1;
        }

        start_hour = natoi(&p_args[parse_index], hour_length);
        parse_index += hour_length;
        parse_index += find_index_of_next_digit(&p_args[parse_index],
                                                args_length - parse_index);

        start_minute = natoi(&p_args[parse_index], minute_length);
        parse_index += minute_length;
        parse_index += find_index_of_next_digit(&p_args[parse_index],
                                                args_length - parse_index);

        start_seconds = natoi(&p_args[parse_index], second_length);
        parse_index += second_length;
        parse_index += find_index_of_next_digit(&p_args[parse_index],
                                                args_length - parse_index);

        end_hour = natoi(&p_args[parse_index], hour_length);
        parse_index += hour_length;
        parse_index += find_index_of_next_digit(&p_args[parse_index],
                                                args_length - parse_index);

        end_minute = natoi(&p_args[parse_index], minute_length);
        parse_index += minute_length;
        parse_index += find_index_of_next_digit(&p_args[parse_index],
                                                args_length - parse_index);

        end_seconds = natoi(&p_args[parse_index], second_length);
        parse_index += second_length;
        parse_index += find_index_of_next_digit(&p_args[parse_index],
                                                args_length - parse_index);

        matched = true;
    }

    if (matched)
    {
        DeviceConfig::time_of_day_range_st
            old_time_of_day_range[MAX_SCHEDULE_TIME_OF_DAY_RANGES];
        DeviceConfig::time_of_day_range_st new_time_of_day_range;

        // Hold the old value in case the new value is invalid
        memcpy(old_time_of_day_range,
               DeviceConfig::p_page->time_of_day_range,
               sizeof(old_time_of_day_range));

        // Parse the value from the string
        new_time_of_day_range.start_second =
            static_cast<uint32_t>(start_hour * SECS_PER_HR +
                                  start_minute * SECS_PER_MIN + start_seconds);
        new_time_of_day_range.end_second = static_cast<uint32_t>(
            end_hour * SECS_PER_HR + end_minute * SECS_PER_MIN + end_seconds);

        if (new_time_of_day_range.start_second >
            new_time_of_day_range.end_second)
        {
            printf("Start time is %d greater than or "
                   "equal to end time %d. "
                   "Setting end time to start time.\n",
                   new_time_of_day_range.start_second,
                   new_time_of_day_range.end_second);
            new_time_of_day_range.end_second =
                new_time_of_day_range.start_second;
        }

        if (time_of_day_index == 0)
        {
            DeviceConfig::reset_time_of_day_ranges();
        }
        DeviceConfig::p_page->time_of_day_range[time_of_day_index] =
            new_time_of_day_range;
        DeviceConfig::sort_time_of_day_ranges(
            DeviceConfig::p_page->time_of_day_range,
            MAX_SCHEDULE_TIME_OF_DAY_RANGES);

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid begin end time %s\n", p_args);
            memcpy(DeviceConfig::p_page->time_of_day_range,
                   old_time_of_day_range,
                   sizeof(old_time_of_day_range));
        }
        else
        {
            updated = true;
        }
    }
    else if (args_length > 0)
    {
        printf("\nInvalid begin end time %s\n", p_args);
    }

    uint32_t num_valid_time_of_day_ranges = 0;
    for (uint32_t index = 0; index < MAX_SCHEDULE_TIME_OF_DAY_RANGES; index++)
    {
        const DeviceConfig::time_of_day_range_st &time_of_day_range =
            DeviceConfig::p_page->time_of_day_range[index];

        if (DeviceConfig::is_valid_time_of_day_range(time_of_day_range))
        {
            printf("\nBegin end time (Index %d) set to "
                   "%02d:%02d:%02d-%02d:%02d:%02d\n",
                   index,
                   time_of_day_range.start_second / SECS_PER_HR,
                   (time_of_day_range.start_second / SECS_PER_MIN) %
                       MINS_PER_HR,
                   time_of_day_range.start_second % SECS_PER_MIN,
                   time_of_day_range.end_second / SECS_PER_HR,
                   (time_of_day_range.end_second / SECS_PER_MIN) % MINS_PER_HR,
                   time_of_day_range.end_second % SECS_PER_MIN);
            num_valid_time_of_day_ranges++;
        }
    }

    if (num_valid_time_of_day_ranges == 0)
    {
        printf("\nNo valid time of day ranges set\n");
    }

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}

/**
 * @brief Set the recording days of the week
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_recording_days_of_week(const char * p_args,
                                       const size_t args_length)
{
    // Format can be M,T,W,R,F,S,U, or X
    // M = Monday, T = Tuesday, W = Wednesday, R = Thursday, F = Friday,
    // S = Saturday, U = Sunday, X = None
    bool updated = false;

    if (args_length > 0)
    {
        // Hold the old value in case the new value is invalid
        uint8_t old_value = DeviceConfig::p_page->day_of_week_mask;

        uint8_t new_value = 0;

        if (((args_length == 1) && (p_args[0] == 'X')) ||
            ((args_length == 2) && (p_args[0] == '"') && (p_args[1] == '"')))
        {
            new_value = DeviceConfig::FULL_WEEK_MASK;
            printf("\nDays of week: X is set to all days (MTWRFSU)\n");
        }
        else
        {
            int match_length = 0;
            int match_index;

            match_index = re_match("\"[MTWRFSU\\s]+\"",
                                   p_args,
                                   args_length,
                                   &match_length);

            bool match  = (match_index == 0) &&
                         (static_cast<size_t>(match_length) == args_length);

            if (!match)
            {
                // Try without quotes
                match_index = re_match("[MTWRFSU\\s]+",
                                       p_args,
                                       args_length,
                                       &match_length);

                match       = (match_index == 0) &&
                        (static_cast<size_t>(match_length) == args_length);
            }

            if (match)
            {
                for (uint32_t index = 0; index < args_length; index++)
                {
                    switch (p_args[index])
                    {
                        case 'M':
                        {
                            new_value |= 0x01;
                            break;
                        }
                        case 'T':
                        {
                            new_value |= 0x02;
                            break;
                        }
                        case 'W':
                        {
                            new_value |= 0x04;
                            break;
                        }
                        case 'R':
                        {
                            new_value |= 0x08;
                            break;
                        }
                        case 'F':
                        {
                            new_value |= 0x10;
                            break;
                        }
                        case 'S':
                        {
                            new_value |= 0x20;
                            break;
                        }
                        case 'U':
                        {
                            new_value |= 0x40;
                            break;
                        }
                        case '"':
                        case ' ':
                        {
                            break;
                        }
                        default:
                        {
                            printf("\nInvalid days of week %s\n", p_args);
                            return;
                        }
                    }
                }
            }
            else
            {
                printf("\nInvalid days of week %s\n", p_args);
                return;
            }
        }

        DeviceConfig::p_page->day_of_week_mask = new_value;

        if (!DeviceConfig::param_validate(true))
        {
            printf("\nInvalid days of week %s\n", p_args);
            DeviceConfig::p_page->day_of_week_mask = old_value;
        }
        else
        {
            updated = true;
        }
    }

    printf("\nDays of week set to \"");
    if (DeviceConfig::p_page->day_of_week_mask & 0x01)
    {
        printf("M");
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x02)
    {
        printf("T");
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x04)
    {
        printf("W");
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x08)
    {
        printf("R");
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x10)
    {
        printf("F");
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x20)
    {
        printf("S");
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x40)
    {
        printf("U");
    }
    printf("\"\n");

    if (updated)
    {
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
}
#endif // STORAGE_CORE != INVALID_CORE_NUM

/**
 * @brief Apply settings to flash
 */
static void apply_settings()
{
    DeviceConfig::write_config_page_to_flash();
    printf("\nDevice configuration is applied after reboot.\n");
}

/**
 * @brief Reboot the device
 */
static void reboot()
{
    printf("\nRebooting device.\n");
    if (!shared_glbl.reboot_flag)
    {
        shared_glbl.reboot_flag    = true;
        shared_glbl.shutdown_stage = shutdown_stage_t::Stage_1;
    }
}

/**
 * @brief Apply settings to flash and reboot
 */
static void apply_and_reboot()
{
    if (!shared_glbl.save_config_flag)
    {
        shared_glbl.save_config_flag = true;
    }
    reboot();
}

/**
 * @brief Set to format the sd card
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void sd_card_format(const char *p_args, const size_t args_length)
{
    int match_length = 0;
    int match_index;

    match_index = re_match("\\d+", p_args, args_length, &match_length);

    if ((match_index == 0) &&
        (static_cast<size_t>(match_length) == args_length))
    {
        if (atoi(p_args) == 0)
        {
            shared_glbl.format_sd_card_flag = false;
            printf("\nSD card format won't happen during reboot\n");
        }
        else
        {
            if (!shared_glbl.format_sd_card_flag)
            {
                shared_glbl.format_sd_card_flag = true;
                printf("\nSD card format will happen during reboot\n");
            }
        }
    }
}

/**
 * @brief Prints the stored assert information
 *
 */
static void assert_info()
{
    if (!shared_glbl.shared_assert_info.asserted)
    {
        printf("There is no assert information \n");
    }
    else
    {
        if (shared_glbl.shared_assert_info.core != CORE_0)
        {
            printf("ASSERT IN CORE %d %s #%d\n",
                   shared_glbl.shared_assert_info.core,
                   shared_glbl.shared_assert_info.file,
                   shared_glbl.shared_assert_info.line);
        }
        else
        {
            if (!shared_glbl.shared_assert_info.assert_in_set)
            {
                printf("ASSERT IN CORE %d %s #%d: %s (Value:0x%llx) %s %s "
                       "(Value:0x%llx)\n",
                       shared_glbl.shared_assert_info.core,
                       shared_glbl.shared_assert_info.file,
                       shared_glbl.shared_assert_info.line,
                       shared_glbl.shared_assert_info.expr1,
                       shared_glbl.shared_assert_info.value1,
                       shared_glbl.shared_assert_info.comparator,
                       shared_glbl.shared_assert_info.expr2,
                       shared_glbl.shared_assert_info.value2);
            }
            else
            {
                printf("ASSERT IN CORE %d %s #%d: %s (Value:0x%llx) not in set "
                       "of %lu "
                       "values\n",
                       shared_glbl.shared_assert_info.core,
                       shared_glbl.shared_assert_info.file,
                       shared_glbl.shared_assert_info.line,
                       shared_glbl.shared_assert_info.expr1,
                       shared_glbl.shared_assert_info.value1,
                       shared_glbl.shared_assert_info.number_of_values);
            }
        }
    }
}

/**
 * @brief Get the battery status object
 *
 */
static void get_battery_status()
{
    printf("Battery_voltage: %d mV\n", shared_glbl.power_mgmt_info.voltage);
    printf("Battery_current: %d mA\n", shared_glbl.power_mgmt_info.current);
    printf("Battery_remaining_capacity: %d mAH\n",
           shared_glbl.power_mgmt_info.remaining_capacity);
    printf("Battery_full_charge_capacity: %d mAH\n",
           shared_glbl.power_mgmt_info.full_charge_capacity);
    printf("Battery_average_current: %d mA\n",
           shared_glbl.power_mgmt_info.average_current);
    printf("Battery_time_to_empty: %d sec\n",
           shared_glbl.power_mgmt_info.time_to_empty);
    printf("Battery_average_power: %d mW\n",
           shared_glbl.power_mgmt_info.average_power);
    printf("Battery_temperature: %d C\n",
           shared_glbl.power_mgmt_info.temperature);
}

#if STORAGE_CORE != INVALID_CORE_NUM
/**
 * @brief Get the sd card storage status object
 *
 */
static void get_sd_card_storage_status()
{
    printf("The total number of bytes written: %lld and the available space: "
           "%lld\n",
           shared_glbl.storage_info.total_bytes_written,
           (shared_glbl.storage_info.total_sd_card_capacity -
            shared_glbl.storage_info.total_bytes_written));
}
#endif // STORAGE_CORE != INVALID_CORE_NUM

/**
 * @brief Get the firmware version information
 */
static void fw_version_info()
{
    print_fw_version_information();
}

/**
 * @brief Get the environment status object
 *
 */
static void get_environment_status()
{
    w_hal_dht_init();
    w_hal_dht_get_env_info(shared_glbl.environment_info.temperature_celsius,
                           shared_glbl.environment_info.relative_humidity);

    printf("Current_Temperature: %.2fC, Current_Humidity: %.2f%%\n",
           shared_glbl.environment_info.temperature_celsius,
           shared_glbl.environment_info.relative_humidity);
}

#if (BLE_CORE != INVALID_CORE_NUM) && !defined(BLE_CORE_CUSTOM)
/**
 * @brief Request BLE pairing/connect window
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void set_ble_pairing_window(const char *p_args, const size_t args_length)
{
    uint32_t window_seconds = BLE_PAIR_WINDOW_DEFAULT_SECONDS;

    if ((p_args != nullptr) && (args_length > 0))
    {
        if (args_length >= MAX_UART_CMD_LEN)
        {
            printf("\nInvalid value. Use: BLEPAIR [seconds]\n");
            return;
        }

        char parse_buffer[MAX_UART_CMD_LEN];
        memcpy(parse_buffer, p_args, args_length);
        parse_buffer[args_length] = '\0';

        char *cursor              = parse_buffer;
        while (isspace(*cursor))
        {
            cursor++;
        }

        if (*cursor == '\0')
        {
            printf("\nInvalid value. Use: BLEPAIR [seconds]\n");
            return;
        }

        char *end_ptr = nullptr;
        long  parsed  = strtol(cursor, &end_ptr, 10);

        while ((end_ptr != nullptr) && isspace(*end_ptr))
        {
            end_ptr++;
        }

        if ((end_ptr == cursor) || (end_ptr == nullptr) || (*end_ptr != '\0') ||
            (parsed <= 0) ||
            (parsed > static_cast<long>(BLE_PAIR_WINDOW_MAX_SECONDS)))
        {
            printf("\nInvalid value. Use: BLEPAIR [1-%u]\n",
                   BLE_PAIR_WINDOW_MAX_SECONDS);
            return;
        }

        window_seconds = static_cast<uint32_t>(parsed);
    }

    shared_glbl.ble_pairing_window_request_seconds = window_seconds;
    shared_glbl.ble_pairing_window_request_pending = 1;

    printf("\nBLE pairing window requested for %u seconds\n", window_seconds);
}
#endif // BLE_CORE != INVALID_CORE_NUM

/**
 * @brief Parse location coordinates from a string.
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 * @param latitude output parameter for latitude
 * @param longitude output parameter for longitude
 * @return true if parsing was successful and the coordinates are valid
 * @return false if parsing failed or the coordinates are invalid
 */
static bool parse_location_coordinates(const char *p_args,
                                       size_t      args_length,
                                       double &    latitude,
                                       double &    longitude)
{
    if ((p_args == nullptr) || (args_length == 0) ||
        (args_length >= MAX_UART_CMD_LEN))
    {
        return false;
    }

    char parse_buffer[MAX_UART_CMD_LEN];
    memcpy(parse_buffer, p_args, args_length);
    parse_buffer[args_length] = '\0';

    char *cursor              = parse_buffer;
    while (isspace(*cursor))
    {
        cursor++;
    }

    if (*cursor == '\0')
    {
        return false;
    }

    char *end_ptr = nullptr;
    latitude      = strtod(cursor, &end_ptr);
    if ((end_ptr == cursor) || !std::isfinite(latitude))
    {
        return false;
    }

    cursor = end_ptr;
    while (isspace(*cursor))
    {
        cursor++;
    }

    if (*cursor == '\0')
    {
        return false;
    }

    longitude = strtod(cursor, &end_ptr);
    if ((end_ptr == cursor) || !std::isfinite(longitude))
    {
        return false;
    }

    while (isspace(*end_ptr))
    {
        end_ptr++;
    }

    return (*end_ptr == '\0');
}

/**
 * @brief Print the current location
 */
static void print_current_location()
{
    if ((DeviceConfig::p_page->latitude_e7 == INVALID_LOCATION_COORDINATE) ||
        (DeviceConfig::p_page->longitude_e7 == INVALID_LOCATION_COORDINATE))
    {
        printf("Current_Location: Not set\n");
        return;
    }

    printf("Current_Location: %.7f, %.7f\n",
           static_cast<double>(DeviceConfig::p_page->latitude_e7) /
               LOCATION_COORDINATE_SCALE,
           static_cast<double>(DeviceConfig::p_page->longitude_e7) /
               LOCATION_COORDINATE_SCALE);
}

/**
 * @brief Set or get the current location
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_location_info(const char *p_args, const size_t args_length)
{
    if ((args_length == 0) || (p_args == nullptr))
    {
        print_current_location();
        return;
    }

    if ((args_length == (sizeof("CLEAR") - 1)) &&
        (strncmp(p_args, "CLEAR", sizeof("CLEAR") - 1) == 0))
    {
        DeviceConfig::p_page->latitude_e7  = INVALID_LOCATION_COORDINATE;
        DeviceConfig::p_page->longitude_e7 = INVALID_LOCATION_COORDINATE;
        printf("\nLocation cleared\n");
        print_current_location();
        printf("\nSend APPLY command and reboot for changes to take effect\n");
        return;
    }

    double latitude  = 0.0;
    double longitude = 0.0;
    if (!parse_location_coordinates(p_args, args_length, latitude, longitude))
    {
        printf("\nInvalid location. Use: LOC <latitude> <longitude>\n");
        print_current_location();
        return;
    }

    if ((latitude < -LATITUDE_LIMIT) || (latitude > LATITUDE_LIMIT) ||
        (longitude < -LONGITUDE_LIMIT) || (longitude > LONGITUDE_LIMIT))
    {
        printf("\nInvalid location range. Latitude [-90,90], Longitude "
               "[-180,180]\n");
        print_current_location();
        return;
    }

    // Scale to E7 and round to nearest int, with correct handling for
    // negatives.
    const double scaled_lat =
        latitude * static_cast<double>(LOCATION_COORDINATE_SCALE);
    const double scaled_lon =
        longitude * static_cast<double>(LOCATION_COORDINATE_SCALE);
    const int32_t latitude_e7 =
        (scaled_lat >= ZERO_OFFSET) ?
            static_cast<int32_t>(std::floor(scaled_lat + ROUND_OFFSET)) :
            static_cast<int32_t>(std::ceil(scaled_lat - ROUND_OFFSET));
    const int32_t longitude_e7 =
        (scaled_lon >= ZERO_OFFSET) ?
            static_cast<int32_t>(std::floor(scaled_lon + ROUND_OFFSET)) :
            static_cast<int32_t>(std::ceil(scaled_lon - ROUND_OFFSET));

    DeviceConfig::p_page->latitude_e7  = latitude_e7;
    DeviceConfig::p_page->longitude_e7 = longitude_e7;

    printf("\nLocation set to %.7f, %.7f\n", latitude, longitude);
    printf("\nSend APPLY command and reboot for changes to take effect\n");
}

/**
 * @brief Set sunrise and sunset based recording
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_sunrise_sunset_info(const char * p_args,
                                    const size_t args_length)
{
    // check if location is set before doing any parsing or calculations
    if ((DeviceConfig::p_page->latitude_e7 == INVALID_LOCATION_COORDINATE) ||
        (DeviceConfig::p_page->longitude_e7 == INVALID_LOCATION_COORDINATE))
    {
        printf("\nLocation is not set. Use: LOC <latitude> <longitude>\n");
        return;
    }

    constexpr double MIDDAY_FRACTIONAL_UT = 0.0;
    int              year                 = 0;
    int              month                = 0;
    int              day                  = 0;

    if ((p_args == nullptr) || (args_length == 0))
    {
        time_t current_time =
            w_hal_rtc_get_time() + DeviceConfig::p_page->time_zone_offset;
        tm tm_struct;
        gmtime_r(&current_time, &tm_struct);

        year  = static_cast<int>(TIME_BASE_YEAR) + tm_struct.tm_year;
        month = static_cast<int>(TIME_BASE_MONTH) + tm_struct.tm_mon;
        day   = tm_struct.tm_mday;
    }
    else
    {
        const char datetime_suffix[] = " 00:00:00 +0000";
        char       datetime_buffer[MAX_UART_CMD_LEN];

        if ((args_length + sizeof(datetime_suffix)) > sizeof(datetime_buffer))
        {
            printf("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
            return;
        }

        int ret_val = snprintf(datetime_buffer,
                               sizeof(datetime_buffer),
                               "%.*s%s",
                               static_cast<int>(args_length),
                               p_args,
                               datetime_suffix);

        if (ret_val <= 0)
        {
            printf("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
            return;
        }

        time_t parsed_time;
        int    parsed_time_zone_offset;
        if (!extract_time_from_datetime_string(datetime_buffer,
                                               strlen(datetime_buffer),
                                               parsed_time,
                                               parsed_time_zone_offset))
        {
            printf("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
            return;
        }

        tm tm_struct;
        gmtime_r(&parsed_time, &tm_struct);

        year  = static_cast<int>(TIME_BASE_YEAR) + tm_struct.tm_year;
        month = static_cast<int>(TIME_BASE_MONTH) + tm_struct.tm_mon;
        day   = tm_struct.tm_mday;
    }

    const double julian_date =
        SunCalculator::compute_julian_date(year,
                                           month,
                                           day,
                                           MIDDAY_FRACTIONAL_UT);
    const double latitude =
        static_cast<double>(DeviceConfig::p_page->latitude_e7) /
        LOCATION_COORDINATE_SCALE;
    const double longitude =
        static_cast<double>(DeviceConfig::p_page->longitude_e7) /
        LOCATION_COORDINATE_SCALE;
    uint32_t sunrise_second = 0;
    uint32_t sunset_second  = 0;
    if (!SunCalculator::compute_sunrise_sunset_seconds(
            julian_date,
            latitude,
            longitude,
            DeviceConfig::p_page->time_zone_offset,
            sunrise_second,
            sunset_second))
    {
        printf("\nSunrise/sunset is not available for this date and "
               "location\n");
        return;
    }

    printf("\nSun date: %04d/%02d/%02d\n", year, month, day);
    printf("Julian date: %.2f\n", julian_date);
    printf("Sunrise time: %02d:%02d:%02d\n",
           sunrise_second / SECS_PER_HR,
           (sunrise_second / SECS_PER_MIN) % MINS_PER_HR,
           sunrise_second % SECS_PER_MIN);
    printf("Sunset time: %02d:%02d:%02d\n",
           sunset_second / SECS_PER_HR,
           (sunset_second / SECS_PER_MIN) % MINS_PER_HR,
           sunset_second % SECS_PER_MIN);
}

/**
 * @brief Set sun recording mode
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void set_sun_recording_mode(const char *p_args, const size_t args_length)
{
    if (args_length == 0)
    {
        // Print current mode
        switch (DeviceConfig::p_page->sun_recording_mode)
        {
            case DeviceConfig::SUN_MODE_SUNRISE:
                printf("\nSun recording mode: Sunrise\n");
                break;
            case DeviceConfig::SUN_MODE_SUNSET:
                printf("\nSun recording mode: Sunset\n");
                break;
            case DeviceConfig::SUN_MODE_SUNRISE_AND_SUNSET:
                printf("\nSun recording mode: Sunrise and Sunset\n");
                break;
            case DeviceConfig::SUN_MODE_SUNRISE_TO_SUNSET:
                printf("\nSun recording mode: Sunrise to Sunset\n");
                break;
            case DeviceConfig::SUN_MODE_SUNSET_TO_SUNRISE:
                printf("\nSun recording mode: Sunset to Sunrise\n");
                break;
            default:
                printf("\nSun recording mode: Disabled\n");
                break;
        }
        return;
    }

    struct
    {
        const char *                       label;
        DeviceConfig::sun_recording_mode_t mode;
    } sun_mode_table[] = {
        {"Sunrise and Sunset", DeviceConfig::SUN_MODE_SUNRISE_AND_SUNSET},
        {"Sunrise to Sunset", DeviceConfig::SUN_MODE_SUNRISE_TO_SUNSET},
        {"Sunset to Sunrise", DeviceConfig::SUN_MODE_SUNSET_TO_SUNRISE},
        {"Sunrise", DeviceConfig::SUN_MODE_SUNRISE},
        {"Sunset", DeviceConfig::SUN_MODE_SUNSET},
        {"Disabled", DeviceConfig::SUN_MODE_DISABLED},
    };

    for (const auto &entry : sun_mode_table)
    {
        const size_t label_len = strlen(entry.label);
        if ((args_length == label_len) &&
            (strncmp(p_args, entry.label, label_len) == 0))
        {
            DeviceConfig::p_page->sun_recording_mode = entry.mode;
            printf("\nSun recording mode set to: %s\n", entry.label);
            printf(
                "\nSend APPLY command and reboot for changes to take effect\n");

            // After setting the mode, if disabled clear all interval fields
            // in flash so stale values from a previous mode do not persist
            if (DeviceConfig::p_page->sun_recording_mode ==
                DeviceConfig::SUN_MODE_DISABLED)
            {
                DeviceConfig::p_page->sunrise_interval_before = 0;
                DeviceConfig::p_page->sunrise_interval_after  = 0;
                DeviceConfig::p_page->sunset_interval_before  = 0;
                DeviceConfig::p_page->sunset_interval_after   = 0;
                DeviceConfig::p_page->sunrise_sunset_rounding = 0;
            }
            return;
        }
    }

    printf("\nInvalid sun recording mode: %.*s\n", (int)args_length, p_args);
    printf("Valid modes: Sunrise, Sunset, Sunrise and Sunset, "
           "Sunrise to Sunset, Sunset to Sunrise, Disabled\n");
}

/**
 * @brief Set the default device configuration
 */
static void set_default_config()
{
    DeviceConfig::p_page->sampling_rate = DEFAULT_AUDIO_SAMPLE_RATE;
    DeviceConfig::p_page->sms_send_upload_frequency_seconds =
        DEFAULT_SMS_UPLOAD_FREQUENCY;
    DeviceConfig::p_page->mic_gain_times_ten   = DEFAULT_MIC_GAIN_TIMES_TEN;
    DeviceConfig::p_page->audio_sampling_width = DEFAULT_BITS_PER_AUDIO_SAMPLE;
    DeviceConfig::p_page->num_audio_channels   = DEFAULT_NUM_AUDIO_CHANNELS;
    DeviceConfig::p_page->recording_length     = DEFAULT_SECONDS_PER_AUDIO_FILE;
    DeviceConfig::p_page->sleep_pause_interval = DEFAULT_SLEEP_PAUSE_INTERVAL;

    DeviceConfig::p_page->folder_structure =
        DeviceConfig::folder_structure_t::HOUR;

    DeviceConfig::p_page->start_date.year  = DeviceConfig::INVALID_YEAR;
    DeviceConfig::p_page->start_date.month = DeviceConfig::INVALID_MONTH;
    DeviceConfig::p_page->start_date.day   = DeviceConfig::INVALID_DAY_IN_MONTH;

    DeviceConfig::p_page->end_date.year    = DeviceConfig::INVALID_YEAR;
    DeviceConfig::p_page->end_date.month   = DeviceConfig::INVALID_MONTH;
    DeviceConfig::p_page->end_date.day     = DeviceConfig::INVALID_DAY_IN_MONTH;

    DeviceConfig::reset_time_of_day_ranges();

    DeviceConfig::p_page->day_of_week_mask =
        DeviceConfig::FULL_WEEK_MASK; // All days of the week

    DeviceConfig::p_page->detector = DeviceConfig::detector_select_t::NONE;
    DeviceConfig::p_page->ai_sensitivity         = 50; // Balanced sensitivity

    DeviceConfig::p_page->lower_cutoff_frequency = 0;
    DeviceConfig::p_page->upper_cutoff_frequency =
        DeviceConfig::p_page->sampling_rate / 2;
    DeviceConfig::p_page->transmission_mode =
        DeviceConfig::transmission_mode_t::MODE_4G;
    DeviceConfig::p_page->low_battery_voltage_cutoff =
        DEFAULT_LOW_BATTERY_VOLTAGE;
    DeviceConfig::p_page->num_battery_cells_used =
        DEFAULT_NUMBER_OF_BATTERY_CELLS;

    DeviceConfig::p_page->latitude_e7        = INVALID_LOCATION_COORDINATE;
    DeviceConfig::p_page->longitude_e7       = INVALID_LOCATION_COORDINATE;
    DeviceConfig::p_page->flags.dw           = 0;

    DeviceConfig::p_page->sun_recording_mode = DeviceConfig::SUN_MODE_DISABLED;
    DeviceConfig::p_page->sunrise_interval_before = 0;
    DeviceConfig::p_page->sunrise_interval_after  = 0;
    DeviceConfig::p_page->sunset_interval_before  = 0;
    DeviceConfig::p_page->sunset_interval_after   = 0;
    DeviceConfig::p_page->sunrise_sunset_rounding = 0;

    DeviceConfig::param_validate(true);

    printf("\nDefault device configurations are set\n");
}

#if RTC_CORE != INVALID_CORE_NUM
/**
 * @brief Reset the RTC time to 2025-01-01 00:00:00 UTC
 */
static void reset_rtc_time()
{
    tm reset_tm;
    // Set the tm structure to 2025-01-01 00:00:00 UTC
    memset(&reset_tm, 0, sizeof(reset_tm));
    reset_tm.tm_year = 2025 - static_cast<int>(TIME_BASE_YEAR);
    reset_tm.tm_mon  = 1 - static_cast<int>(TIME_BASE_MONTH);
    reset_tm.tm_mday = 1;

    w_hal_rtc_time_st reset_time;
    reset_time.time_seconds     = w_hal_rtc_timegm(&reset_tm);
    reset_time.time_nanoseconds = 0;

    wHalErrorCode error         = w_hal_rtc_set_time(reset_time);

    if (error == wHalErrorCode::NO_ERROR)
    {
        DeviceConfig::p_page->program_time = reset_time.time_seconds;
        printf("\nRTC reset to 2025-01-01 00:00:00 UTC\n");
        printf("\nSend APPLY command and reboot for changes to take effect\n");
    }
    else
    {
        printf("\nRTC reset failed with error %d\n", static_cast<int>(error));
    }
}
#endif // RTC_CORE != INVALID_CORE_NUM

/**
 * @brief Wait for I2C command completion
 *
 * @param channel I2C channel
 * @param read true when waiting for read completion
 * @return true if command completed
 */
static bool wait_i2c_cmd_complete(uint8_t channel, bool read)
{
    wHalErrorCode complete_error = read ? wHalErrorCode::I2C_READ_COMPLETE :
                                          wHalErrorCode::I2C_WRITE_COMPLETE;
    wHalErrorCode error;
    uint32_t      retry = 0;

    do
    {
        error = w_hal_i2c_check_status(channel);

        // If the hardware has successfully finished the command, then break out
        // of the loop
        if (error == complete_error)
        {
            break;
        }

        // If we are still in progress (or if the hardware is briefly BUSY
        // setting up), wait for the designated retry period before checking
        // again
        if (retry < I2C_CMD_MAX_RETRY)
        {
            MXC_Delay(MXC_DELAY_MSEC(I2C_CMD_RETRY_MS));
        }

        retry++;
    } while (retry <= I2C_CMD_MAX_RETRY);

    if (error != complete_error)
    {
        printf("\nI2C command failed, status:%d\n", static_cast<int>(error));
        return false;
    }

    error = w_hal_i2c_clear_completion_status(channel);
    if (error != wHalErrorCode::NO_ERROR)
    {
        printf("\nI2C completion clear failed, status:%d\n",
               static_cast<int>(error));
        return false;
    }

    return true;
}

/**
 * @brief Read register from I2C command device
 *
 * @param p_device device info
 * @param reg_address register address
 * @param value read value
 * @return true if read was successful
 */
static bool read_i2c_cmd_register(const i2c_cmd_device_info_t *p_device,
                                  uint32_t                     reg_address,
                                  uint32_t &                   value)
{
    wHalErrorCode error;
    value = 0; // initialize output parameter

    if (p_device->access == i2c_cmd_access_t::SMBUS_WORD)
    {
        error = w_hal_smbus_read_word_data(p_device->bus,
                                           p_device->address,
                                           static_cast<uint16_t>(reg_address));
        if ((error != wHalErrorCode::I2C_READ_COMPLETE) &&
            (error != wHalErrorCode::I2C_READ_IN_PROGRESS))
        {
            printf("\nI2C read failed to start, status:%d\n",
                   static_cast<int>(error));
            return false;
        }

        if (!wait_i2c_cmd_complete(p_device->bus, true))
        {
            return false;
        }

        value = w_hal_smbus_get_read_result(p_device->bus);
        return true;
    }

    uint8_t reg_bytes[I2C_CMD_MAX_BYTES]  = {0}; // Buffer to hold reg address
    uint8_t data_bytes[I2C_CMD_MAX_BYTES] = {0}; // Buffer to hold read data

    for (uint32_t index = 0; index < p_device->reg_address_size_bytes; index++)
    {
        // Convert register address to bytes in big-endian order.
        reg_bytes[index] = static_cast<uint8_t>(
            reg_address >>
            (CHAR_BIT * (p_device->reg_address_size_bytes - 1 - index)));
    }

    error = w_hal_i2c_write_read_data(p_device->bus,
                                      reg_bytes,
                                      p_device->reg_address_size_bytes,
                                      data_bytes,
                                      p_device->reg_data_size_bytes,
                                      false,
                                      p_device->address,
                                      true);
    if ((error != wHalErrorCode::I2C_READ_COMPLETE) &&
        (error != wHalErrorCode::I2C_READ_IN_PROGRESS))
    {
        printf("\nI2C read failed to start, status:%d\n",
               static_cast<int>(error));
        return false;
    }

    if (!wait_i2c_cmd_complete(p_device->bus, true))
    {
        return false;
    }

    for (uint32_t index = 0; index < p_device->reg_data_size_bytes; index++)
    {
        // Combine received data bytes into a uint32_t value
        value |= static_cast<uint32_t>(data_bytes[index])
                 << (CHAR_BIT * (p_device->reg_data_size_bytes - 1 - index));
    }
    return true;
}

/**
 * @brief Write register to I2C command device
 *
 * @param p_device device info
 * @param reg_address register address
 * @param value value to write
 * @return true if write was successful
 */
static bool write_i2c_cmd_register(const i2c_cmd_device_info_t *p_device,
                                   uint32_t                     reg_address,
                                   uint32_t                     value)
{
    wHalErrorCode error;

    if (!p_device->write_allowed)
    {
        printf("\nWrite is disabled for %s\n", p_device->p_name);
        return false;
    }

    // Read before write if required by the device
    if (p_device->read_before_write)
    {
        uint32_t read_value = 0;
        if (!read_i2c_cmd_register(p_device, reg_address, read_value))
        {
            return false;
        }
    }

    if (p_device->access == i2c_cmd_access_t::SMBUS_WORD)
    {
        error = w_hal_smbus_write_word_data(p_device->bus,
                                            p_device->address,
                                            static_cast<uint16_t>(reg_address),
                                            value);
        if ((error != wHalErrorCode::I2C_WRITE_COMPLETE) &&
            (error != wHalErrorCode::I2C_WRITE_IN_PROGRESS))
        {
            printf("\nI2C write failed to start, status:%d\n",
                   static_cast<int>(error));
            return false;
        }

        return wait_i2c_cmd_complete(p_device->bus, false);
    }

    // Buffer to hold register address and data to write
    uint8_t write_bytes[I2C_CMD_MAX_BYTES + I2C_CMD_MAX_BYTES] = {0};

    for (uint32_t index = 0; index < p_device->reg_address_size_bytes; index++)
    {
        // Convert register address to bytes in big-endian order.
        write_bytes[index] = static_cast<uint8_t>(
            reg_address >>
            (CHAR_BIT * (p_device->reg_address_size_bytes - 1 - index)));
    }

    for (uint32_t index = 0; index < p_device->reg_data_size_bytes; index++)
    {
        // Convert value to bytes in big-endian order
        write_bytes[p_device->reg_address_size_bytes + index] =
            static_cast<uint8_t>(
                value >>
                (CHAR_BIT * (p_device->reg_data_size_bytes - 1 - index)));
    }

    error = w_hal_i2c_write_data(
        p_device->bus,
        write_bytes,
        static_cast<uint32_t>(p_device->reg_address_size_bytes +
                              p_device->reg_data_size_bytes),
        false,
        p_device->address,
        true);

    if ((error != wHalErrorCode::I2C_WRITE_COMPLETE) &&
        (error != wHalErrorCode::I2C_WRITE_IN_PROGRESS))
    {
        printf("\nI2C write failed to start, status:%d\n",
               static_cast<int>(error));
        return false;
    }

    return wait_i2c_cmd_complete(p_device->bus, false);
}

/**
 * @brief Set the register page for devices with paged register maps
 *
 * @param p_device device info
 * @param page page number to set
 * @return true if page was set successfully
 */
static bool set_i2c_cmd_page(const i2c_cmd_device_info_t *p_device,
                             uint8_t                      page)
{
    const uint8_t DEFAULT_PAGE_SELECT_REG = 0x00;
    // Page selection register is always register 0x00
    // Only needed for devices with more than one page
    if (p_device->page_count <= 1)
    {
        return true;
    }

    // page select address and page number
    uint8_t       write_bytes[2] = {DEFAULT_PAGE_SELECT_REG, page};
    wHalErrorCode error          = w_hal_i2c_write_data(p_device->bus,
                                               write_bytes,
                                               sizeof(write_bytes),
                                               false,
                                               p_device->address,
                                               true);
    if ((error != wHalErrorCode::I2C_WRITE_COMPLETE) &&
        (error != wHalErrorCode::I2C_WRITE_IN_PROGRESS))
    {
        printf("\nI2C page select failed, status:%d\n",
               static_cast<int>(error));
        return false;
    }

    return wait_i2c_cmd_complete(p_device->bus, false);
}

/**
 * @brief Find the known device name for i2c address
 *
 * @param bus I2C bus
 * @param address I2C address
 * @return device name if known, otherwise unknown
 */
static const char *get_i2c_device_name(uint8_t bus, uint16_t address)
{
    for (uint32_t index = 0;
         index < (sizeof(i2c_device_info) / sizeof(i2c_device_info[0]));
         index++)
    {
        if ((i2c_device_info[index].bus == bus) &&
            (i2c_device_info[index].address == address))
        {
            return i2c_device_info[index].p_name;
        }
    }

    return "Unknown device";
}

/**
 * @brief Scan one i2c bus for devices and print their addresses
 *
 * @param p_bus_info pointer to the i2c bus information structure
 * @return number of devices found on the bus
 */
static uint32_t scan_i2c_bus(const i2c_bus_info_t *p_bus_info)
{
    uint32_t       counter = 0;
    const int      bus     = MXC_I2C_GET_IDX(p_bus_info->p_i2c_regs);

    const uint8_t  I2C_SCAN_START_ADDRESS = 0x08;
    const uint8_t  I2C_SCAN_END_ADDRESS   = 0x77;
    const uint8_t  I2C_MASTER_MODE        = 1;
    const uint32_t I2C_SCAN_DELAY_MS      = 10;

    // Check if bus is idle before scanning
    wHalErrorCode  error = w_hal_i2c_check_status(static_cast<uint8_t>(bus));
    if (error != wHalErrorCode::I2C_NO_ACTIVE_OPERATION)
    {
        printf("\nI2C bus %d busy, status:%d\n", bus, static_cast<int>(error));
        return 0;
    }

    // check if the bus needs to be initialized before scanning
    if (p_bus_info->init_bus)
    {
        int err = MXC_I2C_Init(p_bus_info->p_i2c_regs, I2C_MASTER_MODE, 0);
        if (err != E_NO_ERROR)
        {
            printf("--I2C bus %d (I2C%d) Master Initialization failed, "
                   "error:%d--\n",
                   bus,
                   bus,
                   err);
            return 0;
        }

        printf("\n--I2C bus %d (I2C%d) Master Initialization Complete--\n",
               bus,
               bus);
    }

    printf("Scanning I2C bus %d (I2C%d; %s)\n",
           bus,
           bus,
           p_bus_info->p_pin_info);

    // Set frequency for scanning
    MXC_I2C_SetFrequency(p_bus_info->p_i2c_regs, I2C_SCAN_FREQUENCY);
    mxc_i2c_req_t reqMaster;
    reqMaster.i2c      = p_bus_info->p_i2c_regs;
    reqMaster.addr     = 0;
    reqMaster.tx_buf   = nullptr;
    reqMaster.tx_len   = 0;
    reqMaster.rx_buf   = nullptr;
    reqMaster.rx_len   = 0;
    reqMaster.restart  = 0;
    reqMaster.callback = nullptr;

    for (uint8_t addr = I2C_SCAN_START_ADDRESS; addr <= I2C_SCAN_END_ADDRESS;
         addr++)
    {
        reqMaster.addr = addr;
        if (MXC_I2C_MasterTransaction(&reqMaster) == 0)
        {
            printf("\n  0x%02X (%3d) - %s\n",
                   addr,
                   addr,
                   get_i2c_device_name(static_cast<uint8_t>(bus), addr));
            counter++;
        }
        MXC_Delay(MXC_DELAY_MSEC(I2C_SCAN_DELAY_MS));
    }

    printf("\nI2C bus %d (I2C%d) Scan finished. %u devices found\n",
           bus,
           bus,
           static_cast<unsigned int>(counter));

    return counter;
}

/**
 * @brief Function to scan I2C busses and print detected device addresses
 */
static void scan_i2c_busses()
{
    uint32_t total_counter = 0;
    printf("\n\n******** I2C SLAVE ADDRESS SCANNER *********\n");
    printf("\nThis command finds the addresses of any I2C Slave devices "
           "connected to I2C1 and I2C2.\n");

    for (uint32_t index = 0; index < I2C_BUS_COUNT; index++)
    {
        total_counter += scan_i2c_bus(&i2c_scan_bus_info[index]);
    }
    printf("\n******** END OF I2C SCAN *********\n");
    printf("\nTotal %u devices found in the system\n",
           static_cast<unsigned int>(total_counter));
}

/**
 * @brief Print I2C command usage
 */
static void print_i2c_cmd_usage()
{
    printf("\nUsage:\n");
    printf(" I2C SCAN\n");
    printf(" I2C READ <DEVICE> <REGISTER> [PAGE]\n");
    printf(" I2C WRITE <DEVICE> <REGISTER> [PAGE] <DATA>\n");
    printf("\nDevices:");
    uint32_t limit =
        sizeof(i2c_cmd_device_info) / sizeof(i2c_cmd_device_info[0]);
    for (uint32_t index = 0; index < limit; index++)
    {
        printf(" %s", i2c_cmd_device_info[index].p_alias);
    }
    printf("\n");
}

/**
 * @brief Find I2C command device
 *
 * @param p_alias device alias
 * @return device info if found, otherwise nullptr
 */
static const i2c_cmd_device_info_t *find_i2c_cmd_device(const char *p_alias)
{
    size_t alias_len = strlen(p_alias);
    for (uint32_t i = 0;
         i < sizeof(i2c_cmd_device_info) / sizeof(i2c_cmd_device_info[0]);
         i++)
    {
        if ((strlen(i2c_cmd_device_info[i].p_alias) == alias_len) &&
            (strncmp(i2c_cmd_device_info[i].p_alias, p_alias, alias_len) == 0))
        {
            return &i2c_cmd_device_info[i];
        }
    }
    return nullptr;
}

/**
 * @brief Find I2C command page
 *
 * @param p_alias page alias
 * @return page info if found, otherwise nullptr
 */
static const i2c_cmd_page_info_t *find_i2c_cmd_page(const char *p_alias)
{
    size_t alias_len = strlen(p_alias);
    for (uint32_t i = 0;
         i < sizeof(i2c_cmd_page_info) / sizeof(i2c_cmd_page_info[0]);
         i++)
    {
        if ((strlen(i2c_cmd_page_info[i].p_name) == alias_len) &&
            (strncmp(i2c_cmd_page_info[i].p_name, p_alias, alias_len) == 0))
        {
            return &i2c_cmd_page_info[i];
        }
    }
    return nullptr;
}

/**
 * @brief Handle I2C development command
 *
 * @param p_args arguments to the command
 * @param args_length number of characters in the arguments
 */
static void handle_i2c_command(const char *p_args, const size_t args_length)
{
    if (p_args == nullptr || args_length == 0 ||
        args_length >= MAX_UART_CMD_LEN)
    {
        print_i2c_cmd_usage();
        return;
    }

    const uint32_t READ_CMD_PREFIX_LEN  = 5; // Length of "READ "
    const uint32_t WRITE_CMD_PREFIX_LEN = 6; // Length of "WRITE "
    const size_t   DEV_NAME_BUF_SIZE =
        16; // Buffer size for device names (e.g. "SGTL", "RTC")
    const size_t PAGE_NAME_BUF_SIZE =
        8; // Buffer size for page names (e.g. "PAGE0", "PAGE1")
    const size_t PEEK_TOKEN_BUF_SIZE =
        8; // Buffer size to peek ahead to identify page fields
    const uint32_t BITS_PER_BYTE =
        8; // Bits in a single byte (for shift operations)
    const uint32_t HEX_CHARS_PER_BYTE =
        2; // Formatting: 2 hex characters per byte (e.g. "%0*X")
    const uint32_t MAX_SUPPORTED_BYTES =
        4; // Max byte width (32-bit) supported by uint32_t variables

    // Uppercase entire args for case-insensitive matching
    char args_upper[MAX_UART_CMD_LEN];
    for (size_t i = 0; i < args_length; i++)
    {
        args_upper[i] = toupper(static_cast<unsigned char>(p_args[i]));
    }
    args_upper[args_length] = '\0';

    int match_len           = 0;

    // ---------------------------------------------------------------
    // Match: SCAN
    // ---------------------------------------------------------------
    if (re_match("SCAN", args_upper, args_length, &match_len) == 0 &&
        static_cast<size_t>(match_len) == args_length)
    {
        scan_i2c_busses();
        return;
    }

    // ---------------------------------------------------------------
    // Match: READ <DEVICE> <REGISTER> [PAGE]
    // PAGE is optional - default is PAGE0 if not specified
    // Examples:
    //   I2C READ SGTL 0x1C
    //   I2C READ SGTL 0x1C PAGE1
    //   I2C READ RTC 5
    // ---------------------------------------------------------------
    match_len     = 0;
    int match_idx = re_match("READ [A-Z0-9]+ [A-Z0-9X]+",
                             args_upper,
                             args_length,
                             &match_len);
    if (match_idx == 0)
    {
        // Skip "READ "
        uint32_t offset    = READ_CMD_PREFIX_LEN;

        // Extract device name
        uint32_t dev_start = offset;
        while (offset < args_length && !isspace(args_upper[offset]))
            offset++;
        uint32_t dev_len = offset - dev_start;

        // Skip space before register address
        while (offset < args_length && isspace(args_upper[offset]))
            offset++;
        uint32_t reg_start                   = offset;

        // Copy device name into buffer
        char     dev_name[DEV_NAME_BUF_SIZE] = {0};
        strncpy(dev_name,
                &args_upper[dev_start],
                (dev_len < sizeof(dev_name)) ? dev_len : sizeof(dev_name) - 1);

        // Look up device in table
        const i2c_cmd_device_info_t *p_device = find_i2c_cmd_device(dev_name);
        if (p_device == nullptr)
        {
            printf("\nUnknown I2C device: %s\n", dev_name);
            print_i2c_cmd_usage();
            return;
        }

        // Block direct access to Fuel Gauge to avoid collision with background
        // PM task
        if (strcmp(p_device->p_alias, "BQ34110") == 0)
        {
            printf("\nAccess to Fuel Gauge registers is blocked because it is "
                   "polled in the background.\n");
            printf("Please use the BATT command to read battery status.\n");
            return;
        }

        // Parse register address - supports decimal (5) and hex (0x1C)
        char *   end_ptr = nullptr;
        uint32_t reg_address =
            static_cast<uint32_t>(strtoul(&args_upper[reg_start], &end_ptr, 0));

        if (end_ptr == &args_upper[reg_start])
        {
            printf("\nInvalid register address\n");
            return;
        }

        // Validate register address fits within the device's address width
        const uint32_t reg_addr_max =
            (p_device->reg_address_size_bytes >= MAX_SUPPORTED_BYTES) ?
                0xFFFFFFFFu :
                ((1u << (BITS_PER_BYTE * p_device->reg_address_size_bytes)) -
                 1u);
        if (reg_address > reg_addr_max)
        {
            printf("\nRegister address 0x%X out of range for %s "
                   "(max 0x%0*X for %u-byte address)\n",
                   static_cast<unsigned int>(reg_address),
                   p_device->p_name,
                   p_device->reg_address_size_bytes * HEX_CHARS_PER_BYTE,
                   static_cast<unsigned int>(reg_addr_max),
                   p_device->reg_address_size_bytes);
            return;
        }

        // Advance offset past the register token
        offset = static_cast<uint32_t>(end_ptr - args_upper);

        // Skip space after register
        while (offset < args_length && isspace(args_upper[offset]))
            offset++;

        // Default page is 0 - used if no PAGE argument is given
        uint8_t page = 0;

        // Optional PAGE argument
        if (offset < args_length)
        {
            uint32_t page_start = offset;
            while (offset < args_length && !isspace(args_upper[offset]))
                offset++;
            uint32_t page_len                      = offset - page_start;

            char     page_name[PAGE_NAME_BUF_SIZE] = {0};
            strncpy(page_name,
                    &args_upper[page_start],
                    (page_len < sizeof(page_name)) ? page_len :
                                                     sizeof(page_name) - 1);

            // Look up page name in table
            const i2c_cmd_page_info_t *p_page = find_i2c_cmd_page(page_name);
            if (p_page == nullptr)
            {
                printf("\nUnknown page: %s\n", page_name);
                return;
            }
            page = p_page->page;
        }

        // Reject any trailing garbage after valid arguments
        while (offset < args_length && isspace(args_upper[offset]))
            offset++;
        if (offset < args_length)
        {
            printf("\nUnexpected trailing arguments\n");
            print_i2c_cmd_usage();
            return;
        }

        // setting page register to read from the correct page for devices with
        // paged register maps
        if (!set_i2c_cmd_page(p_device, page))
        {
            return;
        }

        uint32_t value = 0;
        if (!read_i2c_cmd_register(p_device, reg_address, value))
        {
            return;
        }

        printf("\n%s register 0x%0*X page %d = 0x%0*X\n",
               p_device->p_name,
               p_device->reg_address_size_bytes * HEX_CHARS_PER_BYTE,
               static_cast<unsigned int>(reg_address),
               page,
               p_device->reg_data_size_bytes * HEX_CHARS_PER_BYTE,
               static_cast<unsigned int>(value));
        return;
    }

    // ---------------------------------------------------------------
    // Match: WRITE <DEVICE> <REGISTER> <DATA>
    //        WRITE <DEVICE> <REGISTER> <PAGE> <DATA>
    // PAGE is optional - default is PAGE0 if not specified
    // Examples:
    //   I2C WRITE SGTL 0x1C 0xFF
    //   I2C WRITE SGTL 0x1C PAGE1 0xFF
    //   I2C WRITE RTC 5 10
    // ---------------------------------------------------------------
    match_len = 0;
    match_idx = re_match("WRITE [A-Z0-9]+ [A-Z0-9X]+ [A-Z0-9X]+",
                         args_upper,
                         args_length,
                         &match_len);
    if (match_idx == 0)
    {
        // Skip "WRITE "
        uint32_t offset    = WRITE_CMD_PREFIX_LEN;

        // Extract device name
        uint32_t dev_start = offset;
        while (offset < args_length && !isspace(args_upper[offset]))
            offset++;
        uint32_t dev_len = offset - dev_start;

        // Skip space before register address
        while (offset < args_length && isspace(args_upper[offset]))
            offset++;
        uint32_t reg_start                   = offset;

        // Copy device name into buffer
        char     dev_name[DEV_NAME_BUF_SIZE] = {0};
        strncpy(dev_name,
                &args_upper[dev_start],
                (dev_len < sizeof(dev_name)) ? dev_len : sizeof(dev_name) - 1);

        // Look up device in table
        const i2c_cmd_device_info_t *p_device = find_i2c_cmd_device(dev_name);
        if (p_device == nullptr)
        {
            printf("\nUnknown I2C device: %s\n", dev_name);
            print_i2c_cmd_usage();
            return;
        }

        // Block direct access to Fuel Gauge to avoid collision with background
        // PM task
        if (strcmp(p_device->p_alias, "BQ34110") == 0)
        {
            printf("\nAccess to Fuel Gauge registers is blocked because it is "
                   "polled in the background.\n");
            printf("Please use the BATT command to read battery status.\n");
            return;
        }

        // Parse register address - supports decimal (5) and hex (0x1C)
        char *   end_ptr = nullptr;
        uint32_t reg_address =
            static_cast<uint32_t>(strtoul(&args_upper[reg_start], &end_ptr, 0));

        if (end_ptr == &args_upper[reg_start])
        {
            printf("\nInvalid register address\n");
            return;
        }

        // Validate register address fits within the device's address width
        const uint32_t reg_addr_max =
            (p_device->reg_address_size_bytes >= MAX_SUPPORTED_BYTES) ?
                0xFFFFFFFFu :
                ((1u << (BITS_PER_BYTE * p_device->reg_address_size_bytes)) -
                 1u);
        if (reg_address > reg_addr_max)
        {
            printf("\nRegister address 0x%X out of range for %s "
                   "(max 0x%0*X for %u-byte address)\n",
                   static_cast<unsigned int>(reg_address),
                   p_device->p_name,
                   p_device->reg_address_size_bytes * HEX_CHARS_PER_BYTE,
                   static_cast<unsigned int>(reg_addr_max),
                   p_device->reg_address_size_bytes);
            return;
        }

        // Advance offset past the register token
        offset = static_cast<uint32_t>(end_ptr - args_upper);

        // Skip space after register
        while (offset < args_length && isspace(args_upper[offset]))
            offset++;

        // Default page is 0 - check if next token is a PAGE name or data value
        uint8_t  page       = 0;
        uint32_t data_start = offset;

        // Peek at next token to decide if it is a PAGE or data
        uint32_t peek_start = offset;
        while (offset < args_length && !isspace(args_upper[offset]))
            offset++;
        uint32_t peek_len                        = offset - peek_start;

        char     peek_token[PEEK_TOKEN_BUF_SIZE] = {0};
        strncpy(peek_token,
                &args_upper[peek_start],
                (peek_len < sizeof(peek_token)) ? peek_len :
                                                  sizeof(peek_token) - 1);

        // Try to match peek token against known page names
        const i2c_cmd_page_info_t *p_page = find_i2c_cmd_page(peek_token);
        if (p_page != nullptr)
        {
            // Token is a valid page name - consume it and move to data
            page = p_page->page;

            // Skip space after page name
            while (offset < args_length && isspace(args_upper[offset]))
                offset++;
            data_start = offset;
        }
        else
        {
            // Token is not a page name - treat it as the data value
            data_start = peek_start;
            offset     = peek_start;
        }

        // Parse write data - supports decimal and hex
        uint32_t value = static_cast<uint32_t>(
            strtoul(&args_upper[data_start], &end_ptr, 0));

        if (end_ptr == &args_upper[data_start])
        {
            printf("\nInvalid write data\n");
            return;
        }

        // Validate write data fits within the device's data width
        const uint32_t data_max =
            (p_device->reg_data_size_bytes >= MAX_SUPPORTED_BYTES) ?
                0xFFFFFFFFu :
                ((1u << (BITS_PER_BYTE * p_device->reg_data_size_bytes)) - 1u);
        if (value > data_max)
        {
            printf("\nWrite data 0x%X out of range for %s "
                   "(max 0x%0*X for %u-byte data)\n",
                   static_cast<unsigned int>(value),
                   p_device->p_name,
                   p_device->reg_data_size_bytes * HEX_CHARS_PER_BYTE,
                   static_cast<unsigned int>(data_max),
                   p_device->reg_data_size_bytes);
            return;
        }

        // Advance offset past data token
        offset = static_cast<uint32_t>(end_ptr - args_upper);

        // Reject any trailing garbage after valid arguments
        while (offset < args_length && isspace(args_upper[offset]))
            offset++;
        if (offset < args_length)
        {
            printf("\nUnexpected trailing arguments\n");
            print_i2c_cmd_usage();
            return;
        }

        // setting page register to write to the correct page for devices with
        // paged register maps
        if (!set_i2c_cmd_page(p_device, page))
        {
            return;
        }

        if (!write_i2c_cmd_register(p_device, reg_address, value))
        {
            return;
        }

        printf("\n%s register 0x%0*X page %d written with 0x%0*X\n",
               p_device->p_name,
               p_device->reg_address_size_bytes * HEX_CHARS_PER_BYTE,
               static_cast<unsigned int>(reg_address),
               page,
               p_device->reg_data_size_bytes * HEX_CHARS_PER_BYTE,
               static_cast<unsigned int>(value));
        return;
    }

    print_i2c_cmd_usage();
}

#if RTC_CORE != INVALID_CORE_NUM
/**
 * @brief Convert decimal value to BCD (Binary-Coded Decimal)
 *
 * @param val
 * @return uint8_t
 */
static inline uint8_t dec_to_bcd(uint8_t val)
{
    return static_cast<uint8_t>(((val / 10) << 4) | (val % 10));
}

/**
 * @brief Configure test alarm sequence (GPIO 1.7 + RTC writes)
 *
 * @param p_args arguments to the command
 * @param args_length  number of characters in the arguments
 */
static void test_alarm_cmd(const char *p_args, const size_t args_length)
{
    const i2c_cmd_device_info_t *p_device = find_i2c_cmd_device("RTC");
    if (p_device == nullptr)
    {
        printf("\nRTC device not found\n");
        return;
    }

    const uint8_t DELAY_FOR_PIN_LOW_MS =
        100; // Delay for GPIO pin to go low after alarm trigger
    time_t      start_time_utc       = 0;
    int         time_zone_offset     = 0;
    int         alarm_offset_minutes = 2; // Default alarm offset is 2 minutes

    // Trim leading whitespace in arguments
    const char *p_trimmed_args       = p_args;
    size_t      trimmed_len          = args_length;
    while ((trimmed_len > 0) && isspace(p_trimmed_args[0]))
    {
        p_trimmed_args++;
        trimmed_len--;
    }

    if (trimmed_len > 0)
    {
        // Parse format "<Date> <Time> <minutesofAlarm>"
        // Date pattern: \d\d\d\d[/-]?\d\d[/-]?\d\d
        // Time pattern: \d\d:?\d\d:?\d\d
        // Minutes of Alarm pattern: \d+

        int date_match_len = 0;
        int date_match_idx = re_match("\\d\\d\\d\\d[\\-/]?\\d\\d[\\-/]?\\d\\d",
                                      p_trimmed_args,
                                      static_cast<int>(trimmed_len),
                                      &date_match_len);

        if (date_match_idx != 0)
        {
            printf(
                "\nUsage: TEST_ALARM <Date> <Time> <minutesofAlarm>\n"
                "Example: TEST_ALARM 2026-06-18 12:40:00 2\n"
                "Or run TEST_ALARM with no arguments for default settings.\n");
            return;
        }

        const char *p_after_date = p_trimmed_args + date_match_len;
        const char *p_args_end   = p_trimmed_args + trimmed_len;

        // Skip space(s) between Date and Time
        while ((p_after_date < p_args_end) && isspace(*p_after_date))
        {
            p_after_date++;
        }

        int time_match_len = 0;
        int time_match_idx =
            re_match("\\d\\d:?\\d\\d:?\\d\\d",
                     p_after_date,
                     static_cast<int>(p_args_end - p_after_date),
                     &time_match_len);

        if (time_match_idx != 0)
        {
            printf("\nUsage: TEST_ALARM <Date> <Time> <minutesofAlarm>\n"
                   "Example: TEST_ALARM 2026-06-18 12:40:00 2\n");
            return;
        }

        const char *p_after_time = p_after_date + time_match_len;

        // Skip space(s) between Time and Minutes
        while ((p_after_time < p_args_end) && isspace(*p_after_time))
        {
            p_after_time++;
        }

        int minutes_match_len = 0;
        int minutes_match_idx =
            re_match("\\d+",
                     p_after_time,
                     static_cast<int>(p_args_end - p_after_time),
                     &minutes_match_len);

        if (minutes_match_idx != 0)
        {
            printf("\nUsage: TEST_ALARM <Date> <Time> <minutesofAlarm>\n"
                   "Example: TEST_ALARM 2026-06-18 12:40:00 2\n");
            return;
        }

        // Check for any extra trailing characters after the minutes parameter
        const char *p_minutes_end = p_after_time + minutes_match_len;
        while ((p_minutes_end < p_args_end) && isspace(*p_minutes_end))
        {
            p_minutes_end++;
        }

        if (p_minutes_end != p_args_end)
        {
            printf("\nUsage: TEST_ALARM <Date> <Time> <minutesofAlarm>\n"
                   "Example: TEST_ALARM 2026-06-18 12:40:00 2\n");
            return;
        }

        // Combine date and time to a temporary buffer for extraction
        char datetime_temp_buf[64];
        snprintf(datetime_temp_buf,
                 sizeof(datetime_temp_buf),
                 "%.*s %.*s",
                 date_match_len,
                 p_trimmed_args,
                 time_match_len,
                 p_after_date);

        // Extract the datetime using the existing function
        bool success = extract_time_from_datetime_string(
            datetime_temp_buf,
            static_cast<uint32_t>(date_match_len + 1 + time_match_len),
            start_time_utc,
            time_zone_offset);

        if (!success)
        {
            printf("\nInvalid date time format: %s\n", datetime_temp_buf);
            return;
        }

        // Convert input local time to UTC using the system's configured
        // timezone offset
        start_time_utc -= DeviceConfig::p_page->time_zone_offset;

        alarm_offset_minutes = natoi(p_after_time, minutes_match_len);
    }
    else
    {
        // 0 arguments: Default alarm set after 1 minute from the time of
        // entering the command
        w_hal_rtc_time_st current_time;
        w_hal_rtc_get_time(current_time);

        start_time_utc   = current_time.time_seconds + 60; // 1 minute buffer
        time_zone_offset = DeviceConfig::p_page->time_zone_offset;
        alarm_offset_minutes = 2; // alarm duration 2 minutes after start time
    }

    struct tm start_tm;
    gmtime_r(&start_time_utc, &start_tm);

    time_t    alarm_time_utc = start_time_utc + (alarm_offset_minutes * 60);
    struct tm alarm_tm;
    gmtime_r(&alarm_time_utc, &alarm_tm);

    char start_time_buf[32];
    char alarm_time_buf[32];
    // Format the start and alarm times into human-readable strings
    // Added 1900 to tm_year and 1 to tm_mon because struct tm represents years
    // since 1900 and months from 0-11
    snprintf(start_time_buf,
             sizeof(start_time_buf),
             "%04d-%02d-%02d %02d:%02d:%02d",
             start_tm.tm_year + 1900,
             start_tm.tm_mon + 1,
             start_tm.tm_mday,
             start_tm.tm_hour,
             start_tm.tm_min,
             start_tm.tm_sec);
    snprintf(alarm_time_buf,
             sizeof(alarm_time_buf),
             "%04d-%02d-%02d %02d:%02d:%02d",
             alarm_tm.tm_year + 1900,
             alarm_tm.tm_mon + 1,
             alarm_tm.tm_mday,
             alarm_tm.tm_hour,
             alarm_tm.tm_min,
             alarm_tm.tm_sec);

    printf("\nSetting test alarm sequence:\n"
           "  Start Time (UTC): %s\n"
           "  Alarm Time (UTC): %s (offset: %d mins)\n",
           start_time_buf,
           alarm_time_buf,
           alarm_offset_minutes);

    // MCP7940 register addresses
    const uint32_t MCP7940_REG_RTCSEC    = 0x00;
    const uint32_t MCP7940_REG_RTCMIN    = 0x01;
    const uint32_t MCP7940_REG_RTCHOUR   = 0x02;
    const uint32_t MCP7940_REG_RTCWKDAY  = 0x03;
    const uint32_t MCP7940_REG_RTCDATE   = 0x04;
    const uint32_t MCP7940_REG_RTCMTH    = 0x05;
    const uint32_t MCP7940_REG_RTCYEAR   = 0x06;
    const uint32_t MCP7940_REG_CONTROL   = 0x07;
    const uint32_t MCP7940_REG_ALM0SEC   = 0x0A;
    const uint32_t MCP7940_REG_ALM0MIN   = 0x0B;
    const uint32_t MCP7940_REG_ALM0HOUR  = 0x0C;
    const uint32_t MCP7940_REG_ALM0WKDAY = 0x0D;
    const uint32_t MCP7940_REG_ALM0DATE  = 0x0E;
    const uint32_t MCP7940_REG_ALM0MTH   = 0x0F;

    // MCP7940 configuration bit masks
    const uint8_t  MCP7940_VBATEN        = 0x08; // Battery Backup Enable bit
    const uint8_t  MCP7940_ALMPOL_HIGH = 0x80; // Alarm active-high polarity bit
    const uint8_t  MCP7940_ALM0MSK_ALL =
        0x70; // Alarm mask: match sec, min, hr, wday, date, mth (111)
    const uint8_t MCP7940_ALM0_ENABLE =
        0x10; // Control register bit to enable Alarm 0 output pin
    const uint8_t MCP7940_START_OSCILLATOR =
        0x80; // Bit to start oscillator (ST = 1)

    // Prepare RTC register values in BCD format
    uint8_t rtc_vals[7] = {
        dec_to_bcd(
            start_tm.tm_sec), // Seconds (Bit 7 is ST, will write 0 to stop)
        dec_to_bcd(start_tm.tm_min),  // Minutes
        dec_to_bcd(start_tm.tm_hour), // Hours
        static_cast<uint8_t>(MCP7940_VBATEN | ((start_tm.tm_wday + 1) &
                                               0x07)), // Weekday and VBATEN
        dec_to_bcd(start_tm.tm_mday),                  // Date
        dec_to_bcd(start_tm.tm_mon + 1),               // Month
        dec_to_bcd((start_tm.tm_year + 1900) % 100)    // Year
    };

    // Prepare Alarm 0 register values in BCD format
    uint8_t alarm_vals[6] = {
        dec_to_bcd(alarm_tm.tm_sec),  // Alarm 0 Seconds
        dec_to_bcd(alarm_tm.tm_min),  // Alarm 0 Minutes
        dec_to_bcd(alarm_tm.tm_hour), // Alarm 0 Hours
        // Alarm 0 Weekday / Configuration register (0x0D)
        // Bit 7: ALMPOL = 1 (Active-High polarity)
        // Bit 6-4: ALM0MSK = 111 (All registers match)
        // Bit 3: ALM0IF = 0 (Clear Alarm 0 interrupt flag)
        // Bit 2-0: WKDAY (Weekday, 1-7)
        static_cast<uint8_t>(MCP7940_ALMPOL_HIGH | MCP7940_ALM0MSK_ALL |
                             ((alarm_tm.tm_wday + 1) & 0x07)),
        dec_to_bcd(alarm_tm.tm_mday),   // Alarm 0 Date
        dec_to_bcd(alarm_tm.tm_mon + 1) // Alarm 0 Month
    };

    // Configure GPIO Port 1, Pin 7 as digital output (VDDIOH) and set it HIGH
    // to keep SoC powered
    mxc_gpio_cfg_t gpio_1_7;
    gpio_1_7.port  = MXC_GPIO1;
    gpio_1_7.mask  = MXC_GPIO_PIN_7;
    gpio_1_7.func  = MXC_GPIO_FUNC_OUT;
    gpio_1_7.pad   = MXC_GPIO_PAD_NONE;
    gpio_1_7.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&gpio_1_7);

    MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_7);
    printf("GPIO 1.7 set HIGH (securing SoC power during configuration)\n");

    // Establish I2C1 bus ownership via cross-core semaphore
    wHalErrorCode i2c_err =
        w_hal_i2c_init(p_device->bus, whal_i2c_mode::MASTER, 100000);
    if (i2c_err != wHalErrorCode::NO_ERROR)
    {
        printf("TEST_ALARM failed: could not acquire I2C bus (error %d)\n",
               static_cast<int>(i2c_err));
        printf("Power-down aborted. SoC remains powered.\n");
        return;
    }

    // Helper lambda to write a register and abort immediately on failure
    auto write_reg_checked =
        [&](uint32_t reg, uint32_t val, const char *reg_name) -> bool
    {
        if (!write_i2c_cmd_register(p_device, reg, val))
        {
            printf("TEST_ALARM write failed at %s (reg 0x%02lX, val 0x%02lX)\n",
                   reg_name,
                   (unsigned long)reg,
                   (unsigned long)val);
            return false;
        }
        return true;
    };

    bool write_ok = true;

    // Step 1: Stop oscillator before updating time registers
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_RTCSEC, rtc_vals[0], "RTCSEC (ST=0)");
    if (write_ok)
        write_ok = write_reg_checked(MCP7940_REG_RTCMIN, rtc_vals[1], "RTCMIN");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_RTCHOUR, rtc_vals[2], "RTCHOUR");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_RTCWKDAY, rtc_vals[3], "RTCWKDAY");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_RTCDATE, rtc_vals[4], "RTCDATE");
    if (write_ok)
        write_ok = write_reg_checked(MCP7940_REG_RTCMTH, rtc_vals[5], "RTCMTH");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_RTCYEAR, rtc_vals[6], "RTCYEAR");

    // Step 2: Write Alarm 0 registers (only if time writes succeeded)
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_ALM0SEC, alarm_vals[0], "ALM0SEC");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_ALM0MIN, alarm_vals[1], "ALM0MIN");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_ALM0HOUR, alarm_vals[2], "ALM0HOUR");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_ALM0DATE, alarm_vals[4], "ALM0DATE");
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_ALM0MTH, alarm_vals[5], "ALM0MTH");
    if (write_ok)
        write_ok = write_reg_checked(MCP7940_REG_ALM0WKDAY,
                                     alarm_vals[3],
                                     "ALM0WKDAY (Polarity/Mask)");

    // Step 3: Enable Alarm 0 output pin in CONTROL register (ONLY if all alarm
    // values written)
    if (write_ok)
        write_ok = write_reg_checked(MCP7940_REG_CONTROL,
                                     MCP7940_ALM0_ENABLE,
                                     "CONTROL (ALM0EN=1)");

    // Step 4: Restart oscillator (ST = 1) preserving current second
    if (write_ok)
        write_ok =
            write_reg_checked(MCP7940_REG_RTCSEC,
                              MCP7940_START_OSCILLATOR | (rtc_vals[0] & 0x7F),
                              "RTCSEC (ST=1)");

    // Step 5: Read back and verify all programmed registers
    bool readback_ok = false;
    if (write_ok)
    {
        uint32_t rb_sec = 0, rb_min = 0, rb_hour = 0, rb_wkday = 0;
        uint32_t rb_date = 0, rb_mth = 0, rb_ctrl = 0, rb_rtc_sec = 0;

        bool     rb_ok = true;
        rb_ok &= read_i2c_cmd_register(p_device, MCP7940_REG_ALM0SEC, rb_sec);
        rb_ok &= read_i2c_cmd_register(p_device, MCP7940_REG_ALM0MIN, rb_min);
        rb_ok &= read_i2c_cmd_register(p_device, MCP7940_REG_ALM0HOUR, rb_hour);
        rb_ok &= read_i2c_cmd_register(p_device, MCP7940_REG_ALM0DATE, rb_date);
        rb_ok &= read_i2c_cmd_register(p_device, MCP7940_REG_ALM0MTH, rb_mth);
        rb_ok &=
            read_i2c_cmd_register(p_device, MCP7940_REG_ALM0WKDAY, rb_wkday);
        rb_ok &= read_i2c_cmd_register(p_device, MCP7940_REG_CONTROL, rb_ctrl);
        rb_ok &=
            read_i2c_cmd_register(p_device, MCP7940_REG_RTCSEC, rb_rtc_sec);

        if (!rb_ok)
        {
            printf("TEST_ALARM failed: I2C readback error\n");
        }
        else
        {
            printf("Alarm 0 readback: SEC=0x%02lX, MIN=0x%02lX, HOUR=0x%02lX, "
                   "DATE=0x%02lX, MTH=0x%02lX, WKDAY=0x%02lX, CTRL=0x%02lX\n",
                   (unsigned long)rb_sec,
                   (unsigned long)rb_min,
                   (unsigned long)rb_hour,
                   (unsigned long)rb_date,
                   (unsigned long)rb_mth,
                   (unsigned long)rb_wkday,
                   (unsigned long)rb_ctrl);

            bool match =
                (rb_sec == alarm_vals[0]) && (rb_min == alarm_vals[1]) &&
                (rb_hour == alarm_vals[2]) && (rb_date == alarm_vals[4]) &&
                (rb_mth == alarm_vals[5]) && (rb_wkday == alarm_vals[3]) &&
                ((rb_ctrl & MCP7940_ALM0_ENABLE) != 0) &&
                ((rb_rtc_sec & MCP7940_START_OSCILLATOR) != 0);

            if (match)
            {
                readback_ok = true;
            }
            else
            {
                printf("TEST_ALARM readback verification MISMATCH!\n");
                if (rb_sec != alarm_vals[0])
                    printf("  ALM0SEC: expected 0x%02X, got 0x%02lX\n",
                           alarm_vals[0],
                           (unsigned long)rb_sec);
                if (rb_min != alarm_vals[1])
                    printf("  ALM0MIN: expected 0x%02X, got 0x%02lX\n",
                           alarm_vals[1],
                           (unsigned long)rb_min);
                if (rb_hour != alarm_vals[2])
                    printf("  ALM0HOUR: expected 0x%02X, got 0x%02lX\n",
                           alarm_vals[2],
                           (unsigned long)rb_hour);
                if (rb_date != alarm_vals[4])
                    printf("  ALM0DATE: expected 0x%02X, got 0x%02lX\n",
                           alarm_vals[4],
                           (unsigned long)rb_date);
                if (rb_mth != alarm_vals[5])
                    printf("  ALM0MTH: expected 0x%02X, got 0x%02lX\n",
                           alarm_vals[5],
                           (unsigned long)rb_mth);
                if (rb_wkday != alarm_vals[3])
                    printf("  ALM0WKDAY: expected 0x%02X, got 0x%02lX\n",
                           alarm_vals[3],
                           (unsigned long)rb_wkday);
                if ((rb_ctrl & MCP7940_ALM0_ENABLE) == 0)
                    printf("  CONTROL: ALM0EN not set (got 0x%02lX)\n",
                           (unsigned long)rb_ctrl);
                if ((rb_rtc_sec & MCP7940_START_OSCILLATOR) == 0)
                    printf("  RTCSEC: ST not set (got 0x%02lX)\n",
                           (unsigned long)rb_rtc_sec);
            }
        }
    }

    // Release I2C1 bus before power-down or exit
    w_hal_i2c_teardown(p_device->bus);

    if (!write_ok || !readback_ok)
    {
        printf("TEST_ALARM sequence FAILED — aborting power-down (SoC remains "
               "powered)\n");
        return;
    }

    printf("TEST_ALARM sequence completed successfully\n");
    MXC_Delay(MXC_DELAY_MSEC(DELAY_FOR_PIN_LOW_MS));
    printf("GPIO 1.7 set LOW\n");
    // Make GPIO 1.7 low again
    MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_7);
}
#endif // RTC_CORE != INVALID_CORE_NUM

/**
 * @brief Handling of full command string
 *
 * @param p_cmd command string
 * @param length length of the command string
 */
void new_console_command_handler(const char *p_cmd, uint32_t length)
{
    // Skip leading spaces
    while ((length > 0) && isspace(p_cmd[0]))
    {
        p_cmd++;
        length--;
    }

    // Ignore trailing spaces
    while ((length > 0) && isspace(p_cmd[length - 1]))
    {
        length--;
    }

    uint32_t             arg_offset = 0;
    console_config_cmd_t val =
        console_cmd_lookup_func(p_cmd, length, arg_offset);
    SYSTEM_ASSERT_LTEQ(arg_offset, length);

    const char * p_args      = &p_cmd[arg_offset];
    const size_t args_length = length - arg_offset;

    switch (val)
    {
        // HELP command displays all the commands and arguments
        case console_config_cmd_t::HELP:
            display_console_help();
            break;

        // Command to Read serial number
        case console_config_cmd_t::SERIAL_NUMBER:
            set_serial_number(p_args, args_length);
            break;

        case console_config_cmd_t::DEVICE_NAME:
            set_device_name(p_args, args_length);
            break;

        case console_config_cmd_t::PASSWORD:
            set_device_password(p_args, args_length);
            break;

        case console_config_cmd_t::SMS_KEYWORD:
            set_sms_keyword(p_args, args_length);
            break;

        case console_config_cmd_t::SMS_KEY:
            set_sms_key(p_args, args_length);
            break;

#if COMM_CORE != INVALID_CORE_NUM
        case console_config_cmd_t::A2P_PHONE_NUMBER:
            set_a2p_phone_number(p_args, args_length);
            break;

        case console_config_cmd_t::P2A_PHONE_NUMBER:
            set_p2a_phone_number(p_args, args_length);
            break;

        case console_config_cmd_t::SERVER_URL:

            set_server_url(p_args, args_length);
            break;

        case console_config_cmd_t::SERVER_PORT:
            set_server_port(p_args, args_length);
            break;

        case console_config_cmd_t::SERVER_POST_ROUTE:
            set_server_post_route(p_args, args_length);
            break;

        case console_config_cmd_t::SERVER_POST_CONTENT_TYPE:
            set_post_route_content_type(p_args, args_length);
            break;

        case console_config_cmd_t::SERVER_TEST_SETTING:
            set_server_test_settings();
            break;

        case console_config_cmd_t::SERVER_WILDSONIC_SETTING:
            set_server_wildsonic_settings();
            break;

        case console_config_cmd_t::GSMMODE:
            set_gsm_transmission_mode(p_args, args_length);
            break;

        case console_config_cmd_t::SMS_FREQ:
            set_sms_upload_frequency(p_args, args_length);
            break;

        case console_config_cmd_t::SIGNAL_QUALITY:
            get_gsm_signal_quality();
            break;

        case console_config_cmd_t::SEND_STATUS:
            send_status_now();
            break;
#endif

#if AUDIO_CORE != INVALID_CORE_NUM
        // Command to Read recording sampling frequency
        case console_config_cmd_t::SAMF:
            set_sampling_frequency(p_args, args_length);
            break;

        case console_config_cmd_t::BTPS:
            set_sample_width(p_args, args_length);
            break;

        case console_config_cmd_t::NUMCH:
            set_num_audio_channels(p_args, args_length);
            break;

        case console_config_cmd_t::AUDIO:
            set_audio_parameters(p_args, args_length);
            break;

        case console_config_cmd_t::GAIN:
            set_audio_gain(p_args, args_length);
            break;
        case console_config_cmd_t::LOWER_CUTOFF:
            set_lower_cutoff_frequency(p_args, args_length);
            break;
        case console_config_cmd_t::UPPER_CUTOFF:
            set_upper_cutoff_frequency(p_args, args_length);
            break;
#endif

#if STORAGE_CORE != INVALID_CORE_NUM
        // Command to Read recording time length
        case console_config_cmd_t::RECT:
            set_recording_length(p_args, args_length);
            break;

        // Command to Read recording sleep time
        case console_config_cmd_t::SLEEP:
            set_recording_sleep_time(p_args, args_length);
            break;

        // Command to Read/Set daily folder structure flag
        case console_config_cmd_t::FOLDER_STRUCTURE:
            set_folder_structure(p_args, args_length);
            break;

        // Command to Read recording start and end dates
        case console_config_cmd_t::DATE_RANGE:
            set_begin_end_date(p_args, args_length);
            break;

        // Command to Read recording start and end times
        case console_config_cmd_t::TIME_OF_DAY:
            set_begin_end_time(p_args, args_length);
            break;

        // Command to Read recording days of week
        case console_config_cmd_t::DAYS_OF_WEEK:
            set_recording_days_of_week(p_args, args_length);
            break;

        // Command to get battery status information
        case console_config_cmd_t::STORAGE_FILLED:
            get_sd_card_storage_status();
            break;
#endif

#if (BLE_CORE != INVALID_CORE_NUM) && !defined(BLE_CORE_CUSTOM)
        case console_config_cmd_t::BLEPAIR:
            set_ble_pairing_window(p_args, args_length);
            break;
#endif

        // Command to set lower battery voltage cutoff
        case console_config_cmd_t::LOW_BAT:
            set_low_battery_voltage_cutoff(p_args, args_length);
            break;

        // Command to set number of battery cells used
        case console_config_cmd_t::NUM_BAT:
            set_number_of_cells_used(p_args, args_length);
            break;

#if PM_CORE != INVALID_CORE_NUM
        // Command to program fuel gauge full charge capacity
        case console_config_cmd_t::FULL_CAP:
            set_fuel_gauge_full_charge_capacity(p_args, args_length);
            break;
#endif

        // Command to read or set system time
        case console_config_cmd_t::DATETIME:
            set_start_datetime(p_args, args_length);
            break;

        // Commands to read or set the time zone
        case console_config_cmd_t::TIMEZONE:
            set_time_zone(p_args, args_length);
            break;

        // Unknown command sequence
        case console_config_cmd_t::NUM_CMDS:
            display_console_commands();
            break;

        // Command to apply settings
        case console_config_cmd_t::APPLY:
            apply_settings();
            break;

        // Command to reboot
        case console_config_cmd_t::REBOOT:
            reboot();
            break;

        // Command to apply settings and reboot
        case console_config_cmd_t::APPLY_REBOOT:
            apply_and_reboot();
            break;

        // Command to format the SD card
        case console_config_cmd_t::SD_FORMAT:
            sd_card_format(p_args, args_length);
            break;

        // Command to get assert information
        case console_config_cmd_t::ASSERT:
            assert_info();
            break;

        // Command to get battery status information
        case console_config_cmd_t::BATTERY_STATUS:
            get_battery_status();
            break;

        // Command to get version information
        case console_config_cmd_t::VERSION:
            fw_version_info();
            break;

        case console_config_cmd_t::ENVIRONMENT_STATUS:
            get_environment_status();
            break;

        case console_config_cmd_t::LOCATION:
            set_location_info(p_args, args_length);
            break;

        // Command to set sunrise/sunset based recording information
        case console_config_cmd_t::SUN:
            set_sunrise_sunset_info(p_args, args_length);
            break;

        case console_config_cmd_t::SUN_RECORDING_MODE:
            set_sun_recording_mode(p_args, args_length);
            break;

        case console_config_cmd_t::SUNRISE_INTERVAL_BEFORE:
            set_unsigned_value(p_args,
                               args_length,
                               DeviceConfig::p_page->sunrise_interval_before);
            break;

        case console_config_cmd_t::SUNRISE_INTERVAL_AFTER:
            set_unsigned_value(p_args,
                               args_length,
                               DeviceConfig::p_page->sunrise_interval_after);
            break;

        case console_config_cmd_t::SUNSET_INTERVAL_BEFORE:
            set_unsigned_value(p_args,
                               args_length,
                               DeviceConfig::p_page->sunset_interval_before);
            break;

        case console_config_cmd_t::SUNSET_INTERVAL_AFTER:
            set_unsigned_value(p_args,
                               args_length,
                               DeviceConfig::p_page->sunset_interval_after);
            break;

        case console_config_cmd_t::SUNRISE_SUNSET_ROUNDING:
            set_unsigned_value(p_args,
                               args_length,
                               DeviceConfig::p_page->sunrise_sunset_rounding);
            break;
        // Command to set default configuration
        case console_config_cmd_t::DEFAULT:
            set_default_config();
            break;

        case console_config_cmd_t::I2C:
            handle_i2c_command(p_args, args_length);
            break;

#if RTC_CORE != INVALID_CORE_NUM
        // Command to reset RTC time to 2025-01-01 00:00:00 UTC
        case console_config_cmd_t::RTC_RESET:
            reset_rtc_time();
            break;

        case console_config_cmd_t::TEST_ALARM:
            test_alarm_cmd(p_args, args_length);
            break;
#endif
        // Ignore this command
        case console_config_cmd_t::WILDLY:
            break;

        // Unknown command sequence
        default:
            printf("\nUnsupported command : %s\n", uart_console_cmd);
            break;
    }
}

/**
 * @brief Routine to handle the character returned from the console, form a
 * command and process
 *
 * @param c
 */
static void new_character_handler(char c)
{

    // Backspace clears the last entered character
    if (c == '\b')
    {
        if (uart_char_cnt != 0)
        {
            printf("\b \b");
            uart_char_cnt--;

            // Backspace into the command name means we are back to making
            // characters upper case. Set the command length back to zero
            // since the command name is being modified.
            if (uart_char_cnt <= uart_cmd_len)
            {
                uart_cmd_len = 0;
            }
        }
    }
    // Clear console buffer & counter if 'Escape' key is pressed
    else if ((c == 0x1B))
    {
        printf("\nClearing command cache of %d chars\n", uart_char_cnt);
        reset_console_cmd();
        printf(">"); // Move to new line
    }
    // Process command once character limit is reached or return character
    // is entered
    else if ((c == '\r') || (uart_char_cnt == MAX_UART_CMD_LEN - 1))
    {
        printf("\n");
        uart_console_cmd[uart_char_cnt] = '\0';

        new_console_command_handler(uart_console_cmd, uart_char_cnt);

        // Reset character count and buffer
        reset_console_cmd();
        printf(">");
    }
    // Accumulate characters input to form the command
    else if ((uart_char_cnt < MAX_UART_CMD_LEN))
    {
        // Command length is initialized to zero. When a space is observed,
        // it means that the command name has been entered. So, we set the
        // command length to the current character count.
        if (isspace(c) && (uart_cmd_len == 0))
        {
            uart_cmd_len = uart_char_cnt;
        }

        // Ignore spaces at the beginning of the command
        if ((uart_char_cnt > 0) || !isspace(c))
        {
            // Only command should be forced to upper case. Command length of
            // zero means we are still entering the command name, in which case
            // convert the input character to upper case.
            if (uart_cmd_len == 0)
            {
                c = toupper(c);
            }

            printf("%c", c);
            uart_console_cmd[uart_char_cnt++] = c;
        }
    }
}

#endif // __CORE__ == CONSOLE_CORE
