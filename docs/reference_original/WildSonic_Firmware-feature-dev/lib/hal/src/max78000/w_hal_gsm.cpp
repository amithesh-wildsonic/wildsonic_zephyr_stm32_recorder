#if COMM_CORE != INVALID_CORE_NUM

#include "w_hal_gsm.hpp"
#include "w_hal_error_code.hpp" //Header file to enum of errors
#include "w_hal_gsm_internal.hpp"

#include "constants.hpp"
#include "trace.hpp"

#include "re.h"

#if (SIMCOM)
static const uint32_t BAUD_RATE = 115200; // Baud rate to the
                                          // SIMCOM module
#else
static const uint32_t BAUD_RATE = 3000000; // Baud rate to the
                                           // ublox module
#endif
static const uint32_t BAUD_MARGIN_FACTOR  = 9; // Factor specifying
                                               // how much margin is
                                               // needed to keep up
                                               // with rate of data
                                               // that needs to be
                                               // transferred.
static const uint32_t BAUD_MARGIN_DIVISOR = 5; // Divisor specifying
                                               // how much margin is
                                               // needed to keep up
                                               // with rate of data
                                               // that needs to be
                                               // transferred.

#if __CORE__ == COMM_CORE

#include "w_hal_rtc.hpp"
#include "w_hal_system.hpp"
#include "w_hal_uart.hpp"

#include "assert.hpp"

#include <algorithm>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define HAL_GSM_AT_DEBUG 0

static const uint32_t INITIAL_BAUD_RATE  = 115200; // Initial Baud rate to the
                                                   // GSM module
static const uint32_t SARA_MAX_BAUD_RATE = 921600; // Baud rate to use only
                                                   // for SARA ublox module
static const char     ASCII_MIN          = 1;      // Minimum ASCII
                                                   // character
static const char     ASCII_MAX          = 127;    // Maximum ASCII
                                                   // character

static const int      GSM_COMMAND_TIMEOUT_SECONDS = 120; // Timeout time for
                                                         // GSM commands

static const int      MAX_WAIT_FOR_OPERATOR_NAME_SECONDS = 60; // Timeout time
                                                               // for waiting
                                                               // for operator
                                                               // name
static const int      MAX_REGISTRATION_TIME_SECONDS      = 20; // Timeout time
                                                               // for GSM
                                                               // registration
static const int      REATTACH_WAIT_TIME_SECONDS         = 3;  // Time to wait
                                                               // before
                                                               // reattaching
                                                               // to network
                                                               // after
                                                               // receiving URC

static const int      INVALID_SMS_INDEX = -1;  // Invalid SMS index
static const uint32_t MAX_SMS_IN_SYSTEM = 256; // Max number of SMS
                                               // that can be stored
                                               // in system

using sms_index_mask_t                  = uint32_t; // SMS mask type

static const uint32_t SMS_MASK_BIT_WIDTH =
    sizeof(sms_index_mask_t) * CHAR_BIT; // SMS mask bit width

static const uint32_t NUM_SMS_INDEX_MASKS =
    (MAX_SMS_IN_SYSTEM + (SMS_MASK_BIT_WIDTH - 1)) /
    SMS_MASK_BIT_WIDTH; // Number of SMS masks needed to track all SMS in system

// PDP context id type
using pdp_context_id_t                                 = uint8_t;

static const pdp_context_id_t INVALID_PDP_CONTEXT_ID   = 255; // Invalid PDP
                                                              // context ID
static const pdp_context_id_t DEFAULT_PDP_CONTEXT_ID   = 1;   // Default PDP
                                                              // context ID
static const pdp_context_id_t DEFAULT_PDP_CONTEXT_ID_2 = 8;   // Default PDP
                                                              // context ID 2
const uint32_t                NUM_USED_PDP_CONTEXTS    = 2;   // Number of PDP
                                                              // contexts used

static const uint32_t MAX_FILENAME_LENGTH    = 64;   // Self-imposed restriction
                                                     // for space saving. Ublox
                                                     // allows 248.
static const uint32_t BUFF_SIZE              = 256;  // Max internal TX
                                                     // buffer size
static const uint32_t RESP_BUFF_SIZE         = 1024; // Max internal RX
                                                     // buffer size
static const uint32_t IDENTIFIER_LENGTH      = 30;   // max length for all
                                                     // types of identifiers

static const uint32_t MIN_IMEI_LENGTH        = 15; // IMEI number length
                                                   // is either 15 or 16
                                                   // characters
static const uint32_t MAX_IMEI_LENGTH        = 16; // IMEI number length
                                                   // is either 15 or 16
                                                   // characters

static const uint32_t MIN_IMSI_LENGTH        = 14; // IMSI number length is 14
static const uint32_t MAX_IMSI_LENGTH        = 15; // IMSI number length is 15

static const uint32_t MIN_ICCID_LENGTH       = 18; // ICCID number length is 19
static const uint32_t MAX_ICCID_LENGTH       = 22; // ICCID number length is 22

static const uint8_t  INVALID_RSSI_VALUE     = 99;  // Invalid RSSI value
static const uint8_t  INVALID_RXQUAL_VALUE   = 99;  // Invalid RXQUAL value
static const uint8_t  INVALID_RSCP_VALUE     = 255; // Invalid RSCP value
static const uint8_t  INVALID_ECNO_VALUE     = 255; // Invalid ECNO value
static const uint8_t  INVALID_RSRQ_VALUE     = 255; // Invalid RSRQ value
static const uint8_t  INVALID_RSRP_VALUE     = 255; // Invalid RSRP value

static const uint32_t MAX_PDP_ADDRESS_LENGTH = 80; // Max PDP Address length

// Minimum idle time after last response before sending next command
static const w_hal_rtc_time_delta_st MIN_IDLE_TIME_AFTER_LAST_RESPONSE = {
    .seconds     = 0,
    .nanoseconds = 20 * NSEC_PER_MSEC};

static const uint32_t LAC_LENGTH = 4; // Buffer size for
                                      // location area code string
static const uint32_t CI_LENGTH  = 8; // Buffer size for Cell ID code string

static const char     COMMAND_PREFIX[]  = "AT";   // Command prefix for all
                                                  // commands sent to
                                                  // the gsm module
static const char     AT_COMMAND[]      = "AT\r"; // No-op Command

static const char     SET_CHARACTER     = '=';  // Used for Set Command Type
static const char     READ_CHARACTER    = '?';  // Used for Read Command Type
static const char     TEST_CHARACTERS[] = "=?"; // Used for Test Command Type

static const char     IGNORE_CHARACTER  = 255; // Sometimes this comes on
                                               // the UART possibly due to
                                               // pull-up
static const char     S3_CHARACTER = '\r'; // Command line termination character
static const char     S4_CHARACTER = '\n'; // Search character to
                                           // look up till in response
static const uint32_t S3_S4_LENGTH = 2;    // Includes S3 and S4 characters
static const char     DOWNLOAD_RESPONSE_CHARACTER = '>'; // On UDWNFILE command,
                                                         // this character will
                                                         // show when ublox is
                                                         // ready to receive
                                                         // data
static const uint32_t NUM_RESPONSE_TERMINATORS    = 2;   // Max number of
                                                         // possible terminators
                                                         // for end of response

static const char     NULL_TERMINATOR             = '\0'; // Null character

static const char     OK_RESPONSE_MATCH[] =
    "OK[\r\n]+"; // Expected successful response

static const char ERROR_RESPONSE_MATCH[] =
    "[Ee][Rr][Rr][Oo][Rr]\\s*:?\\s*\\d*[\r\n]+"; // Expected failure response

static const char ABORTED_RESPONSE_MATCH[] =
    "[Aa][Bb][Oo][Rr][Tt][Ee][Dd][\r\n]+"; // Expected abort response

const uint32_t   MAX_GSM_HANDLES     = 3; // Max number of handles for open
                                          // http requests

const uint32_t   MAX_HANDLE_TRACKERS = MAX_GSM_HANDLES;
const uint32_t   INVALID_HANDLE_TRACKER_INDEX = MAX_HANDLE_TRACKERS;

// Baud rate values to try and see if modem responds. Prioritize
// the baud rates that this firmware configures first.
static const int init_bauds_rates_to_try[]    = {INITIAL_BAUD_RATE,
                                              BAUD_RATE,
                                              SARA_MAX_BAUD_RATE,
                                              INITIAL_BAUD_RATE,
                                              BAUD_RATE,
                                              SARA_MAX_BAUD_RATE,
                                              1200,
                                              2400,
                                              4800,
                                              9600,
                                              19200,
                                              38400,
                                              57600,
                                              115200,
                                              230400,
                                              460800,
                                              921600,
                                              3000000,
                                              3250000,
                                              6000000,
                                              6500000};

// Number of baud rates to try and see if modem responds
static const int NUM_INIT_BAUD_RATES_TO_TRY =
    sizeof(init_bauds_rates_to_try) / sizeof(init_bauds_rates_to_try[0]);

static const uint32_t SMS_RECEIVE_CHECK_TIMEOUT_SECONDS = 10; // Timeout for
                                                              // checking
                                                              // for SMS
                                                              // messages

/**
 * @brief CME error code enumeration
 */
enum class cme_error_t : uint32_t
{
    PHONE_FAILURE = 0,
    NO_CONNECTION_TO_PHONE,
    PHONE_ADAPTOR_LINK_RESERVED,
    OPERATION_NOT_ALLOWED,
    OPERATION_NOT_SUPPORTED,
    PH_SIM_PIN_REQUIRED,
    PH_FSIM_PIN_REQUIRED,
    PH_FSIM_PUK_REQUIRED,
    SIM_NOT_INSERTED = 10,
    SIM_PIN_REQUIRED,
    SIM_PUK_REQUIRED,
    SIM_FAILURE,
    SIM_BUSY,
    SIM_WRONG,
    INCORRECT_PASSWORD,
    SIM_PIN2_REQUIRED,
    SIM_PUK2_REQUIRED,
    MEMORY_FULL = 20,
    INVALID_INDEX,
};

/**
 * @brief activation error code enum
 * Not all error codes are enumerated here.
 * See ublox commands manual for definitions.
 */
enum activation_error_code_t : uint32_t
{
    ACTIVATION_ERROR_CODE_NONE     = 0,                         // No error
    ACTIVATION_ERROR_CODE_NON_ZERO = static_cast<uint32_t>(-2), // Placeholder
                                                                // value used
                                                                // to mean
                                                                // error is set
                                                                // but this is
                                                                // not defined
                                                                // by ublox
    ACTIVATION_ERROR_CODE_INVALID  = static_cast<uint32_t>(-1), // Undefined
                                                                // or not set
};

/**
 * @brief Result code formatting on response
 * See ublox commands manual for definitions.
 */
enum class at_result_code_format_t : uint8_t
{
    DISABLED,      // Just look for "ERROR"
    NUMERIC_ERROR, // Numeric error code returned
    VERBOSE_ERROR  // Verbose error code returned
};

/**
 * @brief Power saving mode of ublox module
 * See ublox commands manual for definitions.
 */
enum class power_saving_mode_t : uint8_t
{
    DISABLED,
    ENABLED,
    RTS_ENABLED,
    DTR_ENABLED_1,
    DTR_ENABLED_2
};

/**
 * @brief Facility lock status of ublox module
 * See ublox commands manual for definitions.
 */
enum class facility_lock_status_t : uint8_t
{
    NOT_ACTIVE,
    ACTIVE
};

/**
 * @brief Operator mode for +COPS response.
 * See ublox commands manual for definitions.
 */
enum class operator_mode_t : uint8_t
{
    AUTOMATIC,
    MANUAL,
    DEREGISTER,
    SET_FORMAT,
    MANUAL_AUTOMATIC,
    EXT_NETWORK_SEARCH,
    EXT_NETWORK_SEARCH_WO_TAGS,
    NETWORK_TIMING_ADVANCE_SEARCH,
};

/**
 * @brief Operator name format on +COPS response.
 * See ublox commands manual for definitions.
 */
enum class operator_format_t : uint8_t
{
    LONG_ALPHANUMERIC,
    SHORT_FORMAT_ALPHANUMERIC,
    NUMERIC,
};

/**
 * @brief Indicates the state of GPRS attachment.
 * See ublox commands manual for definitions.
 */
enum class gprs_attachment_t : uint8_t
{
    DETACHED, // Detached
    ATTACHED, // Attached (default)
};

/**
 * @brief Registration status on +CREG URC.
 * See ublox commands manual for definitions.
 */
enum class registration_status_t : uint8_t
{
    NOT_REGISTERED,
    REGISTERED,
    SEARCHING,
    DENIED,
    UNKNOWN,
    ROAMING,
    SMS_ONLY,
    ROAMING_SMS_ONLY,
    ATTACHED_EMERGENCY,
    CSFB_NOT_PREFERRED,
    ROAMING_CSFB_NOT_PREFERRED,
    INVALID,
};

/**
 * @brief Radio access technology status on +CREG URC.
 */
enum class radio_access_technology_t : uint8_t
{
    GSM,
    GSM_COMPACT,
    UTRAN,
    EDGE,
    HSDPA,
    HSUPA,
    HSDPA_HSUPA,
    LTE,
    EC_GSM_IOT,
    NB_IOT,
    NUM_RADIO_ACCESS_TECHNOLOGIES,
    INVALID = 255
};

/**
 * @brief Reject cause type on +CREG URC.
 */
enum class reject_cause_type_t : uint8_t
{
    GPRS,
    NUM_REJECT_CAUSE_TYPES,
    INVALID = NUM_REJECT_CAUSE_TYPES
};

/**
 * @brief Registration configuration on +CREG URC.
 * See ublox commands manual for definitions.
 */
enum class registration_urc_config_t : uint8_t
{
    DISABLED,
    NETWORK_ENABLED,
    NETWORK_AND_LOCATION_ENABLED,
    NETWORK_AND_REJECT_CAUSE_ENABLED,
};

/**
 * @brief Signalling connection status on +CSCON URC.
 * See ublox commands manual for definitions.
 */
enum class signalling_connection_status_t : uint8_t
{
    NOT_CONNECTED,
    CONNECTED,
    NUM_SIGNALING_CONNECTION_STATUSES,
    INVALID = NUM_SIGNALING_CONNECTION_STATUSES
};

/**
 * @brief Indicates the event download's tag or proactive command's tag for
 * +UUBIP URC
 */
enum class event_tag_t : uint8_t
{
    CHANNEL_STATUS         = 10,
    OPEN_CHANNEL_PROACTIVE = 64,
    CLOSE_CHANNEL_PROACTIVE,
    RECEIVE_DATA_PROACTIVE,
    SEND_DATA_PROACTIVE,
    NUM_EVENT_TAGS = 5,
    INVALID        = 255,
};

/**
 * @brief Registration configuration on +CGREG URC.
 * See ublox commands manual for definitions.
 */
enum class pdp_type_t : uint8_t
{
    IPV4,
    IPV6,
    IPV4V6,
    NON_IP,
    NUM_PDP_TYPES,
    INVALID = NUM_PDP_TYPES
};

/**
 * @brief HTTP content type
 */
enum class ublox_http_content_type_t : uint8_t
{
    APPLICATION,
    TEXT,
    OCTET_STREAM,
    MULTIPART_FORM_DATA,
    JSON,
    XML,
    USER_DEFINED,
    NUM_HTTP_CONTENT_TYPES,
};

/**
 * @brief Command types with different operating behavior
 */
enum class at_command_type_t : uint8_t
{
    ACTION, // Forces the gsm module to print information text or execute a
            // specific action for the command
    SET,    // Configures the preferred settings for the specified command
    SET_NO_EQUAL, // Configures the preferred settings for the specified
                  // command. There is no equal sign in this command
    READ,         // Provides the current setting of the command parameters
    TEST, // Provides the list of the values allowed by each parameter of the
          // command.
    URC,  // Unsolicited Result Code
    IRC   // Intermediate Result Code
};

/**
 * @brief Check for specific response terminator characters
 */
struct at_response_terminator_flags_t
{
    uint8_t byte;
    struct
    {
        uint8_t s3              : 1; // S3 character
        uint8_t s4              : 1; // S4 character
        uint8_t download_prompt : 1; // Download prompt character
        uint8_t space           : 1; // Space character
        uint8_t ignore          : 1; // Ignore character
        uint8_t rsvd            : 5;
    } bit;
};

enum class command_state_t : uint8_t
{
    NOT_ACTIVE,           // No active command
    SEND_COMMAND,         // Send command to module
    WAITING_FOR_RESPONSE, // Wait for command response from module
};

/**
 * @brief Operator enum.
 * This is not defined by any of the gsm modules
 */
enum class operator_t : uint16_t
{
    FIRST,
    US_TMOBILE = FIRST,
    US_ATT,
    INDIA_AIRTEL,
    INDIA_BSNL,
    INDIA_JIO,
    INDIA_VI,
    NUM_OPERATORS,
};

/**
 * @brief Operator name regex patterns for each operator
 */
static const char *OPERATOR_NAME_RE_PATTERNS[] = {
    "[Tt]-?[Mm][Oo][Bb][Ii][Ll][Ee]",
    "[Aa][Tt]&?[Tt]",
    "[Aa][Ii][Rr][Tt][Ee][Ll]",
    "[Bb][Ss][Nn][Ll]",
    "[Jj][Ii][Oo]",
    "[Vv][Ii]\\s?[Ii][Nn][Dd][Ii][Aa]",
};

static_assert(sizeof(OPERATOR_NAME_RE_PATTERNS) /
                      sizeof(OPERATOR_NAME_RE_PATTERNS[0]) ==
                  static_cast<uint16_t>(operator_t::NUM_OPERATORS),
              "OPERATOR_NAME_RE_PATTERNS size does not match NUM_OPERATORS");

/**
 * @brief APN for each operator
 */
static const char *OPERATOR_APNS[] = {
    "fast.t-mobile.com",
    "NXTGENPHONE",
    "airtelgprs.com",
    "bsnlnet",
    "jionet",
    "www",
};

static_assert(sizeof(OPERATOR_APNS) / sizeof(OPERATOR_APNS[0]) ==
                  static_cast<uint16_t>(operator_t::NUM_OPERATORS),
              "OPERATOR_APNS size does not match NUM_OPERATORS");

/**
 * @brief supported bands expressed as bit masks where the lowest bit
 * represents band 1 and the highest bit represents band 128
 */
struct band_mask_st
{
    uint64_t mask1; // bands 1-64
    uint64_t mask2; // bands 65-128
};

static const band_mask_st OPERATOR_SUPPORTED_LTE_BANDS[] = {
    {.mask1 = 0x81A, .mask2 = 0x42},         // US_TMOBILE - 2,4,5,12,66,71
    {.mask1 = 0x3001281A, .mask2 = 0},       // US_ATT - 2,4,5,12,14,17,29,30
    {.mask1 = 0x8000000085ULL, .mask2 = 0},  // INDIA_AIRTEL - 1,3,8,40
    {.mask1 = 0x10000000081ULL, .mask2 = 0}, // INDIA_BSNL - 1,8,41
    {.mask1 = 0x8000000014ULL, .mask2 = 0},  // INDIA_JIO - 3,5,40
    {.mask1 = 0x85ULL, .mask2 = 0},          // INDIA_VI - 1,3,8
};

static_assert(sizeof(OPERATOR_SUPPORTED_LTE_BANDS) /
                      sizeof(OPERATOR_SUPPORTED_LTE_BANDS[0]) ==
                  static_cast<uint16_t>(operator_t::NUM_OPERATORS),
              "OPERATOR_SUPPORTED_LTE_BANDS size does not match NUM_OPERATORS");

/**
 * @brief Tracker for current command
 */
struct command_tracker_st
{
    command_state_t                command_state;
    at_result_code_format_t        result_code_format;        // Result code
                                                              // format on
                                                              // response
    at_response_terminator_flags_t response_terminator_flags; // Response
                                                              // terminator
                                                              // flags
    bool                           received_response;     // indicates if proper
                                                          // response received
                                                          // for command
    wHalErrorCode                  response_error;        // response code for
                                                          // when response
                                                          // received
    uint32_t                       command_buffer_length; // Number of bytes
                                                          // populated in
                                                          // command buffer
    uint32_t                       response_processed_index; // Indicates number
                                                             // of bytes in
                                                             // response buffer
                                                             // that has been
                                                             // processed
    uint32_t                       response_scan_index; // Next byte in response
                                                        // buffer to scan
                                                        // for S3/S4 or NULL
    w_hal_rtc_time_st              last_response_time;  // Time of last command
                                                        // response received
};

/**
 * @brief Structure for holding response of +CESQ command
 */
union signal_quality_t
{
    uint8_t fields[6];
    struct
    {
        uint8_t rxlev; // Received Signal Strength Indicator index
        uint8_t ber;   // Bit error rate
        uint8_t rscp;  // Received Signal Code Power
        uint8_t ecno;  // Ratio of received energy per network personalization
                       // chip to the total received power spectral density
        uint8_t rsrq;  // Reference Signal Received Quality
        uint8_t rsrp;  // Current Reference Signal Received Power
    };
};

/**
 * @brief Structure for holding response of +CREG command
 */
struct ublox_network_st
{
    registration_urc_config_t registration_urc_config; // Network registration
                                                       // urc configuration
    registration_status_t     registration_status;     // Network registration
                                                       // status
    char location_area_code[LAC_LENGTH + 1];           // Two bytes location
                                                       // area code or
                                                       // tracking area
                                                       // code
                                                       // (if <AcTStatus>=7)
                                                       // in hexadecimal format
    char cell_id[CI_LENGTH + 1];                       // From 2-4 bytes cell
                                                       // ID in hexadecimal
                                                       // format. The value
                                                       // FFFF means that
                                                       // the current value
                                                       // is invalid
    radio_access_technology_t radio_access_tech;       // Indicates the radio
                                                       // access technology
    reject_cause_type_t       reject_cause_type;       // Indicates the type of
                                                       // the reject cause
    uint32_t                  reject_cause;            // Indicates the
                                                       // cause of the network
                                                       // registration
                                                       // rejection
};

struct ublox_command_info_st;
using ublox_command_send_func_t =
    wHalErrorCode (*)(char *, uint32_t, const ublox_command_info_st *);

using ublox_response_func_t = wHalErrorCode (*)(char *,
                                                uint32_t,
                                                char *,
                                                uint32_t);

/**
 * @brief Command information needed to issue command to and process response
 * from the gsm module
 */
struct ublox_command_info_st
{
    const char *      p_command; // Command string (not including AT prefix)
    uint8_t           command_length; // Length of command string
    at_command_type_t type;           // AT command operating type
    at_response_terminator_flags_t response_terminator_flags; // Response
                                                              // terminator
                                                              // flags
    const char *              p_set_value; // Used only for SET command type
    ublox_command_send_func_t command_send_func; // Used for preparing and
                                                 // sending the command
                                                 // in buffer
    ublox_response_func_t     response_func;     // Response function handler
};

/**
 * @brief Tracking information for executing a sequence of commands
 */
struct command_sequence_tracker_st
{
    const ublox_command_info_st *p_sequence; // Array of commands to
                                             // be executed
    uint8_t                      count;      // Number of commands in sequence
    uint8_t                      current_index; // Indicates current command
                                                // to execute or complete
    w_hal_rtc_time_st            last_time;     // Time of last command
                                                // sent or response received
};

/**
 * @brief
 * Static information extracted from responses on initialization
 */
struct ublox_info_st
{
    char manufacturer_name[IDENTIFIER_LENGTH + 1]; // manufacturer name,
                                                   // include one
                                                   // entry for null char
    char model_name[IDENTIFIER_LENGTH + 1];        // Model name, include one
                                                   // entry for null char
    char fw_version[IDENTIFIER_LENGTH + 1];        // Firmware version,
                                                   // include one
                                                   // entry for null char
    char imei[MAX_IMEI_LENGTH + 1];                // IMEI, include one
                                                   // entry for null
                                                   // char
    char imsi[MAX_IMSI_LENGTH + 1];                // IMSI, include one
                                                   // entry for null
                                                   // char
    char iccid[MAX_ICCID_LENGTH + 1];              // ICCID, include one
                                                   // entry for null
                                                   // char
    char pdp_address[MAX_PDP_ADDRESS_LENGTH + 1];  // PDP address,
                                                   // include one
                                                   // entry for
                                                   // null char
    char operator_name[IDENTIFIER_LENGTH + 1];     // Operator name,
                                                   // include one
                                                   // entry for null
                                                   // char
    char phone_number[MAX_PHONE_NUMBER_LENGTH];    // buffer to hold
                                                   // device phone
                                                   // number.
    operator_mode_t        operator_mode;          // operator mode
    operator_format_t      operator_format;        // operator format
    gprs_attachment_t      gprs_attachment;        // GPRS attachment state
    ublox_network_st       network;                // Network information
    ublox_network_st       gprs_network;           // GPRS network info
    ublox_network_st       eps_network;            // EPS network info
    signal_quality_t       signal_quality;         // Signal quality
    power_saving_mode_t    power_saving_mode;      // Power saving mode
    facility_lock_status_t sim_lock_status;        // SIM status
    uint32_t               temperature;          // Temperature in units of 0.1
                                                 // degrees Celsius
    time_t                 init_time_value;      // Time observed during
                                                 // initialization
    time_t                 carrier_time_value;   // Time observed during
                                                 // background polling
    uint32_t               max_file_system_size; // Maximum size of the
                                                 // modem file system
};

/**
 * @brief Tracking for initialization of GSM HAL
 */
struct init_tracker_st
{
    uint32_t     activation_error_code; // CME error code on UPSDA
    operator_t   operator_type;         // Operator type enum
    band_mask_st lte_bands;             // Current lte band configuration
    bool         apn_change_needed[NUM_USED_PDP_CONTEXTS];  // Indicates if APN
                                                            // change is needed
    bool         pdp_context_exists[NUM_USED_PDP_CONTEXTS]; // Indicates if PDP
                                                            // context exists
    bool         pdp_context_active[NUM_USED_PDP_CONTEXTS]; // Indicates if PDP
                                                            // context is active
    bool         has_files_to_delete;       // Indicates the file system
                                            // needs cleanup
    bool         deactivation_needed;       // Indicates if PDP deactivation
                                            // is needed
    bool         activation_needed;         // Indicates if PDP activation
                                            // is needed
    bool         reattach_needed;           // Indicates if modem needs to
                                            // be disabled then enabled
    w_hal_rtc_time_st reattach_needed_time; // Time when it was determined
                                            // that modem needs to be
                                            // disabled then enabled
    int               current_sms_cleanup_index; // Current SMS index to
                                                 // delete
    sms_index_mask_t
           sms_index_cleanup_mask[NUM_SMS_INDEX_MASKS]; // Mask of SMS
                                                        // indices to delete
    int    delete_after_read_sms_index;                 // SMS index to delete
    int    oldest_valid_sms_index;                      // Mask of SMS
                                                        // indices that
                                                        // are valid
    time_t oldest_valid_sms_time;                       // Time of oldest
                                                        // valid SMS
    bool   cusd_done;                                   // Indicates if
                                                        // CUSD response
                                                        // received
    bool   done;                                        // Indicates
                                                        // initialization
                                                        // is done
    bool   gsm_sleep_flag;                              // Flag to check state
                                                        // of the modem
    bool   sim_not_inserted_detected;                   // Indicates if SIM
                                                        // not inserted event
                                                        // detected
};

/**
 * @brief Tracker information for a handle that is active, closing, or deleting
 */
struct handle_tracker_st
{
    wGsmTransferHandle handle;            // Handle value
    w_hal_rtc_time_st  last_command_time; // Time of last command sent
    int      expected_upload_bytes;       // total bytes of upload data for this
                                          // handle. This field must be signed
                                          // to allow for value
                                          // INVALID_GSM_UPLOAD_SIZE.
    uint32_t uploaded_bytes;              // requested data to
                                          // send for this handle
    char     filename[MAX_FILENAME_LENGTH];           // Filename for handle
    char     api_route[MAX_SERVER_POST_ROUTE_LENGTH]; // API Route for handle
    ublox_http_content_type_t http_content_type;      // HTTP content type
};

/**
 * @brief Tracker for SMS
 */
struct sms_tracker_st
{
    bool              is_open;                  // Indicates if
                                                // SMS is open.
    bool              read_write;               // Is current SMS
                                                // activity related
                                                // to reading
                                                // or sending?
    bool              read_completed;           // Indicates if
                                                // SMS read is
                                                // completed
    uint8_t           request_chars_remaining;  // Indicates
                                                // the available
                                                // number of characters
                                                // for SMS.
    uint8_t           transfer_chars_remaining; // Indicates
                                                // the available
                                                // number of characters
                                                // for SMS transfer.
    bool              terminator_sent;          // Indicates if
                                                // terminating
                                                // character has
                                                // been sent
    bool              is_terminated;            // Indicates if
                                                // SMS is terminated
    bool              is_closing;               // Indicates if
                                                // SMS is closing
    char *            p_sms_buffer;             // SMS buffer
    uint32_t          sms_buffer_size;          // SMS buffer size
    uint16_t          read_buffer_fill_index;   // SMS buffer fill
                                                // index for SMS reads
    w_hal_rtc_time_st last_received_time;       // Time of last SMS
                                                // received
};

/**
 * @brief Tracking for requests to HAL
 */
struct request_tracker_st
{
    sms_tracker_st sms_tracker; // SMS tracker
    handle_tracker_st
             handle_trackers[MAX_HANDLE_TRACKERS]; // Current
                                                   // outstanding handles
    uint8_t  deleting_index;                       // Handle to delete
    uint8_t  closing_index;                        // Current closing handle
    uint8_t  upload_index;                         // Current handle
                                                   // for uploading

    bool     http_error_code_needed;                 // Indicates that
                                                     // error reason
                                                     // for last failed
                                                     // command should be
                                                     // retrieved
    uint8_t  closing_queue_head;                     // Closing queue
                                                     // head pointer
    uint8_t  closing_queue_tail;                     // Closing queue
                                                     // tail pointer
    uint8_t  closing_queue[MAX_HANDLE_TRACKERS + 1]; // Closing queue of
                                                     // tracker indices

    uint8_t  deleting_queue_head;                     // Deleting queue
                                                      // head pointer
    uint8_t  deleting_queue_tail;                     // Deleting queue
                                                      // tail pointer
    uint8_t  deleting_queue[MAX_HANDLE_TRACKERS + 1]; // Deleting queue of
                                                      // tracker indices

    uint8_t *p_upload_buffer;           // Current upload buffer
                                        // for post request
    uint32_t upload_buffer_size;        // Upload buffer size
                                        // for post request
    uint32_t upload_buffer_transferred; // Number of bytes transferred
                                        // from the buffer
                                        // in post request
    uint32_t cmd_size;                  // Total bytes to be sent in
                                        // +UDWNFILE command
    uint32_t cmd_bytes_transferred;     // Number of bytes transferred
                                        // for +UDWNFILE command
    uint32_t send_size;                 // Number of bytes to
                                        // send in current command
};

/**
 * @brief Information on how to poll for certain key factors in the background
 */
struct background_poll_item_st
{
    const ublox_command_info_st *p_sequence;     // sequence to execute to
                                                 // obtain background
                                                 // information length of
                                                 // the sequence
    uint8_t                      count;          // Number of commands
                                                 // in sequence
    uint32_t                     idle_wait_time; // idle period between
                                                 // each poll in seconds
};

/**
 * @brief Index for background polling
 */
enum class background_poll_item_index_t : uint8_t
{
    SIGNAL_QUALITY = 0,
    TEMPERATURE,
    TIME,
    OPERATOR_INFO,
    ATTACH,
    ACTIVATION,
    PDP_CONTEXT,
    NUM_ITEMS,
    INVALID = NUM_ITEMS
};

/**
 * @brief Tracking for background polling
 */
struct background_poll_tracker_st
{
    uint32_t          index;
    w_hal_rtc_time_st last_time[static_cast<uint8_t>(
        background_poll_item_index_t::NUM_ITEMS)];
};

/**
 * @brief Interface related parameters
 */
struct interface_params_st
{
    uint32_t data_bit_rate;               // Data bit rate
    uint32_t udwnfile_max_transfer_bytes; // Maximum number of bytes to
                                          // transfer in one +UDWNFILE command

    bool     data_enabled; // Indicates if
                           // data is enabled

    bool     sms_read_enabled; // Indicates if receiving
                               // SMS (A2P) is enabled
    bool     sms_send_enabled; // Indicates if sending
                               // SMS (P2A) is enabled

    bool     a2p_short_code; // Indicates if
                             // A2P short code
                             // is enabled. Special
                             // processing is required
                             // for this feature.
    bool     p2a_short_code; // Indicates if
                             // P2A short code
                             // is enabled. Special
                             // processing is required
                             // for this feature.

    char     a2p_phone_number[MAX_PHONE_NUMBER_LENGTH]; // Server
                                                        // receiver
                                                        // phone
                                                        // number
    char     p2a_phone_number[MAX_PHONE_NUMBER_LENGTH]; // Server
                                                        // receiver
                                                        // phone
                                                        // number
    char     server_url[MAX_SERVER_URL_LENGTH];         // Server URL
    uint32_t server_port;                               // Server port
};

static bool is_sara_module();

static bool is_lara_R6_L6_module();

static bool is_lara_R2_module();

static bool is_sim800l_module();

static bool is_sim7600ei_module();

static bool check_in_middle_of_file_transfer();

static wHalErrorCode
handle_get_free_space_from_file_system_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length);

static wHalErrorCode handle_cgdcont_read_response(char *   p_command_buffer,
                                                  uint32_t command_length,
                                                  char *   p_response_buffer,
                                                  uint32_t response_length);

#if (SIMCOM)

static wHalErrorCode handle_cgpaddr_read_response(char *   p_command_buffer,
                                                  uint32_t command_length,
                                                  char *   p_response_buffer,
                                                  uint32_t response_length);

#endif

static wHalErrorCode get_local_time(char *   p_command_buffer,
                                    uint32_t buffer_size,
                                    const ublox_command_info_st *p_info);

#if !(SIMCOM)
static wHalErrorCode
disable_message_waiting_urc(char *                       p_command_buffer,
                            uint32_t                     buffer_size,
                            const ublox_command_info_st *p_info);

static wHalErrorCode
enable_signalling_connection_status(char *   p_command_buffer,
                                    uint32_t buffer_size,
                                    const ublox_command_info_st *p_info);

static wHalErrorCode
send_uart_parity_setup(char *                       p_command_buffer,
                       uint32_t                     buffer_size,
                       const ublox_command_info_st *p_info);

static wHalErrorCode
send_uband_mask_read_command(char *                       p_command_buffer,
                             uint32_t                     buffer_size,
                             const ublox_command_info_st *p_info);

static wHalErrorCode
                     send_uband_mask_write_command(char *                       p_command_buffer,
                                                   uint32_t                     buffer_size,
                                                   const ublox_command_info_st *p_info);

static wHalErrorCode set_http_server_name(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info);

static wHalErrorCode set_http_server_port(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info);

static wHalErrorCode set_http_timeout(char *   p_command_buffer,
                                      uint32_t buffer_size,
                                      const ublox_command_info_st *p_info);

static wHalErrorCode handle_uart_frame_setup_response(char *   p_command_buffer,
                                                      uint32_t command_length,
                                                      char *p_response_buffer,
                                                      uint32_t response_length);

static wHalErrorCode handle_uband_mask_read_response(char *   p_command_buffer,
                                                     uint32_t command_length,
                                                     char *   p_response_buffer,
                                                     uint32_t response_length);

static wHalErrorCode handle_power_saving_response(char *   p_command_buffer,
                                                  uint32_t command_length,
                                                  char *   p_response_buffer,
                                                  uint32_t response_length);

static wHalErrorCode
handle_init_list_files_for_delete_response(char *   p_command_buffer,
                                           uint32_t command_length,
                                           char *   p_response_buffer,
                                           uint32_t response_length);

static wHalErrorCode handle_delete_file_response(char *   p_command_buffer,
                                                 uint32_t command_length,
                                                 char *   p_response_buffer,
                                                 uint32_t response_length);

static wHalErrorCode
send_gsm_shutdown_command(char *                       p_command_buffer,
                          uint32_t                     buffer_size,
                          const ublox_command_info_st *p_info);
#endif // !(SIMCOM)

static wHalErrorCode generic_send_command(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info);

static wHalErrorCode
                     generic_send_command_data_only(char *                       p_command_buffer,
                                                    uint32_t                     buffer_size,
                                                    const ublox_command_info_st *p_info);

static wHalErrorCode send_uart_baud_setup(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info);

static wHalErrorCode get_temperature(char *   p_command_buffer,
                                     uint32_t buffer_size,
                                     const ublox_command_info_st *p_info);

static wHalErrorCode
                     init_detach_from_network(char *                       p_command_buffer,
                                              uint32_t                     buffer_size,
                                              const ublox_command_info_st *p_info);

static wHalErrorCode send_activate_command(char *   p_command_buffer,
                                           uint32_t buffer_size,
                                           const ublox_command_info_st *p_info);

static wHalErrorCode
                     send_deactivate_command(char *                       p_command_buffer,
                                             uint32_t                     buffer_size,
                                             const ublox_command_info_st *p_info);

static wHalErrorCode set_apn_name(char *                       p_command_buffer,
                                  uint32_t                     buffer_size,
                                  const ublox_command_info_st *p_info);

static wHalErrorCode set_internal_apn_name(char *   p_command_buffer,
                                           uint32_t buffer_size,
                                           const ublox_command_info_st *p_info);

static wHalErrorCode start_sms(char *                       p_command_buffer,
                               uint32_t                     buffer_size,
                               const ublox_command_info_st *p_info);

static wHalErrorCode send_sms_data(char *   p_command_buffer,
                                   uint32_t buffer_size,
                                   const ublox_command_info_st *p_info);

static wHalErrorCode send_sms_terminator(char *   p_command_buffer,
                                         uint32_t buffer_size,
                                         const ublox_command_info_st *p_info);

static wHalErrorCode send_flush_data(char *   p_command_buffer,
                                     uint32_t buffer_size,
                                     const ublox_command_info_st *p_info);

static wHalErrorCode write_to_ublox_file(char *   p_command_buffer,
                                         uint32_t buffer_size,
                                         const ublox_command_info_st *p_info);

static wHalErrorCode
send_init_delete_file_command(char *                       p_command_buffer,
                              uint32_t                     buffer_size,
                              const ublox_command_info_st *p_info);

static wHalErrorCode
send_data_to_ublox_file(char *                       p_command_buffer,
                        uint32_t                     buffer_size,
                        const ublox_command_info_st *p_info);

static wHalErrorCode
                     send_phone_number_enquiry_command(char *                       p_command_buffer,
                                                       uint32_t                     buffer_size,
                                                       const ublox_command_info_st *p_info);

static wHalErrorCode send_post_command(char *   p_command_buffer,
                                       uint32_t buffer_size,
                                       const ublox_command_info_st *p_info);

static wHalErrorCode
                     send_delete_file_command(char *                       p_command_buffer,
                                              uint32_t                     buffer_size,
                                              const ublox_command_info_st *p_info);

static wHalErrorCode send_sms_read_command(char *   p_command_buffer,
                                           uint32_t buffer_size,
                                           const ublox_command_info_st *p_info);

static wHalErrorCode
send_sms_delete_after_read_command(char *   p_command_buffer,
                                   uint32_t buffer_size,
                                   const ublox_command_info_st *p_info);

static wHalErrorCode
                     send_sms_delete_command(char *                       p_command_buffer,
                                             uint32_t                     buffer_size,
                                             const ublox_command_info_st *p_info);

static wHalErrorCode handle_generic_response(char *   p_command_buffer,
                                             uint32_t command_length,
                                             char *   p_response_buffer,
                                             uint32_t response_length);

static wHalErrorCode handle_uart_baud_setup_response(char *   p_command_buffer,
                                                     uint32_t command_length,
                                                     char *   p_response_buffer,
                                                     uint32_t response_length);

static wHalErrorCode handle_cfun_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_cmee_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_cgmi_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_cgmm_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_cgmr_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_imei_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_cimi_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_ccid_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode handle_cops_read_response(char *   p_command_buffer,
                                               uint32_t command_length,
                                               char *   p_response_buffer,
                                               uint32_t response_length);

static wHalErrorCode handle_cgatt_read_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length);

static wHalErrorCode handle_cgact_read_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length);

static wHalErrorCode handle_reattach_response(char *   p_command_buffer,
                                              uint32_t command_length,
                                              char *   p_response_buffer,
                                              uint32_t response_length);

static wHalErrorCode handle_cgact_response(char *   p_command_buffer,
                                           uint32_t command_length,
                                           char *   p_response_buffer,
                                           uint32_t response_length);

static wHalErrorCode handle_cusd_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length);

static wHalErrorCode
handle_extended_signal_quality_response(char *   p_command_buffer,
                                        uint32_t command_length,
                                        char *   p_response_buffer,
                                        uint32_t response_length);

static wHalErrorCode handle_signal_quality_response(char *   p_command_buffer,
                                                    uint32_t command_length,
                                                    char *   p_response_buffer,
                                                    uint32_t response_length);

static wHalErrorCode handle_temperature_response(char *   p_command_buffer,
                                                 uint32_t command_length,
                                                 char *   p_response_buffer,
                                                 uint32_t response_length);

static wHalErrorCode handle_facility_lock_response(char *   p_command_buffer,
                                                   uint32_t command_length,
                                                   char *   p_response_buffer,
                                                   uint32_t response_length);

static wHalErrorCode handle_sim_pin_status(char *   p_command_buffer,
                                           uint32_t command_length,
                                           char *   p_response_buffer,
                                           uint32_t response_length);

static wHalErrorCode handle_real_time_clock_response(char *   p_command_buffer,
                                                     uint32_t command_length,
                                                     char *   p_response_buffer,
                                                     uint32_t response_length);

static wHalErrorCode handle_sms_read_response(char *   p_command_buffer,
                                              uint32_t command_length,
                                              char *   p_response_buffer,
                                              uint32_t response_length);

static wHalErrorCode handle_list_sms_message_response(char *   p_command_buffer,
                                                      uint32_t command_length,
                                                      char *p_response_buffer,
                                                      uint32_t response_length);

static wHalErrorCode handle_download_prompt(char *   p_command_buffer,
                                            uint32_t command_length,
                                            char *   p_response_buffer,
                                            uint32_t response_length);

static wHalErrorCode handle_download_completion(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length);

static wHalErrorCode handle_sms_response(char *   p_command_buffer,
                                         uint32_t command_length,
                                         char *   p_response_buffer,
                                         uint32_t response_length);

static wHalErrorCode handle_http_error_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length);

static void          w_hal_gsm_repeat_close_request();

static const char *  ABORTABLE_COMMANDS[]{"+UPSDA"};

/**
 * @brief Sequence for identification of the gsm module
 */
static const ublox_command_info_st id_sequence[] = {
    // No op
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_generic_response},
    // Disable echo
    {"E",
     sizeof("E") - 1,
     at_command_type_t::SET_NO_EQUAL,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &generic_send_command,
     &handle_generic_response},
    // Get the model name
    {"+CGMM",
     sizeof("+CGMM") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgmm_response},
    // Get manufacturer name
    {"+CGMI",
     sizeof("+CGMI") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgmi_response},
    // Get firmware version
    {"+CGMR",
     sizeof("+CGMR") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgmr_response},
};

/**
 * @brief Sequence for power-on initialization of the gsm module
 */
static const ublox_command_info_st init_sequence[] = {
#if !(SIMCOM)
    // Disable message waiting indicator URC
    {"+UMWI",
     sizeof("+UMWI") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &disable_message_waiting_urc,
     &handle_generic_response},
    // Enable even parity
    {"+ICF",
     sizeof("+ICF") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 1,
              .rsvd            = 0}},
     "2,1",
     &send_uart_parity_setup,
     &handle_uart_frame_setup_response},
#endif
    // Set target baud rate
    {"+IPR",
     sizeof("+IPR") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 1,
              .rsvd            = 0}},
     "%d",
     &send_uart_baud_setup,
     &handle_uart_baud_setup_response},
    // User numeric error values
    {"+CMEE",
     sizeof("+CMEE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_cmee_response},
    // Get IMEI number
    {"+CGSN",
     sizeof("+CGSN") - 1,
#if (SIMCOM)
     at_command_type_t::ACTION,
#else
     at_command_type_t::SET,
#endif
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &generic_send_command,
     &handle_imei_response},
    // To enable SIM card
    {"+CFUN",
     sizeof("+CFUN") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_cfun_response},
    // Enable network registration, location information, and
    // reject cause URC
    {"+CREG",
     sizeof("+CREG") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "2",
     &generic_send_command,
     &handle_generic_response},
    // Enable gprs network registration and location information URC
    {"+CGREG",
     sizeof("+CGREG") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "2",
     &generic_send_command,
     &handle_generic_response},
    // Enable eps network registration, location information, and
    // reject cause URC
    {"+CEREG",
     sizeof("+CEREG") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "2",
     &generic_send_command_data_only,
     &handle_generic_response},
#if !(SIMCOM)
    // Enabled Signalling connection status URC
    {"+CSCON",
     sizeof("+CSCON") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &enable_signalling_connection_status,
     &handle_generic_response},
#endif
    // Route SMS indication to the DTE using +CMTI URC
    {"+CNMI",
     sizeof("+CNMI") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "2,1",
     &generic_send_command,
     &handle_generic_response},
    // Get Extended signal quality information
    {"+CESQ",
     sizeof("+CESQ") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command_data_only,
     &handle_extended_signal_quality_response},
    // Get signal quality information
    {"+CSQ",
     sizeof("+CSQ") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_signal_quality_response},
// Set modem temperature to be Celsius
#if (SIMCOM)
    {"+CMTE",
     sizeof("+CMTE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &get_temperature,
     &handle_temperature_response},
#else
    {"+UTEMP",
     sizeof("+UTEMP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "0",
     &get_temperature,
     &handle_temperature_response},
    // Read power saving value
    {"+UPSV",
     sizeof("+UPSV") - 1,
     at_command_type_t::READ,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "",
     &generic_send_command,
     &handle_power_saving_response},
#endif
// Get profile information
#if (SIMCOM)
    {"+CGPADDR",
     sizeof("+CGPADDR") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command_data_only,
     &handle_cgpaddr_read_response},
#else
    {"+CGDCONT",
     sizeof("+CGDCONT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "",
     &generic_send_command,
     &handle_cgdcont_read_response},
    {"+CPMS",
     sizeof("+CPMS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "\"ME\",\"ME\",\"ME\"",
     &generic_send_command,
     &handle_generic_response},
#endif
    // Set up text mode as preferred message format
    {"+CMGF",
     sizeof("+CMGF") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_generic_response},
    // Set up the local time from network
    {"+CLTS",
     sizeof("+CLTS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &get_local_time,
     &handle_generic_response},
};

/**
 * @brief Sequence for timeout based power saving mode
 */
static const ublox_command_info_st power_saving_mode_sequence[] = {
    // Enabling timeout (13.845s) controlled power saving mode
    {"+UPSV",
     sizeof("+UPSV") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1,3000",
     &generic_send_command,
     &handle_generic_response},
};

/**
 * @brief Sequence that should be run every 5-10 seconds to check if the modem
 * is still attached to the network.
 */
static const ublox_command_info_st get_signal_quality_sequence[] = {
    // Get Extended signal quality information
    {"+CESQ",
     sizeof("+CESQ") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command_data_only,
     &handle_extended_signal_quality_response},
    // Get signal quality information
    {"+CSQ",
     sizeof("+CSQ") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_signal_quality_response},
};

/**
 * @brief Sequence that should be run every 5-10 seconds to check if the modem
 * is still attached to the network.
 */
static const ublox_command_info_st get_temperature_sequence[] = {
// Get modem temperature information
#if (SIMCOM)
    {"+CMTE",
     sizeof("+CMTE") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &get_temperature,
     &handle_temperature_response},
#else
    {"+UTEMP",
     sizeof("+UTEMP") - 1,
     at_command_type_t::READ,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "",
     &get_temperature,
     &handle_temperature_response},
#endif
};

/**
 * @brief Sequence that should be run every 30 seconds to get the current
 * registered PLMN in the configured format.
 */
static const ublox_command_info_st get_operator_info_sequence[] = {
    // Get operator information
    {"+COPS",
     sizeof("+COPS") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cops_read_response},
};

/**
 * @brief Sequence that should be run every 20 seconds to check for any problems
 */
static const ublox_command_info_st check_attach_sequence[] = {
    // Check GPRS attach
    {"+CGATT",
     sizeof("+CGATT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgatt_read_response},
};

/**
 * @brief Sequence that should be run every 20 seconds to check for any problems
 */
static const ublox_command_info_st check_activation_sequence[] = {
    // Check GPRS attach
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgact_read_response},
};

/**
 * @brief Sequence that should be run every 30 seconds to get the active and
 * defined EPS bearers with the APN used and the IP type and addresses.
 */
static const ublox_command_info_st get_pdp_context_sequence[] = {
// Get operator information
#if (SIMCOM)
    {"+CGPADDR",
     sizeof("+CGPADDR") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command_data_only,
     &handle_cgpaddr_read_response},
#else
    {"+CGDCONT",
     sizeof("+CGDCONT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "",
     &generic_send_command,
     &handle_cgdcont_read_response},
#endif
};

/**
 * @brief Sequence for handling unexpected URCs
 */
static const ublox_command_info_st reattach_sequence[] = {
    {"+CGATT",
     sizeof("+CGATT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &generic_send_command,
     &handle_generic_response},
    {"+CFUN",
     sizeof("+CFUN") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &generic_send_command,
     &handle_generic_response},
    {"+CFUN",
     sizeof("+CFUN") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_generic_response},
    {"+CGATT",
     sizeof("+CGATT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_reattach_response},
};

/**
 * @brief Sequence during initialization waiting for operator name
 */
static const ublox_command_info_st check_operator_name_sequence[] = {
    // Get operator information
    {"+COPS",
     sizeof("+COPS") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cops_read_response},
};

/**
 * @brief Sequence for initialization time registration check sequence of module
 */
static const ublox_command_info_st check_registration_sequence[] = {
    // Get operator information
    {"+COPS",
     sizeof("+COPS") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cops_read_response},
    // Check GPRS attach
    {"+CGATT",
     sizeof("+CGATT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgatt_read_response},
#if !(SIMCOM)
    // Band selection
    // FIXME: Restart the cellular functionality (e.g. via
    // AT+CFUN=16 or AT+CFUN=0/1 cycle) to make the setting effective.
    {"+UBANDMASK",
     sizeof("+UBANDMASK") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_uband_mask_read_command,
     &handle_uband_mask_read_response},
#endif
    // Get sim lock status
    {"+CLCK",
     sizeof("+CLCK") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"SC\",2",
     &generic_send_command,
     &handle_facility_lock_response},
    // Get sim pin status
    {"+CPIN",
     sizeof("+CPIN") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_sim_pin_status},
    // Read current time from GSM
    {"+CCLK",
     sizeof("+CCLK") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_real_time_clock_response},
};

/**
 * @brief Sequence for getting sim information
 */
static const ublox_command_info_st get_sim_info_sequence[] = {
    // Get the IMSI number
    {"+CIMI",
     sizeof("+CIMI") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cimi_response},
    // Get the ICCID number
    {"+CCID",
     sizeof("+CCID") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_ccid_response},
};

/**
 * @brief Sequence for real time clock acquisition
 */
static const ublox_command_info_st get_time_sequence[] = {
    // Read current time from GSM
    {"+CCLK",
     sizeof("+CCLK") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_real_time_clock_response},
};

/**
 * @brief Sequence for removing registration for gsm module
 */
static const ublox_command_info_st network_deregistration_sequence[] = {
    // Remove registration from network if not already
    {"+COPS",
     sizeof("+COPS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "2",
     &generic_send_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for initialization time registration setup sequence of the
 * gsm module
 */
static const ublox_command_info_st auto_registration_sequence[] = {
#if !(SIMCOM)
    // Band selection
    {"+UBANDMASK",
     sizeof("+UBANDMASK") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "3,%llu,%llu",
     &send_uband_mask_write_command,
     &handle_generic_response},
#endif
    // Start network registration
    {"+COPS",
     sizeof("+COPS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &generic_send_command,
     &handle_generic_response},
    // Read current time from GSM
    {"+CCLK",
     sizeof("+CCLK") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_real_time_clock_response},
};

/**
 * @brief Sequence for deactivation time registration setup sequence of the
 * gsm module
 */
static const ublox_command_info_st external_pdp_deactivation_sequence[] = {
    // Deactivate External PDP Context
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &generic_send_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for runtime activation sequence. This is not used for
 * initialization.
 */
static const ublox_command_info_st external_pdp_activation_sequence[] = {
    // Activate both External PDP Contexts
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1,1",
     &generic_send_command,
     &handle_cgact_response},
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1,8",
     &generic_send_command,
     &handle_cgact_response},
};

/**
 * @brief Sequence for initialization time apn setup sequence of the gsm
 * module for external pdp context
 */
static const ublox_command_info_st external_pdp_apn_sequence[] = {
    // Check which profile needs an APN change or needs to be set up
    {"+CGDCONT",
     sizeof("+CGDCONT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgdcont_read_response},
    // If any PDP context needs an APN change, then
    // detach from the network first to allow for
    // deactivation of PDP context prior to
    // changing APN.
    {"+CGATT",
     sizeof("+CGATT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0",
     &init_detach_from_network,
     &handle_generic_response},
    // Read Activation Status
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgact_read_response},
    // Deactivate an External PDP Context
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,%d",
     &send_deactivate_command,
     &handle_generic_response},
    // Deactivate another External PDP Context
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,%d",
     &send_deactivate_command,
     &handle_generic_response},
    // Read Activation Status after deactivation
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgact_read_response},
    // Set APN for a PDP Context
    {"+CGDCONT",
     sizeof("+CGDCONT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "%d,\"%s\",\"%s\"",
     &set_apn_name,
     &handle_generic_response},
    // Set APN for another PDP Context
    {"+CGDCONT",
     sizeof("+CGDCONT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "%d,\"%s\",\"%s\"",
     &set_apn_name,
     &handle_generic_response},
    // Re-attach to the network
    {"+CGATT",
     sizeof("+CGATT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_generic_response},
    // Read Activation Status
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgact_read_response},
    // Activate an External PDP Context
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1,%d",
     &send_activate_command,
     &handle_cgact_response},
    // Activate another External PDP Context
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1,%d",
     &send_activate_command,
     &handle_cgact_response},
    // Read Activation Status
    {"+CGACT",
     sizeof("+CGACT") - 1,
     at_command_type_t::READ,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_cgact_read_response},
};

/**
 * @brief Sequence for deactivation time registration setup sequence of the
 * gsm module
 */
static const ublox_command_info_st internal_pdp_deactivation_sequence[] = {
    // Deactivate Internal PDP Context
    {"+UPSDA",
     sizeof("+UPSDA") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,4",
     &generic_send_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for runtime activation sequence. This is not used for
 * initialization.
 */
static const ublox_command_info_st internal_pdp_activation_sequence[] = {
    // Activate Internal PDP Context
    {"+UPSDA",
     sizeof("+UPSDA") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,3",
     &generic_send_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for initialization time apn setup sequence of the gsm
 * module for internal pdp context
 */
static const ublox_command_info_st internal_pdp_apn_sequence[] = {
    // Deactivate Internal PDP Context
    {"+UPSDA",
     sizeof("+UPSDA") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,4",
     &generic_send_command,
     &handle_generic_response},
    // Profile #0 is mapped on CID=8.
    {"+UPSD",
     sizeof("+UPSD") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,100,8",
     &generic_send_command,
     &handle_generic_response},
    // IP-V4V6 with IP-V4 preferred for internal sockets
    {"+UPSD",
     sizeof("+UPSD") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,0,2",
     &generic_send_command,
     &handle_generic_response},
    // APN for internal PDP Context
    {"+UPSD",
     sizeof("+UPSD") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,1,\"%s\"",
     &set_internal_apn_name,
     &handle_generic_response},
    // Activate Internal PDP Context
    {"+UPSDA",
     sizeof("+UPSDA") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "0,3",
     &generic_send_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for initialization test sequence of the gsm
 * module
 */
static const ublox_command_info_st test_sequence[] = {
#if (SIMCOM)
    {"+HTTPINIT",
     sizeof("+HTTPINIT") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_generic_response},
    {"+HTTPPARA",
     sizeof("+HTTPPARA") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"URL\",\"http://httpbin.org\"",
     &generic_send_command,
     &handle_generic_response},
    {"+HTTPACTION",
     sizeof("+HTTPACTION") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "2",
     &generic_send_command,
     &handle_generic_response},
#else
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "0",
     &generic_send_command,
     &handle_generic_response},
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "0,1,\"httpbin.org\"",
     &generic_send_command,
     &handle_generic_response},
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "0,5,80",
     &generic_send_command,
     &handle_generic_response},
    {"+UHTTPC",
     sizeof("+UHTTPC") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "0,5,\"/post\",\"post.ffs\",\"name_post=MyName&age_post=30\",0",
     &generic_send_command,
     &handle_generic_response},
#endif
};

/**
 * @brief Sequence for initialization time file cleanup sequence of the gsm
 * filesystem
 */
static const ublox_command_info_st init_file_cleanup_sequence[] = {
#if (SIMCOM)
    // Delete first file
    {"+FSDEL",
     sizeof("+FSDEL") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "*.*",
     &send_init_delete_file_command,
     &handle_generic_response},
#else
    // List files in file system
    {"+ULSTFILE",
     sizeof("+ULSTFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "0",
     &generic_send_command,
     &handle_init_list_files_for_delete_response},
    // Delete first file
    {"+UDELFILE",
     sizeof("+UDELFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "%s",
     &send_init_delete_file_command,
     &handle_generic_response},
#endif
};

/**
 * @brief Sequence for getting free space from file system of the modem
 */
static const ublox_command_info_st get_file_size_sequence[] = {
// get free space available in the file system
#if !(SIMCOM)
    {"+ULSTFILE",
     sizeof("+ULSTFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_get_free_space_from_file_system_response},
#else
    {"+FSMEM",
     sizeof("+FSMEM") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "",
     &generic_send_command_data_only,
     &handle_get_free_space_from_file_system_response}
#endif
};

/**
 * @brief Scan modem for all messages for the oldest message from the server
 */
static const ublox_command_info_st sms_receive_sequence[] = {
    // List SMS messages first
    {"+CMGL",
     sizeof("+CMGL") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"ALL\"",
     &generic_send_command,
     &handle_list_sms_message_response},
    // Read targeted SMS from preferred storage
    {"+CMGR",
     sizeof("+CMGR") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "%d",
     &send_sms_read_command,
     &handle_sms_read_response},
};

/**
 * @brief Sequence for sms cleanup of the gsm module
 */
static const ublox_command_info_st sms_cleanup_after_read_sequence[] = {
    // Delete SMS from preferred storage
    {"+CMGD",
     sizeof("+CMGD") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "%d",
     &send_sms_delete_after_read_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for sms cleanup of the gsm module
 */
static const ublox_command_info_st sms_cleanup_sequence[] = {
    // Delete SMS from preferred storage
    {"+CMGD",
     sizeof("+CMGD") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "%d",
     &send_sms_delete_command,
     &handle_generic_response},
};

/**
 * @brief Sequence for starting an sms to the gsm module
 */
static const ublox_command_info_st sms_send_full_content_sequence[] = {
    // Keep SMS relay protocol link alive
    {"+CMMS",
     sizeof("+CMMS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_generic_response},
    // Send command to start send sms
    {"+CMGS",
     sizeof("+CMGS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 1,
              .space           = 1,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"%s\"",
     &start_sms,
     &handle_download_prompt},
    // Send characters in sms message content
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_data,
     nullptr},
    // Send Ctrl-Z
    {"\x1A",
     sizeof("\x1A") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_terminator,
     &handle_sms_response},
};

/**
 * @brief Sequence for starting an sms to the gsm module
 */
static const ublox_command_info_st sms_send_first_content_sequence[] = {
    // Keep SMS relay protocol link alive
    {"+CMMS",
     sizeof("+CMMS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_generic_response},
    // Send command to start send sms
    {"+CMGS",
     sizeof("+CMGS") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 1,
              .space           = 1,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"%s\"",
     &start_sms,
     &handle_download_prompt},
    // Send characters in sms message content
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_data,
     nullptr},
};

/**
 * @brief Sequence for sending sms content to the gsm module after opening.
 */
static const ublox_command_info_st sms_send_content_sequence[] = {
    // Send characters in sms message content
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_data,
     nullptr},
};

/**
 * @brief Sequence for sending the last sms content to the gsm module for the
 * given message.
 */
static const ublox_command_info_st sms_send_last_content_sequence[] = {
    // Send characters in sms message content
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_data,
     nullptr},
    // Send Ctrl-Z
    {"\x1A",
     sizeof("\x1A") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_terminator,
     &handle_sms_response},
};

/**
 * @brief Sequence for sending the termination for sms message
 */
static const ublox_command_info_st sms_send_termination_sequence[] = {
    // Send Ctrl-Z
    {"\x1A",
     sizeof("\x1A") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_sms_terminator,
     &handle_sms_response},
};

/**
 * @brief Sequence for sending file data to the gsm module
 */
static const ublox_command_info_st file_send_sequence[] = {
// Send command to download file to file system
#if (SIMCOM)
    {"+CFTRANRX",
     sizeof("+CFTRANRX") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 1,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"C:/post_data_%x_%s\",%d",
     &write_to_ublox_file,
     &handle_download_prompt},
#else
    {"+UDWNFILE",
     sizeof("+UDWNFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 1,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "\"post_data_%x_%s\",%d",
     &write_to_ublox_file,
     &handle_download_prompt},
#endif
    // Send data for file download
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_data_to_ublox_file,
     &handle_download_completion},
};

/**
 * @brief Sequence for sending file data to the gsm module
 */
static const ublox_command_info_st file_portion_send_sequence[] = {
    // Send data for file download
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {0, 0},
     "",
     &send_data_to_ublox_file,
     nullptr},
};

/**
 * @brief Sequence for sending file data to the gsm module
 */
static const ublox_command_info_st file_first_portion_send_sequence[] = {
// Send command to download file to file system
#if (SIMCOM)
    {"+CFTRANRX",
     sizeof("+CFTRANRX") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 1,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"C:/post_data_%x_%s\",%d",
     &write_to_ublox_file,
     &handle_download_prompt},
#else
    {"+UDWNFILE",
     sizeof("+UDWNFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 1,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "\"post_data_%x_%s\",%d",
     &write_to_ublox_file,
     &handle_download_prompt},
#endif
    // Send data for file download
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {0, 0},
     "",
     &send_data_to_ublox_file,
     nullptr},
};

/**
 * @brief Sequence for sending file data to the gsm module
 */
static const ublox_command_info_st file_last_portion_send_sequence[] = {
    // Send data for file download
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_data_to_ublox_file,
     &handle_download_completion},
};

/**
 * @brief Sequence for initialization post sequence of the gsm module
 */
static const ublox_command_info_st http_init_profile_sequence[] = {
#if (SIMCOM)
    // Reset HTTP profile
    {"+HTTPINIT",
     sizeof("+HTTPINIT") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &generic_send_command,
     &handle_generic_response},
    // Set HTTP server name
    {"+HTTPPARA",
     sizeof("+HTTPPARA") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"URL\",\"http://httpbin.org/:80\"",
     &generic_send_command,
     &handle_generic_response},
#else
    // Reset HTTP profile #1
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "1",
     &generic_send_command,
     &handle_generic_response},
    // Set HTTP server name
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "1,%d,\"%s\"",
     &set_http_server_name,
     &handle_generic_response},
    // Set HTTP server port
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "1,5,%d",
     &set_http_server_port,
     &handle_generic_response},
    // Set HTTP timeout
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "1,7,%d",
     &set_http_timeout,
     &handle_generic_response},
    // Set no username/password authentication
    {"+UHTTP",
     sizeof("+UHTTP") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "1,4,0",
     &generic_send_command,
     &handle_generic_response},
#endif
};

/**
 * @brief Sequence for post sequence of the gsm module
 */
static const ublox_command_info_st http_post_sequence[] = {
// Trigger HTTP command
#if (SIMCOM)
    {"+HTTPPOSTFILE",
     sizeof("+HTTPPOSTFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"post.ffs\",1,0",
     &send_post_command,
     &handle_generic_response},
#else
    {"+UHTTPC",
     sizeof("+UHTTPC") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "1,4,\"%s\",\"post.ffs\",\"post_data_%x_%s\",%d",
     &send_post_command,
     &handle_generic_response},
#endif
};

/**
 * @brief Sequence for getting protocol error of last http request
 */
static const ublox_command_info_st http_protocol_error_sequence[] = {
    // Get protocol error after HTTP command failure
    {"+UHTTPER",
     sizeof("+UHTTPER") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "1",
     &generic_send_command,
     &handle_http_error_response},
};

/**
 * @brief Sequence for file cleanup of posted file in the gsm module
 */
static const ublox_command_info_st file_cleanup_sequence[] = {
// Delete file from filesystem
#if (SIMCOM)
    {"+FSDEL",
     sizeof("+FSDEL") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "\"post_data_%x_%s\"",
     &send_delete_file_command,
     &handle_generic_response},
#else
    {"+UDELFILE",
     sizeof("+UDELFILE") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "\"post_data_%x_%s\"",
     &send_delete_file_command,
     &handle_delete_file_response},
#endif
};

/**
 * @brief Sequence for flushing the current UDWNFILE command with zeros
 */
static const ublox_command_info_st flush_no_response_sequence[] = {
    // Flush some zeros. Will not necessarily fully flush due to buffer
    // limitations so we have to call this multiple times.
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_flush_data,
     nullptr},
};

/**
 * @brief Sequence for flushing the current UDWNFILE command with zeros and
 * waiting for response
 */
static const ublox_command_info_st flush_with_response_sequence[] = {
    {"",
     sizeof("") - 1,
     at_command_type_t::ACTION,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "",
     &send_flush_data,
     &handle_generic_response},
};

/**
 * @brief Sequence for shutdown the gsm module
 */
static const ublox_command_info_st shutdown_sequence[] = {
// Shutdown and reset command
#if (SIMCOM)
    {"+CFUN",
     sizeof("+CFUN") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "4,1",
     &generic_send_command,
     &handle_generic_response},
#else
    {"+CFUN",
     sizeof("+CFUN") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "16,0",
     &generic_send_command,
     &handle_generic_response},
    {"+CFUN",
     sizeof("+CFUN") - 1,
     at_command_type_t::SET,
     {.bit = {.s3 = 1,
              .s4 = 1,
              .download_prompt = 0,
              .space = 0,
              .ignore = 0,
              .rsvd = 0}},
     "10,0",
     &send_gsm_shutdown_command,
     &handle_generic_response},
#endif
};

/**
 * @brief Sequence for getting phone number from carrier
 */
static const ublox_command_info_st get_phone_number_sequence[] = {
    // Get phone number
    {"+CUSD",
     sizeof("+CUSD") - 1,
     at_command_type_t::SET,
     {.bit = {.s3              = 1,
              .s4              = 1,
              .download_prompt = 0,
              .space           = 0,
              .ignore          = 0,
              .rsvd            = 0}},
     "%s",
     &send_phone_number_enquiry_command,
     &handle_cusd_response},
};

/**
 * @brief Array of background poll items
 */
static const background_poll_item_st background_poll_items[] = {
    {get_signal_quality_sequence,
     sizeof(get_signal_quality_sequence) / sizeof(ublox_command_info_st),
     5},
    {get_temperature_sequence,
     sizeof(get_temperature_sequence) / sizeof(ublox_command_info_st),
     5},
    {get_time_sequence,
     sizeof(get_time_sequence) / sizeof(ublox_command_info_st),
     5},
    {get_operator_info_sequence,
     sizeof(get_operator_info_sequence) / sizeof(ublox_command_info_st),
     30},
    {check_attach_sequence,
     sizeof(check_attach_sequence) / sizeof(ublox_command_info_st),
     20},
    {check_activation_sequence,
     sizeof(check_activation_sequence) / sizeof(ublox_command_info_st),
     20},
    {get_pdp_context_sequence,
     sizeof(get_pdp_context_sequence) / sizeof(ublox_command_info_st),
     30},
};

static_assert(
    sizeof(background_poll_items) / sizeof(background_poll_item_st) ==
        static_cast<uint8_t>(background_poll_item_index_t::NUM_ITEMS),
    "Background poll items array size does not match constant size\n");

static interface_params_st        interface_params;
static background_poll_tracker_st background_poll_tracker;
static init_tracker_st            init_tracker;
static request_tracker_st         request_tracker;
static command_tracker_st         command_tracker; // Command tracker
static command_sequence_tracker_st
    foreground_command_sequence_tracker; // Sequence tracker
static command_sequence_tracker_st
                     internal_command_sequence_tracker; // Sequence tracker
                                                        // for background
                                                        // operations

static ublox_info_st ublox_info; // Static data retrieved from gsm module

static char          command_buffer[BUFF_SIZE];       // Used to populate entire
                                                      // command in buffer
static char          response_buffer[RESP_BUFF_SIZE]; // Used to populate
                                                      // response in buffer
static char          scratch_buffer[BUFF_SIZE];       // Used as intermediate
                                                      // between commands

/**
 * @brief Indicates whether the registration status is a working status
 *
 * @param status the registration status
 * @return true if the status is a working status
 * @return false otherwise
 */
static bool is_working_network_registration_status(registration_status_t status)
{
    return (status != registration_status_t::NOT_REGISTERED) &&
           (status != registration_status_t::SEARCHING) &&
           (status != registration_status_t::DENIED) &&
           (status != registration_status_t::UNKNOWN);
}

/**
 * @brief Flag the module for reattach
 */
static void w_hal_gsm_flag_for_reattach()
{
    init_tracker.reattach_needed      = true;
    init_tracker.reattach_needed_time = w_hal_rtc_get_time();
    init_tracker.deactivation_needed  = true;
    init_tracker.activation_needed    = true;
}

/**
 * @brief Clear the reattach flag
 */
static void w_hal_gsm_reattach_done()
{
    init_tracker.reattach_needed = false;
    init_tracker.reattach_needed_time.invalidate();

    if (request_tracker.closing_index != INVALID_HANDLE_TRACKER_INDEX)
    {
        w_hal_gsm_repeat_close_request();
    }
}

/**
 * @brief Skip over adjust length characters in the string buffer
 *
 * @param p_string_buffer [in/out] :  the string buffer to parse. Will be
 * updated by adjust length
 * @param string_length [in/out] : the length of the string buffer. Will be
 * updated to not include adjust_length.
 * @param adjust_length adjust length to move buffer forward
 */
static void adjust_string_buffer_position(char *&   p_string_buffer,
                                          uint32_t &string_length,
                                          uint32_t  adjust_length)
{
    p_string_buffer += adjust_length;
    SYSTEM_ASSERT_GTEQ(string_length, adjust_length);
    string_length -= adjust_length;
}

/**
 * @brief Skip over a comma and any spaces before or after it
 *
 * @param p_string_buffer [in/out] :  the string buffer to parse. Will be
 * updated past spaces and commas.
 * @param string_length [in/out] : the length of the string buffer. Will be
 * updated to not include spaces and commas.
 * @return bool true if a comma and spaces were skipped, false otherwise
 */
static int skip_comma_spaces(char *&p_string_buffer, uint32_t &string_length)
{
    int match_length;
    int match_index =
        re_match("\\s*,\\s*", p_string_buffer, string_length, &match_length);

    // If the beginning of the string does not match, then we would be
    // skipping more then just the comma and spaces.
    if (match_index == 0)
    {
        adjust_string_buffer_position(p_string_buffer,
                                      string_length,
                                      match_length);

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Parse the time string from the gsm module
 *
 * @param p_time_buffer string buffer containing time
 * @param time_length length of time buffer
 * @return time_t the time in seconds since epoch
 */
static time_t parse_ublox_time_string(char *p_time_buffer, uint32_t time_length)
{
    const uint32_t num_time_fields = 7;
    uint32_t       field_index;
    int            clk_field[num_time_fields];

    for (field_index = 0; field_index < num_time_fields; field_index++)
    {
        int match_length;
        int match_index;

        // Look for field, skipping over delimiters
        match_index = re_match("\\-?\\d+[/,:\"]?",
                               p_time_buffer,
                               time_length,
                               &match_length);

        if (match_index == RE_NO_MATCH_INDEX)
        {
            break;
        }

        // Get field
        clk_field[field_index] = atoi(&p_time_buffer[match_index]);

        adjust_string_buffer_position(p_time_buffer,
                                      time_length,
                                      match_index + match_length);
    }

    if (field_index != num_time_fields)
    {
        return static_cast<time_t>(0);
    }

    // In string form, year is only two digits representing
    // years since 2000. In the time structure, year is the
    // number of years since 1900
    const uint32_t base_year = 2000 - 1900;

    tm             time;
    memset(&time, 0, sizeof(time));

    time.tm_year   = clk_field[0] + base_year;
    time.tm_mon    = clk_field[1] - 1; // 0-based
    time.tm_mday   = clk_field[2];

    // The Time Zone information (last field) is expressed
    // in steps of 15 minutes
    int time_shift = clk_field[6];
    time.tm_hour   = clk_field[3] - (time_shift / 4);
    time.tm_min    = clk_field[4] - (15 * (time_shift % 4));
    time.tm_sec    = clk_field[5];

    return w_hal_rtc_timegm(&time);
}

/**
 * @brief Check if response terminates with S3/S4 characters
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response terminates with S3/S4 characters
 * @return false otherwise
 */
static bool is_response_S_terminated(char *   p_response_buffer,
                                     uint32_t response_length)
{
    return (response_length >= 1) &&
           ((p_response_buffer[response_length - 1] == S4_CHARACTER) ||
            (p_response_buffer[response_length - 1] == S3_CHARACTER));
}

/**
 * @brief Check if response is "OK"
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response matches "OK"
 * @return false otherwise
 */
static bool is_response_OK(char *p_response_buffer, uint32_t response_length)
{
    int match_length;
    int match_index = re_match(OK_RESPONSE_MATCH,
                               p_response_buffer,
                               response_length,
                               &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if response is "ERROR"
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response matches "ERROR"
 * @return false otherwise
 */
static bool is_response_ERROR(char *p_response_buffer, uint32_t response_length)
{
    int match_length;
    int match_index = re_match(ERROR_RESPONSE_MATCH,
                               p_response_buffer,
                               response_length,
                               &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        const char prefix_pattern[] = "\\+CME ERROR\\s*:\\s*";

        // Check if expected prefix has shown up
        match_index                 = re_match(prefix_pattern,
                               p_response_buffer,
                               response_length,
                               &match_length);

        if (match_index != RE_NO_MATCH_INDEX)
        {
            // Skip over prefix
            adjust_string_buffer_position(p_response_buffer,
                                          response_length,
                                          match_index + match_length);
            int error = atoi(p_response_buffer);

            // If it is SIM not inserted error, set flag
            if (error == static_cast<int>(cme_error_t::SIM_NOT_INSERTED))
            {
                init_tracker.sim_not_inserted_detected = true;
            }
        }
        return true;
    }
    return false;
}

/**
 * @brief Check if response is "ABORTED"
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response matches "ABORTED"
 * @return false otherwise
 */
static bool is_response_ABORTED(char *   p_response_buffer,
                                uint32_t response_length)
{
    int match_length;
    int match_index = re_match(ABORTED_RESPONSE_MATCH,
                               p_response_buffer,
                               response_length,
                               &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if command and response match
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if command and response match
 * @return false otherwise
 */
static bool is_command_echo(char *   p_command_buffer,
                            uint32_t command_length,
                            char *   p_response_buffer,
                            uint32_t response_length)
{
    int match_length;
    int match_index =
        re_match("[\r\n]+", p_response_buffer, response_length, &match_length);

    // command does not have S4 character, so just compare up to S3 character
    if ((command_length == static_cast<uint32_t>(match_index) + 1) &&
        !strncmp(p_command_buffer, p_response_buffer, command_length))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Check if response only has S3, S4 characters
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is empty
 * @return false otherwise
 */
static bool is_response_empty(char *p_response_buffer, uint32_t response_length)
{
    int match_length;
    int match_index =
        re_match("[\r\n]+", p_response_buffer, response_length, &match_length);

    return (match_index == 0) &&
           (static_cast<uint32_t>(match_length) == response_length);
}

/**
 * @brief Check if response is a potential URC. Sometimes responses
 * to forward commands start with a plus.
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is a URC
 * @return false otherwise
 */
static bool is_potential_urc_response(char *   p_response_buffer,
                                      uint32_t response_length)
{
    int match_length;
    int match_index;

    // Check if response starts with a plus
    match_index =
        re_match("\\s*\\+", p_response_buffer, response_length, &match_length);

    return (match_index == 0);
}

/**
 * @brief Check if response is +CSCON URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +CSCON URC
 * @return false otherwise
 */
static bool is_connection_status_response(char *   p_response_buffer,
                                          uint32_t response_length)
{
    int match_length;
    int match_index;

    match_index  = re_match("\\+CSCON\\s*:\\s*",
                           p_response_buffer,
                           response_length,
                           &match_length);

    bool matched = (match_index != RE_NO_MATCH_INDEX);

    // Parse signalling connection status
    if (matched)
    {
        // skip over the prefix
        adjust_string_buffer_position(p_response_buffer,
                                      response_length,
                                      match_index + match_length);

        // Parse signalling connection status
        match_index =
            re_match("\\d+", p_response_buffer, response_length, &match_length);

        if (match_index != RE_NO_MATCH_INDEX)
        {
            // Get signalling connection status
            int connection_status = atoi(&p_response_buffer[match_index]);

            // Disconnected?
            if (connection_status == 0)
            {
                // Can't trust whether +CSCON is latest status
                // Flag background polling of activation status
                background_poll_tracker
                    .last_time[static_cast<uint8_t>(
                        background_poll_item_index_t::ACTIVATION)]
                    .clear();
            }
        }
    }

    return matched;
}

/**
 * @brief Check if response is +UMWI URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +UMWI URC
 * @return false otherwise
 */
static bool is_message_waiting_indicator_response(char *   p_response_buffer,
                                                  uint32_t response_length)
{
    int match_length;

    return (re_match("\\+UMWI",
                     p_response_buffer,
                     response_length,
                     &match_length) != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if response is +CREG URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +CREG URC
 * @return false otherwise
 */
static bool is_network_registration_status_response(char *   p_response_buffer,
                                                    uint32_t response_length)
{
    const char        prefix_pattern[]      = "\\+CREG\\s*:\\s*";
    const char        gprs_prefix_pattern[] = "\\+CGREG\\s*:\\s*";
    const char        eps_prefix_pattern[]  = "\\+CEREG\\s*:\\s*";

    ublox_network_st *pNetwork;

    int               prefix_length;
    int               prefix_index;

    // Check if expected prefix has shown up
    prefix_index = re_match(prefix_pattern,
                            p_response_buffer,
                            response_length,
                            &prefix_length);

    if (prefix_index != RE_NO_MATCH_INDEX)
    {
        pNetwork = &ublox_info.network;
    }
    else
    {
        // Check if expected prefix has shown up
        prefix_index = re_match(gprs_prefix_pattern,
                                p_response_buffer,
                                response_length,
                                &prefix_length);

        if (prefix_index != RE_NO_MATCH_INDEX)
        {
            pNetwork = &ublox_info.gprs_network;
        }
        else
        {
            // Check if expected prefix has shown up
            prefix_index = re_match(eps_prefix_pattern,
                                    p_response_buffer,
                                    response_length,
                                    &prefix_length);

            if (prefix_index != RE_NO_MATCH_INDEX)
            {
                pNetwork = &ublox_info.eps_network;
            }
            else
            {
                // Not a network registration status
                return false;
            }
        }
    }

    // skip over the prefix
    adjust_string_buffer_position(p_response_buffer,
                                  response_length,
                                  prefix_index + prefix_length);

    int match_length;
    int match_index;

    // Check if response has network registration URC configuration
    if (re_match("\\d+\\s*,\\s*\\d+\\s*,?",
                 p_response_buffer,
                 response_length,
                 &match_length) != RE_NO_MATCH_INDEX)
    {
        // Parse network registration URC configuration
        match_index = re_match("\\d+\\s*,\\s*",
                               p_response_buffer,
                               response_length,
                               &match_length);

        SYSTEM_ASSERT_NOT_EQUAL(match_index, RE_NO_MATCH_INDEX)

        // Get network registration URC configuration
        pNetwork->registration_urc_config =
            static_cast<registration_urc_config_t>(
                atoi(&p_response_buffer[match_index]));

        adjust_string_buffer_position(p_response_buffer,
                                      response_length,
                                      match_index + match_length);
    }

    // Parse network status
    match_index = re_match("\\d+\\s*,?\\s*",
                           p_response_buffer,
                           response_length,
                           &match_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return true;
    }

    // Get network registration status
    pNetwork->registration_status = static_cast<registration_status_t>(
        atoi(&p_response_buffer[match_index]));

    adjust_string_buffer_position(p_response_buffer,
                                  response_length,
                                  match_index + match_length);

    // Parse location area code. Because of a bug in LARA-R6 where sometimes
    // the opening quote is missing, the regex pattern does not scan for
    // any quote.
    match_index = re_match("[a-fA-F0-9]+",
                           p_response_buffer,
                           response_length,
                           &match_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return true;
    }

    // Get location area code
    SYSTEM_ASSERT_LTEQ(static_cast<uint32_t>(match_length), LAC_LENGTH);

    // copy location area code
    memcpy(pNetwork->location_area_code,
           &p_response_buffer[match_index],
           match_length);
    pNetwork->location_area_code[match_length] = NULL_TERMINATOR;

    adjust_string_buffer_position(p_response_buffer,
                                  response_length,
                                  match_index + match_length);

    // Skip over end quote if it is there
    if (p_response_buffer[0] == '\"')
    {
        p_response_buffer++;
        response_length--;
    }

    // Skip over spaces and commas
    if (!skip_comma_spaces(p_response_buffer, response_length))
    {
        return true;
    }

    // Parse cell ID. Because of a bug in LARA-R6 where sometimes
    // the opening quote is missing, the regex pattern does not scan for
    // any quote.
    match_index = re_match("[a-fA-F0-9]+",
                           p_response_buffer,
                           response_length,
                           &match_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return true;
    }

    // Get cell ID
    SYSTEM_ASSERT_LTEQ(static_cast<uint32_t>(match_length), CI_LENGTH);
    // copy cellular identifier
    memcpy(pNetwork->cell_id, &p_response_buffer[match_index], match_length);
    pNetwork->cell_id[match_length] = NULL_TERMINATOR;

    adjust_string_buffer_position(p_response_buffer,
                                  response_length,
                                  match_index + match_length);

    // Skip over end quote if it is there
    if (p_response_buffer[0] == '\"')
    {
        p_response_buffer++;
        response_length--;
    }

    // Skip over spaces and commas
    if (!skip_comma_spaces(p_response_buffer, response_length))
    {
        return true;
    }

    // Parse ActStatus
    match_index =
        re_match("\\d+", p_response_buffer, response_length, &match_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return true;
    }

    // Get radio access technology
    pNetwork->radio_access_tech = static_cast<radio_access_technology_t>(
        atoi(&p_response_buffer[match_index]));

    adjust_string_buffer_position(p_response_buffer,
                                  response_length,
                                  match_index + match_length);

    // Skip over spaces and commas
    if (!skip_comma_spaces(p_response_buffer, response_length))
    {
        return true;
    }

    // Skip over cause type
    match_index =
        re_match("\\d+", p_response_buffer, response_length, &match_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return true;
    }

    // Get Reject Cause Type
    pNetwork->reject_cause_type =
        static_cast<reject_cause_type_t>(atoi(&p_response_buffer[match_index]));

    // Skip over spaces and commas
    if (!skip_comma_spaces(p_response_buffer, response_length))
    {
        return true;
    }

    // Parse Reject Cause
    match_index =
        re_match("\\d+", p_response_buffer, response_length, &match_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return true;
    }

    // Get Reject Cause
    pNetwork->reject_cause = atoi(&p_response_buffer[match_index]);

    return true;
}

/**
 * @brief Check if response is +UUPSDD or +UUPSDA URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +UUPSDD or +UUPSDA URC
 * @return false otherwise
 */
static bool is_psd_activation_response(char *   p_response_buffer,
                                       uint32_t response_length)
{
    const char activation_pattern[]    = "\\+UUPSDA\\s*:\\s*";
    const char deactivation_pattern1[] = "\\+UUPSDD\\s*:\\s*";
    // The below pattern is not documented in spec, but has been observed
    const char deactivation_pattern2[] = "\\+UUPSD";

    int        prefix_length;
    int        prefix_index;

    // Check if expected prefix has shown up
    prefix_index = re_match(activation_pattern,
                            p_response_buffer,
                            response_length,
                            &prefix_length);

    if (prefix_index != RE_NO_MATCH_INDEX)
    {
        if (is_lara_R2_module() || is_sara_module())
        {
            // skip over the prefix
            adjust_string_buffer_position(p_response_buffer,
                                          response_length,
                                          prefix_index + prefix_length);

            prefix_index = re_match("\\d+",
                                    p_response_buffer,
                                    response_length,
                                    &prefix_length);

            if (prefix_index != RE_NO_MATCH_INDEX)
            {
                // Activation result
                init_tracker.activation_error_code = atoi(p_response_buffer);

                // If activation error didn't happen during initialization
                // then retry
                init_tracker.activation_needed =
                    (init_tracker.activation_error_code !=
                     ACTIVATION_ERROR_CODE_NONE);
            }
        }

        return true;
    }
    else if ((re_match(deactivation_pattern1,
                       p_response_buffer,
                       response_length,
                       &prefix_length) != RE_NO_MATCH_INDEX) ||
             (re_match(deactivation_pattern2,
                       p_response_buffer,
                       response_length,
                       &prefix_length) != RE_NO_MATCH_INDEX))
    {
        if (is_lara_R2_module() || is_sara_module())
        {
            // Set activation error code to be non-zero
            init_tracker.activation_error_code = ACTIVATION_ERROR_CODE_NON_ZERO;

            // Deactivation is done. Don't need to do it again.
            init_tracker.deactivation_needed   = false;
            init_tracker.activation_needed     = true;
        }

        return true;
    }

    return false;
}

/**
 * @brief Check if response is +CMTI URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +CMTI URC
 * @return false otherwise
 */
static bool is_new_sms_response(char *   p_response_buffer,
                                uint32_t response_length)
{
    int match_length;

    return (re_match("\\+CMTI",
                     p_response_buffer,
                     response_length,
                     &match_length) != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if response is +CUSD URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +CUSD URC
 * @return false otherwise
 */
static bool is_cusd_urc_response(char *   p_response_buffer,
                                 uint32_t response_length)
{
    const char prefix_pattern[] = "\\+CUSD\\s*:\\s*\\d+";
    int        match_length;
    int        match_index;

    // Check if expected prefix has shown up
    match_index = re_match(prefix_pattern,
                           p_response_buffer,
                           response_length,
                           &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        init_tracker.cusd_done = true;

        // Skip over prefix
        adjust_string_buffer_position(p_response_buffer,
                                      response_length,
                                      match_index + match_length);

        skip_comma_spaces(p_response_buffer, response_length);

        match_index =
            re_match("\\d+", p_response_buffer, response_length, &match_length);

        if (match_index != RE_NO_MATCH_INDEX)
        {
            strncpy(ublox_info.phone_number,
                    &p_response_buffer[match_index],
                    match_length);
        }

        return true;
    }

    return false;
}

#if !(SIMCOM)
/**
 * @brief Parse a pdp type from a string buffer
 *
 * @param p_string_buffer string buffer to parse
 * @param string_length length of the string (may not include null
 * terminator)
 * @param skip_length number of bytes to skip over
 * @return pdp_type_t the pdp type or INVALID if not found
 */
static pdp_type_t
parse_pdp_type(char *p_string_buffer, uint32_t string_length, int &skip_length)
{
    pdp_type_t pdp_type     = pdp_type_t::INVALID;
    int        prefix_index = re_match("\"IPV?\\d?V?\\d?\"",
                                p_string_buffer,
                                string_length,
                                &skip_length);

    if (prefix_index != RE_NO_MATCH_INDEX)
    {
        // skip lengths include quotes and commas
        if (skip_length == 8)
        {
            pdp_type = pdp_type_t::IPV4V6;
        }
        else if (skip_length == 6)
        {
            pdp_type = pdp_type_t::IPV6;
        }
        else if (skip_length == 4)
        {
            pdp_type = pdp_type_t::IPV4;
        }

        // The match length is already included in skip_length. Add the match
        // index to calculate the end position of the match.
        skip_length += prefix_index;
    }

    return pdp_type;
}
#endif

/**
 * @brief Parse a pdp address from a string buffer
 *
 * @param p_string_buffer string buffer to parse
 * @param string_length length of the string (may not include null terminator)
 * @param match_length length of the match
 * @return int the index of the match or RE_NO_MATCH_INDEX if not found
 */
static int parse_pdp_address(char *   p_string_buffer,
                             uint32_t string_length,
                             int &    match_length)
{
    // IPV4 address contains one IPV4 address
    // IPV6 address contains one IPV6 address
    // IPV4V6 address contains both IPV4 address and IPV6 address
    int prefix_index = re_match("\"[0-9\\.]+\\s?[0-9\\.]*\"",
                                p_string_buffer,
                                string_length,
                                &match_length);

    return prefix_index;
}

/**
 * @brief Parse a pdp address from a string buffer
 *
 * @param p_pdp_address_buffer string buffer to parse
 * @param pdp_address_length length of the string (may not include null
 * terminator)
 * @return bool true if valid pdp address, false otherwise
 */
static bool check_pdp_address_valid(char *   p_pdp_address_buffer,
                                    uint32_t pdp_address_length)
{
    int         match_length;
    bool        valid                = true;

    // These are patterns for IPV4 addresses that are not valid
    const char *p_invalid_patterns[] = {
        "\"\"",                                 // empty string
        "\"255\\.255\\.255\\.[0-9\\.]+[\\s\"]", // broadcast address
        "\"127\\.[0-9\\.]+[\\s\"]",             // loopback
        "\"169\\.254\\.[0-9\\.]+[\\s\"]",     // automatic private ip addresses
        "\"192\\.168\\.*[\\s\"]",             // private network ip addresses
        "\"172\\.1[6-9]\\.*[\\s\"]",          // private network ip addresses
        "\"172\\.2[0-9]\\.*[\\s\"]",          // private network ip addresses
        "\"172\\.31\\.*[\\s\"]",              // private network ip addresses
        "\"192\\.88\\.99\\.[0-9\\.]+[\\s\"]", // Relay use only
    };

    for (uint32_t index = 0;
         index < sizeof(p_invalid_patterns) / sizeof(char *);
         index++)
    {
        int match_index = re_match(p_invalid_patterns[index],
                                   p_pdp_address_buffer,
                                   pdp_address_length,
                                   &match_length);

        if (match_index != RE_NO_MATCH_INDEX)
        {
            valid = false;
            break;
        }
    }

    return valid;
}

/**
 * @brief Parse a pdp context id from a string buffer
 *
 * @param p_string_buffer string buffer to parse
 * @param string_length length of the string (may not include null terminator)
 * @param skip_length number of bytes to skip over
 * @return pdp_context_id_t the pdp context id or INVALID_PDP_CONTEXT_ID if not
 * found
 */
static pdp_context_id_t parse_pdp_context_id(char *   p_string_buffer,
                                             uint32_t string_length,
                                             int &    skip_length)
{
    int match_index =
        re_match("\\d+", p_string_buffer, string_length, &skip_length);

    if (match_index == RE_NO_MATCH_INDEX)
    {
        return INVALID_PDP_CONTEXT_ID;
    }
    else
    {
        // The match length is already included in skip_length. Add the match
        // index to calculate the end position of the match.
        skip_length += match_index;

        return atoi(p_string_buffer + match_index);
    }
}

/**
 * @brief Repeat +UHTTPC command for the same file if it failed
 */
static void w_hal_gsm_repeat_close_request()
{
    uint8_t closing_index = request_tracker.closing_index;
    SYSTEM_ASSERT_NOT_EQUAL(closing_index, INVALID_HANDLE_TRACKER_INDEX);
    SYSTEM_ASSERT_LT(closing_index, MAX_HANDLE_TRACKERS);

    // Clear last command time
    request_tracker.handle_trackers[closing_index]
        .last_command_time.invalidate();

    // Enqueue to the front of the closing queue
    // to retry. Decrement the head pointer and place
    // it there.
    if (request_tracker.closing_queue_head == 0)
    {
        request_tracker.closing_queue_head = MAX_HANDLE_TRACKERS;
    }
    else
    {
        --request_tracker.closing_queue_head;
    }

    // Queue shouldn't go from full to empty (overflow)
    SYSTEM_ASSERT_NOT_EQUAL(request_tracker.closing_queue_head,
                            request_tracker.closing_queue_tail);

    request_tracker.closing_queue[request_tracker.closing_queue_head] =
        closing_index;

    // Clear closing handle to allow another close request
    request_tracker.closing_index = INVALID_HANDLE_TRACKER_INDEX;
}

/**
 * @brief Check if response is +UHTTPC URC
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return true if response is +UHTTPC URC
 * @return false otherwise
 */
static bool is_http_response(char *p_response_buffer, uint32_t response_length)
{
#if (SIMCOM)
    const char prefix_pattern[] = "\\+HTTPACTION\\s*:\\s*";
#else
    const char prefix_pattern[] = "\\+UUHTTPCR\\s*:\\s*";
#endif
    int prefix_length;
    int prefix_index;

    prefix_index = re_match(prefix_pattern,
                            p_response_buffer,
                            response_length,
                            &prefix_length);

    bool matched = (prefix_index != RE_NO_MATCH_INDEX);
    if (matched)
    {
        int good_http_status = false;

        adjust_string_buffer_position(p_response_buffer,
                                      response_length,
                                      prefix_index + prefix_length);

        // Skip over profile id and command type
        prefix_index = re_match("\\d+\\s*,\\s*\\d+\\s*,\\s*",
                                p_response_buffer,
                                response_length,
                                &prefix_length);

        if (prefix_index != RE_NO_MATCH_INDEX)
        {
            adjust_string_buffer_position(p_response_buffer,
                                          response_length,
                                          prefix_index + prefix_length);

            prefix_index     = re_match("\\d+",
                                    p_response_buffer,
                                    response_length,
                                    &prefix_length);

            good_http_status = atoi(p_response_buffer);
        }

        if (init_tracker.done)
        {
            uint8_t closing_index = request_tracker.closing_index;
            SYSTEM_ASSERT_NOT_EQUAL(closing_index,
                                    INVALID_HANDLE_TRACKER_INDEX);
            SYSTEM_ASSERT_LT(closing_index, MAX_HANDLE_TRACKERS);

            // Clear last command time
            request_tracker.handle_trackers[closing_index]
                .last_command_time.invalidate();

            if (good_http_status)
            {
                // Clear closing handle from active handles
                SYSTEM_ASSERT_NOT_EQUAL(
                    request_tracker.handle_trackers[closing_index].handle,
                    INVALID_GSM_HANDLE);

                // Clear closing handle and mark it for delete
                request_tracker
                    .deleting_queue[request_tracker.deleting_queue_tail] =
                    request_tracker.closing_index;
                if (++request_tracker.deleting_queue_tail ==
                    MAX_HANDLE_TRACKERS + 1)
                {
                    request_tracker.deleting_queue_tail = 0;
                }
                SYSTEM_ASSERT_NOT_EQUAL(request_tracker.deleting_queue_head,
                                        request_tracker.deleting_queue_tail);

                request_tracker.closing_index = INVALID_HANDLE_TRACKER_INDEX;
            }
            else
            {
                LOG_MSG_WITH_ARGS(
                    TraceFile::W_HAL_GSM_CPP,
                    "Failed to perform HTTP request for file %s. "
                    "Retrying...\n",
                    request_tracker.handle_trackers[closing_index].filename);

                // Flag to send +UHTTPER to check error status
                request_tracker.http_error_code_needed = true;

                w_hal_gsm_repeat_close_request();
            }
        }
        else
        {
            if (good_http_status)
            {
                init_tracker.activation_error_code = ACTIVATION_ERROR_CODE_NONE;
                init_tracker.deactivation_needed   = false;
                init_tracker.activation_needed     = false;
            }
            else
            {
                // If this is during test sequence, then flag to move onto
                // another operator
                init_tracker.activation_error_code =
                    ACTIVATION_ERROR_CODE_NON_ZERO;
                init_tracker.deactivation_needed = true;
                init_tracker.activation_needed   = true;
            }
        }
    }

    return matched;
}

/**
 * @brief Check if model name is SARA
 *
 * @return true if model name is SARA
 * @return false otherwise
 */
static bool is_sara_module()
{
    int match_index;
    int match_length;

    SYSTEM_ASSERT_NOT_EQUAL(ublox_info.model_name[0], NULL_TERMINATOR);

    match_index = re_match("SARA",
                           ublox_info.model_name,
                           sizeof(ublox_info.model_name),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if model name is LARA-R6
 *
 * @return true if model name is LARA-R6
 * @return false otherwise
 */
static bool is_lara_R6_module()
{
    int match_index;
    int match_length;

    SYSTEM_ASSERT_NOT_EQUAL(ublox_info.model_name[0], NULL_TERMINATOR);

    match_index = re_match("LARA-R6",
                           ublox_info.model_name,
                           sizeof(ublox_info.model_name),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if model name is LARA-R6 or LARA-L6
 *
 * @return true if model name is LARA-R6 or LARA-L6
 * @return false otherwise
 */
static bool is_lara_R6_L6_module()
{
    int match_index;
    int match_length;

    SYSTEM_ASSERT_NOT_EQUAL(ublox_info.model_name[0], NULL_TERMINATOR);

    match_index = re_match("LARA-[LR]6",
                           ublox_info.model_name,
                           sizeof(ublox_info.model_name),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if model name is LARA-R2
 *
 * @return true if model name is LARA-R2
 * @return false otherwise
 */
static bool is_lara_R2_module()
{
    int match_index;
    int match_length;

    SYSTEM_ASSERT_NOT_EQUAL(ublox_info.model_name[0], NULL_TERMINATOR);

    match_index = re_match("LARA-R2",
                           ublox_info.model_name,
                           sizeof(ublox_info.model_name),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if model name is SIMCOM_SIM800L
 *
 * @return true if model name is SIMCOM_SIM800L
 * @return false otherwise
 */
static bool is_sim800l_module()
{
    int match_index;
    int match_length;

    SYSTEM_ASSERT_NOT_EQUAL(ublox_info.model_name[0], NULL_TERMINATOR);

    match_index = re_match("SIMCOM_SIM800L",
                           ublox_info.model_name,
                           sizeof(ublox_info.model_name),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Check if model name is SIMCOM_SIM7600EI
 *
 * @return true if model name is SIMCOM_SIM7600EI
 * @return false otherwise
 */
static bool is_sim7600ei_module()
{
    int match_index;
    int match_length;

    SYSTEM_ASSERT_NOT_EQUAL(ublox_info.model_name[0], NULL_TERMINATOR);

    match_index = re_match("SIMCOM_SIM7600EI",
                           ublox_info.model_name,
                           sizeof(ublox_info.model_name),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief Perform preliminary checks on response buffer. This may generate
 * an error code if applicable
 *
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @param error_code [out] : if done, then this is the error code to be
 * returned
 * @param error_code_valid [out] : Indicates that the response is a
 * predefined string
 * @return true if response is valid
 * @return false if response is invalid
 */
static bool response_precheck(char *         p_response_buffer,
                              uint32_t       response_length,
                              wHalErrorCode &error_code,
                              bool &         error_code_valid)
{
    bool valid       = false;
    error_code_valid = false;

    if (is_response_S_terminated(p_response_buffer, response_length))
    {
        valid = true;

        if (is_response_empty(p_response_buffer, response_length))
        {
            error_code       = wHalErrorCode::BUSY;
            error_code_valid = true;
        }
        else if (is_response_OK(p_response_buffer, response_length))
        {
            error_code       = wHalErrorCode::NO_ERROR;
            error_code_valid = true;
        }
        else if (is_response_ABORTED(p_response_buffer, response_length) ||
                 is_response_ERROR(p_response_buffer, response_length))
        {
            // Since response buffer not null-terminated, copy to another
            // buffer where it can be null-terminated

            const uint32_t TEMP_BUFFER_SIZE = 32;
            char           temp_cmd_buffer[TEMP_BUFFER_SIZE + 1];
            char           temp_resp_buffer[TEMP_BUFFER_SIZE + 1];

            strncpy(temp_cmd_buffer, command_buffer, TEMP_BUFFER_SIZE);

            uint32_t copy_length = std::min(TEMP_BUFFER_SIZE, response_length);
            memcpy(temp_resp_buffer, p_response_buffer, copy_length);
            temp_resp_buffer[copy_length] = NULL_TERMINATOR;

            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                              "%s\nOn CMD:%s\n",
                              &temp_resp_buffer[0],
                              &temp_cmd_buffer[0]);

            error_code       = wHalErrorCode::GSM_ERROR;
            error_code_valid = true;
        }
    }

    // This response may not be S3/S4 terminated
    if (!error_code_valid)
    {
        if (is_potential_urc_response(p_response_buffer, response_length))
        {
            using urc_check_func_t = bool (*)(char *, uint32_t);
            const urc_check_func_t urc_check_functions[] = {
                is_connection_status_response,
                is_message_waiting_indicator_response,
                is_http_response,
                is_network_registration_status_response,
                is_psd_activation_response,
                is_new_sms_response,
                is_cusd_urc_response,
            };

            bool expected_urc = false;

            for (uint32_t index = 0;
                 index < sizeof(urc_check_functions) / sizeof(urc_check_func_t);
                 index++)
            {
                if (urc_check_functions[index](p_response_buffer,
                                               response_length))
                {
                    expected_urc = true;
                    break;
                }
            }

            if (expected_urc)
            {
                valid            = true;
                error_code       = wHalErrorCode::BUSY;
                error_code_valid = true;
            }
        }
    }

    return valid;
}

/**
 * @brief Handler for non-specific command processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if nothing to do, NO_ERROR if successful
 * response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_generic_response(char *   p_command_buffer,
                                             uint32_t command_length,
                                             char *   p_response_buffer,
                                             uint32_t response_length)
{
    wHalErrorCode error_code;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error_code;
        }
        // Ignore echo
        else if (is_command_echo(p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length))
        {

            return wHalErrorCode::BUSY;
        }
    }

    return wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
}

#if !(SIMCOM)
/**
 * @brief Handler for +ICF command response processing
 *
 * @param p_command_buffer the command sent to gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if nothing to do, NO_ERROR if successful
 * response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_uart_frame_setup_response(char *   p_command_buffer,
                                                      uint32_t command_length,
                                                      char *p_response_buffer,
                                                      uint32_t response_length)
{
    wHalErrorCode error;

    error = handle_generic_response(p_command_buffer,
                                    command_length,
                                    p_response_buffer,
                                    response_length);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // Wait 100 ms before adjusting settings
        w_hal_system_delay_ms(100);

        // Now adjust the parity on this side
        error = w_hal_uart_set_parity_bits(GSM_UART_CHANNEL,
                                           whal_uart_parity::EVEN_1);

        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR); // wait for S4
    }

    return error;
}
#endif

/**
 * @brief Handler for +IPR command response processing
 *
 * @param p_command_buffer the command sent to gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if nothing to do, NO_ERROR if successful
 * response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_uart_baud_setup_response(char *   p_command_buffer,
                                                     uint32_t command_length,
                                                     char *   p_response_buffer,
                                                     uint32_t response_length)
{
    wHalErrorCode error;

    error = handle_generic_response(p_command_buffer,
                                    command_length,
                                    p_response_buffer,
                                    response_length);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // On the UART AT interface, after the reception of the "OK" result
        // code for the +IPR command, the DTE shall wait for at least 100 ms
        // before issuing a new AT command; this is to guarantee a proper
        // baud rate reconfiguration.
        w_hal_system_delay_ms(100);

        // Now adjust the baud rate on DTE side
        // SARA only supports lower baud rate
        uint32_t baud_rate = is_sara_module() ? SARA_MAX_BAUD_RATE : BAUD_RATE;
        error = w_hal_uart_set_baud_rate(GSM_UART_CHANNEL, baud_rate);

        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return error;
}

/**
 * @brief Handler for +CFUN SIM reset command response processing
 *
 * @param p_command_buffer the command sent to gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if nothing to do, NO_ERROR if successful
 * response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cfun_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    wHalErrorCode error = handle_generic_response(p_command_buffer,
                                                  command_length,
                                                  p_response_buffer,
                                                  response_length);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // Wait 1s to allow SIM card to initialize
        w_hal_system_delay(1);
    }

    return error;
}

/**
 * @brief Handler for +CMEE command response processing
 *
 * @param p_command_buffer the command sent to gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if nothing to do, NO_ERROR if successful
 * response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cmee_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    wHalErrorCode error = handle_generic_response(p_command_buffer,
                                                  command_length,
                                                  p_response_buffer,
                                                  response_length);

    if (error == wHalErrorCode::NO_ERROR)
    {
        command_tracker.result_code_format =
            at_result_code_format_t::NUMERIC_ERROR;
    }

    return error;
}

/**
 * @brief Handler function for responses that need to be stored
 * in a buffer
 *
 * @param p_store_buffer permanent storage response buffer
 * @param store_buffer_size size of response storage buffer
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @param p_skip_prefix prefix to skip over
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_store_response(char *      p_store_buffer,
                                           uint32_t    store_buffer_size,
                                           char *      p_command_buffer,
                                           uint32_t    command_length,
                                           char *      p_response_buffer,
                                           uint32_t    response_length,
                                           const char *p_skip_prefix = nullptr)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            if (p_skip_prefix != nullptr)
            {
                int prefix_length;
                int prefix_index;

                prefix_index = re_match(p_skip_prefix,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index != RE_NO_MATCH_INDEX)
                {
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  prefix_index + prefix_length);
                }
            }

            // Copy over identifier
            // Don't include S3 or S4 suffix
            uint32_t copy_length =
                std::min(store_buffer_size - 1, response_length - 1);
            memcpy(p_store_buffer, p_response_buffer, copy_length);
            p_store_buffer[copy_length] = NULL_TERMINATOR;

            // Wait for OK response
            error                       = wHalErrorCode::BUSY;
        }
    }

    return error;
}

/**
 * @brief Handler for +CGMI command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cgmi_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    return handle_store_response(ublox_info.manufacturer_name,
                                 sizeof(ublox_info.manufacturer_name),
                                 p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length);
}

/**
 * @brief Handler for +CGMM command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cgmm_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    return handle_store_response(ublox_info.model_name,
                                 sizeof(ublox_info.model_name),
                                 p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length);
}

/**
 * @brief Handler for +CGMR command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cgmr_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    const char *p_prefix_pattern;
    if (is_sim800l_module())
    {
        p_prefix_pattern = "\\Revision\\s*:\\s*";
    }
    else
    {
        p_prefix_pattern = "\\+CGMR\\s*:\\s*";
    }
    return handle_store_response(ublox_info.fw_version,
                                 sizeof(ublox_info.fw_version),
                                 p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length,
                                 p_prefix_pattern);
}

/**
 * @brief Handler for command response processing for IMEI number request
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_imei_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    return handle_store_response(ublox_info.imei,
                                 sizeof(ublox_info.imei),
                                 p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length);
}

/**
 * @brief Handler for command response processing for IMSI number request
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cimi_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    return handle_store_response(ublox_info.imsi,
                                 sizeof(ublox_info.imsi),
                                 p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length);
}

/**
 * @brief Handler for command response processing for ICCID number request
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_ccid_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    const char prefix_pattern[] = "\\+CCID\\s*:\\s*";
    return handle_store_response(ublox_info.iccid,
                                 sizeof(ublox_info.iccid),
                                 p_command_buffer,
                                 command_length,
                                 p_response_buffer,
                                 response_length,
                                 prefix_pattern);
}

/**
 * @brief Handler for +COPS command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cops_read_response(char *   p_command_buffer,
                                               uint32_t command_length,
                                               char *   p_response_buffer,
                                               uint32_t response_length)
{
    wHalErrorCode error_code = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error_code;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+COPS\\s*:\\s*";
                int        match_length;
                int        match_index;

                // Check if expected prefix has shown up
                match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Wait for OK response
                error_code = wHalErrorCode::BUSY;

                // Skip over prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Get operator mode
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                ublox_info.operator_mode = static_cast<operator_mode_t>(
                    atoi(&p_response_buffer[match_index]));

                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Skip over spaces and commas
                if (!skip_comma_spaces(p_response_buffer, response_length))
                {
                    break;
                }

                // Get operator format
                match_index = re_match("\\d+\\s*,\\s*\"",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                ublox_info.operator_format = static_cast<operator_format_t>(
                    atoi(&p_response_buffer[match_index]));

                // Skip over operator format and punctuation
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Get operator name or MCC/MNC
                match_index = re_match("[^\"]+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                memcpy(ublox_info.operator_name,
                       &p_response_buffer[match_index],
                       match_length);

                // Add null terminator
                ublox_info.operator_name[match_length] = NULL_TERMINATOR;

                for (operator_t op = operator_t::FIRST;
                     op != operator_t::NUM_OPERATORS;
                     op = static_cast<operator_t>(static_cast<int>(op) + 1))
                {
                    int op_match_length;

                    if (re_match(
                            OPERATOR_NAME_RE_PATTERNS[static_cast<int>(op)],
                            ublox_info.operator_name,
                            match_length,
                            &op_match_length) != RE_NO_MATCH_INDEX)
                    {
                        init_tracker.operator_type = op;
                        break;
                    }
                }

            } while (false);
        }
    }

    return error_code;
}

/**
 * @brief Handler for +CGATT command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cgatt_read_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length)
{
    wHalErrorCode error_code = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error_code;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CGATT\\s*:\\s*";
                int        match_length;
                int        match_index;

                // Check if expected prefix has shown up
                match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Skip over prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Get GPRS attachment state
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                ublox_info.gprs_attachment = static_cast<gprs_attachment_t>(
                    atoi(&p_response_buffer[match_index]));

                if (init_tracker.done)
                {
                    // If we are done with initialization, then we need to
                    // reattach if we are not attached
                    if (ublox_info.gprs_attachment ==
                        gprs_attachment_t::DETACHED)
                    {
                        LOG_MSG(TraceFile::W_HAL_GSM_CPP,
                                "GPRS not attached. Reattaching...\n");
                        w_hal_gsm_flag_for_reattach();
                    }
                }

                // Wait for OK response
                error_code = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error_code;
}

/**
 * @brief Handler for +CGACT command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cgact_read_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length)
{
    wHalErrorCode error_code = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error_code;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CGACT\\s*:\\s*";
                int        match_length;
                int        match_index;

                // Check if expected prefix has shown up
                match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Wait for OK response
                error_code = wHalErrorCode::BUSY;

                if (is_lara_R2_module() || is_sara_module())
                {
                    // For LARA R2 and SARA modules, we only care about the
                    // internal pdp context, so we can skip over the other
                    // pdp contexts.
                    break;
                }

                // Skip over prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Parse pdp context id
                pdp_context_id_t cid = parse_pdp_context_id(p_response_buffer,
                                                            response_length,
                                                            match_length);

                // If we could not parse the pdp context id or it is not the
                // pdp context that was configured earlier, then break
                if ((cid == INVALID_PDP_CONTEXT_ID) ||
                    ((cid != DEFAULT_PDP_CONTEXT_ID) &&
                     (cid != DEFAULT_PDP_CONTEXT_ID_2)))
                {
                    break;
                }

                if (cid == DEFAULT_PDP_CONTEXT_ID)
                {
                    init_tracker.pdp_context_exists[0] = true;
                }
                else if (cid == DEFAULT_PDP_CONTEXT_ID_2)
                {
                    init_tracker.pdp_context_exists[1] = true;
                }

                // Skip over PDP context ID
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_length);

                // Skip over spaces and commas
                skip_comma_spaces(p_response_buffer, response_length);

                // Get PDP context status
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                int status = atoi(&p_response_buffer[match_index]);

                if (cid == DEFAULT_PDP_CONTEXT_ID)
                {
                    init_tracker.pdp_context_active[0] = (status == 1);
                }
                else if (cid == DEFAULT_PDP_CONTEXT_ID_2)
                {
                    init_tracker.pdp_context_active[1] = (status == 1);
                }

                bool all_active = true;
                for (uint32_t index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
                {
                    all_active =
                        all_active && init_tracker.pdp_context_active[index];
                }

                if (!all_active)
                {
                    // Reactivate the pdp context
                    init_tracker.deactivation_needed = false;
                    init_tracker.activation_needed   = true;
                }
            } while (false);
        }
    }

    return error_code;
}

/**
 * @brief Handler for +CGDCONT command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 * @return wHalErrorCode
 */
static wHalErrorCode handle_cgdcont_read_response(char *   p_command_buffer,
                                                  uint32_t command_length,
                                                  char *   p_response_buffer,
                                                  uint32_t response_length)
{
    wHalErrorCode error_code = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error_code;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CGDCONT\\s*:\\s*";
                int        match_length;
                int        match_index;

                // Check if expected prefix has shown up
                match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Wait for OK response
                error_code = wHalErrorCode::BUSY;

                // Skip over prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Get PDP context ID
                pdp_context_id_t cid = parse_pdp_context_id(p_response_buffer,
                                                            response_length,
                                                            match_length);

                // If we could not parse the pdp context id or it is not the
                // pdp context that was configured earlier, then break
                if ((cid == INVALID_PDP_CONTEXT_ID) ||
                    ((cid != DEFAULT_PDP_CONTEXT_ID) &&
                     (cid != DEFAULT_PDP_CONTEXT_ID_2)))
                {
                    break;
                }

                if (cid == DEFAULT_PDP_CONTEXT_ID)
                {
                    init_tracker.pdp_context_exists[0] = true;
                }
                else if (cid == DEFAULT_PDP_CONTEXT_ID_2)
                {
                    init_tracker.pdp_context_exists[1] = true;
                }

                // Skip over PDP context ID
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_length);

                // Skip over spaces and commas
                skip_comma_spaces(p_response_buffer, response_length);

                // Get PDP context type
                pdp_type_t pdp_type = parse_pdp_type(p_response_buffer,
                                                     response_length,
                                                     match_length);

                if (pdp_type == pdp_type_t::INVALID)
                {
                    break;
                }

                // Check for PDP context type mismatch.
                // If the current pdp type is unexpected, then we need to
                // reconfigure the pdp context. Set the flag to indicate
                // that we need to reconfigure the pdp context.
                bool pdp_type_mismatch = false;
                if (cid == DEFAULT_PDP_CONTEXT_ID)
                {
                    pdp_type_mismatch = (pdp_type != pdp_type_t::IPV4V6);
                    init_tracker.apn_change_needed[0] = pdp_type_mismatch;
                }
                else if (cid == DEFAULT_PDP_CONTEXT_ID_2)
                {
                    pdp_type_mismatch = (pdp_type != pdp_type_t::IPV4);
                    init_tracker.apn_change_needed[1] = pdp_type_mismatch;
                }

                // Skip over PDP context type
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_length);

                // Skip over spaces and commas
                skip_comma_spaces(p_response_buffer, response_length);

                // Check for APN mismatch
                match_index = re_match("\"[^\"]*\"",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Compare APN to see if it is the same as the one that was
                // configured on boot. If it is not, then we need to
                // reconfigure the pdp context.
                bool apn_change_needed =
                    (strlen(OPERATOR_APNS[static_cast<int>(
                         init_tracker.operator_type)]) !=
                     static_cast<size_t>(match_length - 2)) ||
                    (strncmp(OPERATOR_APNS[static_cast<int>(
                                 init_tracker.operator_type)],
                             p_response_buffer + match_index + 1,
                             match_length - 2) != 0);

                // If the APN is different or the pdp type is different,
                // then we need to reconfigure the pdp context
                if (cid == DEFAULT_PDP_CONTEXT_ID)
                {
                    init_tracker.apn_change_needed[0] =
                        apn_change_needed || pdp_type_mismatch;
                }
                else if (cid == DEFAULT_PDP_CONTEXT_ID_2)
                {
                    init_tracker.apn_change_needed[1] =
                        apn_change_needed || pdp_type_mismatch;
                }

                // Skip over APN
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Skip over spaces and commas
                skip_comma_spaces(p_response_buffer, response_length);

                match_index = parse_pdp_address(p_response_buffer,
                                                response_length,
                                                match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // An invalid ip address is unexpected
                // If the ip address is invalid, then we need to reconfigure
                // the pdp context
                if (!check_pdp_address_valid(p_response_buffer + match_index,
                                             match_length))
                {
                    LOG_MSG(
                        TraceFile::W_HAL_GSM_CPP,
                        "Invalid PDP address. Reconfiguring pdp context...\n");
                    init_tracker.deactivation_needed = true;
                    init_tracker.activation_needed   = true;
                }

                // Don't include quotes in pdp address parsing
                SYSTEM_ASSERT_GT(match_length, 2);
                match_index += 1;
                match_length -= 2;

                // Copy over pdp address
                memcpy(ublox_info.pdp_address,
                       p_response_buffer + match_index,
                       match_length);
                ublox_info.pdp_address[match_length] = NULL_TERMINATOR;
            } while (false);
        }
    }

    return error_code;
}

#if SIMCOM
/**
 * @brief Handler for +CGPADDR command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 * @return wHalErrorCode
 */
static wHalErrorCode handle_cgpaddr_read_response(char *   p_command_buffer,
                                                  uint32_t command_length,
                                                  char *   p_response_buffer,
                                                  uint32_t response_length)
{
    wHalErrorCode error_code = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error_code;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CGPADDR\\s*:\\s*";
                int        match_length;
                int        match_index;

                // Check if expected prefix has shown up
                match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Skip over prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Get PDP context ID
                pdp_context_id_t cid = parse_pdp_context_id(p_response_buffer,
                                                            response_length,
                                                            match_length);

                // If we could not parse the pdp context id or it is not the
                // pdp context that was configured earlier, then break
                if ((cid == INVALID_PDP_CONTEXT_ID) ||
                    (cid != DEFAULT_PDP_CONTEXT_ID))
                {
                    break;
                }

                // Skip over PDP context ID
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_length);

                // Skip over spaces and commas
                skip_comma_spaces(p_response_buffer, response_length);

                match_index = parse_pdp_address(p_response_buffer,
                                                response_length,
                                                match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // An invalid ip address is unexpected
                // If the ip address is invalid, then we need to reconfigure
                // the pdp context
                if (!check_pdp_address_valid(p_response_buffer + match_index,
                                             match_length))
                {
                    LOG_MSG(
                        TraceFile::W_HAL_GSM_CPP,
                        "Invalid PDP address. Reconfiguring pdp context...\n");
                    init_tracker.deactivation_needed = true;
                    init_tracker.activation_needed   = true;
                }

                // Don't include quotes in pdp address parsing
                SYSTEM_ASSERT_GT(match_length, 2);
                match_index += 1;
                match_length -= 2;

                // Copy over pdp address
                memcpy(ublox_info.pdp_address,
                       p_response_buffer + match_index,
                       match_length);
                ublox_info.pdp_address[match_length] = NULL_TERMINATOR;

                // Wait for OK response
                error_code                           = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error_code;
}
#endif

/**
 * @brief Handler for +CFUN command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_reattach_response(char *   p_command_buffer,
                                              uint32_t command_length,
                                              char *   p_response_buffer,
                                              uint32_t response_length)
{
    wHalErrorCode error = handle_generic_response(p_command_buffer,
                                                  command_length,
                                                  p_response_buffer,
                                                  response_length);

    if (error != wHalErrorCode::BUSY)
    {
        if (error == wHalErrorCode::NO_ERROR)
        {
            w_hal_gsm_reattach_done();
        }
        else
        {
            LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Reattach failed. Retrying...\n");
            w_hal_gsm_flag_for_reattach();
        }
    }

    return error;
}

/**
 * @brief Handler for +CGACT command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cgact_response(char *   p_command_buffer,
                                           uint32_t command_length,
                                           char *   p_response_buffer,
                                           uint32_t response_length)
{
    wHalErrorCode error = handle_generic_response(p_command_buffer,
                                                  command_length,
                                                  p_response_buffer,
                                                  response_length);

    if ((error != wHalErrorCode::BUSY) && !is_lara_R2_module() &&
        !is_sara_module())
    {
        if (error == wHalErrorCode::NO_ERROR)
        {
            if (init_tracker.activation_error_code ==
                ACTIVATION_ERROR_CODE_INVALID)
            {
                bool all_active = true;
                for (uint32_t index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
                {
                    all_active =
                        all_active && init_tracker.pdp_context_active[index];
                }

                // Only set the activation error code to none if all pdp
                // contexts are active. It should only be set once.
                if (all_active)
                {
                    init_tracker.activation_error_code =
                        ACTIVATION_ERROR_CODE_NONE;
                }
            }
        }
        else
        {
            LOG_MSG(TraceFile::W_HAL_GSM_CPP,
                    "PDP context activation failed. Retrying...\n");
            init_tracker.activation_error_code = ACTIVATION_ERROR_CODE_NON_ZERO;
        }

        // If activation error didn't happen during initialization
        // then retry
        bool reactivation                = (error != wHalErrorCode::NO_ERROR);
        init_tracker.deactivation_needed = reactivation;
        init_tracker.activation_needed   = reactivation;
    }

    return error;
}

#if !(SIMCOM)
/**
 * @brief Handler for +UBANDMASK command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_uband_mask_read_response(char *   p_command_buffer,
                                                     uint32_t command_length,
                                                     char *   p_response_buffer,
                                                     uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {

            const char prefix_pattern[] = "\\+UBANDMASK\\s*:\\s*";

            int        prefix_length;
            int        prefix_index;

            // Check if expected prefix has shown up
            prefix_index = re_match(prefix_pattern,
                                    p_response_buffer,
                                    response_length,
                                    &prefix_length);

            if (prefix_index != RE_NO_MATCH_INDEX)
            {
                const int lte_rat = 3;

                while (true)
                {

                    // Response is a list with each entry having the form
                    // <RAT>,<BAND1_MASK>,<BAND2_MASK>. RAT is a number from 0
                    // to 7. BAND1_MASK and BAND2_MASK are 64 bit decimal
                    // numbers
                    int match_length;
                    int match_index;

                    // Scan for RAT
                    match_index = re_match("\\d+",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index == RE_NO_MATCH_INDEX)
                    {
                        break;
                    }

                    // Parse the RAT
                    int rat = atoi(&p_response_buffer[match_index]);

                    // Skip over RAT
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);

                    // Skip over spaces and commas
                    if (!skip_comma_spaces(p_response_buffer, response_length))
                    {
                        break;
                    }

                    // Scan for BAND1_MASK
                    match_index = re_match("\\d+",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index == RE_NO_MATCH_INDEX)
                    {
                        break;
                    }

                    // Parse the BAND1_MASK
                    if (rat == lte_rat)
                    {
                        init_tracker.lte_bands.mask1 =
                            strtoull(&p_response_buffer[match_index],
                                     nullptr,
                                     10);
                    }

                    // Skip over BAND1_MASK
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);

                    // Skip over spaces and commas
                    if (!skip_comma_spaces(p_response_buffer, response_length))
                    {
                        break;
                    }

                    // Only look at the LTE bands
                    if (rat != lte_rat)
                    {
                        // Other RATs only describe BAND1 mask, which why the
                        // code jumps out here
                        continue;
                    }

                    // Scan for BAND2_MASK
                    match_index = re_match("\\d+",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index == RE_NO_MATCH_INDEX)
                    {
                        break;
                    }

                    // Parse the BAND2_MASK
                    init_tracker.lte_bands.mask2 =
                        strtoull(&p_response_buffer[match_index], nullptr, 10);

                    // Wait for OK response
                    error = wHalErrorCode::BUSY;
                    break;
                }
            }
        }
    }

    return error;
}
#endif

/**
 * @brief Handler for +CUSD command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_cusd_response(char *   p_command_buffer,
                                          uint32_t command_length,
                                          char *   p_response_buffer,
                                          uint32_t response_length)
{
    wHalErrorCode error;

    error = handle_generic_response(p_command_buffer,
                                    command_length,
                                    p_response_buffer,
                                    response_length);

    if ((error != wHalErrorCode::BUSY) && (error != wHalErrorCode::NO_ERROR))
    {
        // If there was an error, then the response is not valid
        init_tracker.cusd_done = true;
    }

    return error;
}

/**
 * @brief Handler for +CSQ command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_signal_quality_response(char *   p_command_buffer,
                                                    uint32_t command_length,
                                                    char *   p_response_buffer,
                                                    uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CSQ\\s*:\\s*\"?";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);
                int match_length;
                int match_index;

                // Look for RSSI field
                match_index = re_match("\\d+,?",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get field
                int       signal_power = atoi(&p_response_buffer[match_index]);
                const int invalid_signal_power = 99;

                if (signal_power != invalid_signal_power)
                {
                    // Signal power is RSSI in GSM and LTE RATs,
                    // and is RSCP in UMTS RATs
                    switch (ublox_info.network.radio_access_tech)
                    {
                        case radio_access_technology_t::GSM:
                        case radio_access_technology_t::GSM_COMPACT:
                        case radio_access_technology_t::EDGE:
                        case radio_access_technology_t::LTE:
                        case radio_access_technology_t::EC_GSM_IOT:
                        case radio_access_technology_t::NB_IOT:
                        {
                            ublox_info.signal_quality.rxlev = signal_power;
                            break;
                        }
                        case radio_access_technology_t::UTRAN:
                        case radio_access_technology_t::HSDPA:
                        case radio_access_technology_t::HSUPA:
                        case radio_access_technology_t::HSDPA_HSUPA:
                        {
                            ublox_info.signal_quality.rscp = signal_power;
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                }

                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                skip_comma_spaces(p_response_buffer, response_length);

                // Look for BER field
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get field
                int       qual         = atoi(&p_response_buffer[match_index]);
                const int invalid_qual = 99;

                if (qual != invalid_qual)
                {
                    switch (ublox_info.network.radio_access_tech)
                    {
                        case radio_access_technology_t::GSM:
                        case radio_access_technology_t::GSM_COMPACT:
                        case radio_access_technology_t::EDGE:
                        case radio_access_technology_t::EC_GSM_IOT:
                        {
                            // Log ber or mean bit error probability
                            // for 2G RAT CS/GPRS or 2G RAT EGPRS respectively
                            ublox_info.signal_quality.ber = qual;
                            break;
                        }
                        case radio_access_technology_t::UTRAN:
                        case radio_access_technology_t::HSDPA:
                        case radio_access_technology_t::HSUPA:
                        case radio_access_technology_t::HSDPA_HSUPA:
                        case radio_access_technology_t::LTE:
                        case radio_access_technology_t::NB_IOT:
                        default:
                        {
                            break;
                        }
                    }
                }

                // Wait for OK response
                error = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error;
}

/**
 * @brief Handler for +CESQ command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode
handle_extended_signal_quality_response(char *   p_command_buffer,
                                        uint32_t command_length,
                                        char *   p_response_buffer,
                                        uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CESQ\\s*:\\s*\"?";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);

                const uint32_t num_fields =
                    sizeof(ublox_info.signal_quality.fields);

                uint32_t field_index;
                for (field_index = 0; field_index < num_fields; field_index++)
                {
                    int match_length;
                    int match_index;

                    // Look for field
                    match_index = re_match("\\d+,?",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index == RE_NO_MATCH_INDEX)
                    {
                        break;
                    }

                    // Get field
                    ublox_info.signal_quality.fields[field_index] =
                        atoi(&p_response_buffer[match_index]);

                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);
                }

                if (field_index == num_fields)
                {
                    // Wait for OK response
                    error = wHalErrorCode::BUSY;
                }
            } while (false);
        }
    }

    return error;
}

/**
 * @brief Handler for +UTEMP command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_temperature_response(char *   p_command_buffer,
                                                 uint32_t command_length,
                                                 char *   p_response_buffer,
                                                 uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
#if (SIMCOM)
                const char prefix_pattern[] = "\\+CMTE\\s*:\\s*";
#else
                const char prefix_pattern[] = "\\+UTEMP\\s*:\\s*";
#endif

                int match_length;
                int match_index;

                // Check if expected prefix has shown up
                match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // Look for field
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);
#if (SIMCOM)
                // skip over
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                match_index = re_match("-?\\d+\\.?\\d?",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index != RE_NO_MATCH_INDEX)
                {
                    ublox_info.temperature =
                        10 * atoi(&p_response_buffer[match_index]);

                    // See if there are any digits after the decimal point
                    match_index = re_match("\\.\\d",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index != RE_NO_MATCH_INDEX)
                    {
                        // convert char to int
                        ublox_info.temperature +=
                            p_response_buffer[match_index + 1] - '0';
                    }
                }

#else
                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get field
                ublox_info.temperature = atoi(&p_response_buffer[match_index]);
#endif

                // Wait for OK response
                error = wHalErrorCode::BUSY;

            } while (false);
        }
    }

    return error;
}

#if !(SIMCOM)
/**
 * @brief Handler for +UPSV command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_power_saving_response(char *   p_command_buffer,
                                                  uint32_t command_length,
                                                  char *   p_response_buffer,
                                                  uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+UPSV\\s*:\\s*";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);

                int match_length;
                int match_index;

                // Look for number
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get number
                ublox_info.power_saving_mode = static_cast<power_saving_mode_t>(
                    atoi(&p_response_buffer[match_index]));

                // Wait for OK response
                error = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error;
}
#endif

/**
 * @brief Handler for +CLCK command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_facility_lock_response(char *   p_command_buffer,
                                                   uint32_t command_length,
                                                   char *   p_response_buffer,
                                                   uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CLCK\\s*:\\s*";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);

                int match_length;
                int match_index;

                // Look for number
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get number
                ublox_info.sim_lock_status =
                    static_cast<facility_lock_status_t>(
                        atoi(&p_response_buffer[match_index]));

                // Wait for OK response
                error = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error;
}

/**
 * @brief Handler for +CPIN command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_sim_pin_status(char *   p_command_buffer,
                                           uint32_t command_length,
                                           char *   p_response_buffer,
                                           uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            int match_length;
            int match_index;

            // Parse status
            match_index = re_match("\\+CPIN\\s*:\\s*READY",
                                   p_response_buffer,
                                   response_length,
                                   &match_length);

            if (match_index == RE_NO_MATCH_INDEX)
            {
                // Unhandled code
                error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
            }
            else
            {
                error = wHalErrorCode::BUSY;
            }
        }
    }

    return error;
}

/**
 * @brief Handler for +CCLK command response processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_real_time_clock_response(char *   p_command_buffer,
                                                     uint32_t command_length,
                                                     char *   p_response_buffer,
                                                     uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+CCLK\\s*:\\s*\"";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);

                time_t time =
                    parse_ublox_time_string(p_response_buffer, response_length);

                if (time == static_cast<time_t>(0))
                {
                    // Invalid time
                    break;
                }

                ublox_info.carrier_time_value = time;
                if (!init_tracker.done)
                {
                    ublox_info.init_time_value = time;
                }

                // Wait for OK response
                error = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error;
}

#if !(SIMCOM)
/**
 * @brief Handler for +ULSTFILE on initialization time command response
 * processing
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode
handle_init_list_files_for_delete_response(char *   p_command_buffer,
                                           uint32_t command_length,
                                           char *   p_response_buffer,
                                           uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+ULSTFILE:\\s*";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);

                int len;
                int first_quote_index;

                // Check if filename has shown up
                first_quote_index =
                    re_match("\"", p_response_buffer, response_length, &len);

                if (first_quote_index == RE_NO_MATCH_INDEX)
                {
                    // No files to delete
                    init_tracker.has_files_to_delete = false;
                }
                else
                {
                    int end_quote_index =
                        re_match("\"",
                                 &p_response_buffer[first_quote_index + 1],
                                 response_length - (first_quote_index + 1),
                                 &len);

                    if (end_quote_index == RE_NO_MATCH_INDEX)
                    {
                        // No files to delete
                        init_tracker.has_files_to_delete = false;
                    }
                    else
                    {
                        // Increment by 1 since we started searching at
                        // offset 1
                        end_quote_index++;

                        int filename_length =
                            end_quote_index - first_quote_index + 1;

                        memcpy(scratch_buffer,
                               &p_response_buffer[first_quote_index],
                               filename_length);
                        scratch_buffer[filename_length] = NULL_TERMINATOR;
                    }
                }
                // Wait for OK response
                error = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error;
}

/**
 * @brief Handler for +ULSTFILE on getting free space in file system command
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_UNEXPECTED_RESPONSE if unknown response
 * received
 */
static wHalErrorCode
handle_get_free_space_from_file_system_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+ULSTFILE:\\s*";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index != RE_NO_MATCH_INDEX)
                {
                    // skip over the prefix
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  prefix_index + prefix_length);

                    // Parse file size
                    prefix_index = re_match("\\d+",
                                            p_response_buffer,
                                            response_length,
                                            &prefix_length);

                    if (prefix_index != 0)
                    {
                        break;
                    }

                    // Getting maximum size of the file system
                    ublox_info.max_file_system_size = static_cast<uint32_t>(
                        atoi(p_response_buffer + prefix_index));
                }
            } while (false);
        }
    }
    return wHalErrorCode::NO_ERROR;
}

#else  // SIMCOM

/**
 * @brief Handler for +FSMEM on getting free space in file system command
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_UNEXPECTED_RESPONSE if unknown response
 * received
 */
static wHalErrorCode
handle_get_free_space_from_file_system_response(char *p_command_buffer,
                                                uint32_t command_length,
                                                char *p_response_buffer,
                                                uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+FSMEM\\s*:\\s*";

                int prefix_length;
                int prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index != RE_NO_MATCH_INDEX)
                {
                    // skip over the prefix
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  prefix_index + prefix_length);

                    // Remove initial unwanted characters
                    prefix_index = re_match("[a-zA-Z]\\s*:\\s*\\(",
                                            p_response_buffer,
                                            response_length,
                                            &prefix_length);

                    if (prefix_index != RE_NO_MATCH_INDEX)
                    {
                        // skip over the prefix
                        adjust_string_buffer_position(p_response_buffer,
                                                      response_length,
                                                      prefix_index +
                                                          prefix_length);

                        // Parse file size
                        prefix_index = re_match("\\d+",
                                                p_response_buffer,
                                                response_length,
                                                &prefix_length);

                        if (prefix_index != 0)
                        {
                            break;
                        }

                        // Getting maximum size of the file system
                        ublox_info.max_file_system_size = static_cast<uint32_t>(
                            atoi(p_response_buffer + prefix_index));
                    }
                }
            } while (false);
        }
    }
    return wHalErrorCode::NO_ERROR;
}
#endif // SIMCOM

/**
 * @brief Check that the SMS tracker is in the expected state for a
 * SMS read response.
 */
static void assert_sms_read_response_state(void)
{
    SYSTEM_ASSERT_TRUE(request_tracker.sms_tracker.is_open);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.read_write);
    SYSTEM_ASSERT_TRUE(request_tracker.sms_tracker.is_closing);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.read_completed);

    // Reads never touch the following variable so they should
    // remain unchanged
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.transfer_chars_remaining,
                        0);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
}

/**
 * @brief Handler for +CMGR command response
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_sms_read_response(char *   p_command_buffer,
                                              uint32_t command_length,
                                              char *   p_response_buffer,
                                              uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            if ((error == wHalErrorCode::NO_ERROR) ||
                (error == wHalErrorCode::GSM_ERROR))
            {
                assert_sms_read_response_state();

                request_tracker.sms_tracker.read_completed = true;
            }
        }
        else
        {
            assert_sms_read_response_state();

            const char *prefix_pattern = "\\+CMGR\\s*:\\s*";

            int         match_length;
            int         match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

            if (match_index != RE_NO_MATCH_INDEX)
            {
                error = wHalErrorCode::BUSY;
            }
            else
            {
                // This is not an "OK" or "+CMGR" SMS message header.
                // This is a message that matches the server phone number
                // and we have a valid index for the oldest message.
                // Copy the data to the buffer.
                uint32_t read_offset =
                    request_tracker.sms_tracker.read_buffer_fill_index;
                SYSTEM_ASSERT_LTEQ(read_offset + response_length,
                                   request_tracker.sms_tracker.sms_buffer_size);
                SYSTEM_ASSERT_GTEQ(
                    request_tracker.sms_tracker.request_chars_remaining,
                    response_length);

                if ((read_offset > 0) &&
                    (request_tracker.sms_tracker
                         .p_sms_buffer[read_offset - 1] != '\n') &&
                    (request_tracker.sms_tracker
                         .p_sms_buffer[read_offset - 1] != '\r'))
                {
                    // Add newline to separate lines within the same SMS
                    request_tracker.sms_tracker.p_sms_buffer[read_offset] =
                        '\n';
                    read_offset++;

                    SYSTEM_ASSERT_LTEQ(
                        read_offset + response_length,
                        request_tracker.sms_tracker.sms_buffer_size);
                }

                strncpy(request_tracker.sms_tracker.p_sms_buffer + read_offset,
                        p_response_buffer,
                        response_length);
                request_tracker.sms_tracker.request_chars_remaining -=
                    response_length;
                request_tracker.sms_tracker.read_buffer_fill_index =
                    read_offset + response_length;
            }
        }
    }

    return error;
}

/**
 * @brief Handler for +CMGL command response
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_list_sms_message_response(char *   p_command_buffer,
                                                      uint32_t command_length,
                                                      char *p_response_buffer,
                                                      uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    assert_sms_read_response_state();

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (!error_code_valid)
        {
            const char *prefix_pattern = "\\+CMGL\\s*:\\s*";

            int         match_length;
            int         match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

            if (match_index != RE_NO_MATCH_INDEX)
            {
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                do
                {
                    // Parse SMS index
                    match_index = re_match("\\d+",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index != 0)
                    {
                        break;
                    }

                    uint32_t sms_index = static_cast<uint32_t>(
                        atoi(p_response_buffer + match_index));
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);
                    skip_comma_spaces(p_response_buffer, response_length);

                    // Skip over string type field
                    match_index = re_match("\"[A-Za-z0-9\\s]*\"",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index != 0)
                    {
                        break;
                    }

                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);
                    skip_comma_spaces(p_response_buffer, response_length);

                    // Get phone number
                    match_index = re_match("\"[^\",]*\"",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index != 0)
                    {
                        break;
                    }

                    // Check if this phone number is a short code with a two
                    // character prefix and dash
                    int short_match_length;
                    int short_match_index =
                        re_match("\"[A-Za-z][A-Za-z]\\-[^\",]+\"",
                                 p_response_buffer,
                                 response_length,
                                 &short_match_length);

                    // If short code prefix is found, skip over it
                    if (short_match_index == 0)
                    {
                        // Skip beginning quote and prefix
                        adjust_string_buffer_position(p_response_buffer,
                                                      response_length,
                                                      4);

                        match_length -= 4;
                    }
                    else
                    {
                        // Just skip beginning quote
                        adjust_string_buffer_position(p_response_buffer,
                                                      response_length,
                                                      1);

                        match_length--;
                    }

                    // Compare phone number against whitelist, ignoring end
                    // quote
                    if (strncmp(interface_params.a2p_phone_number,
                                p_response_buffer,
                                strlen(interface_params.a2p_phone_number)) != 0)
                    {
                        // Messages not from server should be marked for delete
                        if (sms_index < MAX_SMS_IN_SYSTEM)
                        {
                            // Mark SMS for delete
                            init_tracker
                                .sms_index_cleanup_mask[sms_index /
                                                        SMS_MASK_BIT_WIDTH] |=
                                (1 << (sms_index % SMS_MASK_BIT_WIDTH));
                        }

                        break;
                    }

                    // This is the phone number of the server!
                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);
                    skip_comma_spaces(p_response_buffer, response_length);

                    // Skip alpha tag
                    match_index = re_match("\"?[A-Za-z0-9\\s\\+]*\"?,",
                                           p_response_buffer,
                                           response_length,
                                           &match_length);

                    if (match_index != 0)
                    {
                        break;
                    }

                    adjust_string_buffer_position(p_response_buffer,
                                                  response_length,
                                                  match_index + match_length);

                    // Parse time of message
                    time_t time = parse_ublox_time_string(p_response_buffer,
                                                          response_length);

                    if (time == static_cast<time_t>(0))
                    {
                        // Invalid time
                        break;
                    }

                    // We only care about the oldest message
                    if ((time < init_tracker.oldest_valid_sms_time) ||
                        (init_tracker.oldest_valid_sms_time ==
                         static_cast<time_t>(-1)))
                    {
                        init_tracker.oldest_valid_sms_index = sms_index;
                        init_tracker.oldest_valid_sms_time  = time;
                    }

                } while (false);
            }

            // Wait for OK response
            error = wHalErrorCode::BUSY;
        }
    }

    return error;
}

/**
 * @brief Handler for +UDWNFILE command response prompt
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_download_prompt(char *   p_command_buffer,
                                            uint32_t command_length,
                                            char *   p_response_buffer,
                                            uint32_t response_length)
{
    if ((response_length == 1) &&
        (*p_response_buffer == DOWNLOAD_RESPONSE_CHARACTER))
    {
        // Don't wait idle period after udwnfile download prompt
        command_tracker.last_response_time.clear();

        // Sequence should be set up such that after this command response,
        // file data should be sent
        return wHalErrorCode::NO_ERROR;
    }
    else
    {
        // This is to handle potential URCs that may come in before prompt
        return handle_generic_response(p_command_buffer,
                                       command_length,
                                       p_response_buffer,
                                       response_length);
    }
}

/**
 * @brief On buffer transfer completion, increment counts, check if the
 * buffer needs to be released, and check if the required bytes for for
 * +UDWNFILE has been sent.
 */
static void write_transfer_completion(void)
{
    // Check if write transfer is for SMS
    if (request_tracker.sms_tracker.is_open &&
        request_tracker.sms_tracker.read_write)
    {
        SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
        SYSTEM_ASSERT_GT(request_tracker.sms_tracker.sms_buffer_size, 0);
        SYSTEM_ASSERT_GTEQ(request_tracker.sms_tracker.transfer_chars_remaining,
                           request_tracker.sms_tracker.sms_buffer_size);
        request_tracker.sms_tracker.transfer_chars_remaining -=
            request_tracker.sms_tracker.sms_buffer_size;

        SYSTEM_ASSERT_EQUAL(
            request_tracker.sms_tracker.request_chars_remaining,
            request_tracker.sms_tracker.transfer_chars_remaining);

        request_tracker.sms_tracker.p_sms_buffer    = nullptr;
        request_tracker.sms_tracker.sms_buffer_size = 0;
    }
    else if (request_tracker.upload_buffer_size > 0)
    {
        handle_tracker_st *p_handle_tracker =
            &request_tracker.handle_trackers[request_tracker.upload_index];

        p_handle_tracker->uploaded_bytes += request_tracker.send_size;
        request_tracker.cmd_bytes_transferred += request_tracker.send_size;
        request_tracker.upload_buffer_transferred += request_tracker.send_size;

        if (request_tracker.upload_buffer_transferred ==
            request_tracker.upload_buffer_size)
        {
            // release buffer
            request_tracker.upload_index       = INVALID_HANDLE_TRACKER_INDEX;
            request_tracker.p_upload_buffer    = nullptr;
            request_tracker.upload_buffer_size = 0;
            request_tracker.upload_buffer_transferred = 0;
        }

        if (request_tracker.cmd_bytes_transferred == request_tracker.cmd_size)
        {
            request_tracker.cmd_size              = 0;
            request_tracker.cmd_bytes_transferred = 0;
        }

        request_tracker.send_size = 0;
    }
}

/**
 * @brief Handler for completion of sending data for +UDWNFILE command
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_download_completion(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length)
{
    wHalErrorCode error = handle_generic_response(p_command_buffer,
                                                  command_length,
                                                  p_response_buffer,
                                                  response_length);

    if ((error == wHalErrorCode::NO_ERROR) ||
        (error == wHalErrorCode::GSM_ERROR))
    {
        write_transfer_completion();
    }

    return error;
}

/**
 * @brief Reset SMS send tracker
 */
static void reset_sms_send_tracker()
{
    request_tracker.sms_tracker.terminator_sent = false;
    request_tracker.sms_tracker.is_terminated   = false;
    request_tracker.sms_tracker.is_closing      = false;
    request_tracker.sms_tracker.is_open         = false;
}

/**
 * @brief Handler for completion of sending data for +CMGS command
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_sms_response(char *   p_command_buffer,
                                         uint32_t command_length,
                                         char *   p_response_buffer,
                                         uint32_t response_length)
{

    wHalErrorCode error_code = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error_code,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            if (error_code == wHalErrorCode::GSM_ERROR)
            {
                // For now, just ignore errors and continue
                SYSTEM_ASSERT_TRUE(request_tracker.sms_tracker.terminator_sent);
                SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
                request_tracker.sms_tracker.is_terminated = true;

                if (request_tracker.sms_tracker.is_closing)
                {
                    // Reset tracker
                    reset_sms_send_tracker();
                }
            }

            return error_code;
        }
        else
        {
            const char *prefix_pattern = "\\+CMGS\\s*:\\s*";

            int         match_length;
            int         match_index = re_match(prefix_pattern,
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

            if (match_index != RE_NO_MATCH_INDEX)
            {
                SYSTEM_ASSERT_TRUE(request_tracker.sms_tracker.is_open);
                SYSTEM_ASSERT_TRUE(request_tracker.sms_tracker.terminator_sent);
                SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.sms_buffer_size,
                                    0);
                SYSTEM_ASSERT_EQUAL(
                    request_tracker.sms_tracker.transfer_chars_remaining,
                    0);
                SYSTEM_ASSERT_EQUAL(
                    request_tracker.sms_tracker.request_chars_remaining,
                    0);
                SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.sms_buffer_size,
                                    0);

                SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
                request_tracker.sms_tracker.is_terminated = true;

                if (request_tracker.sms_tracker.is_closing)
                {
                    // Reset tracker
                    reset_sms_send_tracker();
                }

                // Wait for OK response
                error_code = wHalErrorCode::BUSY;
            }
        }
    }

    return error_code;
}

#if !(SIMCOM)
/**
 * @brief Handler for completion of deleting file on gsm filesystem
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_delete_file_response(char *   p_command_buffer,
                                                 uint32_t command_length,
                                                 char *   p_response_buffer,
                                                 uint32_t response_length)
{
    wHalErrorCode error = handle_generic_response(p_command_buffer,
                                                  command_length,
                                                  p_response_buffer,
                                                  response_length);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // release handle
        uint8_t deleting_index = request_tracker.deleting_index;
        SYSTEM_ASSERT_NOT_EQUAL(deleting_index, INVALID_HANDLE_TRACKER_INDEX);

        // Clear tracker here
        request_tracker.handle_trackers[deleting_index].handle =
            INVALID_GSM_HANDLE;
        request_tracker.handle_trackers[deleting_index].filename[0] =
            NULL_TERMINATOR;

        request_tracker.deleting_index = INVALID_HANDLE_TRACKER_INDEX;
    }

    return error;
}
#endif

/**
 * @brief Handler for parsing +UHTTPER response
 *
 * @param p_command_buffer the command sent to the gsm module
 * @param command_length the length of the command string
 * @param p_response_buffer the response from the gsm module
 * @param response_length the length of the response string
 * @return wHalErrorCode BUSY if command not complete yet, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode handle_http_error_response(char *   p_command_buffer,
                                                uint32_t command_length,
                                                char *   p_response_buffer,
                                                uint32_t response_length)
{
    wHalErrorCode error = wHalErrorCode::GSM_UNEXPECTED_RESPONSE;
    bool          error_code_valid;

    if (response_precheck(p_response_buffer,
                          response_length,
                          error,
                          error_code_valid))
    {
        if (error_code_valid)
        {
            return error;
        }
        else
        {
            do
            {
                const char prefix_pattern[] = "\\+UHTTPER\\s*:\\s*";

                int        prefix_length;
                int        prefix_index;

                // Check if expected prefix has shown up
                prefix_index = re_match(prefix_pattern,
                                        p_response_buffer,
                                        response_length,
                                        &prefix_length);

                if (prefix_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // skip over the prefix
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              prefix_index + prefix_length);

                int match_length;
                int match_index;

                // Look for number
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get number
                int protocol_error_class =
                    static_cast<int>(atoi(&p_response_buffer[match_index]));

                // skip over the number
                adjust_string_buffer_position(p_response_buffer,
                                              response_length,
                                              match_index + match_length);

                // skip over the commas and spaces
                skip_comma_spaces(p_response_buffer, response_length);

                // Look for number
                match_index = re_match("\\d+",
                                       p_response_buffer,
                                       response_length,
                                       &match_length);

                if (match_index == RE_NO_MATCH_INDEX)
                {
                    break;
                }

                // Get number
                int protocol_error_code =
                    static_cast<int>(atoi(&p_response_buffer[match_index]));

                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                  "HTTP Protocol Error Class: %d, Code: %d\n",
                                  protocol_error_class,
                                  protocol_error_code);

                // Wait for OK response
                error = wHalErrorCode::BUSY;
            } while (false);
        }
    }

    return error;
}

/**
 * @brief Check for response
 *
 * @param response_handler response handler for specific command
 * @return wHalErrorCode BUSY if response not fulfilled, NO_ERROR if
 * successful response received, GSM_ERROR if error response received,
 * GSM_UNEXPECTED_RESPONSE if unknown response received
 */
static wHalErrorCode check_for_response(ublox_response_func_t response_handler)
{
    wHalErrorCode status_error;

    status_error = w_hal_uart_check_write_status(GSM_UART_CHANNEL);

    switch (status_error)
    {
        case wHalErrorCode::UART_WRITE_IN_PROGRESS:
            // Still waiting for write data to be transferred
            return wHalErrorCode::BUSY;
        case wHalErrorCode::UART_WRITE_COMPLETE:
            w_hal_uart_clear_write_completion_status(GSM_UART_CHANNEL);
            if (response_handler == nullptr)
            {
                // This is a write transfer that is part of a larger
                // +UDWNFILE command
                write_transfer_completion();
                return wHalErrorCode::NO_ERROR;
            }
            // FALL-THROUGH
        case wHalErrorCode::UART_NO_ACTIVE_WRITE:
            break;
        default:
            SYSTEM_ASSERT_TRUE(false);
    }

    status_error = w_hal_uart_check_read_status(GSM_UART_CHANNEL);

    wHalErrorCode response_error;

    if (status_error == wHalErrorCode::UART_NO_ACTIVE_READ)
    {
        // Scan index and processed index should be the same
        // since no read is in progress
        SYSTEM_ASSERT_EQUAL(command_tracker.response_scan_index, 0);
        SYSTEM_ASSERT_EQUAL(command_tracker.response_processed_index, 0);

        // Always read max length and abort when last line of response
        // has been received
        wHalErrorCode error =
            w_hal_uart_read_data(GSM_UART_CHANNEL,
                                 reinterpret_cast<uint8_t *>(response_buffer),
                                 RESP_BUFF_SIZE);

        // This would be an unhandled exception
        SYSTEM_ASSERT_NOT_EQUAL(error, wHalErrorCode::UART_READ_FAILED);
        SYSTEM_ASSERT_NOT_EQUAL(error, wHalErrorCode::BUSY);

        if (error != wHalErrorCode::UART_READ_COMPLETE)
        {
            return wHalErrorCode::BUSY;
        }

        // Check read status
        status_error = w_hal_uart_check_read_status(GSM_UART_CHANNEL);

        SYSTEM_ASSERT_EQUAL(status_error, wHalErrorCode::UART_READ_COMPLETE);
    }

    const wHalErrorCode valid_status_errors[] = {
        wHalErrorCode::UART_READ_IN_PROGRESS,
        wHalErrorCode::UART_READ_COMPLETE};
    SYSTEM_ASSERT_IN_SET(status_error,
                         valid_status_errors,
                         sizeof(valid_status_errors) / sizeof(wHalErrorCode));

    response_error                            = wHalErrorCode::BUSY;

    const wHalErrorCode valid_stop_statuses[] = {
        wHalErrorCode::UART_READ_STOPPED,
        wHalErrorCode::UART_READ_COMPLETE};

    // Search for the end
    uint32_t scan_index                = command_tracker.response_scan_index;
    bool     found_response_terminator = false;
    bool     found_null_terminator     = false;
    for (scan_index = scan_index; scan_index < RESP_BUFF_SIZE; scan_index++)
    {
        volatile char next_char;

        if (scan_index + 1 < RESP_BUFF_SIZE)
        {
            next_char = response_buffer[scan_index + 1];
        }
        else
        {
            next_char = NULL_TERMINATOR;
        }

        // Acquire the next char before this one to make the assert checks pass.
        volatile char response_char = response_buffer[scan_index];

        // If null terminator is found, then there is nothing more to scan
        if (response_char == NULL_TERMINATOR)
        {
            // Sometimes LARA-R6 will send sequence 0x00-0xFF as a response
            // to a command. This is not a valid response, so it should be
            // ignored.
            if (next_char == 0xFF)
            {
                // Known invalid character sequences in a
                // response can be easily skipped.
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                  "Likely Frame error: Unexpected 0x00-0xFF at "
                                  "byte positions %d in "
                                  "response buffer and %d in "
                                  "most recent line.\n",
                                  scan_index,
                                  scan_index -
                                      command_tracker.response_processed_index);
                command_tracker.response_scan_index += 2;
                command_tracker.response_processed_index += 2;

                // scan_index will be incremented once more by loop
                scan_index = scan_index + 1;

                // Move on to next characters
                continue;
            }
            else
            {
                found_null_terminator = true;
                // If corruption did not happen, then next character should also
                // be null terminator.
                SYSTEM_ASSERT_EQUAL(next_char, NULL_TERMINATOR);
                break;
            }
        }
        else if (response_char == 0xFF)
        {
            // LARA can sometimes send a single 0xFF character as a response
            // to a command. This is not a valid response, so it should be
            // ignored.

            // Known invalid character sequences at the beginning of a
            // response can be easily skipped.
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                              "Likely Frame error: Unexpected 0xFF at byte "
                              "position %d in "
                              "response buffer and %d in "
                              "most recent line.\n",
                              scan_index,
                              scan_index -
                                  command_tracker.response_processed_index);
            command_tracker.response_scan_index += 1;
            command_tracker.response_processed_index += 1;

            // scan_index will be incremented by loop
            continue;
        }

        // Check if character is in range of ASCII characters
        SYSTEM_ASSERT_GTEQ(response_char, ASCII_MIN);
        SYSTEM_ASSERT_LTEQ(response_char, ASCII_MAX);

        // If null terminator has not been seen, but response terminator has
        // been seen, then break
        if (found_response_terminator)
        {
            break;
        }

        if (command_tracker.response_terminator_flags.bit.s3 &&
            (response_char == S3_CHARACTER))
        {
            found_response_terminator = true;
        }
        else if (command_tracker.response_terminator_flags.bit.s4 &&
                 (response_char == S4_CHARACTER))
        {
            found_response_terminator = true;
        }
        else if (command_tracker.response_terminator_flags.bit
                     .download_prompt &&
                 (response_char == DOWNLOAD_RESPONSE_CHARACTER))
        {
            found_response_terminator = true;
        }
        else if (command_tracker.response_terminator_flags.bit.space &&
                 (response_char == ' '))
        {
            found_response_terminator = true;
        }
        else if (command_tracker.response_terminator_flags.bit.ignore &&
                 (response_char == IGNORE_CHARACTER))
        {
            found_response_terminator = true;
        }
    }

    if (scan_index != command_tracker.response_scan_index)
    {
        // Update structure with new scan index
        command_tracker.response_scan_index = scan_index;

        // Need to wait 20 ms before sending next command to
        // avoid collision with a URC response.
        // Record the time that the response was received.
        w_hal_rtc_get_time(command_tracker.last_response_time);
    }

    if (found_response_terminator)
    {
#if HAL_GSM_AT_DEBUG
        char     print_buffer[64];
        uint32_t strncpy_length =
            scan_index - command_tracker.response_processed_index;

        // Trim off any trailing S3 or S4 characters
        while ((strncpy_length > 0) &&
               ((response_buffer[command_tracker.response_processed_index +
                                 strncpy_length - 1] == S3_CHARACTER) ||
                (response_buffer[command_tracker.response_processed_index +
                                 strncpy_length - 1] == S4_CHARACTER)))
        {
            strncpy_length--;
        }

        if (strncpy_length > sizeof(print_buffer) - 1)
        {
            strncpy_length = sizeof(print_buffer) - 1;
        }

        if (strncpy_length > 0)
        {
            // String to print should be greater than 0 and it should
            strncpy(print_buffer,
                    &response_buffer[command_tracker.response_processed_index],
                    strncpy_length);
            print_buffer[strncpy_length] = NULL_TERMINATOR;

            printf("AT_RSP: %s\n", print_buffer);
        }
#endif

        response_error = response_handler(
            command_buffer,
            command_tracker.command_buffer_length,
            &response_buffer[command_tracker.response_processed_index],
            command_tracker.response_scan_index -
                command_tracker.response_processed_index);

        command_tracker.response_processed_index =
            command_tracker.response_scan_index;

        // BUSY code means no actual response information received
        if (response_error != wHalErrorCode::BUSY)
        {
            command_tracker.received_response = true;

            // Any error code will supersede NO_ERROR code
            if (command_tracker.response_error != wHalErrorCode::NO_ERROR)
            {
                command_tracker.response_error = response_error;
            }
        }

        if (command_tracker.received_response && found_null_terminator &&
            (status_error == wHalErrorCode::UART_READ_IN_PROGRESS))
        {
            // Try to stop read
            wHalErrorCode stop_status = w_hal_uart_stop_read(GSM_UART_CHANNEL);
            SYSTEM_ASSERT_IN_SET(stop_status,
                                 valid_stop_statuses,
                                 sizeof(valid_stop_statuses) /
                                     sizeof(wHalErrorCode));

            // If more characters came in after stopping the read,
            // then resume the read to get the remaining response
            if ((scan_index < RESP_BUFF_SIZE) &&
                (response_buffer[scan_index] != NULL_TERMINATOR))
            {
                // Since read needs to continue, flags need to be
                // adjusted to not move on to next command
                found_null_terminator = false;
                response_error        = wHalErrorCode::BUSY;

                // Search for the last character received
                uint32_t resume_index;
                for (resume_index = scan_index; resume_index < RESP_BUFF_SIZE;
                     resume_index++)
                {
                    if (response_buffer[resume_index] == NULL_TERMINATOR)
                    {
                        break;
                    }
                }

                // Don't resume read if resume index is at the end of the
                // buffer
                if (resume_index < RESP_BUFF_SIZE)
                {
                    // Resume read from after last character
                    w_hal_uart_read_data(GSM_UART_CHANNEL,
                                         reinterpret_cast<uint8_t *>(
                                             &response_buffer[resume_index]),
                                         RESP_BUFF_SIZE - resume_index);
                }
            }
        }
    }
    else if (scan_index == RESP_BUFF_SIZE)
    {
        // Not possible to have a response that is longer than the buffer
        SYSTEM_ASSERT_FALSE(found_null_terminator);

        wHalErrorCode error = w_hal_uart_check_read_status(GSM_UART_CHANNEL);

        switch (error)
        {
            case wHalErrorCode::UART_READ_COMPLETE:
            {
                w_hal_uart_clear_read_completion_status(GSM_UART_CHANNEL);
                break;
            }
            case wHalErrorCode::UART_READ_IN_PROGRESS:
            {
                // Try to stop read
                wHalErrorCode stop_status =
                    w_hal_uart_stop_read(GSM_UART_CHANNEL);
                SYSTEM_ASSERT_IN_SET(stop_status,
                                     valid_stop_statuses,
                                     sizeof(valid_stop_statuses) /
                                         sizeof(wHalErrorCode));
                break;
            }
            default:
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::UART_NO_ACTIVE_READ);
                break;
            }
        }

        // Try to restart the read while keeping the portion that has not
        // yet been processed.
        scan_index = RESP_BUFF_SIZE - command_tracker.response_processed_index;
        uint32_t read_resume_size = command_tracker.response_processed_index;

        // Keep the unprocessed portion of the buffer
        memcpy(response_buffer,
               &response_buffer[command_tracker.response_processed_index],
               scan_index);
        memset(&response_buffer[scan_index], 0, read_resume_size);

        command_tracker.response_processed_index = 0;
        command_tracker.response_scan_index      = scan_index;

        // Resume read from after last character
        w_hal_uart_read_data(
            GSM_UART_CHANNEL,
            reinterpret_cast<uint8_t *>(&response_buffer[scan_index]),
            read_resume_size);
    }

    // If response has been received and processed, reset tracker
    if (command_tracker.received_response && found_null_terminator &&
        (scan_index == command_tracker.response_processed_index))
    {
        SYSTEM_ASSERT_EQUAL(scan_index, command_tracker.response_scan_index);

        // Use response code from the line that gave actual
        // response information
        response_error = command_tracker.response_error;

        // If error, then leave the response buffer for further debug
        if (response_error == wHalErrorCode::NO_ERROR)
        {
            memset(response_buffer, 0, scan_index);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                              "Response error code: %u\n",
                              response_error);
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                              "Command buffer:\n%s\n",
                              command_buffer);
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                              "Response buffer:\n%s\n",
                              response_buffer);
        }

        // Reset tracker variables except for last response time
        command_tracker.response_error           = wHalErrorCode::NO_ERROR;
        command_tracker.received_response        = false;
        command_tracker.command_buffer_length    = 0;
        command_tracker.response_processed_index = 0;
        command_tracker.response_scan_index      = 0;

        // Always keep a read going on to make sure any URC is received
        wHalErrorCode error =
            w_hal_uart_read_data(GSM_UART_CHANNEL,
                                 reinterpret_cast<uint8_t *>(response_buffer),
                                 RESP_BUFF_SIZE);

        // This would be an unhandled exception
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::UART_READ_IN_PROGRESS);
    }
    else
    {
        response_error = wHalErrorCode::BUSY;
    }

    return response_error;
}

/**
 * @brief Send command to the gsm module over UART
 *
 * @tparam Args typename for array of set value args
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_command command (not including AT prefix)
 * @param length length of the command (not including null-terminator)
 * @param type AT command type
 * @param p_set_value_format parameter format string for SET AT command type
 * @param args arguments to set value format
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
template <typename... Args>
wHalErrorCode send_command(char *            p_command_buffer,
                           uint32_t          buffer_size,
                           const char *      p_command,
                           uint32_t          length,
                           at_command_type_t type,
                           const char *      p_set_value_format = "",
                           Args... set_value_args)
{
    uint32_t total_size    = 0;

    // Set up prefix
    uint32_t prefix_length = sizeof(COMMAND_PREFIX) - 1;
    SYSTEM_ASSERT_LT(prefix_length, buffer_size - total_size);
    memcpy(p_command_buffer, COMMAND_PREFIX, prefix_length);
    total_size += prefix_length;

    SYSTEM_ASSERT_LT(length, buffer_size - total_size);
    strncpy(&p_command_buffer[total_size], p_command, length);
    total_size += length;

    SYSTEM_ASSERT_GT(buffer_size - total_size, 1);
    switch (type)
    {
        case at_command_type_t::ACTION:
        {
            // No added parameters
            // needed here
            break;
        }
        case at_command_type_t::SET:
        {
            p_command_buffer[total_size] = SET_CHARACTER;
            total_size++;
        }
        // FALL-THROUGH
        case at_command_type_t::SET_NO_EQUAL:
        {
            uint32_t set_str_length = snprintf(&p_command_buffer[total_size],
                                               buffer_size - total_size,
                                               p_set_value_format,
                                               set_value_args...);
            SYSTEM_ASSERT_LT(set_str_length, buffer_size - total_size);
            total_size += set_str_length;
            break;
        }
        case at_command_type_t::READ:
        {
            p_command_buffer[total_size] = READ_CHARACTER;
            total_size++;
            break;
        }
        case at_command_type_t::TEST:
        {
            uint32_t test_str_length = sizeof(TEST_CHARACTERS) - 1;
            SYSTEM_ASSERT_LT(test_str_length, buffer_size - total_size);
            memcpy(&p_command_buffer[total_size],
                   TEST_CHARACTERS,
                   test_str_length);
            total_size += test_str_length;
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    // Add carriage return to complete command
    SYSTEM_ASSERT_GT(buffer_size - total_size, 1);
    p_command_buffer[total_size] = S3_CHARACTER;
    total_size++;

    // Set length of the command buffer
    command_tracker.command_buffer_length = total_size;

    // Null-terminator (not sent over UART but used for comparison
    // against response)
    p_command_buffer[total_size]          = NULL_TERMINATOR;

#if HAL_GSM_AT_DEBUG
    printf("AT_CMD: %s\n", p_command_buffer);
#endif

    return w_hal_uart_write_data(GSM_UART_CHANNEL,
                                 reinterpret_cast<uint8_t *>(p_command_buffer),
                                 total_size);
}

/**
 * @brief Send command to the gsm module over UART
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode generic_send_command(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info)
{
    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value);
}

/**
 * @brief Send command to the gsm module over UART if data is enabled.
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
generic_send_command_data_only(char *                       p_command_buffer,
                               uint32_t                     buffer_size,
                               const ublox_command_info_st *p_info)
{
    if (!interface_params.data_enabled)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value);
}

#if !(SIMCOM)
/**
 * @brief Send command to the gsm module over UART
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_gsm_shutdown_command(char *                       p_command_buffer,
                          uint32_t                     buffer_size,
                          const ublox_command_info_st *p_info)
{
    if (is_lara_R2_module())
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value);
}
#endif // !(SIMCOM)

/**
 * @brief Send command to the gsm module over UART
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode get_local_time(char *   p_command_buffer,
                                    uint32_t buffer_size,
                                    const ublox_command_info_st *p_info)
{
    if (!is_sim800l_module())
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value);
}

#if !(SIMCOM)
/**
 * @brief Send command to disable message waiting indicator URC
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
disable_message_waiting_urc(char *                       p_command_buffer,
                            uint32_t                     buffer_size,
                            const ublox_command_info_st *p_info)
{
    // LARA-L6 and LARA-R6 do not support MT storage
    if (is_lara_R6_L6_module())
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value);
}

/**
 * @brief Send command to enable +CSCON URC
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
enable_signalling_connection_status(char *   p_command_buffer,
                                    uint32_t buffer_size,
                                    const ublox_command_info_st *p_info)
{
    // LARA-R2 and SARA do not support +CSCON command
    if (!is_lara_R6_L6_module())
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
    else
    {
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value);
    }
}

/**
 * @brief Send command to the gsm module over UART to setup uart frame
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_uart_parity_setup(char *   p_command_buffer,
                                            uint32_t buffer_size,
                                            const ublox_command_info_st *p_info)
{
    if (is_lara_R6_L6_module())
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
    else
    {
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value);
    }
}
#endif

/**
 * @brief Send command to the gsm module over UART to set uart baud rate
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_uart_baud_setup(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info)
{
    if (BAUD_RATE == INITIAL_BAUD_RATE)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    uint32_t baud_rate = is_sara_module() ? SARA_MAX_BAUD_RATE : BAUD_RATE;

    // Incoming audio data rate plus margin cannot exceed baud rate
    if (interface_params.data_enabled)
    {
        SYSTEM_ASSERT_LTEQ(
            (interface_params.data_bit_rate * BAUD_MARGIN_FACTOR) /
                BAUD_MARGIN_DIVISOR,
            baud_rate);
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        baud_rate);
}

/**
 * @brief Send command to the gsm module over UART to get temperature
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode get_temperature(char *   p_command_buffer,
                                     uint32_t buffer_size,
                                     const ublox_command_info_st *p_info)
{
    if (is_lara_R2_module() || is_sara_module())
    {
        // Workaround for SARA and LARA-R2
        // They do not support AT+UTEMP command, so skip this step
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
    else if (is_sim800l_module())
    {
        // Send AT+CMTE command
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value);
    }
    else
    {
        // Send AT+UTEMP command
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value);
    }
}

#if !(SIMCOM)
/**
 * @brief Read the configured bands for this modem
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_uband_mask_read_command(char *                       p_command_buffer,
                             uint32_t                     buffer_size,
                             const ublox_command_info_st *p_info)
{
    if (!is_lara_R6_L6_module())
    {
        // Only LARA-L6 and LARA-R6 support this command
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
    else
    {
        // Send AT+UBANDMASK? command
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type);
    }
}

/**
 * @brief Write the configured bands for this modem
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_uband_mask_write_command(char *                       p_command_buffer,
                              uint32_t                     buffer_size,
                              const ublox_command_info_st *p_info)
{
    if (!is_lara_R6_module())
    {
        // Only LARA-R6 supports this command
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
    else
    {
        if (init_tracker.operator_type == operator_t::NUM_OPERATORS)
        {
            // Skip setting band mask since operator type is not set
            return wHalErrorCode::GSM_SKIP_COMMAND;
        }

        SYSTEM_ASSERT_LT(static_cast<uint32_t>(init_tracker.operator_type),
                         static_cast<uint32_t>(operator_t::NUM_OPERATORS));

        // Set band mask
        const uint64_t band_mask_1 =
            OPERATOR_SUPPORTED_LTE_BANDS[static_cast<uint32_t>(
                                             init_tracker.operator_type)]
                .mask1;
        const uint64_t band_mask_2 =
            OPERATOR_SUPPORTED_LTE_BANDS[static_cast<uint32_t>(
                                             init_tracker.operator_type)]
                .mask2;

        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                          "Setting band masks 1: %llu %llu\n",
                          band_mask_1,
                          band_mask_2);

        // Send AT+UBANDMASK command
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value,
                            band_mask_1,
                            band_mask_2);
    }
}
#endif //!(SIMCOM)

/**
 * @brief Send detach command to the gsm module over UART
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
init_detach_from_network(char *                       p_command_buffer,
                         uint32_t                     buffer_size,
                         const ublox_command_info_st *p_info)
{
    uint32_t index;

    // Find an active PDP context with the wrong APN
    for (index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
    {
        if (!init_tracker.pdp_context_exists[index] &&
            init_tracker.apn_change_needed[index])
        {

            break;
        }
    }

    // If no PDP context with wrong APN is found, then skip this command
    if (index == NUM_USED_PDP_CONTEXTS)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
    else
    {
        // Send AT+CGATT=0 command
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value);
    }
}

/**
 * @brief Send activate command to the gsm module over UART
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_activate_command(char *   p_command_buffer,
                                           uint32_t buffer_size,
                                           const ublox_command_info_st *p_info)
{
    pdp_context_id_t pdp_context_id = INVALID_PDP_CONTEXT_ID;

    // Find an inactive PDP context with the correct APN set
    for (uint32_t index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
    {
        if (init_tracker.pdp_context_exists[index] &&
            !init_tracker.pdp_context_active[index] &&
            !init_tracker.apn_change_needed[index])
        {
            if (index == 0)
            {
                pdp_context_id = DEFAULT_PDP_CONTEXT_ID;
            }
            else
            {
                pdp_context_id = DEFAULT_PDP_CONTEXT_ID_2;
            }

            init_tracker.pdp_context_active[index] = true;

            break;
        }
    }

    if (pdp_context_id == INVALID_PDP_CONTEXT_ID)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    // Send AT+CGACT or AT+UPSDA command
    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        static_cast<int>(pdp_context_id));
}

/**
 * @brief Send deactivate command to the gsm module over UART
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_deactivate_command(char *                       p_command_buffer,
                        uint32_t                     buffer_size,
                        const ublox_command_info_st *p_info)
{
    pdp_context_id_t pdp_context_id = INVALID_PDP_CONTEXT_ID;

    // Find an active PDP context with the wrong APN
    for (uint32_t index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
    {
        if (init_tracker.pdp_context_exists[index] &&
            init_tracker.pdp_context_active[index] &&
            init_tracker.apn_change_needed[index])
        {
            if (index == 0)
            {
                pdp_context_id = DEFAULT_PDP_CONTEXT_ID;
            }
            else
            {
                pdp_context_id = DEFAULT_PDP_CONTEXT_ID_2;
            }

            init_tracker.pdp_context_active[index] = false;

            break;
        }
    }

    if (pdp_context_id == INVALID_PDP_CONTEXT_ID)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    // Send AT+CGACT or AT+UPSDA command
    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        static_cast<int>(pdp_context_id));
}

/**
 * @brief Set the apn name for the operator
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode set_apn_name(char *                       p_command_buffer,
                                  uint32_t                     buffer_size,
                                  const ublox_command_info_st *p_info)
{
    pdp_context_id_t pdp_context_id = INVALID_PDP_CONTEXT_ID;

    for (uint32_t index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
    {
        if (!init_tracker.pdp_context_exists[index] ||
            (!init_tracker.pdp_context_active[index] &&
             init_tracker.apn_change_needed[index]))
        {
            if (index == 0)
            {
                pdp_context_id = DEFAULT_PDP_CONTEXT_ID;
            }
            else
            {
                pdp_context_id = DEFAULT_PDP_CONTEXT_ID_2;
            }

            init_tracker.pdp_context_exists[index] = true;
            init_tracker.apn_change_needed[index]  = false;

            break;
        }
    }

    if (pdp_context_id == INVALID_PDP_CONTEXT_ID)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    const char *const pdp_type_string =
        (pdp_context_id == DEFAULT_PDP_CONTEXT_ID) ? "IPV4V6" : "IP";

    // Send AT+CGDCONT command
    return send_command(
        p_command_buffer,
        buffer_size,
        p_info->p_command,
        p_info->command_length,
        p_info->type,
        p_info->p_set_value,
        static_cast<int>(pdp_context_id),
        pdp_type_string,
        OPERATOR_APNS[static_cast<int>(init_tracker.operator_type)]);
}

/**
 * @brief Set the internal PDP context apn name for the operator
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode set_internal_apn_name(char *   p_command_buffer,
                                           uint32_t buffer_size,
                                           const ublox_command_info_st *p_info)
{
    // Send AT+UPSD command
    return send_command(
        p_command_buffer,
        buffer_size,
        p_info->p_command,
        p_info->command_length,
        p_info->type,
        p_info->p_set_value,
        OPERATOR_APNS[static_cast<int>(init_tracker.operator_type)]);
}

#if !(SIMCOM)
/**
 * @brief Send command to the gsm module over UART to set http server name
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode set_http_server_name(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info)
{
    int uhttp_opcode = 1; // 0: HTTP server IP address, 1: HTTP server name

    // Check if server url is an IP address
    int match_length = 0;
    if (re_match("[0-9\\.]+",
                 interface_params.server_url,
                 MAX_SERVER_URL_LENGTH,
                 &match_length) == 0)
    {
        uhttp_opcode = 0;
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        uhttp_opcode,
                        interface_params.server_url);
}

/**
 * @brief Send command to the gsm module over UART to set http server port
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode set_http_server_port(char *   p_command_buffer,
                                          uint32_t buffer_size,
                                          const ublox_command_info_st *p_info)
{
    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        interface_params.server_port);
}

/**
 * @brief Send command to the gsm module over UART to set http request_timeout
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode set_http_timeout(char *   p_command_buffer,
                                      uint32_t buffer_size,
                                      const ublox_command_info_st *p_info)
{
    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        GSM_COMMAND_TIMEOUT_SECONDS);
}
#endif // !(SIMCOM)

/**
 * @brief Send +CMGS command to the gsm module over UART to send SMS message
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode start_sms(char *                       p_command_buffer,
                               uint32_t                     buffer_size,
                               const ublox_command_info_st *p_info)
{
    const char *p_phone_number = interface_params.p2a_phone_number;
    SYSTEM_ASSERT_NOT_EQUAL(p_phone_number[0], NULL_TERMINATOR);

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        p_phone_number);
}

/**
 * @brief Send SMS data to the gsm module over UART to send SMS message
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_sms_data(char *   p_command_buffer,
                                   uint32_t buffer_size,
                                   const ublox_command_info_st *p_info)
{
    SYSTEM_ASSERT_NOT_EQUAL(request_tracker.sms_tracker.p_sms_buffer,
                            static_cast<char *>(nullptr));
    SYSTEM_ASSERT_GT(request_tracker.sms_tracker.sms_buffer_size, 0);

    return w_hal_uart_write_data(GSM_UART_CHANNEL,
                                 reinterpret_cast<const uint8_t *>(
                                     request_tracker.sms_tracker.p_sms_buffer),
                                 request_tracker.sms_tracker.sms_buffer_size);
}

/**
 * @brief Send flush data to the gsm module over UART to fill the remaining
 * buffer
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_flush_data(char *   p_command_buffer,
                                     uint32_t buffer_size,
                                     const ublox_command_info_st *p_info)
{
    wHalErrorCode error = wHalErrorCode::GSM_SKIP_COMMAND;

    if (check_in_middle_of_file_transfer())
    {
        memset(command_buffer, 0, sizeof(command_buffer));

        uint32_t min_buffer_length = std::min(
            static_cast<uint32_t>(sizeof(command_buffer)),
            request_tracker.cmd_size - request_tracker.cmd_bytes_transferred);

        error =
            w_hal_uart_write_data(GSM_UART_CHANNEL,
                                  reinterpret_cast<uint8_t *>(command_buffer),
                                  min_buffer_length);

        request_tracker.cmd_bytes_transferred += min_buffer_length;
    }

    return error;
}

/**
 * @brief Send SMS terminator to the gsm module over UART to send SMS message
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_sms_terminator(char *   p_command_buffer,
                                         uint32_t buffer_size,
                                         const ublox_command_info_st *p_info)
{
    SYSTEM_ASSERT_LT(p_info->command_length, buffer_size);
    strncpy(p_command_buffer, p_info->p_command, p_info->command_length);

    return w_hal_uart_write_data(
        GSM_UART_CHANNEL,
        reinterpret_cast<const uint8_t *>(p_command_buffer),
        p_info->command_length);
}

/**
 * @brief Send command to the gsm module over UART to write data to the
 * gsm filesystem
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode write_to_ublox_file(char *   p_command_buffer,
                                         uint32_t buffer_size,
                                         const ublox_command_info_st *p_info)
{
    uint8_t upload_index = request_tracker.upload_index;
    SYSTEM_ASSERT_NOT_EQUAL(upload_index, INVALID_HANDLE_TRACKER_INDEX);

    wGsmTransferHandle upload_handle =
        request_tracker.handle_trackers[upload_index].handle;
    SYSTEM_ASSERT_NOT_EQUAL(upload_handle, INVALID_GSM_HANDLE);

    char *p_upload_filename =
        request_tracker.handle_trackers[upload_index].filename;
    SYSTEM_ASSERT_NOT_EQUAL(p_upload_filename[0], NULL_TERMINATOR);

    SYSTEM_ASSERT_GT(request_tracker.upload_buffer_size, 0);
    SYSTEM_ASSERT_GT(request_tracker.cmd_size, 0);

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        static_cast<uint32_t>(upload_handle),
                        p_upload_filename,
                        request_tracker.cmd_size);
}

/**
 * @brief Send write data to the gsm filesystem
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_data_to_ublox_file(char *                       p_command_buffer,
                        uint32_t                     buffer_size,
                        const ublox_command_info_st *p_info)
{
    SYSTEM_ASSERT_GT(request_tracker.send_size, 0);

    return w_hal_uart_write_data(GSM_UART_CHANNEL,
                                 request_tracker.p_upload_buffer +
                                     request_tracker.upload_buffer_transferred,
                                 request_tracker.send_size);
}

/**
 * @brief Send post command data to the gsm module
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_post_command(char *   p_command_buffer,
                                       uint32_t buffer_size,
                                       const ublox_command_info_st *p_info)
{
    uint8_t closing_index = request_tracker.closing_index;

    SYSTEM_ASSERT_NOT_EQUAL(closing_index, INVALID_HANDLE_TRACKER_INDEX);

    wGsmTransferHandle closing_handle =
        request_tracker.handle_trackers[closing_index].handle;
    SYSTEM_ASSERT_NOT_EQUAL(closing_handle, INVALID_GSM_HANDLE);

    char *p_closing_filename =
        request_tracker.handle_trackers[closing_index].filename;
    SYSTEM_ASSERT_NOT_EQUAL(p_closing_filename[0], NULL_TERMINATOR);

    request_tracker.handle_trackers[closing_index].last_command_time =
        w_hal_rtc_get_time();

    const char *p_api_route =
        request_tracker.handle_trackers[closing_index].api_route;

    ublox_http_content_type_t http_content_type =
        request_tracker.handle_trackers[closing_index].http_content_type;

    if (http_content_type == ublox_http_content_type_t::MULTIPART_FORM_DATA)
    {
        return send_command(
            p_command_buffer,
            buffer_size,
            p_info->p_command,
            p_info->command_length,
            p_info->type,
            "1,4,\"%s\",\"post.ffs\",\"post_data_%x_%s\",3,\"%s\"",
            p_api_route,
            static_cast<uint32_t>(closing_handle),
            p_closing_filename,
            p_closing_filename);
    }
    else
    {
        return send_command(p_command_buffer,
                            buffer_size,
                            p_info->p_command,
                            p_info->command_length,
                            p_info->type,
                            p_info->p_set_value,
                            p_api_route,
                            static_cast<uint32_t>(closing_handle),
                            p_closing_filename,
                            static_cast<uint32_t>(http_content_type));
    }
}

/**
 * @brief Send delete file command data to the gsm filesystem
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_init_delete_file_command(char *                       p_command_buffer,
                              uint32_t                     buffer_size,
                              const ublox_command_info_st *p_info)
{
    if (init_tracker.has_files_to_delete)
    {
        // Send AT+UDELFILE command
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                          "Deleting file %s\n",
                          scratch_buffer);

        if (is_sim7600ei_module())
        {
            // All files will be deleted in one command
            init_tracker.has_files_to_delete = false;
            return send_command(p_command_buffer,
                                buffer_size,
                                p_info->p_command,
                                p_info->command_length,
                                p_info->type,
                                p_info->p_set_value);
        }
        else
        {
            return send_command(p_command_buffer,
                                buffer_size,
                                p_info->p_command,
                                p_info->command_length,
                                p_info->type,
                                p_info->p_set_value,
                                scratch_buffer);
        }
    }
    else
    {
        // No file to delete. Skip sending AT command
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }
}

/**
 * @brief Send delete file command data to the gsm filesystem
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_delete_file_command(char *                       p_command_buffer,
                         uint32_t                     buffer_size,
                         const ublox_command_info_st *p_info)
{
    uint8_t deleting_tracker_index = request_tracker.deleting_index;
    SYSTEM_ASSERT_NOT_EQUAL(deleting_tracker_index,
                            INVALID_HANDLE_TRACKER_INDEX);

    wGsmTransferHandle deleting_handle =
        request_tracker.handle_trackers[deleting_tracker_index].handle;
    SYSTEM_ASSERT_NOT_EQUAL(deleting_handle, INVALID_GSM_HANDLE);

    char *p_deleting_filename =
        request_tracker.handle_trackers[deleting_tracker_index].filename;
    SYSTEM_ASSERT_NOT_EQUAL(p_deleting_filename[0], NULL_TERMINATOR);

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        static_cast<uint32_t>(deleting_handle),
                        p_deleting_filename);
}

/**
 * @brief Send read SMS command data to the gsm module
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode send_sms_read_command(char *   p_command_buffer,
                                           uint32_t buffer_size,
                                           const ublox_command_info_st *p_info)
{
    // Update last received time
    w_hal_rtc_get_time(request_tracker.sms_tracker.last_received_time);

    if (init_tracker.oldest_valid_sms_index == INVALID_SMS_INDEX)
    {
        SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.read_completed);
        request_tracker.sms_tracker.read_completed = true;

        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    return send_command(p_command_buffer,
                        buffer_size,
                        p_info->p_command,
                        p_info->command_length,
                        p_info->type,
                        p_info->p_set_value,
                        init_tracker.oldest_valid_sms_index);
}

/**
 * @brief Send delete sms command data to the gsm module
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_sms_delete_after_read_command(char *   p_command_buffer,
                                   uint32_t buffer_size,
                                   const ublox_command_info_st *p_info)
{
    if (init_tracker.delete_after_read_sms_index == INVALID_SMS_INDEX)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    wHalErrorCode error =
        send_command(p_command_buffer,
                     buffer_size,
                     p_info->p_command,
                     p_info->command_length,
                     p_info->type,
                     p_info->p_set_value,
                     init_tracker.delete_after_read_sms_index);

    init_tracker.delete_after_read_sms_index = INVALID_SMS_INDEX;

    return error;
}

/**
 * @brief Send delete sms command data to the gsm module
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_sms_delete_command(char *                       p_command_buffer,
                        uint32_t                     buffer_size,
                        const ublox_command_info_st *p_info)
{
    if (init_tracker.current_sms_cleanup_index == INVALID_SMS_INDEX)
    {
        return wHalErrorCode::GSM_SKIP_COMMAND;
    }

    wHalErrorCode error                    = send_command(p_command_buffer,
                                       buffer_size,
                                       p_info->p_command,
                                       p_info->command_length,
                                       p_info->type,
                                       p_info->p_set_value,
                                       init_tracker.current_sms_cleanup_index);

    // Clear after issuing delete
    init_tracker.current_sms_cleanup_index = INVALID_SMS_INDEX;

    return error;
}

/**
 * @brief Send command sequence serially to the gsm module and
 * handle response after each command
 *
 * @param command_sequence_tracker Command sequence tracker
 * @param p_sequence Array of commands to issue
 * @param num_commands Number of commands in array
 * @param block_for_response If true, block until response is received
 */
static void send_sequence(command_sequence_tracker_st &command_sequence_tracker,
                          const ublox_command_info_st *p_sequence,
                          uint32_t                     num_commands,
                          bool                         block_for_response)
{
    uint32_t index;
    for (index = command_sequence_tracker.current_index; index < num_commands;
         index++)
    {
        wHalErrorCode error;
        bool          break_from_loop = false;

        switch (command_tracker.command_state)
        {
            case command_state_t::NOT_ACTIVE:
            {
                // This portion is to handle URCs that may come before a
                // command has been sent
                do
                {
                    check_for_response(handle_generic_response);
                    error = w_hal_uart_check_read_status(GSM_UART_CHANNEL);
                } while ((block_for_response &&
                          (command_tracker.response_processed_index !=
                           command_tracker.response_scan_index)) ||
                         (error == wHalErrorCode::UART_READ_COMPLETE));

                if (error == wHalErrorCode::UART_NO_ACTIVE_READ)
                {
                    // Always read max length and abort when last line of
                    // response has been received
                    error = w_hal_uart_read_data(
                        GSM_UART_CHANNEL,
                        reinterpret_cast<uint8_t *>(response_buffer),
                        RESP_BUFF_SIZE);

                    // This would be an unhandled exception
                    SYSTEM_ASSERT_NOT_EQUAL(error,
                                            wHalErrorCode::UART_READ_FAILED);
                }

                if (block_for_response)
                {
                    // Usually when blocking for response, this is not a
                    // performance critical operation, so simply idling for the
                    // given amount of time is fine
                    w_hal_system_delay(
                        MIN_IDLE_TIME_AFTER_LAST_RESPONSE.seconds);
                    w_hal_system_delay_ms(
                        MIN_IDLE_TIME_AFTER_LAST_RESPONSE.nanoseconds /
                        NSEC_PER_MSEC);
                }
                else
                {
                    int cts_value;
                    error = w_hal_uart_check_cts(GSM_UART_CHANNEL, cts_value);
                    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                    w_hal_rtc_time_st current_time;
                    w_hal_rtc_get_time(current_time);

                    if (cts_value)
                    {
                        // This ensures that FW waits an idle time after CTS
                        // goes from high to low
                        command_tracker.last_response_time = current_time;

                        break_from_loop                    = true;
                        break;
                    }
                    else
                    {
                        // Check if fw has waited long enough for a response
                        // before sending the next command
                        w_hal_rtc_time_delta_st time_delta;

                        current_time.subtract(
                            command_tracker.last_response_time,
                            time_delta);

                        if (time_delta < MIN_IDLE_TIME_AFTER_LAST_RESPONSE)
                        {
                            break_from_loop = true;
                            break;
                        }
                    }
                }

                command_tracker.command_state = command_state_t::SEND_COMMAND;
            }
            // FALL-THROUGH
            case command_state_t::SEND_COMMAND:
            {
                error =
                    p_sequence[index].command_send_func(command_buffer,
                                                        sizeof(command_buffer),
                                                        &p_sequence[index]);

                if (error == wHalErrorCode::GSM_SKIP_COMMAND)
                {
                    command_tracker.command_state = command_state_t::NOT_ACTIVE;
                    break;
                }

                const wHalErrorCode valid_errors[] = {
                    wHalErrorCode::UART_WRITE_COMPLETE,
                    wHalErrorCode::UART_WRITE_IN_PROGRESS};
                SYSTEM_ASSERT_IN_SET(error,
                                     valid_errors,
                                     sizeof(valid_errors) /
                                         sizeof(wHalErrorCode));

                command_tracker.response_terminator_flags =
                    p_sequence[index].response_terminator_flags;
                command_tracker.command_state =
                    command_state_t::WAITING_FOR_RESPONSE;
            }
            // FALL-THROUGH
            case command_state_t::WAITING_FOR_RESPONSE:
            {
                do
                {
                    error = check_for_response(p_sequence[index].response_func);
                } while (block_for_response && (error == wHalErrorCode::BUSY));

                if (error == wHalErrorCode::BUSY)
                {
                    break_from_loop = true;
                }
                else
                {
                    command_tracker.command_state = command_state_t::NOT_ACTIVE;
                    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
                }
                break;
            }
            default:
            {
                SYSTEM_ASSERT_TRUE(false);
                break;
            }
        }

        if (break_from_loop)
        {
            break;
        }
    }

    command_sequence_tracker.current_index = index;

    // If all commands have been issued, sequence tracking can be cleared
    if (index == command_sequence_tracker.count)
    {
        command_sequence_tracker.p_sequence = nullptr;
    }
}

/**
 * @brief Command to get the phone number based on the carrier
 *
 * @param p_command_buffer command write buffer
 * @param buffer_size size of command buffer in bytes
 * @param p_info command information to prepare into write buffer
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if
 * another write is occurring, UART_WRITE_COMPLETE if write completed,
 * NO_ERROR otherwise
 */
static wHalErrorCode
send_phone_number_enquiry_command(char *                       p_command_buffer,
                                  uint32_t                     buffer_size,
                                  const ublox_command_info_st *p_info)
{
    // FIXME: For airtel, we need to send a USSD string to get the phone
    // number. But this is not working right now, hence why it is disabled.
#if 0
    if (!is_lara_R2_module())
    {
        if (init_tracker.operator_type == operator_t::INDIA_AIRTEL)
        {
            return send_command(p_command_buffer,
                                buffer_size,
                                p_info->p_command,
                                p_info->command_length,
                                p_info->type,
                                p_info->p_set_value,
                                "1,\"*282#\",15");
        }
    }
#endif

    // Skip sending AT command.
    init_tracker.cusd_done = true;
    return wHalErrorCode::GSM_SKIP_COMMAND;
}

/**
 * @brief Send foreground sequence
 *
 * @param block_for_response If true, block until response is received
 */
static void w_hal_gsm_send_foreground_sequence(bool block_for_response)
{
    send_sequence(foreground_command_sequence_tracker,
                  foreground_command_sequence_tracker.p_sequence,
                  foreground_command_sequence_tracker.count,
                  block_for_response);
}

/**
 * @brief Send background internal sequence
 *
 * @param block_for_response If true, block until response is received
 */
static void w_hal_gsm_send_internal_sequence(bool block_for_response)
{
    send_sequence(internal_command_sequence_tracker,
                  internal_command_sequence_tracker.p_sequence,
                  internal_command_sequence_tracker.count,
                  block_for_response);
}

/**
 * @brief Set up an array of commands to be issued in order
 *
 * @param command_sequence_tracker Tracker to use for sequence
 * @param p_sequence Array of commands to issue
 * @param num_commands Number of commands in array
 */
static void
w_hal_gsm_setup_sequence(command_sequence_tracker_st &command_sequence_tracker,
                         const ublox_command_info_st *p_sequence,
                         uint32_t                     num_commands)
{
    SYSTEM_ASSERT_EQUAL(command_sequence_tracker.p_sequence,
                        static_cast<const ublox_command_info_st *>(nullptr));

    command_sequence_tracker.p_sequence    = p_sequence;
    command_sequence_tracker.count         = num_commands;
    command_sequence_tracker.current_index = 0;
    command_sequence_tracker.last_time     = w_hal_rtc_get_time();
}

/**
 * @brief Set up an array of commands to be issued in order
 *
 * @param p_sequence Array of commands to issue
 * @param num_commands Number of commands in array
 */
static void
w_hal_gsm_setup_foreground_sequence(const ublox_command_info_st *p_sequence,
                                    uint32_t                     num_commands)
{
    w_hal_gsm_setup_sequence(foreground_command_sequence_tracker,
                             p_sequence,
                             num_commands);
}

/**
 * @brief Set up an array of commands to be issued in order
 *
 * @param p_sequence Array of commands to issue
 * @param num_commands Number of commands in array
 */
static void
w_hal_gsm_setup_internal_sequence(const ublox_command_info_st *p_sequence,
                                  uint32_t                     num_commands)
{
    w_hal_gsm_setup_sequence(internal_command_sequence_tracker,
                             p_sequence,
                             num_commands);
}

/**
 * @brief Get the activation sequence to use for pdp deactivation
 *
 * @param p_sequence [out] : The setup sequence
 * @param num_commands_in_sequence [out] : The number of commands in the
 * sequence
 */
static void
w_hal_gsm_get_deactivation_sequence(const ublox_command_info_st *&p_sequence,
                                    uint32_t &num_commands_in_sequence)
{
    if (is_lara_R2_module() || is_sara_module())
    {
        p_sequence               = internal_pdp_deactivation_sequence;
        num_commands_in_sequence = sizeof(internal_pdp_deactivation_sequence) /
                                   sizeof(ublox_command_info_st);
    }
    else
    {
        p_sequence               = external_pdp_deactivation_sequence;
        num_commands_in_sequence = sizeof(external_pdp_deactivation_sequence) /
                                   sizeof(ublox_command_info_st);
    }
}

/**
 * @brief Get the apn sequence to use for set up
 *
 * @param p_apn_sequence [out] : The apn setup sequence
 * @param num_commands_in_apn_sequence [out] : The number of commands in the
 * sequence
 */
static void
w_hal_gsm_get_apn_sequence(const ublox_command_info_st *&p_apn_sequence,
                           uint32_t &num_commands_in_apn_sequence)
{
    if (is_lara_R2_module() || is_sara_module())
    {
        p_apn_sequence = internal_pdp_apn_sequence;
        num_commands_in_apn_sequence =
            sizeof(internal_pdp_apn_sequence) / sizeof(ublox_command_info_st);
    }
    else
    {
        p_apn_sequence = external_pdp_apn_sequence;
        num_commands_in_apn_sequence =
            sizeof(external_pdp_apn_sequence) / sizeof(ublox_command_info_st);
    }
}

/**
 * @brief Get the runtime activation sequence to use for set up
 *
 * @param p_apn_sequence [out] : The apn setup sequence
 * @param num_commands_in_apn_sequence [out] : The number of commands in the
 * sequence
 */
static void w_hal_gsm_get_runtime_pdp_activation_sequence(
    const ublox_command_info_st *&p_apn_sequence,
    uint32_t &                    num_commands_in_apn_sequence)
{
    if (is_lara_R6_L6_module())
    {
        p_apn_sequence = external_pdp_activation_sequence;
        num_commands_in_apn_sequence =
            sizeof(external_pdp_activation_sequence) /
            sizeof(ublox_command_info_st);
    }
    else
    {
        p_apn_sequence = internal_pdp_activation_sequence;
        num_commands_in_apn_sequence =
            sizeof(internal_pdp_activation_sequence) /
            sizeof(ublox_command_info_st);
    }
}

/**
 * @brief Try to setup apn for specified operator
 *
 * @param op operator type
 * @return true if the apn setup was successful
 * @return false if apn setup was not successful
 */
static bool w_hal_gsm_try_apn(operator_t op)
{
    init_tracker.activation_error_code = ACTIVATION_ERROR_CODE_INVALID;
    init_tracker.operator_type         = op;

    const ublox_command_info_st *p_apn_sequence;
    uint32_t                     num_commands_in_apn_sequence;

    w_hal_gsm_get_apn_sequence(p_apn_sequence, num_commands_in_apn_sequence);

    // Setup sequence tracker
    w_hal_gsm_setup_internal_sequence(p_apn_sequence,
                                      num_commands_in_apn_sequence);

    w_hal_gsm_send_internal_sequence(true);

    // Wait for +UUPSDA URC or +CGACT response
    while (init_tracker.activation_error_code == ACTIVATION_ERROR_CODE_INVALID)
    {
        check_for_response(handle_generic_response);
    }

    // If activation not successful, try another operator
    if (init_tracker.activation_error_code != ACTIVATION_ERROR_CODE_NONE)
    {
        return false;
    }

    init_tracker.activation_error_code = ACTIVATION_ERROR_CODE_INVALID;

    // Run test sequence to confirm everything is set up correctly
    w_hal_gsm_setup_internal_sequence(test_sequence,
                                      sizeof(test_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    // Wait for +UUHTTPCR URC
    while (init_tracker.activation_error_code == ACTIVATION_ERROR_CODE_INVALID)
    {
        check_for_response(handle_generic_response);
    }

    // Check test sequence result here
    if (init_tracker.activation_error_code == ACTIVATION_ERROR_CODE_NONE)
    {
        // Successful, so done finding the appropriate operator
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Find and set up the APN that works with SIM
 */
static void w_hal_gsm_find_apn()
{
    // First try apn from detected operator
    if (init_tracker.operator_type != operator_t::NUM_OPERATORS)
    {
        if (w_hal_gsm_try_apn(init_tracker.operator_type))
        {
            return;
        }
    }

    do
    {
        operator_t op;
        for (op = operator_t::FIRST; op < operator_t::NUM_OPERATORS;
             op = static_cast<operator_t>(static_cast<int>(op) + 1))
        {
            if (w_hal_gsm_try_apn(op))
            {
                break;
            }
        }
    } while (init_tracker.activation_error_code != ACTIVATION_ERROR_CODE_NONE);
}

/**
 * @brief Keep sending AT command until device responds with OK
 *
 * @return wHalErrorCode NO_ERROR
 */
static wHalErrorCode w_hal_gsm_wait_for_modem_ready()
{
    static char   init_command[] = "AT\r";
    wHalErrorCode error;
    int           baud_rate_index         = 0;
    int           number_of_tries_at_baud = 0;
    const int     MAX_TRIES_PER_BAUD      = 10; // 1 second of time

    // Always wait for at least one second to avoid glitches in CTS
    w_hal_system_delay(1);

    // Wait for CTS to clear before issuing AT commands
#if !(SIMCOM)
    int cts_value = 1;
    while (cts_value)
    {
        error = w_hal_uart_check_cts(GSM_UART_CHANNEL, cts_value);
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }
#if defined(BOARD_FTHR_REVA)
    // Re-setup flow control
    error = w_hal_uart_set_flow_control(GSM_UART_CHANNEL, true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
#endif
#endif // !(SIMCOM)

    error = w_hal_uart_read_data(GSM_UART_CHANNEL,
                                 reinterpret_cast<uint8_t *>(response_buffer),
                                 RESP_BUFF_SIZE);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::UART_READ_IN_PROGRESS);

    while (true)
    {
        error = w_hal_uart_write_data(GSM_UART_CHANNEL,
                                      reinterpret_cast<uint8_t *>(init_command),
                                      sizeof(init_command) - 1);
        SYSTEM_ASSERT_NOT_EQUAL(error, wHalErrorCode::UART_WRITE_FAILED);
        SYSTEM_ASSERT_NOT_EQUAL(error, wHalErrorCode::BUSY);

        // This should be enough time for command to go out and get a
        // response
        w_hal_system_delay_ms(100);

        error = w_hal_uart_check_write_status(GSM_UART_CHANNEL);
        SYSTEM_ASSERT_NOT_EQUAL(error, wHalErrorCode::UART_WRITE_IN_PROGRESS);

        if (error == wHalErrorCode::UART_WRITE_COMPLETE)
        {
            w_hal_uart_clear_write_completion_status(GSM_UART_CHANNEL);
        }

        // Got some response?
        if (response_buffer[0] != NULL_TERMINATOR)
        {
            bool done = is_response_OK(response_buffer, RESP_BUFF_SIZE);

            error     = w_hal_uart_stop_read(GSM_UART_CHANNEL);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::UART_READ_STOPPED);
            memset(response_buffer, 0, sizeof(response_buffer));

            if (done)
            {
                break;
            }

            // restart read
            error = w_hal_uart_read_data(
                GSM_UART_CHANNEL,
                reinterpret_cast<uint8_t *>(response_buffer),
                RESP_BUFF_SIZE);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::UART_READ_IN_PROGRESS);
        }
        else
        {
            if (++number_of_tries_at_baud >= MAX_TRIES_PER_BAUD)
            {
                number_of_tries_at_baud = 0;

                // stop outstanding read so baud rate can be changed
                error = w_hal_uart_stop_read(GSM_UART_CHANNEL);
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::UART_READ_STOPPED);
                memset(response_buffer, 0, sizeof(response_buffer));

                // Select next baud rate
                if (++baud_rate_index >= NUM_INIT_BAUD_RATES_TO_TRY)
                {
                    baud_rate_index = 0;
                }
                int next_baud_rate = init_bauds_rates_to_try[baud_rate_index];

                // Don't log these messages as it will overflow the log
                printf("Try switching to baud rate %d\n", next_baud_rate);

                // Change UART to use next baud rate
                error =
                    w_hal_uart_set_baud_rate(GSM_UART_CHANNEL, next_baud_rate);
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                // restart read that was previously halted
                error = w_hal_uart_read_data(
                    GSM_UART_CHANNEL,
                    reinterpret_cast<uint8_t *>(response_buffer),
                    RESP_BUFF_SIZE);
                SYSTEM_ASSERT_EQUAL(error,
                                    wHalErrorCode::UART_READ_IN_PROGRESS);
            }
        }
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Set up phone number for use in HAL
 *
 * @param enable [out] : Set to enable use of this phone number
 * @param enable_short_code [out] : Set to indicate if short code is detected
 * @param p_config_phone_number : Phone number from config
 * @param p_hal_phone_number [out] : Phone number to use in HAL
 * @param hal_phone_number_size : Size of HAL phone number buffer
 */
static void w_hal_gsm_setup_phone_number(bool &      enable,
                                         bool &      enable_short_code,
                                         const char *p_config_phone_number,
                                         char *      p_hal_phone_number,
                                         uint32_t    hal_phone_number_size)
{
    enable                            = false;
    enable_short_code                 = false;

    size_t config_phone_number_length = strlen(p_config_phone_number);

    if ((p_config_phone_number == nullptr) || (config_phone_number_length == 0))
    {
        return;
    }

    int match_length = 0;
    int match_index  = re_match("\\+\\d+",
                               p_config_phone_number,
                               config_phone_number_length,
                               &match_length);

    if ((match_index == 0) &&
        (static_cast<size_t>(match_length) == config_phone_number_length))
    {
        enable = true;
    }
    else
    {
        // Long code format check failed, try short code format
        if ((config_phone_number_length >=
             MIN_SHORT_CODE_PHONE_NUMBER_LENGTH) &&
            (config_phone_number_length <= MAX_SHORT_CODE_PHONE_NUMBER_LENGTH))
        {
            match_index = re_match("[A-Za-z0-9]+",
                                   p_config_phone_number,
                                   config_phone_number_length,
                                   &match_length);

            if ((match_index == 0) && (static_cast<size_t>(match_length) ==
                                       config_phone_number_length))
            {
                enable            = true;
                enable_short_code = true;
            }
        }
    }

    if (enable)
    {
        // Copy Phone number to HAL buffer
        SYSTEM_ASSERT_LT(config_phone_number_length, hal_phone_number_size);
        strcpy(p_hal_phone_number, p_config_phone_number);
    }
}

/**
 * @brief Initialize GSM module
 *
 * @param device_config device configuration
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_gsm_init(DeviceConfig::config_page_st &device_config)
{
    command_tracker.command_state         = command_state_t::NOT_ACTIVE;
    command_tracker.result_code_format    = at_result_code_format_t::DISABLED;
    command_tracker.received_response     = false;
    command_tracker.response_error        = wHalErrorCode::NO_ERROR;
    command_tracker.command_buffer_length = 0;
    command_tracker.response_processed_index = 0;
    command_tracker.response_scan_index      = 0;
    command_tracker.last_response_time.clear();

    background_poll_tracker.index = 0;
    for (uint32_t index = 0;
         index < static_cast<uint8_t>(background_poll_item_index_t::NUM_ITEMS);
         index++)
    {
        // Initialize to oldest time to immediately acquire all info after boot
        background_poll_tracker.last_time[index].clear();
    }

    request_tracker.sms_tracker.is_open                  = false;
    request_tracker.sms_tracker.read_write               = false;
    request_tracker.sms_tracker.read_completed           = false;
    request_tracker.sms_tracker.request_chars_remaining  = 0;
    request_tracker.sms_tracker.transfer_chars_remaining = 0;
    request_tracker.sms_tracker.terminator_sent          = false;
    request_tracker.sms_tracker.is_terminated            = false;
    request_tracker.sms_tracker.is_closing               = false;
    request_tracker.sms_tracker.p_sms_buffer             = nullptr;
    request_tracker.sms_tracker.sms_buffer_size          = 0;
    request_tracker.sms_tracker.read_buffer_fill_index   = 0;
    request_tracker.sms_tracker.last_received_time.clear();

    for (uint32_t index = 0; index < MAX_HANDLE_TRACKERS; index++)
    {
        request_tracker.handle_trackers[index].handle = INVALID_GSM_HANDLE;
        request_tracker.handle_trackers[index].expected_upload_bytes = 0;
        request_tracker.handle_trackers[index].uploaded_bytes        = 0;
        memset(request_tracker.handle_trackers[index].filename,
               NULL_TERMINATOR,
               MAX_FILENAME_LENGTH);
        request_tracker.handle_trackers[index].last_command_time.invalidate();
    }

    request_tracker.deleting_index            = INVALID_HANDLE_TRACKER_INDEX;
    request_tracker.closing_index             = INVALID_HANDLE_TRACKER_INDEX;
    request_tracker.upload_index              = INVALID_HANDLE_TRACKER_INDEX;
    request_tracker.p_upload_buffer           = nullptr;
    request_tracker.upload_buffer_size        = 0;
    request_tracker.upload_buffer_transferred = 0;
    request_tracker.cmd_size                  = 0;
    request_tracker.cmd_bytes_transferred     = 0;
    request_tracker.closing_queue_head        = 0;
    request_tracker.closing_queue_tail        = 0;
    request_tracker.deleting_queue_head       = 0;
    request_tracker.deleting_queue_tail       = 0;
    request_tracker.http_error_code_needed    = false;

    foreground_command_sequence_tracker.count = 0;
    foreground_command_sequence_tracker.current_index = 0;
    foreground_command_sequence_tracker.p_sequence    = nullptr;
    foreground_command_sequence_tracker.last_time.invalidate();

    internal_command_sequence_tracker.count         = 0;
    internal_command_sequence_tracker.current_index = 0;
    internal_command_sequence_tracker.p_sequence    = nullptr;
    internal_command_sequence_tracker.last_time.invalidate();

    init_tracker.activation_error_code = 0;
    for (uint32_t index = 0; index < NUM_USED_PDP_CONTEXTS; index++)
    {
        init_tracker.apn_change_needed[index]  = false;
        init_tracker.pdp_context_exists[index] = false;
        init_tracker.pdp_context_active[index] = false;
    }
    init_tracker.done                = false;
    init_tracker.deactivation_needed = false;
    init_tracker.activation_needed   = true; // First time activation
    init_tracker.reattach_needed     = false;
    init_tracker.reattach_needed_time.invalidate();
    init_tracker.current_sms_cleanup_index = INVALID_SMS_INDEX;
    init_tracker.gsm_sleep_flag            = false;
    memset(init_tracker.sms_index_cleanup_mask,
           0,
           sizeof(init_tracker.sms_index_cleanup_mask));
    init_tracker.delete_after_read_sms_index = INVALID_SMS_INDEX;
    init_tracker.oldest_valid_sms_index      = INVALID_SMS_INDEX;
    init_tracker.oldest_valid_sms_time       = static_cast<time_t>(-1LL);
    init_tracker.has_files_to_delete         = false;
    init_tracker.operator_type               = operator_t::NUM_OPERATORS;
    init_tracker.cusd_done                   = false;
    init_tracker.sim_not_inserted_detected   = false;

    memset(&ublox_info, 0, sizeof(ublox_info));
    ublox_info.operator_mode                  = operator_mode_t::DEREGISTER;
    ublox_info.eps_network.reject_cause_type  = reject_cause_type_t::INVALID;
    ublox_info.gprs_network.reject_cause_type = reject_cause_type_t::INVALID;
    ublox_info.network.reject_cause_type      = reject_cause_type_t::INVALID;
    ublox_info.signal_quality.rxlev           = INVALID_RSSI_VALUE;
    ublox_info.signal_quality.ber             = INVALID_RXQUAL_VALUE;
    ublox_info.signal_quality.rscp            = INVALID_RSCP_VALUE;
    ublox_info.signal_quality.ecno            = INVALID_ECNO_VALUE;
    ublox_info.signal_quality.rsrq            = INVALID_RSRQ_VALUE;
    ublox_info.signal_quality.rsrp            = INVALID_RSRP_VALUE;

    memset(command_buffer, 0, sizeof(command_buffer));
    memset(response_buffer, 0, sizeof(response_buffer));
    memset(scratch_buffer, 0, sizeof(scratch_buffer));

    wHalErrorCode error;

    // Setup for the  channel
    error = w_hal_uart_init(GSM_UART_CHANNEL, INITIAL_BAUD_RATE);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error = w_hal_uart_set_data_bits(GSM_UART_CHANNEL, 8);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error = w_hal_uart_set_stop_bits(GSM_UART_CHANNEL,
                                     whal_uart_stop_bits::ONE_BIT);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error =
        w_hal_uart_set_parity_bits(GSM_UART_CHANNEL, whal_uart_parity::NONE);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error = w_hal_uart_set_flow_control(GSM_UART_CHANNEL, SIMCOM == 0);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Select Voltage according to board used
    whal_voltage_select voltage_select;
    w_hal_gsm_get_voltage(voltage_select);

    error = w_hal_uart_set_voltage(GSM_UART_CHANNEL, voltage_select);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    error = w_hal_uart_flush(GSM_UART_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Reset the gsm module
    w_hal_gsm_reset();

    // Wait for gsm ready
    w_hal_gsm_wait_for_modem_ready();

    // Setup sequence tracker
    w_hal_gsm_setup_internal_sequence(id_sequence,
                                      sizeof(id_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    // Checking whether the module is sim800l or not to configure for
    // transmission mode
    if (is_sim800l_module())
    {
        DeviceConfig::max_transmission_mode =
            DeviceConfig::transmission_mode_t::MODE_2G;
    }

    w_hal_gsm_validate_config(device_config, true);

    // Set up A2P and P2A phone numbers
    w_hal_gsm_setup_phone_number(interface_params.sms_read_enabled,
                                 interface_params.a2p_short_code,
                                 device_config.a2p_phone_number,
                                 interface_params.a2p_phone_number,
                                 sizeof(interface_params.a2p_phone_number));

    w_hal_gsm_setup_phone_number(interface_params.sms_send_enabled,
                                 interface_params.p2a_short_code,
                                 device_config.p2a_phone_number,
                                 interface_params.p2a_phone_number,
                                 sizeof(interface_params.p2a_phone_number));

    interface_params.data_enabled = false;
    if ((device_config.server_url[0] != '\0') &&
        (device_config.transmission_mode !=
         DeviceConfig::transmission_mode_t::MODE_2G))
    {
        interface_params.data_enabled = true;

        strncpy(interface_params.server_url,
                device_config.server_url,
                sizeof(interface_params.server_url));
        interface_params.server_port = device_config.server_port;
    }

    interface_params.data_bit_rate               = 0;
    interface_params.udwnfile_max_transfer_bytes = 0;
    if (interface_params.data_enabled)
    {
        SYSTEM_ASSERT_LTEQ(
            (interface_params.data_bit_rate * BAUD_MARGIN_FACTOR) /
                BAUD_MARGIN_DIVISOR,
            BAUD_RATE);

        interface_params.data_bit_rate =
            DeviceConfig::get_data_bit_rate(device_config);
        const uint32_t audio_data_byte_rate =
            interface_params.data_bit_rate / CHAR_BIT;

        interface_params.udwnfile_max_transfer_bytes =
            std::min(audio_data_byte_rate,
                     128ul * 1024); // Either every 1
                                    // second or 128K.
                                    // This allows for
                                    // opportunity to
                                    // do housekeeping
                                    // between transfers
    }

    // Setup sequence tracker
    w_hal_gsm_setup_internal_sequence(init_sequence,
                                      sizeof(init_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    // Return error if initialization failed due to SIM not inserted
    if (init_tracker.sim_not_inserted_detected)
    {
        return wHalErrorCode::GSM_SIM_NOT_INSERTED;
    }

    LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Waiting for operator...\n");

    for (int delay_index = 0; delay_index < MAX_WAIT_FOR_OPERATOR_NAME_SECONDS;
         delay_index++)
    {
        // Check if operator name has been received
        if (ublox_info.operator_name[0] != NULL_TERMINATOR)
        {
            break;
        }

        // Setup sequence tracker
        w_hal_gsm_setup_internal_sequence(check_operator_name_sequence,
                                          sizeof(check_operator_name_sequence) /
                                              sizeof(ublox_command_info_st));

        w_hal_gsm_send_internal_sequence(true);

        // Delay to allow for operator name to be received
        w_hal_system_delay(1);
    }

    if (ublox_info.operator_name[0] == NULL_TERMINATOR)
    {
        LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Operator name not received\n");
    }

    LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Begin Registration\n");

    // Keep trying to register until successful
    while (true)
    {
        // Setup check registration tracker
        w_hal_gsm_setup_internal_sequence(check_registration_sequence,
                                          sizeof(check_registration_sequence) /
                                              sizeof(ublox_command_info_st));

        w_hal_gsm_send_internal_sequence(true);

        // If there are any anomalies with operator setup, try to
        // register
        // Bands are only checked for LARA-R6 modules
        tm tm_struct;
        gmtime_r(&ublox_info.init_time_value, &tm_struct);
        if ((ublox_info.operator_mode == operator_mode_t::AUTOMATIC) &&
            (strlen(ublox_info.operator_name) > 0) &&
            ((TIME_BASE_YEAR + static_cast<uint32_t>(tm_struct.tm_year)) >
             2022) &&
            is_working_network_registration_status(
                ublox_info.network.registration_status) &&
            (init_tracker.operator_type != operator_t::NUM_OPERATORS) &&
            (!is_lara_R6_module() ||
             ((init_tracker.lte_bands.mask1 ==
               OPERATOR_SUPPORTED_LTE_BANDS[static_cast<int>(
                                                init_tracker.operator_type)]
                   .mask1) &&
              (init_tracker.lte_bands.mask2 ==
               OPERATOR_SUPPORTED_LTE_BANDS[static_cast<int>(
                                                init_tracker.operator_type)]
                   .mask2))))
        {
            break;
        }

        // Remove registration from network
        w_hal_gsm_setup_internal_sequence(
            network_deregistration_sequence,
            sizeof(network_deregistration_sequence) /
                sizeof(ublox_command_info_st));

        w_hal_gsm_send_internal_sequence(true);

        // Wait for URC to indicate device is not registered
        // to any network.
        while (ublox_info.network.registration_status !=
               registration_status_t::NOT_REGISTERED)
        {
            check_for_response(handle_generic_response);
        }

        // Re-register
        ublox_info.network.registration_status = registration_status_t::INVALID;
        w_hal_gsm_setup_internal_sequence(auto_registration_sequence,
                                          sizeof(auto_registration_sequence) /
                                              sizeof(ublox_command_info_st));

        w_hal_gsm_send_internal_sequence(true);

        // Wait for URC
        while (ublox_info.network.registration_status ==
               registration_status_t::INVALID)
        {
            check_for_response(handle_generic_response);
        }

        // Wait some time for additional registration URCs
        const int delay_ms = 100;
        for (int delay_index = 0; delay_index < MAX_REGISTRATION_TIME_SECONDS *
                                                    MSEC_PER_SEC / delay_ms;
             delay_index++)
        {
            if (is_working_network_registration_status(
                    ublox_info.network.registration_status) &&
                ((device_config.transmission_mode ==
                  DeviceConfig::transmission_mode_t::MODE_2G) ||
                 is_working_network_registration_status(
                     ublox_info.eps_network.registration_status)))
            {
                if (!is_working_network_registration_status(
                        ublox_info.gprs_network.registration_status))
                {
                    // GPRS is not an absolute requirement, but if it
                    // is not registered, then log it
                    LOG_MSG_WITH_ARGS(
                        TraceFile::W_HAL_GSM_CPP,
                        "Unexpected GPRS network registration status: %d\n",
                        ublox_info.gprs_network.registration_status);
                }

                break;
            }

            check_for_response(handle_generic_response);

            // RTC Time not available yet, so wait for a fixed amount of time
            // instead
            w_hal_system_delay_ms(delay_ms);
        }
    }

    // Run cleanup sequence until files are deleted
    if (interface_params.data_enabled)
    {
        init_tracker.has_files_to_delete = true;
        while (init_tracker.has_files_to_delete)
        {
            w_hal_gsm_setup_internal_sequence(
                init_file_cleanup_sequence,
                sizeof(init_file_cleanup_sequence) /
                    sizeof(ublox_command_info_st));

            w_hal_gsm_send_internal_sequence(true);
        }

        LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Identifying appropriate APN\n");

        // Find the right operator and set the appropriate APN
        w_hal_gsm_find_apn();
    }
    else
    {
        init_tracker.activation_needed = false;
    }

    LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Cleanup files on file system\n");

    if (interface_params.data_enabled)
    {
        // Run cleanup sequence until files are deleted
        init_tracker.has_files_to_delete = true;
        while (init_tracker.has_files_to_delete)
        {
            w_hal_gsm_setup_internal_sequence(
                init_file_cleanup_sequence,
                sizeof(init_file_cleanup_sequence) /
                    sizeof(ublox_command_info_st));

            w_hal_gsm_send_internal_sequence(true);
        }

        // To get the total size of the file system
        w_hal_gsm_setup_internal_sequence(get_file_size_sequence,
                                          sizeof(get_file_size_sequence) /
                                              sizeof(ublox_command_info_st));
        w_hal_gsm_send_internal_sequence(true);
    }

    LOG_MSG(TraceFile::W_HAL_GSM_CPP, "Final setup\n");

    if (interface_params.data_enabled)
    {
        // Setup http profile
        w_hal_gsm_setup_internal_sequence(http_init_profile_sequence,
                                          sizeof(http_init_profile_sequence) /
                                              sizeof(ublox_command_info_st));

        w_hal_gsm_send_internal_sequence(true);
    }

    // Get sim information sequence
    w_hal_gsm_setup_internal_sequence(get_sim_info_sequence,
                                      sizeof(get_sim_info_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    w_hal_gsm_setup_internal_sequence(get_phone_number_sequence,
                                      sizeof(get_phone_number_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    while (!init_tracker.cusd_done)
    {
        check_for_response(handle_generic_response);
    }

    // Get the latest signal quality as the second to last step
    w_hal_gsm_setup_internal_sequence(get_signal_quality_sequence,
                                      sizeof(get_signal_quality_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    // Setup power saving sequence tracker
    w_hal_gsm_setup_internal_sequence(power_saving_mode_sequence,
                                      sizeof(power_saving_mode_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    // Get the latest time as the last step
    w_hal_gsm_setup_internal_sequence(get_time_sequence,
                                      sizeof(get_time_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    init_tracker.done = true;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get time value that was read during initialization
 *
 * @return time_t the time value in GMT
 */
time_t w_hal_gsm_get_init_time()
{
    return ublox_info.init_time_value;
}

/**
 * @brief Get tracker index for gsm handle
 *
 * @param handle handle for open post request
 * @return tracker index for gsm handle, MAX_HANDLE_TRACKERS otherwise
 */
static uint8_t w_hal_gsm_get_tracker_index(wGsmTransferHandle handle)
{
    if (handle == INVALID_GSM_HANDLE)
    {
        return MAX_HANDLE_TRACKERS;
    }

    // Check if handle is a valid handle
    uint32_t index;
    for (index = 0; index < MAX_HANDLE_TRACKERS; index++)
    {
        if (request_tracker.handle_trackers[index].handle == handle)
        {
            return index;
        }
    }

    return MAX_HANDLE_TRACKERS;
}

/**
 * @brief Get the next transfer size to send over to the GSM
 *
 * @param p_handle_tracker handle tracker
 * @return uint32_t the size of the next transfer
 */
static uint32_t get_next_transfer_size(handle_tracker_st *p_handle_tracker)
{
    uint32_t size = std::min(request_tracker.upload_buffer_size -
                                 request_tracker.upload_buffer_transferred,
                             request_tracker.cmd_size -
                                 request_tracker.cmd_bytes_transferred);
    size          = std::min(size,
                    p_handle_tracker->expected_upload_bytes -
                        p_handle_tracker->uploaded_bytes);

    return size;
}

/**
 * @brief Check if SMS data is being sent
 *
 * @return true if SMS data is being sent
 * @return false otherwise
 */
static bool check_in_middle_of_send_sms_transfer()
{
    return request_tracker.sms_tracker.is_open &&
           request_tracker.sms_tracker.read_write &&
           ((request_tracker.sms_tracker.transfer_chars_remaining <
             MAX_SMS_LENGTH) ||
            (request_tracker.sms_tracker.sms_buffer_size > 0));
}

/**
 * @brief Check if file data is being sent
 *
 * @return true if file data is being sent
 * @return false otherwise
 */
static bool check_in_middle_of_file_transfer()
{
    return (request_tracker.cmd_bytes_transferred != request_tracker.cmd_size);
}

/**
 * @brief Check if SMS that has been read needs to be deleted
 */
static bool w_hal_gsm_check_delete_after_receive_sms()
{
    if ((foreground_command_sequence_tracker.p_sequence != nullptr) ||
        (internal_command_sequence_tracker.p_sequence != nullptr))
    {
        // Current sequence is active
        return false;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue in between udwnfile command transfers
        return false;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't issue in between sms transfers
        return false;
    }

    if (init_tracker.delete_after_read_sms_index == INVALID_SMS_INDEX)
    {
        // No SMS to delete
        return false;
    }

    w_hal_gsm_setup_internal_sequence(sms_cleanup_after_read_sequence,
                                      sizeof(sms_cleanup_after_read_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(false);

    return true;
}

/**
 * @brief Check if SMS needs to be read and handle it.
 */
static void w_hal_gsm_check_receive_sms()
{
    if ((foreground_command_sequence_tracker.p_sequence != nullptr) ||
        (internal_command_sequence_tracker.p_sequence != nullptr))
    {
        // Current sequence is active
        return;
    }

    if (!request_tracker.sms_tracker.is_open ||
        request_tracker.sms_tracker.read_write)
    {
        // Not open nor currently doing sms write request
        return;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue in between udwnfile command transfers
        return;
    }

    // Use the is_closing flag to determine if the
    // receive sequence has been started
    if (request_tracker.sms_tracker.is_closing)
    {
        // No sms buffer to send and don't need to send termination
        return;
    }

    if (init_tracker.delete_after_read_sms_index != INVALID_SMS_INDEX)
    {
        // Delete last SMS message before processing the next one
        return;
    }

    // Use the is_closing flag to determine if the
    // receive sequence has been started
    request_tracker.sms_tracker.is_closing = true;
    init_tracker.oldest_valid_sms_index    = INVALID_SMS_INDEX;
    init_tracker.oldest_valid_sms_time     = static_cast<time_t>(-1LL);

    w_hal_gsm_setup_foreground_sequence(sms_receive_sequence,
                                        sizeof(sms_receive_sequence) /
                                            sizeof(ublox_command_info_st));

    w_hal_gsm_send_foreground_sequence(false);
}

/**
 * @brief Check if SMS needs to be done and handle it.
 */
static void w_hal_gsm_check_send_sms()
{
    if ((foreground_command_sequence_tracker.p_sequence != nullptr) ||
        (internal_command_sequence_tracker.p_sequence != nullptr))
    {
        // Current sequence is active
        return;
    }

    if (!request_tracker.sms_tracker.is_open ||
        !request_tracker.sms_tracker.read_write)
    {
        // Not open or sms read request
        return;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue sms in between udwnfile command transfers
        return;
    }

    if ((request_tracker.sms_tracker.sms_buffer_size == 0) &&
        (request_tracker.sms_tracker.terminator_sent ||
         !request_tracker.sms_tracker.is_closing))
    {
        // No sms buffer to send and don't need to send termination
        return;
    }

    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);

    if (request_tracker.sms_tracker.transfer_chars_remaining == MAX_SMS_LENGTH)
    {
        if (request_tracker.sms_tracker.sms_buffer_size == MAX_SMS_LENGTH)
        {
            // Send command and all data
            // Response is expected.
            w_hal_gsm_setup_foreground_sequence(
                sms_send_full_content_sequence,
                sizeof(sms_send_full_content_sequence) /
                    sizeof(ublox_command_info_st));

            SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);
            request_tracker.sms_tracker.terminator_sent = true;
        }
        else
        {
            SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);

            // Send command and first portion of data.
            // No response is expected.
            w_hal_gsm_setup_foreground_sequence(
                sms_send_first_content_sequence,
                sizeof(sms_send_first_content_sequence) /
                    sizeof(ublox_command_info_st));
        }
    }
    else if (!request_tracker.sms_tracker.terminator_sent)
    {
        if (request_tracker.sms_tracker.transfer_chars_remaining == 0)
        {
            SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.sms_buffer_size, 0);
            w_hal_gsm_setup_foreground_sequence(
                sms_send_termination_sequence,
                sizeof(sms_send_termination_sequence) /
                    sizeof(ublox_command_info_st));
            request_tracker.sms_tracker.terminator_sent = true;
        }
        else if (request_tracker.sms_tracker.transfer_chars_remaining ==
                 request_tracker.sms_tracker.sms_buffer_size)
        {
            w_hal_gsm_setup_foreground_sequence(
                sms_send_last_content_sequence,
                sizeof(sms_send_last_content_sequence) /
                    sizeof(ublox_command_info_st));
            request_tracker.sms_tracker.terminator_sent = true;
        }
        else
        {
            SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);

            w_hal_gsm_setup_foreground_sequence(
                sms_send_content_sequence,
                sizeof(sms_send_content_sequence) /
                    sizeof(ublox_command_info_st));
        }
    }

    if (foreground_command_sequence_tracker.p_sequence != nullptr)
    {
        w_hal_gsm_send_foreground_sequence(false);
    }
}

/**
 * @brief Check if buffer needs to be uploaded for file
 */
static void w_hal_gsm_check_file_upload()
{
    if (!interface_params.data_enabled)
    {
        return;
    }

    if ((foreground_command_sequence_tracker.p_sequence != nullptr) ||
        (internal_command_sequence_tracker.p_sequence != nullptr))
    {
        // Current sequence is active
        return;
    }

    // None of the handles match argument handle?
    uint8_t tracker_index = request_tracker.upload_index;
    if (tracker_index == INVALID_HANDLE_TRACKER_INDEX)
    {
        return;
    }

    // No buffer available to transfer
    if (request_tracker.upload_buffer_transferred ==
        request_tracker.upload_buffer_size)
    {
        return;
    }

    // Can't send file data in middle of sms
    if (check_in_middle_of_send_sms_transfer())
    {
        return;
    }

    handle_tracker_st *p_handle_tracker =
        &request_tracker.handle_trackers[tracker_index];

    // If the total upload size is unknown, then treat each upload as a
    // separate UDWNFILE command
    if (p_handle_tracker->expected_upload_bytes == INVALID_GSM_UPLOAD_SIZE)
    {
        request_tracker.cmd_size = request_tracker.upload_buffer_size;
        request_tracker.cmd_bytes_transferred = 0;
        request_tracker.send_size = request_tracker.upload_buffer_size;

        // Setup sequence tracker
        w_hal_gsm_setup_foreground_sequence(file_send_sequence,
                                            sizeof(file_send_sequence) /
                                                sizeof(ublox_command_info_st));
    }
    else if (request_tracker.cmd_bytes_transferred == 0)
    {
        const ublox_command_info_st *p_sequence;
        uint32_t                     sequence_count;

        uint32_t                     remaining_bytes_in_file =
            static_cast<uint32_t>(p_handle_tracker->expected_upload_bytes -
                                  p_handle_tracker->uploaded_bytes);

        request_tracker.cmd_size =
            std::min(remaining_bytes_in_file,
                     interface_params.udwnfile_max_transfer_bytes);
        if ((request_tracker.cmd_size == remaining_bytes_in_file) &&
            ((request_tracker.upload_buffer_size -
              request_tracker.upload_buffer_transferred) >=
             remaining_bytes_in_file))
        {
            // Send command and wait for download completion
            p_sequence = file_send_sequence;
            sequence_count =
                sizeof(file_send_sequence) / sizeof(ublox_command_info_st);
        }
        else
        {
            p_sequence     = file_first_portion_send_sequence;
            sequence_count = sizeof(file_first_portion_send_sequence) /
                             sizeof(ublox_command_info_st);
        }
        request_tracker.cmd_bytes_transferred = 0;

        request_tracker.send_size = get_next_transfer_size(p_handle_tracker);
        SYSTEM_ASSERT_NOT_EQUAL(request_tracker.send_size, 0);

        // Send command and first portion of data. No response is
        // expected.
        w_hal_gsm_setup_foreground_sequence(p_sequence, sequence_count);
    }
    else
    {
        request_tracker.send_size = get_next_transfer_size(p_handle_tracker);
        SYSTEM_ASSERT_NOT_EQUAL(request_tracker.send_size, 0);

        SYSTEM_ASSERT_LTEQ(request_tracker.cmd_bytes_transferred +
                               request_tracker.send_size,
                           request_tracker.cmd_size);

        if (request_tracker.cmd_bytes_transferred + request_tracker.send_size ==
            request_tracker.cmd_size)
        {
            // Just send last portion of data. A response is expected.
            w_hal_gsm_setup_foreground_sequence(
                file_last_portion_send_sequence,
                sizeof(file_last_portion_send_sequence) /
                    sizeof(ublox_command_info_st));
        }
        else
        {
            // Just send data. No response is expected.
            w_hal_gsm_setup_foreground_sequence(
                file_portion_send_sequence,
                sizeof(file_portion_send_sequence) /
                    sizeof(ublox_command_info_st));
        }
    }

    w_hal_gsm_send_foreground_sequence(false);
}

/**
 * @brief Check if file that has been posted can now be deleted
 *
 * @return true if file cleanup in progress, false otherwise
 */
static bool w_hal_gsm_check_file_cleanup()
{
    if (!interface_params.data_enabled)
    {
        return false;
    }

    if (internal_command_sequence_tracker.p_sequence != nullptr)
    {
        // Current sequence is active
        return false;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue delete file in between file transfers
        return false;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't send file data in middle of sms
        return false;
    }

    if (request_tracker.deleting_queue_head ==
        request_tracker.deleting_queue_tail)
    {
        // Nothing to process here
        return false;
    }

    // Dequeue tracker index
    uint8_t tracker_index =
        request_tracker.deleting_queue[request_tracker.deleting_queue_head];
    if (++request_tracker.deleting_queue_head == MAX_HANDLE_TRACKERS + 1)
    {
        request_tracker.deleting_queue_head = 0;
    }

    request_tracker.deleting_index = tracker_index;

    // Setup sequence tracker for file cleanup
    w_hal_gsm_setup_internal_sequence(file_cleanup_sequence,
                                      sizeof(file_cleanup_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(false);

    return true;
}

/**
 * @brief Check if SMS cleanup is needed
 *
 * @return true if SMS cleanup in progress, false otherwise
 */
static bool w_hal_gsm_check_sms_cleanup()
{
    if (internal_command_sequence_tracker.p_sequence != nullptr)
    {
        // Current sequence is active
        return false;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue command in between file transfers
        return false;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't send file data in middle of sms
        return false;
    }

    int sms_cleanup_index = INVALID_SMS_INDEX;

    for (uint32_t index = 0; index < NUM_SMS_INDEX_MASKS; index++)
    {
        if (init_tracker.sms_index_cleanup_mask[index])
        {
            // Find the first set bit
            for (uint32_t bit_index = 0; bit_index < SMS_MASK_BIT_WIDTH;
                 bit_index++)
            {
                if (init_tracker.sms_index_cleanup_mask[index] &
                    (1 << bit_index))
                {
                    sms_cleanup_index = index * SMS_MASK_BIT_WIDTH + bit_index;

                    // Clear bit from cleanup mask
                    init_tracker.sms_index_cleanup_mask[index] &=
                        ~(1 << bit_index);

                    break;
                }
            }

            break;
        }
    }

    if (sms_cleanup_index != INVALID_SMS_INDEX)
    {
        init_tracker.current_sms_cleanup_index = sms_cleanup_index;

        // Setup sequence tracker for sms cleanup
        w_hal_gsm_setup_internal_sequence(sms_cleanup_sequence,
                                          sizeof(sms_cleanup_sequence) /
                                              sizeof(ublox_command_info_st));

        w_hal_gsm_send_internal_sequence(false);

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Check if http error reason needs to be retrieved
 *
 * @return true if http error reason in progress, false otherwise
 */
static bool w_hal_gsm_check_http_error_reason()
{
    if ((is_sim800l_module()) || (is_sim7600ei_module()))
    {
        return false;
    }
    if (internal_command_sequence_tracker.p_sequence != nullptr)
    {
        // Current sequence is active
        return false;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue command in between file transfers
        return false;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't send file data in middle of sms
        return false;
    }

    if (!request_tracker.http_error_code_needed)
    {
        // No error reason to retrieve
        return false;
    }

    // Setup sequence tracker for http error reason
    w_hal_gsm_setup_internal_sequence(http_protocol_error_sequence,
                                      sizeof(http_protocol_error_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(false);

    // Clear flag since the sequence has been setup
    request_tracker.http_error_code_needed = false;

    return true;
}

/**
 * @brief Check if reactivation of PDP context is needed
 *
 * @return true if reactivation is needed, false otherwise
 */
static bool w_hal_gsm_check_reactivation()
{
    if (!interface_params.data_enabled)
    {
        return false;
    }

    if (internal_command_sequence_tracker.p_sequence != nullptr)
    {
        // Current sequence is active
        return false;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue command in between file transfers
        return false;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't send file data in middle of sms
        return false;
    }

    if (!init_tracker.reattach_needed && !init_tracker.deactivation_needed &&
        !init_tracker.activation_needed)
    {
        return false;
    }

    if (init_tracker.reattach_needed)
    {
        // Wait for a few seconds before sending reattach command sequence
        if ((w_hal_rtc_get_time() - init_tracker.reattach_needed_time) <
            REATTACH_WAIT_TIME_SECONDS)
        {
            // Don't allow foreground operation to continue if reattach
            // is still needed, but hasn't been performed
            return true;
        }

        w_hal_gsm_setup_internal_sequence(reattach_sequence,
                                          sizeof(reattach_sequence) /
                                              sizeof(ublox_command_info_st));
    }
    else
    {
        const ublox_command_info_st *p_sequence;
        uint32_t                     num_commands_in_sequence;
        if (init_tracker.deactivation_needed)
        {
            w_hal_gsm_get_deactivation_sequence(p_sequence,
                                                num_commands_in_sequence);
        }
        else
        {
            SYSTEM_ASSERT_TRUE(init_tracker.activation_needed);
            w_hal_gsm_get_runtime_pdp_activation_sequence(
                p_sequence,
                num_commands_in_sequence);
        }

        // Setup sequence tracker
        w_hal_gsm_setup_internal_sequence(p_sequence, num_commands_in_sequence);
    }

    w_hal_gsm_send_internal_sequence(false);

    return true;
}

/**
 * @brief Check if file needs to be posted
 */
static void w_hal_gsm_check_handle_close()
{
    if (!interface_params.data_enabled)
    {
        return;
    }

    if ((internal_command_sequence_tracker.p_sequence != nullptr) ||
        (foreground_command_sequence_tracker.p_sequence != nullptr))
    {
        // Current sequence is active
        return;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue delete file in between file transfers
        return;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't send file data in middle of sms
        return;
    }

    if (request_tracker.closing_index != INVALID_HANDLE_TRACKER_INDEX)
    {
        // Closing another file already
        return;
    }

    if (request_tracker.closing_queue_head ==
        request_tracker.closing_queue_tail)
    {
        // Nothing to process here
        return;
    }

    // Dequeue tracker index
    uint8_t tracker_index =
        request_tracker.closing_queue[request_tracker.closing_queue_head];
    if (++request_tracker.closing_queue_head == MAX_HANDLE_TRACKERS + 1)
    {
        request_tracker.closing_queue_head = 0;
    }

    request_tracker.closing_index = tracker_index;

    // Setup sequence tracker
    w_hal_gsm_setup_foreground_sequence(http_post_sequence,
                                        sizeof(http_post_sequence) /
                                            sizeof(ublox_command_info_st));

    w_hal_gsm_send_foreground_sequence(false);

    // Record time of command for timeout purposes
    w_hal_rtc_get_time(
        request_tracker.handle_trackers[tracker_index].last_command_time);
}

/**
 * @brief Check if there are any available file handles
 *
 * @return uint32_t number of available file handles
 */
uint32_t w_hal_gsm_get_available_post_requests()
{
    if (!interface_params.data_enabled)
    {
        return 0;
    }

    uint32_t available_handles = 0;
    for (uint32_t index = 0; index < MAX_HANDLE_TRACKERS; index++)
    {
        if (request_tracker.handle_trackers[index].handle == INVALID_GSM_HANDLE)
        {
            available_handles++;
        }
    }

    return available_handles;
}

/**
 * @brief Get a handle tracker
 *
 * @return uint32_t handle tracker index, MAX_HANDLE_TRACKERS if no handle
 */
static uint32_t get_available_handle_tracker()
{
    if (!interface_params.data_enabled)
    {
        return MAX_HANDLE_TRACKERS;
    }

    uint32_t index;
    for (index = 0; index < MAX_HANDLE_TRACKERS; index++)
    {
        if (request_tracker.handle_trackers[index].handle == INVALID_GSM_HANDLE)
        {
            break;
        }
    }

    return index;
}

/**
 * @brief Issue request to modem to check and read oldest SMS
 *
 * @param p_sms SMS buffer to hold content
 * @param sms_size SMS buffer size
 * @return wHalErrorCode GSM_SMS_NOT_ENABLED if not enabled,
 * BUSY if SMS read request is not open, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_read_sms_request(char *p_sms, uint32_t sms_size)
{
    // Can't read sms if sms is not enabled
    if (!interface_params.sms_read_enabled)
    {
        return wHalErrorCode::GSM_SMS_NOT_ENABLED;
    }

    // Can't read and send sms at same time
    if (request_tracker.sms_tracker.is_open)
    {
        return wHalErrorCode::BUSY;
    }

    // Empty phone number not valid
    SYSTEM_ASSERT_NOT_EQUAL(interface_params.a2p_phone_number[0],
                            NULL_TERMINATOR);

    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_closing);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.request_chars_remaining, 0);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.transfer_chars_remaining,
                        0);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.sms_buffer_size, 0);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.p_sms_buffer,
                        static_cast<char *>(nullptr));

    request_tracker.sms_tracker.is_open                 = true;
    request_tracker.sms_tracker.read_write              = false;
    request_tracker.sms_tracker.request_chars_remaining = MAX_SMS_LENGTH;

    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.p_sms_buffer,
                        static_cast<char *>(nullptr));
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.read_buffer_fill_index, 0);
    request_tracker.sms_tracker.p_sms_buffer    = p_sms;
    request_tracker.sms_tracker.sms_buffer_size = sms_size;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check if SMS read is done
 *
 * @param sms_size[out] characters in received SMS
 * @return wHalErrorCode  GSM_SMS_NOT_ENABLED if not enabled,
 * GSM_SMS_READ_NOT_OPEN if SMS read request is not open,
 * BUSY if SMS read request is not open, NO_ERROR
 * otherwise.
 */
wHalErrorCode w_hal_gsm_check_sms_read_done(uint32_t &sms_size)
{
    // Can't read sms if sms is not enabled
    if (!interface_params.sms_read_enabled)
    {
        return wHalErrorCode::GSM_SMS_NOT_ENABLED;
    }

    if (!request_tracker.sms_tracker.is_open)
    {
        sms_size = 0;
        return wHalErrorCode::GSM_SMS_READ_NOT_OPEN;
    }

    if (request_tracker.sms_tracker.read_write)
    {
        // Send SMS in progress
        return wHalErrorCode::BUSY;
    }

    if (!request_tracker.sms_tracker.read_completed)
    {
        // SMS read not completed
        return wHalErrorCode::BUSY;
    }

    // The last fill position is the number of characters in the SMS
    sms_size = request_tracker.sms_tracker.read_buffer_fill_index;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Clear SMS read done flag
 *
 * @return wHalErrorCode  GSM_SMS_NOT_ENABLED if not enabled,
 * GSM_SMS_READ_NOT_OPEN if SMS read request is not open,
 * BUSY if SMS read request is not open, NO_ERROR
 * otherwise.
 */
wHalErrorCode w_hal_gsm_clear_sms_read_done()
{
    uint32_t      sms_size;
    wHalErrorCode error = w_hal_gsm_check_sms_read_done(sms_size);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // Reset sms tracker
        request_tracker.sms_tracker.is_closing              = false;
        request_tracker.sms_tracker.is_open                 = false;
        request_tracker.sms_tracker.read_write              = false;
        request_tracker.sms_tracker.read_completed          = false;
        request_tracker.sms_tracker.request_chars_remaining = 0;
        request_tracker.sms_tracker.p_sms_buffer            = NULL;
        request_tracker.sms_tracker.sms_buffer_size         = 0;
        request_tracker.sms_tracker.read_buffer_fill_index  = 0;

        // Mark the SMS message for deletion
        init_tracker.delete_after_read_sms_index =
            init_tracker.oldest_valid_sms_index;
        init_tracker.oldest_valid_sms_index = INVALID_SMS_INDEX;
        init_tracker.oldest_valid_sms_time  = static_cast<time_t>(-1LL);
    }

    return error;
}

/**
 * @brief Open a handle to a SMS request
 *
 * @param sms_size remaining characters in SMS request
 * @return wHalErrorCode BUSY if SMS request is already
 * open, GSM_SMS_NOT_ENABLED if not enabled, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_write_sms_request(uint32_t &sms_size)
{
    if (!interface_params.sms_send_enabled)
    {
        return wHalErrorCode::GSM_SMS_NOT_ENABLED;
    }

    if (request_tracker.sms_tracker.is_open)
    {
        return wHalErrorCode::BUSY;
    }

    // Empty phone number not valid
    SYSTEM_ASSERT_NOT_EQUAL(interface_params.p2a_phone_number[0],
                            NULL_TERMINATOR);

    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_closing);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.request_chars_remaining, 0);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.transfer_chars_remaining,
                        0);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.sms_buffer_size, 0);
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.p_sms_buffer,
                        static_cast<char *>(nullptr));

    request_tracker.sms_tracker.is_open                  = true;
    request_tracker.sms_tracker.read_write               = true;
    request_tracker.sms_tracker.request_chars_remaining  = MAX_SMS_LENGTH;
    request_tracker.sms_tracker.transfer_chars_remaining = MAX_SMS_LENGTH;

    sms_size = request_tracker.sms_tracker.transfer_chars_remaining;
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Add text to SMS content
 *
 * @param p_text the text to add to SMS content
 * @param size number of characters in p_text
 * @param remaining_size[out] remaining characters in SMS request
 * @return wHalErrorCode BUSY if hal is busy, GSM_SMS_REQUEST_NOT_OPEN if SMS
 * request is not open, GSM_SMS_REQUEST_ALREADY_CLOSING if SMS request is
 * already closing, GSM_SMS_REQUEST_TOO_LONG if SMS request is too long,
 * NO_ERROR otherwise.
 */
wHalErrorCode w_hal_gsm_add_sms_content(const char *p_text,
                                        uint32_t    size,
                                        uint32_t &  remaining_size)
{
    w_hal_gsm_handler();

    // If sequence is ongoing, reject request
    if (foreground_command_sequence_tracker.p_sequence != nullptr)
    {
        return wHalErrorCode::BUSY;
    }

    // Can't add content before uploading
    if (!request_tracker.sms_tracker.is_open)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_NOT_OPEN;
    }

    // Can't add content to read request
    if (!request_tracker.sms_tracker.read_write)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_NOT_OPEN;
    }

    // Can't add content to closing request
    if (request_tracker.sms_tracker.is_closing)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_ALREADY_CLOSING;
    }

    // Can't add more content than what is available
    if (size > request_tracker.sms_tracker.request_chars_remaining)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_TOO_LONG;
    }

    // Size must be greater than 0
    if (size == 0)
    {
        return wHalErrorCode::GSM_INVALID_SIZE;
    }

    // Can't add content if HAL is working on previous transferring of content
    if (request_tracker.sms_tracker.sms_buffer_size > 0)
    {
        return wHalErrorCode::BUSY;
    }

    // If SMS is terminated, then it is already sent. Can't add content.
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.terminator_sent);
    SYSTEM_ASSERT_FALSE(request_tracker.sms_tracker.is_terminated);
    SYSTEM_ASSERT_GT(request_tracker.sms_tracker.transfer_chars_remaining, 0);

    // Transfer bytes remaining should always be behind request bytes remaining
    SYSTEM_ASSERT_LTEQ(request_tracker.sms_tracker.request_chars_remaining,
                       request_tracker.sms_tracker.transfer_chars_remaining);

    request_tracker.sms_tracker.request_chars_remaining -= size;

    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.p_sms_buffer,
                        static_cast<char *>(nullptr));
    request_tracker.sms_tracker.p_sms_buffer    = const_cast<char *>(p_text);
    request_tracker.sms_tracker.sms_buffer_size = size;

    remaining_size = request_tracker.sms_tracker.request_chars_remaining;

    w_hal_gsm_handler();

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check if sms content for active handle is still uploading
 *
 * @return true if sms content for active handle is still uploading
 * @return false otherwise
 */
bool w_hal_gsm_check_sms_content_uploading()
{
    // If sms not open, then no sms content uploading
    if (!request_tracker.sms_tracker.is_open)
    {
        return false;
    }

    // If sms is open, but no buffer provided, then no sms content uploading
    return (request_tracker.sms_tracker.sms_buffer_size > 0);
}

/**
 * @brief Close SMS request
 *
 * @return wHalErrorCode BUSY if hal is busy, GSM_SMS_REQUEST_NOT_OPEN if SMS
 * request is not open, GSM_SMS_REQUEST_ALREADY_CLOSING if SMS request is
 * already closing, NO_ERROR otherwise.
 */
wHalErrorCode w_hal_gsm_close_sms_write_request()
{
    w_hal_gsm_handler();

    // Can't add content before uploading
    if (!request_tracker.sms_tracker.is_open)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_NOT_OPEN;
    }

    if (request_tracker.sms_tracker.is_closing)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_ALREADY_CLOSING;
    }

    if (request_tracker.sms_tracker.sms_buffer_size > 0)
    {
        return wHalErrorCode::BUSY;
    }

    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.p_sms_buffer,
                        static_cast<char *>(nullptr));
    SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.sms_buffer_size, 0);

    if (request_tracker.sms_tracker.is_terminated)
    {
        SYSTEM_ASSERT_TRUE(request_tracker.sms_tracker.terminator_sent);
        SYSTEM_ASSERT_EQUAL(
            request_tracker.sms_tracker.transfer_chars_remaining,
            0);
        SYSTEM_ASSERT_EQUAL(request_tracker.sms_tracker.request_chars_remaining,
                            0);

        // SMS is already terminated and sent. Done with SMS request.
        // Reset tracker
        reset_sms_send_tracker();
    }
    else
    {
        // If sequence is ongoing, reject request
        if (foreground_command_sequence_tracker.p_sequence != nullptr)
        {
            return wHalErrorCode::BUSY;
        }

        // Mark SMS for closing.
        if (request_tracker.sms_tracker.terminator_sent)
        {
            // Terminator has been sent but have not gotten response yet
            request_tracker.sms_tracker.transfer_chars_remaining =
                request_tracker.sms_tracker.request_chars_remaining =
                    request_tracker.sms_tracker.sms_buffer_size;
        }
        else
        {
            SYSTEM_ASSERT_EQUAL(
                request_tracker.sms_tracker.request_chars_remaining,
                request_tracker.sms_tracker.transfer_chars_remaining);
            request_tracker.sms_tracker.request_chars_remaining  = 0;
            request_tracker.sms_tracker.transfer_chars_remaining = 0;
        }
        request_tracker.sms_tracker.is_closing = true;

        w_hal_gsm_handler();
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check if post request for handle is not open
 *
 * @param handle handle for open post request
 * @return true if post request is still active
 * @return false if post request does not exist or has closed
 */
bool w_hal_gsm_check_sms_request_closed()
{
    return !request_tracker.sms_tracker.is_open;
}

/**
 * @brief Check if a post request can be opened
 *
 * @return wHalErrorCode GSM_TRANSFER_NO_HANDLE_AVAILABLE if no handle,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_check_can_open_post_request()
{
    uint32_t index = get_available_handle_tracker();

    if (index == MAX_HANDLE_TRACKERS)
    {
        return wHalErrorCode::GSM_TRANSFER_NO_HANDLE_AVAILABLE;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Open a post request to be performed
 *
 * @param p_api_route : route for post request
 * @param content_type : content type for post request
 * @param p_file_name : file name for post request
 * @param handle [out] : handle for open post request
 * @param total_size : total size of all the transfers (optional)
 * @return wHalErrorCode GSM_TRANSFER_NO_HANDLE_AVAILABLE if no handle
 * available. BUSY if HAL is busy, NO_ERROR otherwise.
 */
wHalErrorCode w_hal_gsm_open_post_request(const char *        p_api_route,
                                          http_content_type_t content_type,
                                          const char *        p_file_name,
                                          wGsmTransferHandle &handle,
                                          int                 total_size)
{
    // Empty filename not valid
    SYSTEM_ASSERT_NOT_EQUAL(p_file_name, static_cast<const char *>(nullptr));
    SYSTEM_ASSERT_NOT_EQUAL(p_file_name[0], NULL_TERMINATOR);

    uint32_t index = get_available_handle_tracker();
    if (index == MAX_HANDLE_TRACKERS)
    {
        return wHalErrorCode::GSM_TRANSFER_NO_HANDLE_AVAILABLE;
    }

    // Don't allow opening of POST requests if SMS is open
    if (request_tracker.sms_tracker.is_open)
    {
        return wHalErrorCode::BUSY;
    }

    handle = w_hal_rtc_get_time();
    SYSTEM_ASSERT_NOT_EQUAL(handle, INVALID_GSM_HANDLE);

    request_tracker.handle_trackers[index].handle                = handle;
    request_tracker.handle_trackers[index].expected_upload_bytes = total_size;
    request_tracker.handle_trackers[index].uploaded_bytes        = 0;

    // File name length does not include NULL character
    uint32_t file_name_length = strlen(p_file_name) + 1;
    SYSTEM_ASSERT_LT(file_name_length, MAX_FILENAME_LENGTH);
    strcpy(request_tracker.handle_trackers[index].filename, p_file_name);

    // Route length does not include NULL character
    uint32_t route_length = strlen(p_api_route) + 1;
    SYSTEM_ASSERT_LT(route_length, MAX_SERVER_POST_ROUTE_LENGTH);
    strcpy(request_tracker.handle_trackers[index].api_route, p_api_route);

    switch (content_type)
    {
        case http_content_type_t::OCTET_STREAM:
            request_tracker.handle_trackers[index].http_content_type =
                ublox_http_content_type_t::OCTET_STREAM;
            break;
        case http_content_type_t::MULTIPART_FORM_DATA:
            request_tracker.handle_trackers[index].http_content_type =
                ublox_http_content_type_t::MULTIPART_FORM_DATA;
            break;
        default:
            SYSTEM_ASSERT_TRUE(false);
            break;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Add data to post request
 *
 * @param p_buffer buffer data to be transferred
 * @param size size of buffer data
 * @param handle handle for open post request
 * @return wHalErrorCode BUSY if request cannot be serviced now,
 * GSM_INVALID_HANDLE if handle is not open or is invalid, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_gsm_upload(const uint8_t *    p_buffer,
                               uint32_t           size,
                               wGsmTransferHandle handle)
{
    w_hal_gsm_handler();

    // If sequence is ongoing, reject request
    if (foreground_command_sequence_tracker.p_sequence != nullptr)
    {
        return wHalErrorCode::BUSY;
    }

    // Size must be greater than 0
    if (size == 0)
    {
        return wHalErrorCode::GSM_INVALID_SIZE;
    }

    // None of the handles match argument handle?
    uint8_t tracker_index = w_hal_gsm_get_tracker_index(handle);
    if (tracker_index == INVALID_HANDLE_TRACKER_INDEX)
    {
        return wHalErrorCode::GSM_INVALID_HANDLE;
    }

    // Make sure size is multiple of 4
    SYSTEM_ASSERT_EQUAL(size % sizeof(uint32_t), 0);
    request_tracker.upload_index              = tracker_index;
    request_tracker.p_upload_buffer           = const_cast<uint8_t *>(p_buffer);
    request_tracker.upload_buffer_size        = size;
    request_tracker.upload_buffer_transferred = 0;

    w_hal_gsm_handler();

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check if post request for handle is actively uploading
 *
 * @param handle handle for open post request
 * @return true if post request is still open and upload is happening for
 * handle
 * @return false otherwise
 */
bool w_hal_gsm_check_active_upload(wGsmTransferHandle handle)
{
    uint8_t            upload_index  = request_tracker.upload_index;
    wGsmTransferHandle upload_handle = INVALID_GSM_HANDLE;
    if (upload_index != INVALID_HANDLE_TRACKER_INDEX)
    {
        upload_handle = request_tracker.handle_trackers[upload_index].handle;
    }

    return (handle == upload_handle) &&
           (request_tracker.upload_buffer_size != 0);
}

/**
 * @brief Close post request for handle
 *
 * @param handle handle for open post request
 * @return wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_close_post_request(wGsmTransferHandle handle)
{
    // None of the active handles match argument handle?
    uint8_t tracker_index = w_hal_gsm_get_tracker_index(handle);
    if (tracker_index == INVALID_HANDLE_TRACKER_INDEX)
    {
        return wHalErrorCode::GSM_INVALID_HANDLE;
    }

    // We should have no more data to send
    if (request_tracker.handle_trackers[tracker_index].expected_upload_bytes !=
        INVALID_GSM_UPLOAD_SIZE)
    {
        SYSTEM_ASSERT_EQUAL(
            request_tracker.handle_trackers[tracker_index].uploaded_bytes,
            static_cast<uint32_t>(request_tracker.handle_trackers[tracker_index]
                                      .expected_upload_bytes));
    }

    request_tracker.closing_queue[request_tracker.closing_queue_tail] =
        tracker_index;
    if (++request_tracker.closing_queue_tail == MAX_HANDLE_TRACKERS + 1)
    {
        request_tracker.closing_queue_tail = 0;
    }
    SYSTEM_ASSERT_NOT_EQUAL(request_tracker.closing_queue_head,
                            request_tracker.closing_queue_tail);

    w_hal_gsm_handler();

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check if post request for handle is not open
 *
 * @param handle handle for open post request
 * @return true if post request is still active
 * @return false if post request does not exist or has closed
 */
bool w_hal_gsm_check_post_request_closed(wGsmTransferHandle handle)
{
    uint8_t            closing_index  = request_tracker.closing_index;
    wGsmTransferHandle closing_handle = INVALID_GSM_HANDLE;
    if (closing_index != INVALID_HANDLE_TRACKER_INDEX)
    {
        closing_handle = request_tracker.handle_trackers[closing_index].handle;
    }

    uint8_t            deleting_index  = request_tracker.deleting_index;
    wGsmTransferHandle deleting_handle = INVALID_GSM_HANDLE;
    if (deleting_index != INVALID_HANDLE_TRACKER_INDEX)
    {
        deleting_handle =
            request_tracker.handle_trackers[deleting_index].handle;
    }

    bool    closing_handle_queued  = false;
    bool    deleting_handle_queued = false;
    uint8_t tracker_index          = w_hal_gsm_get_tracker_index(handle);
    if (tracker_index != INVALID_HANDLE_TRACKER_INDEX)
    {
        for (uint8_t head = request_tracker.closing_queue_head;
             head != request_tracker.closing_queue_tail;)
        {
            if (tracker_index == request_tracker.closing_queue[head])
            {
                closing_handle_queued = true;
                break;
            }

            if (++head == MAX_HANDLE_TRACKERS + 1)
            {
                head = 0;
            }
        }

        for (uint8_t head = request_tracker.deleting_queue_head;
             head != request_tracker.deleting_queue_tail;)
        {
            if (tracker_index == request_tracker.deleting_queue[head])
            {
                deleting_handle_queued = true;
                break;
            }

            if (++head == MAX_HANDLE_TRACKERS + 1)
            {
                head = 0;
            }
        }
    }

    return (tracker_index == INVALID_HANDLE_TRACKER_INDEX) ||
           (handle == closing_handle) || (handle == deleting_handle) ||
           closing_handle_queued || deleting_handle_queued;
}

/**
 * @brief Poll background health information
 *
 * @return true if a background health sequence was set up, false otherwise
 */
static bool w_hal_gsm_background_health_poll()
{
    if (!interface_params.data_enabled)
    {
        return false;
    }

    // If we already have a sequence running, we should not start
    // another one
    if (internal_command_sequence_tracker.p_sequence != nullptr)
    {
        return false;
    }

    if (check_in_middle_of_file_transfer())
    {
        // Can't issue command in between file transfers
        return false;
    }

    if (check_in_middle_of_send_sms_transfer())
    {
        // Can't send file data in middle of sms
        return false;
    }

    // Get the current time
    uint32_t                       current_time = w_hal_rtc_get_time();

    // Start polling from the last poll index
    uint32_t &                     poll_index   = background_poll_tracker.index;
    const background_poll_item_st *p_items      = background_poll_items;

    bool                           command_sequence_set = false;
    const uint8_t                  num_items =
        static_cast<uint8_t>(background_poll_item_index_t::NUM_ITEMS);
    for (uint32_t index = 0; index < num_items; index++)
    {
        SYSTEM_ASSERT_NOT_EQUAL(
            p_items[poll_index].p_sequence,
            static_cast<const ublox_command_info_st *>(nullptr));

        // If it is time to poll this item of information, do so
        if (current_time - background_poll_tracker.last_time[poll_index] >=
            p_items[poll_index].idle_wait_time)
        {
            w_hal_gsm_setup_internal_sequence(p_items[poll_index].p_sequence,
                                              p_items[poll_index].count);

            w_hal_gsm_send_internal_sequence(false);

            background_poll_tracker.last_time[poll_index] = current_time;

            command_sequence_set                          = true;
        }

        if (++poll_index >= num_items)
        {
            poll_index = 0;
        }

        // We should only set up one sequence at a time, wait for it to
        // finish. Then we can set up the next sequence.
        if (command_sequence_set)
        {
            break;
        }
    }

    return command_sequence_set;
}

/**
 * @brief Check if command is abortable
 *
 * @param p_command command to check
 * @return true if command is abortable, false otherwise
 * @return false otherwise
 */
static bool is_command_abortable(const char *p_command)
{
    for (uint32_t abort_index = 0;
         abort_index <
         (sizeof(ABORTABLE_COMMANDS) / sizeof(ABORTABLE_COMMANDS[0]));
         abort_index++)
    {
        if ((strlen(p_command) == strlen(ABORTABLE_COMMANDS[abort_index])) &&
            (strcmp(p_command, ABORTABLE_COMMANDS[abort_index]) == 0))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Abort the current command
 *
 * @param p_command command to abort
 */
static bool abort_current_command(const char *p_command)
{
    // If command is abortable, abort it
    bool abortable = is_command_abortable(p_command);

    if (abortable)
    {
        // Send an AT command to abort the current command
        memcpy(command_buffer, AT_COMMAND, sizeof(AT_COMMAND));
        w_hal_uart_write_data(GSM_UART_CHANNEL,
                              reinterpret_cast<uint8_t *>(command_buffer),
                              sizeof(AT_COMMAND) - 1);
    }

    return abortable;
}

/**
 * @brief Check if the current command has timed out.
 * If it has and it is abortable, abort it.
 */
static void handle_internal_sequence_timeout()
{
    // If we are still in the middle of a command sequence,
    // check if we need to timeout
    if ((internal_command_sequence_tracker.p_sequence != nullptr) &&
        (command_tracker.command_state ==
         command_state_t::WAITING_FOR_RESPONSE))
    {
        w_hal_rtc_time_st &last_command_time =
            internal_command_sequence_tracker.last_time;

        // Last command time not being valid means the command has
        // not been sent yet
        if (last_command_time.is_valid())
        {
            // Check for timeout
            w_hal_rtc_time_st current_time;
            w_hal_rtc_get_time(current_time);

            w_hal_rtc_time_delta_st time_delta;
            current_time.subtract(last_command_time, time_delta);

            if (time_delta.seconds >= 1)
            {
                // Try to abort the current command
                if (abort_current_command(
                        internal_command_sequence_tracker
                            .p_sequence[internal_command_sequence_tracker
                                            .current_index]
                            .p_command))
                {
                    // Take evasive action
                    LOG_MSG_WITH_ARGS(
                        TraceFile::W_HAL_GSM_CPP,
                        "Command %s timed out and was aborted...\n",
                        internal_command_sequence_tracker.p_sequence
                            ->p_command);
                }

                // Reset the last command time to avoid repeating
                // this block
                last_command_time = current_time;
            }
        }
    }
}

/**
 * @brief Run background activities for GSM HAL
 *
 * @return wHalErrorCode BUSY if there is work to be done, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_gsm_handler()
{
    wHalErrorCode error = wHalErrorCode::NO_ERROR;

    // Prioritize internal command sequence first
    if (internal_command_sequence_tracker.p_sequence != nullptr)
    {
        w_hal_gsm_send_internal_sequence(false);
        error = wHalErrorCode::BUSY;

        handle_internal_sequence_timeout();
    }
    // Cannot initiate any internal commands if in the middle of a
    // foreground command sequence
    else if (((foreground_command_sequence_tracker.p_sequence == nullptr) ||
              (foreground_command_sequence_tracker.current_index == 0)) &&
             (command_tracker.command_state == command_state_t::NOT_ACTIVE) &&
             (request_tracker.cmd_bytes_transferred == 0) &&
             (request_tracker.send_size == 0) &&
             !check_in_middle_of_send_sms_transfer())
    {
        // If an HTTP request failed on a protocol error, we need to
        // get the error reason
        if (w_hal_gsm_check_http_error_reason())
        {
            error = wHalErrorCode::BUSY;
        }
        // If we are not in the middle of a sequence or a +UDWNFILE command,
        // we can start a background operation
        else if (w_hal_gsm_check_reactivation())
        {
            error = wHalErrorCode::BUSY;
        }
        else
        {
            // If there is a sms to delete from the file system
            if (w_hal_gsm_check_delete_after_receive_sms())
            {
                error = wHalErrorCode::BUSY;
            }

            // If there is a file to delete from the file system
            if (w_hal_gsm_check_file_cleanup())
            {
                error = wHalErrorCode::BUSY;
            }

            // If there are sms messages to delete from the mt storage
            if (w_hal_gsm_check_sms_cleanup())
            {
                error = wHalErrorCode::BUSY;
            }

            // Poll the module for any issues
            if (w_hal_gsm_background_health_poll())
            {
                error = wHalErrorCode::BUSY;
            }
        }
    }

    if (error != wHalErrorCode::BUSY)
    {
        if (foreground_command_sequence_tracker.p_sequence != nullptr)
        {
            w_hal_gsm_send_foreground_sequence(false);
        }
        else
        {
            check_for_response(handle_generic_response);

            w_hal_gsm_check_receive_sms();
            w_hal_gsm_check_send_sms();
            w_hal_gsm_check_handle_close();
            w_hal_gsm_check_file_upload();
        }
    }

    // If there was some work needed, a sequence would have been
    // triggered
    if ((internal_command_sequence_tracker.p_sequence != nullptr) ||
        (foreground_command_sequence_tracker.p_sequence != nullptr))
    {
        error = wHalErrorCode::BUSY;
    }

    return error;
}

/**
 * @brief Get the gsm modem name string
 *
 * @param p_modem_name[out]: GSM modem name string
 * @param modem_size[out]: size of the modem name string
 */
void w_hal_gsm_get_modem_name(const char *&p_modem_name, uint32_t &modem_size)
{
    p_modem_name = ublox_info.model_name;
    modem_size   = strlen(ublox_info.model_name) + 1;
}

/**
 * @brief Get the gsm firmware version string
 *
 * @param p_fw_version[out]: GSM firmware version string
 * @param version_size[out]: size of firmware version string
 */
void w_hal_gsm_get_fw_version(const char *&p_fw_version, uint32_t &version_size)
{
    p_fw_version = ublox_info.fw_version;
    version_size = strlen(ublox_info.fw_version) + 1;
}

/**
 * @brief Get the gsm imei number string
 *
 * @param p_imei[out]: imei number string
 * @param imei_size[out]: size of imei number string
 */
void w_hal_gsm_get_imei(const char *&p_imei, uint32_t &imei_size)
{
    p_imei    = ublox_info.imei;
    imei_size = strlen(ublox_info.imei) + 1;
}

/**
 * @brief Get the gsm imsi number string
 *
 * @param p_imsi[out]: imsi number string
 * @param imsi_size[out]: size of imsi number string
 */
void w_hal_gsm_get_imsi(const char *&p_imsi, uint32_t &imsi_size)
{
    p_imsi    = ublox_info.imsi;
    imsi_size = strlen(ublox_info.imsi) + 1;
}

/**
 * @brief Get the gsm iccid string
 *
 * @param p_iccid[out]: iccid string
 * @param iccid_size[out]: size of iccid string
 */
void w_hal_gsm_get_iccid(const char *&p_iccid, uint32_t &iccid_size)
{
    p_iccid    = ublox_info.iccid;
    iccid_size = strlen(ublox_info.iccid) + 1;
}

/**
 * @brief Get the gsm operator name string
 *
 * @param p_operator_name[out]: operator name string
 * @param operator_size[out]: size of operator name string
 */
void w_hal_gsm_get_operator_name(const char *&p_operator_name,
                                 uint32_t &   operator_size)
{
    p_operator_name = ublox_info.operator_name;
    operator_size   = strlen(ublox_info.operator_name) + 1;
}

/**
 * @brief Get the gsm gprs status
 *
 * @return uint8_t: Returns the gprs attachment status
 */
uint8_t w_hal_gsm_get_gprs_status()
{
    return static_cast<uint8_t>(ublox_info.gprs_attachment);
}

/**
 * @brief Get the gsm power status
 *
 * @return uint8_t: Returns the gsm power status
 */
uint8_t w_hal_gsm_get_power_status()
{
    return static_cast<uint8_t>(ublox_info.power_saving_mode);
}

/**
 * @brief Get the gsm sim status
 *
 * @return uint8_t: Returns SIM status
 */
uint8_t w_hal_gsm_get_sim_status()
{
    return static_cast<uint8_t>(ublox_info.sim_lock_status);
}

/**
 * @brief Get the gsm signal quality
 *
 * @param rssi[out]: received signal strength indication
 * @param rxqual[out]:  bit error rate
 * @param rscp[out]: received signal code power
 * @param ecno[out]: Ratio of received energy per PN chip to the
 * total received power spectral density
 * @param rsrp[out]: reference signal received power
 * @param rsrq[out]: reference signal received quality
 */
void w_hal_gsm_get_signal_quality(int32_t &rssi,
                                  int32_t &rxqual,
                                  int32_t &rscp,
                                  int32_t &ecno,
                                  int32_t &rsrp,
                                  int32_t &rsrq)
{
    // received signal strength indication
    if (is_sim800l_module())
    {
        if (ublox_info.signal_quality.rxlev <= 1)
        {
            rssi = -115 + 4 * ublox_info.signal_quality.rxlev;
        }
        else if (ublox_info.signal_quality.rxlev < 32)
        {
            rssi = -114 + 2 * ublox_info.signal_quality.rxlev;
        }
        else
        {
            rssi = INT8_MIN;
        }
    }
    else
    {
        if (ublox_info.signal_quality.rxlev < 32)
        {
            rssi = -113 + 2 * ublox_info.signal_quality.rxlev;
        }
        else if (ublox_info.signal_quality.rxlev < INVALID_RSSI_VALUE)
        {
            rssi = -51;
        }
        else
        {
            rssi = INT8_MIN;
        }
    }

    // bit error rate
    if (ublox_info.signal_quality.ber < 8)
    {
        rxqual = ublox_info.signal_quality.ber;
    }
    else
    {
        rxqual = INT8_MIN;
    }

    // received signal code power
    if (ublox_info.signal_quality.rscp < 96)
    {
        rscp = -121 + ublox_info.signal_quality.rscp;
    }
    else if (ublox_info.signal_quality.rscp < INVALID_RSCP_VALUE)
    {
        rscp = -25;
    }
    else
    {
        rscp = INT8_MIN;
    }

    // Ratio of received energy per PN chip to the total received power
    // spectral density
    if (ublox_info.signal_quality.ecno < 49)
    {
        ecno = (-49 + ublox_info.signal_quality.ecno) / 2;
    }
    else if (ublox_info.signal_quality.ecno < INVALID_ECNO_VALUE)
    {
        ecno = 0;
    }
    else
    {
        ecno = INT8_MIN;
    }

    // reference signal received power
    if (ublox_info.signal_quality.rsrp < 97)
    {
        rsrp = -141 + ublox_info.signal_quality.rsrp;
    }
    else if (ublox_info.signal_quality.rsrp < INVALID_RSRP_VALUE)
    {
        rsrp = -44;
    }
    else
    {
        rsrp = INT16_MIN;
    }

    // reference signal received quality
    if (ublox_info.signal_quality.rsrq < 34)
    {
        rsrq = (-40 + ublox_info.signal_quality.rsrq) / 2;
    }
    else if (ublox_info.signal_quality.rsrq < INVALID_RSRQ_VALUE)
    {
        rsrq = -3;
    }
    else
    {
        rsrq = INT8_MIN;
    }
}

/**
 * @brief Get the gsm carrier time
 *
 * @return time_t: Returns the Carrier time
 */
time_t w_hal_gsm_get_carrier_time()
{
    return ublox_info.carrier_time_value;
}

/**
 * @brief Get the gsm pdp address string
 *
 * @param p_pdp_address[out]: PDP address
 * @param pdp_address_size[out]: size of the PDP address string
 */
void w_hal_gsm_get_pdp_address(const char *&p_pdp_address,
                               uint32_t &   pdp_address_size)
{
    p_pdp_address    = ublox_info.pdp_address;
    pdp_address_size = strlen(ublox_info.pdp_address) + 1;
}

/**
 * @brief Get the gsm location area code string
 *
 * @param p_location_area_code[out]: location area code
 * @param location_area_code_size[out]: size of the location area code
 * string
 */
void w_hal_gsm_get_location_area_code(const char *&p_location_area_code,
                                      uint32_t &   location_area_code_size)
{
    p_location_area_code    = ublox_info.network.location_area_code;
    location_area_code_size = strlen(ublox_info.network.location_area_code) + 1;
}

/**
 * @brief Get the gsm cell id string
 *
 * @param p_cell_id[out]: cell id
 * @param cell_id_size[out]: size of the cell id string
 */
void w_hal_gsm_get_cell_id(const char *&p_cell_id, uint32_t &cell_id_size)
{
    p_cell_id    = ublox_info.network.cell_id;
    cell_id_size = strlen(ublox_info.network.cell_id) + 1;
}

/**
 * @brief To get the phone number from the SIM
 *
 * @param p_phone_number[out]: phone number
 * @param phone_number_size[out]: size of the phone number string
 */
void w_hal_gsm_get_phone_number(const char *&p_phone_number,
                                uint32_t &   phone_number_size)
{
    p_phone_number    = ublox_info.phone_number;
    phone_number_size = strlen(ublox_info.phone_number) + 1;
}

/**
 * @brief To get the max file system size for post request
 *
 * @return uint32_t: Returns the max supported post request size
 */
uint32_t w_hal_gsm_get_max_post_request_size()
{
    return (ublox_info.max_file_system_size / 2);
}

/**
 * @brief Print GSM Information obtained during Initialization
 */
void w_hal_gsm_print_info()
{
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Modem Manufacturer Name: %s, Model Name: %s\n",
                      ublox_info.manufacturer_name,
                      ublox_info.model_name);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Modem FW Version: %s\n",
                      ublox_info.fw_version);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "IMEI: %s, IMSI: %s, ICCID: %s\n",
                      ublox_info.imei,
                      ublox_info.imsi,
                      ublox_info.iccid);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "GPRS attachment state: %d\n",
                      ublox_info.gprs_attachment);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Operator Mode: %d, Format: %d, Name: %s, "
                      "Type detected: %d\n",
                      ublox_info.operator_mode,
                      ublox_info.operator_format,
                      ublox_info.operator_name,
                      init_tracker.operator_type);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Reg URC Config: %d, Status: %d, LAC: %s, "
                      "Cell ID: %s, RAT Status: %d\n",
                      ublox_info.network.registration_urc_config,
                      ublox_info.network.registration_status,
                      ublox_info.network.location_area_code,
                      ublox_info.network.cell_id,
                      ublox_info.network.radio_access_tech);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "GPRS Reg URC Config: %d, Status: %d, LAC: %s, "
                      "Cell ID: %s, RAT Status: %d\n",
                      ublox_info.gprs_network.registration_urc_config,
                      ublox_info.gprs_network.registration_status,
                      ublox_info.gprs_network.location_area_code,
                      ublox_info.gprs_network.cell_id,
                      ublox_info.gprs_network.radio_access_tech);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "EPS Reg URC Config: %d, Status: %d, LAC: %s, "
                      "Cell ID: %s, RAT Status: %d\n",
                      ublox_info.eps_network.registration_urc_config,
                      ublox_info.eps_network.registration_status,
                      ublox_info.eps_network.location_area_code,
                      ublox_info.eps_network.cell_id,
                      ublox_info.eps_network.radio_access_tech);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "RSSI index: %d, BER: %d, RSCP: %d, "
                      "ECNO :%d, RSRQ: %d, RSRP: %d\n",
                      ublox_info.signal_quality.rxlev,
                      ublox_info.signal_quality.ber,
                      ublox_info.signal_quality.rscp,
                      ublox_info.signal_quality.ecno,
                      ublox_info.signal_quality.rsrq,
                      ublox_info.signal_quality.rsrp);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Modem Power Saving Mode: %d\n",
                      ublox_info.power_saving_mode);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Sim Lock Status: %d\n",
                      ublox_info.sim_lock_status);

    tm tm_struct;
    gmtime_r(&ublox_info.init_time_value, &tm_struct);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                      "Start Time: %04d/%02d/%02d_%02d:%02d:%02d\n",
                      TIME_BASE_YEAR + static_cast<uint32_t>(tm_struct.tm_year),
                      TIME_BASE_MONTH + static_cast<uint32_t>(tm_struct.tm_mon),
                      tm_struct.tm_mday,
                      tm_struct.tm_hour,
                      tm_struct.tm_min,
                      tm_struct.tm_sec);
}

/**
 * @brief Teardown of the GSM
 *
 * @return wHalErrorCode BUSY if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_teardown()
{
    wHalErrorCode error = wHalErrorCode::BUSY;

    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_gsm_handler();
    }

    // shutdown setup for gsm
    w_hal_gsm_setup_internal_sequence(shutdown_sequence,
                                      sizeof(shutdown_sequence) /
                                          sizeof(ublox_command_info_st));

    w_hal_gsm_send_internal_sequence(true);

    w_hal_system_delay_ms(100);

    // UART Teardown function
    w_hal_uart_teardown(GSM_UART_CHANNEL);

    return error;
}

/**
 * @brief GSM file cleanup before shutting it down
 */
void w_hal_gsm_cleanup(void)
{
    // Wait for idle
    wHalErrorCode error = wHalErrorCode::BUSY;
    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_gsm_handler();
    }

    while (check_in_middle_of_file_transfer())
    {
        const ublox_command_info_st *p_sequence;
        uint32_t                     num_commands;

        // Use sequence that expects a response for the last piece of flush
        // data
        if (request_tracker.cmd_size - request_tracker.cmd_bytes_transferred <=
            sizeof(command_buffer))
        {
            p_sequence   = flush_with_response_sequence;
            num_commands = sizeof(flush_with_response_sequence) /
                           sizeof(ublox_command_info_st);
        }
        else
        {
            p_sequence   = flush_no_response_sequence;
            num_commands = sizeof(flush_no_response_sequence) /
                           sizeof(ublox_command_info_st);
        }

        // shutdown setup for flush
        w_hal_gsm_setup_internal_sequence(p_sequence, num_commands);

        w_hal_gsm_send_internal_sequence(true);
    }

    // Wait for idle
    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_gsm_handler();
    }
}

/**
 * @brief To enter to the sleep mode for gsm
 *
 * @param sleep flag to set for sleep mode
 * @return wHalErrorCode wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_sleep(bool sleep)
{
    if (init_tracker.gsm_sleep_flag == sleep)
    {
        return wHalErrorCode::NO_ERROR;
    }

    wHalErrorCode error = wHalErrorCode::BUSY;
    error               = w_hal_gsm_handler();

    // Setup GPIO for sleep mode
    if (error == wHalErrorCode::NO_ERROR)
    {
        w_hal_gsm_enter_sleep_mode(sleep);
        // Setting the sleep flag
        init_tracker.gsm_sleep_flag = sleep;
    }

    return error;
}

/**
 * @brief to check if gsm receives any SMS
 *
 * @return true: SMS received
 * @return false: SMS not received
 */
bool w_hal_gsm_need_to_check_for_receive_sms()
{
#if USE_RING_INDICATOR
    return w_hal_gsm_has_ring();
#else
    // If no ring indicator, we need to poll for SMS
    w_hal_rtc_time_st current_time;
    w_hal_rtc_time_delta_st time_delta;

    w_hal_rtc_get_time(current_time);
    current_time.subtract(request_tracker.sms_tracker.last_received_time,
                          time_delta);

    return (time_delta.seconds >= SMS_RECEIVE_CHECK_TIMEOUT_SECONDS);
#endif
}

/**
 * @brief To get the sleep state of an modem
 *
 * @return true: If it is in sleep
 * @return false: If it is in wakeup state
 */
bool w_hal_gsm_get_sleep_state()
{
    return init_tracker.gsm_sleep_flag;
}

#endif // __CORE__ == COMM_CORE

/**
 * @brief Validate phone number
 *
 * @param p_phone_number phone number string to validate
 * @param fix indicates if the phone number should be fixed
 * @return wHalErrorCode : return enum type error code value
 */
static wHalErrorCode w_hal_gsm_validate_phone_number(char *p_phone_number,
                                                     bool  fix)
{
    const uint32_t phone_length = strlen(p_phone_number);
    if (phone_length > 0)
    {
        int match_index;
        int match_length = 0;

        // Check for long code pattern match
        match_index =
            re_match("\\+\\d+", p_phone_number, phone_length, &match_length);

        bool match = (match_index == 0) &&
                     (static_cast<size_t>(match_length) == phone_length);

        if (!match)
        {
            // Long code format check failed, try short code format
            if ((phone_length >= MIN_SHORT_CODE_PHONE_NUMBER_LENGTH) &&
                (phone_length <= MAX_SHORT_CODE_PHONE_NUMBER_LENGTH))
            {
                match_index = re_match("[A-Za-z0-9]+",
                                       p_phone_number,
                                       phone_length,
                                       &match_length);

                match       = (match_index == 0) &&
                        (static_cast<size_t>(match_length) == phone_length);
            }
        }

        if (!match)
        {
            // Something is wrong with the phone number
            if (fix)
            {
                // Clear it out
                memset(p_phone_number, 0, MAX_PHONE_NUMBER_LENGTH);
            }
            else
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                  "Server phone number %s is not "
                                  "valid. It must start with a country "
                                  "code (e.g. +91) "
                                  "followed immediately by phone "
                                  "digits with no spaces.\n",
                                  p_phone_number);

                return wHalErrorCode::GSM_INVALID_CONFIG;
            }
        }
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Validate GSM configuration
 *
 * @param device_config device configuration
 * @param fix indicates if the configuration should be fixed
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_gsm_validate_config(DeviceConfig::config_page_st &device_config, bool fix)
{

    // Validate the A2P server phone number
    wHalErrorCode error =
        w_hal_gsm_validate_phone_number(device_config.a2p_phone_number, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Validate the P2A server phone number
    error =
        w_hal_gsm_validate_phone_number(device_config.p2a_phone_number, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Validate the GSM mode supported by modem
    if (static_cast<uint32_t>(device_config.transmission_mode) >
        static_cast<uint32_t>(DeviceConfig::max_transmission_mode))
    {
        if (fix)
        {
            printf("Mentioned GSM mode won't be supported\n");
            device_config.transmission_mode =
                DeviceConfig::max_transmission_mode;
        }
        else
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_GSM_CPP,
                "Transmission mode %d exceeds max transmission mode %d\n",
                static_cast<uint32_t>(device_config.transmission_mode),
                static_cast<uint32_t>(DeviceConfig::max_transmission_mode));

            return wHalErrorCode::GSM_INVALID_CONFIG;
        }
    }

    if (device_config.transmission_mode !=
        DeviceConfig::transmission_mode_t::MODE_2G)
    {
        uint32_t data_bit_rate = DeviceConfig::get_data_bit_rate(device_config);

        uint32_t baud_requirement =
            (data_bit_rate * BAUD_MARGIN_FACTOR) / BAUD_MARGIN_DIVISOR;

        if (baud_requirement > BAUD_RATE)
        {
            if (fix)
            {
                do
                {
                    // First try lowering the sampling width, then the number of
                    // channels, then the sampling rate
                    if (device_config.audio_sampling_width >
                        BITS_PER_AUDIO_SAMPLE_16)
                    {
                        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                          "Baud requirement %d at %d "
                                          "bits per sample is not supported."
                                          " Lowering to %d bits per sample\n",
                                          baud_requirement,
                                          device_config.audio_sampling_width,
                                          BITS_PER_AUDIO_SAMPLE_16);

                        device_config.audio_sampling_width =
                            BITS_PER_AUDIO_SAMPLE_16;

                        data_bit_rate =
                            DeviceConfig::get_data_bit_rate(device_config);

                        baud_requirement =
                            (data_bit_rate * BAUD_MARGIN_FACTOR) /
                            BAUD_MARGIN_DIVISOR;

                        if (baud_requirement <= BAUD_RATE)
                        {
                            break;
                        }
                    }

                    if (device_config.num_audio_channels > 1)
                    {
                        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                          "Baud requirement %d at "
                                          "%d channels is not supported. "
                                          "Lowering to 1 channel\n",
                                          baud_requirement,
                                          device_config.num_audio_channels);

                        device_config.num_audio_channels = 1;

                        data_bit_rate =
                            DeviceConfig::get_data_bit_rate(device_config);

                        baud_requirement =
                            (data_bit_rate * BAUD_MARGIN_FACTOR) /
                            BAUD_MARGIN_DIVISOR;

                        if (baud_requirement <= BAUD_RATE)
                        {
                            break;
                        }
                    }

                    if (device_config.sampling_rate > AUDIO_SAMPLE_RATE_16KHz)
                    {
                        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                          "Baud requirement %d at %d Hz "
                                          "is not supported. "
                                          "Lowering to %d Hz\n",
                                          baud_requirement,
                                          device_config.sampling_rate,
                                          AUDIO_SAMPLE_RATE_16KHz);

                        device_config.sampling_rate = AUDIO_SAMPLE_RATE_16KHz;

                        data_bit_rate =
                            DeviceConfig::get_data_bit_rate(device_config);

                        baud_requirement =
                            (data_bit_rate * BAUD_MARGIN_FACTOR) /
                            BAUD_MARGIN_DIVISOR;

                        if (baud_requirement <= BAUD_RATE)
                        {
                            break;
                        }
                    }

                    return wHalErrorCode::GSM_INVALID_CONFIG;

                } while (false);
            }
            else
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                  "Baud requirement %d at "
                                  "%d bits per sample, %d "
                                  "channels, and %d Hz is not supported.\n",
                                  baud_requirement,
                                  device_config.audio_sampling_width,
                                  device_config.num_audio_channels,
                                  device_config.sampling_rate);

                return wHalErrorCode::GSM_INVALID_CONFIG;
            }
        }

        // Validate the server URL
        const uint32_t url_length = strlen(device_config.server_url);
        if (url_length > 0)
        {
            int match_index;
            int match_length = 0;

            match_index      = re_match("[a-zA-Z0-9_\\.\\-]+",
                                   device_config.server_url,
                                   sizeof(device_config.server_url),
                                   &match_length);

            if ((match_index == RE_NO_MATCH_INDEX) || (match_index != 0) ||
                (static_cast<uint32_t>(match_length) != url_length))
            {
                // Something is wrong with the URL
                if (fix)
                {
                    // Clear it out
                    memset(device_config.server_url, 0, MAX_SERVER_URL_LENGTH);
                }
                else
                {
                    LOG_MSG_WITH_ARGS(
                        TraceFile::W_HAL_GSM_CPP,
                        "Server URL %s is not valid. It must exclude http:// "
                        "or https:// and must end with a domain name (e.g. "
                        ".com) "
                        "with no spaces.\n",
                        device_config.server_url);

                    return wHalErrorCode::GSM_INVALID_CONFIG;
                }
            }
        }

        // Validate post api route
        const uint32_t post_api_route_length =
            strlen(device_config.server_post_route);
        if (post_api_route_length > 0)
        {
            int match_index;
            int match_length = 0;

            match_index      = re_match("[a-zA-Z0-9_\\.\\-\\/]+",
                                   device_config.server_post_route,
                                   sizeof(device_config.server_post_route),
                                   &match_length);

            if ((match_index == RE_NO_MATCH_INDEX) || (match_index != 0) ||
                (static_cast<uint32_t>(match_length) != post_api_route_length))
            {
                // Something is wrong with the URL
                if (fix)
                {
                    // Clear it out
                    memset(device_config.server_post_route,
                           0,
                           MAX_SERVER_POST_ROUTE_LENGTH);
                }
                else
                {
                    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP,
                                      "Post API route %s is not valid.\n",
                                      device_config.server_post_route);

                    return wHalErrorCode::GSM_INVALID_CONFIG;
                }
            }
        }

        // Validate http content type
        if ((device_config.http_content_type !=
             http_content_type_t::MULTIPART_FORM_DATA) &&
            (device_config.http_content_type !=
             http_content_type_t::OCTET_STREAM))
        {
            if (fix)
            {
                device_config.http_content_type =
                    http_content_type_t::OCTET_STREAM;
            }
            else
            {
                LOG_MSG_WITH_ARGS(
                    TraceFile::W_HAL_GSM_CPP,
                    "HTTP content type %d is not valid. It must be "
                    "either multipart/form-data or application/octet-stream.\n",
                    device_config.http_content_type);

                return wHalErrorCode::GSM_INVALID_CONFIG;
            }
        }
    }

    return wHalErrorCode::NO_ERROR;
}

#endif // COMM_CORE != INVALID_CORE_NUM