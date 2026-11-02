#ifndef LIB_CONFIG_CONSTANTS_HPP
#define LIB_CONFIG_CONSTANTS_HPP

#include <climits>
#include <stdint.h>

static const float        PI                       = 3.14159265;

static constexpr uint32_t BITS_PER_DWORD_LOG2      = 5;
static constexpr uint32_t BITS_PER_DWORD           = (1 << BITS_PER_DWORD_LOG2);

static constexpr uint32_t BITS_PER_AUDIO_SAMPLE_8  = 8;  // 8-bit
static constexpr uint32_t BITS_PER_AUDIO_SAMPLE_16 = 16; // 16-bit
static constexpr uint32_t BITS_PER_AUDIO_SAMPLE_24 = 24; // 24-bit
static constexpr uint32_t BITS_PER_AUDIO_SAMPLE_32 = 32; // 32-bit

static constexpr uint32_t SIZE_1KB                 = 1024;
static constexpr uint32_t SIZE_1MB                 = SIZE_1KB * SIZE_1KB;
static constexpr uint32_t SIZE_1GB                 = SIZE_1MB * SIZE_1KB;

static constexpr uint32_t MILLISECONDS_PER_SECOND  = 1000;

static constexpr uint32_t MICROSECONDS_PER_MILLISECOND = 1000;
static constexpr uint32_t MICROSECONDS_PER_SECOND =
    MICROSECONDS_PER_MILLISECOND * MILLISECONDS_PER_SECOND;

static constexpr uint32_t NANOSECONDS_PER_MICROSECOND = 1000;
static constexpr uint32_t NANOSECONDS_PER_MILLISECOND =
    NANOSECONDS_PER_MICROSECOND * MICROSECONDS_PER_MILLISECOND;
static constexpr uint32_t NANOSECONDS_PER_SECOND =
    NANOSECONDS_PER_MICROSECOND * MICROSECONDS_PER_SECOND;

static constexpr uint32_t MONTHS_PER_YEAR        = 12;
static constexpr uint32_t MAX_DAYS_PER_MONTH     = 31;

static constexpr uint32_t TIME_BASE_YEAR         = 1900; // The base offset year
                                                         // to be used with
                                                         // time.h api
static constexpr uint32_t TIME_BASE_MONTH        = 1; // The base offset month
                                                      // to be used with
                                                      // time.h api
static constexpr uint32_t MIN_CONFIGURATION_YEAR = 2025; // Minimum year
                                                         // allowed for
                                                         // configuration
                                                         // time

static constexpr uint32_t SECTOR_SIZE_512        = 512;
static constexpr uint32_t SECTOR_SIZE_4K         = 4096;

static constexpr uint32_t AUDIO_SAMPLE_RATE_8KHz = 8000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_16KHz    = 16000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_22_05KHz = 22050;
static constexpr uint32_t AUDIO_SAMPLE_RATE_24KHz    = 24000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_32KHz    = 32000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_44_1KHz  = 44100;
static constexpr uint32_t AUDIO_SAMPLE_RATE_48KHz    = 48000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_64KHz    = 64000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_96KHz    = 96000;
static constexpr uint32_t AUDIO_SAMPLE_RATE_192KHz   = 192000;

static constexpr uint32_t DEFAULT_AUDIO_SAMPLE_RATE  = AUDIO_SAMPLE_RATE_48KHz;

static constexpr uint8_t  AUDIO_GAIN_DIVIDER = 10; // To get the original value
                                                   // from the stored value

static constexpr uint32_t DEFAULT_SECONDS_PER_AUDIO_FILE = 600;
static constexpr uint32_t DEFAULT_SLEEP_PAUSE_INTERVAL   = 0;

static constexpr uint16_t DEFAULT_NUM_AUDIO_CHANNELS     = 1;

static constexpr int      DEFAULT_MIC_GAIN_TIMES_TEN     = 0;

static constexpr uint32_t MIN_SMS_UPLOAD_FREQUENCY     = 300; // Min SMS upload
                                                              // frequency
static constexpr uint32_t DEFAULT_SMS_UPLOAD_FREQUENCY = 60 * 60; // Default SMS
                                                                  // upload
                                                                  // frequency

static constexpr uint32_t A2P_MESSAGE_COUNT_TO = 60; // Idle time to wait
                                                     // to receive all
                                                     // the messages

// Size of audio sample to store in buffer
static constexpr uint32_t DEFAULT_BITS_PER_AUDIO_SAMPLE =
    BITS_PER_AUDIO_SAMPLE_16;

static constexpr uint32_t MIN_SHORT_CODE_PHONE_NUMBER_LENGTH = 5;  // Min short
                                                                   // code
                                                                   // length
static constexpr uint32_t MAX_SHORT_CODE_PHONE_NUMBER_LENGTH = 6;  // Max short
                                                                   // code
                                                                   // length
static constexpr uint32_t MAX_PHONE_NUMBER_LENGTH            = 16; // Max phone
                                                                   // number
                                                                   // length

static constexpr uint32_t MAX_SMS_LENGTH                = 160; // Max SMS length
static constexpr uint32_t MAX_SERVER_URL_LENGTH         = 64;  // Max server URL
                                                               // length
static constexpr uint32_t MAX_SERVER_POST_ROUTE_LENGTH  = 64; // Max server post
                                                              // route length

static constexpr uint32_t MAX_SMS_KEYWORD_LENGTH        = 16; // Max SMS keyword
                                                              // length

static constexpr uint32_t MAX_SERIAL_NUMBER_LENGTH      = 3; // Max serial
                                                             // number length

static constexpr uint32_t MAX_DEVICE_NAME_LENGTH        = 6; // Max device name
                                                             // length

static constexpr uint32_t MAX_PASSWORD_LENGTH           = 64; // Max password
                                                              // length

static constexpr uint32_t MAX_SMS_KEY_LENGTH            = 64; // Max device
                                                              // key length

static constexpr uint32_t MAX_FILE_SYSTEM_FORMAT_LENGTH = 10; // MAX sd card
                                                              // file format
                                                              // length

static constexpr uint32_t MAX_SCHEDULE_TIME_OF_DAY_RANGES = 10; // Max number
                                                                // of time of
                                                                // day ranges

static constexpr uint32_t DEFAULT_LOW_BATTERY_VOLTAGE = 2700; // Voltage in mV
static constexpr uint32_t LOW_BATTERY_RESTORE_VOLTAGE = 2900; // Voltage in mV

static constexpr uint32_t DEFAULT_NUMBER_OF_BATTERY_CELLS = 5; // number of
                                                               // battery cells

static constexpr int32_t  INVALID_LOCATION_COORDINATE = INT32_MIN; // Invalid
                                                                   // location
                                                                   // marker
static constexpr int32_t  LOCATION_COORDINATE_SCALE   = 10000000;  // Degrees *
                                                                   // 1e7
                                                                   // scale
static constexpr uint32_t SECONDS_PER_DAY             = 86400;     // Number
                                                                   // of seconds
                                                                   // in a day

static constexpr uint32_t BLE_PAIR_WINDOW_DEFAULT_SECONDS =
    30; // Default BLE
        // pairing window
        // duration in
        // seconds

#endif // LIB_CONFIG_CONSTANTS_HPP
