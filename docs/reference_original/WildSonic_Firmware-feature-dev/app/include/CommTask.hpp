#ifndef _COMM_TASK_HPP
#define _COMM_TASK_HPP

#include "FifoQueue.hpp"
#include "Messages.hpp"
#include "Task.hpp"
#include "WAVFile.hpp"
#include "bson_encode.hpp"
#include "w_hal_gsm.hpp"

class CommTask :
    public Task
{
  public:
    /**
     * @brief Data sections that will be part of binary to send
     */
    enum class data_section_t : uint8_t
    {
        SERIAL_NUMBER,
        SMS_KEY,
        SMS_UPLOAD_FREQUENCY_SECONDS,
        GIT_SHA,
        FW_VERSION,
        RECORD_TIME,
        CREATED_AT,
        DEVICE_ID,
        DEVICE_UNIQUE_ID,
        MIC_GAIN,
        SAMPLE_WIDTH,
        NUM_CHANNELS,
        SAMPLING_RATE,
        CUTOFF_FREQUENCY,
        RECORDING_LENGTH,
        SLEEP_PAUSE_INTERVAL,
        START_END_DATE,
        START_END_TIME_OF_DAY_0,
        START_END_TIME_OF_DAY_1,
        START_END_TIME_OF_DAY_2,
        START_END_TIME_OF_DAY_3,
        START_END_TIME_OF_DAY_4,
        START_END_TIME_OF_DAY_5,
        START_END_TIME_OF_DAY_6,
        START_END_TIME_OF_DAY_7,
        START_END_TIME_OF_DAY_8,
        START_END_TIME_OF_DAY_9,
        DAYS_OF_WEEK,
        DETECTOR_SELECT,
        AI_SENSITIVITY,
        SAVE_ONLY_DETECTED_SOUNDS,
        TRANSMIT_ONLY_DETECTED_SOUNDS,
        FOLDER_STRUCTURE,
        GSM_TRANSMISSION_MODE,
        GSM_MODEM_NAME,
        GSM_FIRMWARE_VERSION,
        GSM_IMEI_NUMBER,
        GSM_IMSI,
        GSM_ICCID,
        GSM_OPERATOR_NAME,
        GSM_GPRS_ATTACH,
        GSM_POWER_STATUS,
        GSM_SIM_STATUS,
        GSM_RSSI,
        GSM_RXQUAL,
        GSM_RSCP,
        GSM_ECNO,
        GSM_RSRP,
        GSM_RSRQ,
        GSM_BOOT_TIME,
        GSM_CARRIER_TIME,
        GSM_PDP_ADDRESS,
        GSM_LOCATION_AREA_CODE,
        GSM_CELL_ID,
        PHONE_NUMBER,
        STORAGE_TOTAL_BYTES_WRITTEN,
        STORAGE_TOTAL_CAPACITY,
        BATTERY_VOLTAGE,
        BATTERY_CURRENT,
        BATTERY_REMAINING_CAPACITY,
        BATTERY_FULL_CHARGE_CAPACITY,
        BATTERY_AVERAGE_CURRENT,
        BATTERY_TIME_TO_EMPTY,
        BATTERY_AVERAGE_POWER,
        BATTERY_TEMPERATURE,
        NUMBER_OF_BATTERY_CELLS,
        SD_CARD_FILE_SYSTEM_FORMAT,
        SECONDS_RECORDED,
        DUMMY_ALIGN,
        WAV,
        NUM_DATA_SECTIONS
    };

    /**
     * @brief Flags for each data section
     */
    union section_flags_st
    {
        struct
        {
            uint8_t data_status       : 1; // When time to send
                                           // status over 3G/4G/LTE,
                                           // send this section.
            uint8_t every_sms_status  : 1; // When time to send
                                           // status over SMS,
                                           // send this section.
            uint8_t every_sms_message : 1; // Status is broken
                                           // into multiple SMS
                                           // messages. Section is
                                           // required to be sent
                                           // in every SMS
                                           // message.
            uint8_t first_sms_status  : 1; // Section is required
                                           // to be sent in first
                                           // SMS status. Superseded
                                           // by every_sms_status.
            uint8_t reserved          : 4; // Reserved
        } bits;
        uint8_t flags;
    };

    static const uint32_t NUM_DATA_SECTIONS =
        static_cast<uint32_t>(data_section_t::NUM_DATA_SECTIONS);

    static const char *SECTION_NAMES[NUM_DATA_SECTIONS]; // Name of each section
    static const section_flags_st SECTION_FLAGS[NUM_DATA_SECTIONS]; // Flag to
                                                                    // include
                                                                    // section
                                                                    // in SMS

    // Default constructor
    CommTask() = default;

    // Set up and initialization for Comm Task
    void          initialize(queues_info_t &queues_info);

    // Main routine for Comm Task
    TaskErrorCode run() override;

  private:
    static const uint32_t FILE_NAME_MAX_LENGTH = 40;

    static const uint32_t SECTION_NAMES_SIZE   = 64; // Size of all
                                                     // section name
                                                     // strings

    static const uint32_t BSON_BUFFER_SIZE     = 1536;
    static const uint32_t SMS_BUFFER_SIZE      = BSON_BUFFER_SIZE; // Using same
                                                                   // buffer
                                                                   // for SMS
                                                                   // and BSON

    static const uint32_t DUMMY_ALIGN_SIZE     = 4;                // Size of
                                                                   // dummy
                                                                   // alignment
    static const uint32_t DUMMY_ALIGN_MASK = DUMMY_ALIGN_SIZE - 1; // Mask to
                                                                   // align to
                                                                   // 4-byte
                                                                   // boundary

    static const char     FILE_FORMAT_STRING[]; // File name
                                                // format string

    static const uint8_t  DEVICE_ID_SIZE                   = 16; // Size of
                                                                 // the device
                                                                 // id
    static const uint8_t  DEVICE_UNIQUE_ID_SIZE            = 30; // Size of the
                                                                 // device
                                                                 // unique id

    static const uint32_t IDLE_STATE_CHECK_TIMEOUT_SECONDS = 5; // Timeout for
                                                                // checking
                                                                // for SMS
                                                                // messages
    static const uint32_t SMS_EMPTY_STATE_CHECK_TIMEOUT_SECONDS = 5; // Timeout
                                                                     // to
                                                                     // check
                                                                     // empty
                                                                     // SMS
    static const uint32_t EMPTY_SMS_LIMIT_BEFORE_SLEEP     = 2; // Max limit
                                                                // to check for
                                                                // sms before
                                                                // going to
                                                                // sleep
    static const uint32_t SMS_SEND_STATUS_TIMEOUT_SECONDS  = 15 * 60; // Timeout
                                                                      // for
                                                                      // sending
                                                                      // status
                                                                      // over
                                                                      // SMS

    static const uint32_t MAX_WAV_SECONDS_PER_POST_REQUEST = 10; // Max number
                                                                 // of seconds
                                                                 // that can
                                                                 // be sent
                                                                 // per post
                                                                 // request.

    static const uint32_t MAX_WAV_DATA_PER_POST_REQUEST = 700000; // Max number
                                                                  // of bytes
                                                                  // that can
                                                                  // be sent
                                                                  // per post
                                                                  // request

    static const uint32_t CUTOFF_FREQ_RANGE             = 15; // Size of cutoff
                                                              // frequency range

    static const uint32_t START_END_DATE_STRING_SIZE  = 22; // Size of start/end
                                                            // date string
    static const uint32_t START_END_TIME_STRING_SIZE  = 18; // Size of start/end
                                                            // time string
    static const uint32_t DAY_OF_WEEK_STRING_SIZE     = 8;  // Size of day of
                                                            // week mask string
                                                            // e.g "MTWRFSU"
    static const uint32_t DETECTOR_SELECT_STRING_SIZE = 30; // Size of detector
                                                            // select string
    static const uint32_t FOLDER_STRUCTURE_STRING_SIZE  = 10; // Size of folder
                                                              // structure
                                                              // string
    static const uint32_t TRANSMISSION_MODE_STRING_SIZE = 10; // Size of
                                                              // transmission
                                                              // mode string
    static const uint32_t MAX_RECENT_SECONDS_SIZE       = 12; // Size of
                                                              // recent seconds
                                                              // string

    /**
     * @brief State of execution in Comm Task
     */
    enum class State : uint8_t
    {
        FIRST,
        WAIT_FOR_COMM_REQUEST = FIRST,          // Initial State -
                                                // waiting for request
        OPEN_POST_REQUEST,                      // Open Post request
                                                // if not already open
        PREPARE_DATA_BUFFER,                    // Populate local data buffer
                                                // with items to transfer
                                                // (except WAV file data)
        TRANSFER_DATA_BUFFER,                   // Make upload to HAL
        WAIT_FOR_TRANSFER_DATA_BUFFER_RESPONSE, // Wait for data buffer
                                                // transfer response
        TRANSFER_WAV_DATA,                      // Make upload to HAL
        WAITING_FOR_TRANSFER_WAV_DATA_RESPONSE, // Wait for upload response
        PREPARE_END_OF_DATA,                    // Populate local data buffer
                                                // with end of data
                                                // information
        CLOSE_POST_REQUEST,                     // Close post request if
                                                // last buffer in
                                                // request series
        PREPARE_SMS_SOURCE_DATA,                // Prepare SMS source data
        OPEN_SMS_REQUEST,                       // Call HAL to open SMS
                                                // request
        PREPARE_SMS_BUFFER,                     // Populate SMS buffer
        TRANSFER_SMS_BUFFER,                    // Call HAL to transfer SMS
                                                // buffer
        WAIT_FOR_SMS_TRANSFER_RESPONSE,         // Wait for SMS transfer
                                                // response
        CLOSE_SMS_REQUEST,                      // Call HAL to close SMS
                                                // request
        WAIT_FOR_SMS_CLOSE_RESPONSE,            // Wait for SMS close
                                                // response
        CHECK_FOR_RECEIVE_SMS,                  // Check for receive SMS
                                                // messages
        WAIT_FOR_RECEIVE_SMS,                   // Wait for receive SMS
                                                // messages
    };

    /**
     * @brief State of execution in Comm Task
     */
    enum class sms_state_t : uint8_t
    {
        FIRST,
        CHECK_TIMEOUT = FIRST,
        OPEN_SMS_REQUEST,
        PREPARE_SMS_BUFFER,
        TRANSFER_SMS_BUFFER,
        WAIT_FOR_SMS_TRANSFER_RESPONSE,
        CLOSE_SMS_REQUEST,
        WAIT_FOR_SMS_CLOSE_RESPONSE,
    };

    /**
     * @brief Structure for all GSM information
     */
    struct gsm_info_st
    {
        uint8_t  gprs_status;  // Variable to get GPRS status
        uint8_t  sim_status;   // Variable to get SIM status
        uint32_t power_status; // Variable to get GSM power status

        // Variables to get GSM signal quality
        int32_t  rssi;
        int32_t  rxqual;
        int32_t  rscp;
        int32_t  ecno;
        int32_t  rsrp;
        int32_t  rsrq;
        // Variables to get date and time
        time_t   boot_time_ms;
        time_t   carrier_time_ms;
    };

    /**
     * @brief Structure for all GSM information
     */
    struct device_config_params_st
    {
        double   mic_gain;                          // Mic gain
        uint32_t sample_width;                      // Sample width
        uint32_t num_audio_channels;                // Number of audio channels
        uint32_t sampling_rate;                     // Sampling rate
        uint32_t recording_length;                  // Recording length
        uint32_t sleep_pause_interval;              // Sleep pause interval
        uint32_t sms_send_upload_frequency_seconds; // SMS send/upload
                                                    // frequency
                                                    // in seconds
        char     cutoff_freq_range[CUTOFF_FREQ_RANGE]; // Cutoff
                                                       // frequency range
        char
            start_end_date_string[START_END_DATE_STRING_SIZE]; // Start/end date
        char
            start_end_time_string[MAX_SCHEDULE_TIME_OF_DAY_RANGES]
                                 [START_END_TIME_STRING_SIZE]; // Start/end time
        char day_of_week_string[DAY_OF_WEEK_STRING_SIZE];      // Day of week
                                                               // mask
        char detector_select_string[DETECTOR_SELECT_STRING_SIZE];   // Detector
                                                                    // select
                                                                    // string
        char folder_structure_string[FOLDER_STRUCTURE_STRING_SIZE]; // folder
                                                                    // structure
        char transmission_mode_string
            [TRANSMISSION_MODE_STRING_SIZE];    // Transmission
                                                // mode
                                                // string
        uint32_t ai_sensitivity;                // AI sensitivity
        bool     save_only_detected_sounds;     // Save only detected sounds
        bool     transmit_only_detected_sounds; // Transmit only detected sounds
    };

    // Parse and process received SMS
    void               parse_received_sms(char *p_buffer, uint32_t length);

    // Indicates whether comm requests should be aborted
    bool               should_abort_comm_requests();

    // Handle aborting of comm requests if applicable
    bool               abort_current_comm_request();

    // Handle aborting of comm requests if applicable
    bool               handle_abort_comm_requests();

    // Handle high priority queue requests
    bool               handle_hp_comm_requests();

    // Compute the bson document size based on the data sections
    uint32_t           compute_bson_document_size();

    // Populate the data footer for transfer
    void               populate_end_of_document_field();

    // Collect section data from gsm
    void               prepare_gsm_status_info();

    // Prepare source data to be transferred over SMS
    void               prepare_sms_source_data();

    // Update seconds recorded since last status
    void               update_seconds_recorded();

    // Populate the sms field in buffer for transfer
    bool               fill_sms_field_in_buffer(char *              p_buffer,
                                                uint32_t            buffer_size,
                                                const char *        p_section_name,
                                                const char *        p_section_pointer,
                                                bson_encode::type_t data_type,
                                                uint32_t &          filled);

    // Prepare SMS buffer for transfer
    void               prepare_sms_buffer();

    // Prepare data header for transfer
    uint32_t           prepare_data_header();

    // Prepare a data buffer for transfer
    void               prepare_data_buffer();

    State              m_state;                 // State of execution
                                                // in Comm Task
    wGsmTransferHandle m_handle;                // Transfer handle
    comm_request_id_t  m_request_id;            // Request tracker
    uint32_t           m_last_seconds_recorded; // Seconds recorded
                                                // up till the last
                                                // data/sms sent
    char m_recent_seconds_recorded[MAX_RECENT_SECONDS_SIZE]; // Current seconds
                                                             // recorded since
                                                             // the last
                                                             // data/sms sent
    uint32_t          m_empty_sms_count_before_sleep;        // Count to check
                                             // for empty messages
    w_hal_rtc_time_st m_last_check_receive_sms_time;   // Last time check
                                                       // for SMS  messages
                                                       // was performed
    w_hal_rtc_time_st m_last_send_status_time;         // Last time status
                                                       // was sent
    uint64_t          m_comm_request_time_ms;          // Time of comm
                                                       // request
    uint64_t          m_comm_request_transfer_time_ms; // Time of comm request
                                                       // transfer
    wav_header_t      m_wav_header;                    // Header for WAV file
    uint32_t          m_max_wav_data_size_in_post_request; // Max wav data that
                                                           // can be sent in one
                                                           // post request
    uint16_t          m_remaining_audio_buffers_to_write;  // Remaining audio
                                                           // buffers to
                                                           // transfer
    gsm_info_st       m_gsm_info;                          // Contains all GSM
                                                           // status info
    uint32_t          m_section_index;                     // Indicates current
                                                           // section that is
                                                           // being transferred

    const uint8_t *   m_section_pointers[NUM_DATA_SECTIONS]; // Section data
                                                             // pointers
    uint32_t          m_section_sizes[NUM_DATA_SECTIONS];    // Sizes of each
                                                             // data section
    uint32_t          m_bson_field_sizes[NUM_DATA_SECTIONS]; // Sizes of each
                                                             // bson field
    bson_encode::type_t m_section_types[NUM_DATA_SECTIONS];  // Types of each
                                                             // data section

    uint8_t             m_bson_buffer[BSON_BUFFER_SIZE]; // BSON buffer
    bool                m_bson_header_populated;         // Indicates if BSON
                                                         // header has been
                                                         // populated yet
    uint32_t            m_bson_buffer_bytes_populated;   // Indicates how many
                                                         // bytes have been
                                                         // populated in BSON
                                                         // buffer
    uint32_t            m_bson_document_size;            // Size of BSON
                                                         // document to send

    bool                m_abort_requests; // Flag to abort
                                          // requests
    bool                m_first_sms_sent; // Flag to indicate
                                          // first SMS
                                          // message has been sent

    char                m_device_id[DEVICE_ID_SIZE];    // Device ID
    char     m_device_unique_id[DEVICE_UNIQUE_ID_SIZE]; // Device unique
                                                        // serial number

    bool     m_a2p_phone_number_exists; // Server A2P phone
                                        // number exists
    bool     m_p2a_phone_number_exists; // Server P2A phone
                                        // number exists
    bool     m_server_api_route_exists; // Server API route
                                        // exists
    uint32_t m_sms_chars_remaining;     // Number of characters
                                        // remaining in SMS
                                        // request

    uint8_t  m_number_of_msgs_received; // Number of
                                        // messages received
    uint8_t  m_number_of_msgs_total;    // Total number of message sent by the
                                        // application

    device_config_params_st m_device_config; // Parameters from
                                             // device config

    char m_serial_number[MAX_SERIAL_NUMBER_LENGTH]; // Serial number string
    char m_sms_keyword[MAX_SMS_KEYWORD_LENGTH];     // SMS keyword string
    char m_sms_key[MAX_SMS_KEY_LENGTH];             // Device key string
    char m_file_name[FILE_NAME_MAX_LENGTH];         // File name string

    char m_server_api_route[MAX_SERVER_POST_ROUTE_LENGTH]; // Server post
                                                           // route string
    http_content_type_t m_http_content_type;               // HTTP content
                                                           // type

    uint8_t             m_dummy_align[DUMMY_ALIGN_SIZE]; // Dummy alignment
                                                         // to 4 byte boundary

    CommRequestMsg *    m_p_comm_msgs;    // Array of comm
                                          // request messages
    CommHpRequestMsg *  m_p_comm_hp_msgs; // Array of comm hp
                                          // request messages

    CommFifoQueue *     m_p_comm_request_queue;      // request queue from
                                                     // audio task
    CommFifoQueue *     m_p_comm_response_queue;     // response queue to
                                                     // audio task
    CommHpFifoQueue *   m_p_comm_hp_request_queue;   // request queue from
                                                     // audio task
    CommHpFifoQueue *   m_p_comm_hp_response_queue;  // response queue to
                                                     // audio task
    time_t              m_last_gsm_info_logged_time; // Last time modem was
                                                     // queried for gsm info
};

#endif // _COMM_TASK_HPP