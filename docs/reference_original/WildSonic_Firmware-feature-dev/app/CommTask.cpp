#if __CORE__ == COMM_CORE

#include "CommTask.hpp"
#include "assert.hpp"
#include "bson_encode.hpp"
#include "console.hpp"
#include "constants.hpp"
#include "re.h"
#include "shared.hpp"
#include "trace.hpp"
#include "version.hpp"

#include "w_hal_gsm.hpp"
#include "w_hal_rtc.hpp"

#include <inttypes.h>
#include <string.h>

const char CommTask::FILE_FORMAT_STRING[] =
    "%04d%02d%02d_%02d%02d%02d_%03d.bson";

const char *CommTask::SECTION_NAMES[CommTask::NUM_DATA_SECTIONS] = {
    "sn",    // SERIAL_NUMBER
    "skey",  // SMS_KEY
    "sfq",   // SMS_UPLOAD_FREQUENCY_SECONDS
    "sha",   // GIT_SHA
    "ver",   // FW_VERSION
    "r_tm",  // RECORD_TIME
    "c_at",  // CREATED_AT
    "id",    // DEVICE_ID
    "uid",   // DEVICE_UNIQUE_ID
    "gain",  // MIC_GAIN
    "sw",    // SAMPLE_WIDTH
    "nch",   // NUM_CHANNELS
    "sr",    // SAMPLING_RATE
    "filt",  // CUTOFF_FREQUENCY
    "rect",  // RECORDING_LENGTH
    "slpi",  // SLEEP_PAUSE_INTERVAL
    "rdt",   // START_END_DATE
    "rtm0",  // START_END_TIME_OF_DAY_0
    "rtm1",  // START_END_TIME_OF_DAY_1
    "rtm2",  // START_END_TIME_OF_DAY_2
    "rtm3",  // START_END_TIME_OF_DAY_3
    "rtm4",  // START_END_TIME_OF_DAY_4
    "rtm5",  // START_END_TIME_OF_DAY_5
    "rtm6",  // START_END_TIME_OF_DAY_6
    "rtm7",  // START_END_TIME_OF_DAY_7
    "rtm8",  // START_END_TIME_OF_DAY_8
    "rtm9",  // START_END_TIME_OF_DAY_9
    "day",   // DAYS_OF_WEEK
    "dsel",  // DETECTOR_SELECT
    "sen",   // AI_SENSITIVITY
    "sdet",  // SAVE_ONLY_DETECTED_SOUNDS
    "xdet",  // TRANSMIT_ONLY_DETECTED_SOUNDS
    "fldr",  // FOLDER_STRUCTURE
    "gm",    // GSM_TRANSMISSION_MODE
    "gmod",  // GSM_MODEM_NAME
    "gfw",   // GSM_FIRMWARE_VERSION
    "imei",  // GSM_IMEI_NUMBER
    "imsi",  // GSM_IMSI
    "iccid", // GSM_ICCID
    "gop",   // GSM_OPERATOR_NAME
    "gprs",  // GSM_GPRS_ATTACH
    "psts",  // GSM_POWER_STATUS
    "ssts",  // GSM_SIM_STATUS
    "rssi",  // GSM_RSSI
    "rxq",   // GSM_RXQUAL
    "rscp",  // GSM_RSCP
    "ecno",  // GSM_ECNO
    "rsrp",  // GSM_RSRP
    "rsrq",  // GSM_RSRQ
    "bttm",  // GSM_BOOT_TIME
    "crtm",  // GSM_CARRIER_TIME
    "pdp",   // GSM_PDP_ADDRESS
    "lac",   // GSM_LOCATION_AREA_CODE
    "cid",   // GSM_CELL_ID
    "ph",    // PHONE_NUMBER
    "stbw",  // STORAGE_TOTAL_BYTES_WRITTEN
    "stot",  // STORAGE_TOTAL_CAPACITY
    "bv",    // BATTERY_VOLTAGE
    "bcur",  // BATTERY_CURRENT
    "brc",   // BATTERY_REMAINING_CAPACITY
    "bfcc",  // BATTERY_FULL_CHARGE_CAPACITY
    "bac",   // BATTERY_AVERAGE_CURRENT
    "bt2e",  // BATTERY_TIME_TO_EMPTY
    "bap",   // BATTERY_AVERAGE_POWER
    "btem",  // BATTERY_TEMPERATURE
    "bnum",  // NUMBER_OF_BATTERY_CELLS
    "sdfs",  // SD_CARD_FILE_SYSTEM_FORMAT
    "secr",  // SECONDS_RECORDED
    "dummy", // DUMMY_ALIGN
    "audio", // WAV
};

static_assert(sizeof(CommTask::SECTION_NAMES) /
                      sizeof(CommTask::SECTION_NAMES[0]) ==
                  CommTask::NUM_DATA_SECTIONS,
              "SECTION_NAMES size mismatch");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Right now, everything is sent over data. Some sections are sent over SMS.
// SMS is limited to 160 characters, so we break up the sections into multiple
// SMS messages. Every SMS message includes the device name and created_at.
const CommTask::section_flags_st
    CommTask::SECTION_FLAGS[CommTask::NUM_DATA_SECTIONS] = {
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SERIAL_NUMBER
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 1,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // SMS_KEY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SMS_UPLOAD_FREQUENCY_SECONDS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // GIT_SHA
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // FW_VERSION
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // RECORD_TIME
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // CREATED_AT
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // DEVICE_ID
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // DEVICE_UNIQUE_ID
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // MIC_GAIN
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SAMPLE_WIDTH
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // NUM_CHANNELS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SAMPLING_RATE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // CUTOFF_FREQUENCY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // RECORDING_LENGTH
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SLEEP_PAUSE_INTERVAL
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_DATE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_0
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_1
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_2
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_3
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_4
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_5
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_6
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_7
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_8
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // START_END_TIME_OF_DAY_9
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // DAYS_OF_WEEK
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // DETECTOR_SELECT
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // AI_SENSITIVITY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SAVE_ONLY_DETECTED_SOUNDS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // TRANSMIT_ONLY_DETECTED_SOUNDS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // FOLDER_STRUCTURE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // GSM_TRANSMISSION_MODE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_MODEM_NAME
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_FIRMWARE_VERSION
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_IMEI_NUMBER
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_IMSI
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_ICCID
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // GSM_OPERATOR_NAME
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_GPRS_ATTACH
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_POWER_STATUS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // GSM_SIM_STATUS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_RSSI
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_RXQUAL
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_RSCP
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_ECNO
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_RSRP
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_RSRQ
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // GSM_BOOT_TIME
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_CARRIER_TIME
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_PDP_ADDRESS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_LOCATION_AREA_CODE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // GSM_CELL_ID
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // PHONE_NUMBER
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // STORAGE_TOTAL_BYTES_WRITTEN
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // STORAGE_TOTAL_CAPACITY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_VOLTAGE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_CURRENT
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_REMAINING_CAPACITY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_FULL_CHARGE_CAPACITY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_AVERAGE_CURRENT
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_TIME_TO_EMPTY
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_AVERAGE_POWER
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // BATTERY_TEMPERATURE
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // NUMBER_OF_BATTERY_CELLS
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 1,
                  .reserved          = 0}}, // SD_CARD_FILE_SYSTEM_FORMAT
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 1,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // SECONDS_RECORDED
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // DUMMY_ALIGN
        {.bits = {.data_status       = 1,
                  .every_sms_status  = 0,
                  .every_sms_message = 0,
                  .first_sms_status  = 0,
                  .reserved          = 0}}, // WAV
};
#pragma GCC diagnostic pop

static_assert(sizeof(CommTask::SECTION_FLAGS) /
                      sizeof(CommTask::SECTION_FLAGS[0]) ==
                  CommTask::NUM_DATA_SECTIONS,
              "SECTION_FLAGS size mismatch");

/**
 * @brief
 * Set up and initialization for Comm Task
 *
 * @param queues_info queue information
 */
void CommTask::initialize(queues_info_t &queues_info)
{
    Task::initialize();

    m_state                         = State::FIRST;
    m_handle                        = INVALID_GSM_HANDLE;
    m_request_id                    = INVALID_COMM_MSG_ID;
    m_empty_sms_count_before_sleep  = 0;
    m_comm_request_time_ms          = 0;
    m_comm_request_transfer_time_ms = 0;
    m_last_seconds_recorded         = 0;
    m_a2p_phone_number_exists =
        (DeviceConfig::p_page->a2p_phone_number[0] != '\0');
    m_p2a_phone_number_exists =
        (DeviceConfig::p_page->p2a_phone_number[0] != '\0');
    m_sms_chars_remaining     = 0;
    m_number_of_msgs_received = 0;
    m_number_of_msgs_total    = 0;

    // Check immediately for SMS
    m_last_check_receive_sms_time.clear();

    shared_glbl.send_status_now = false;

    if ((DeviceConfig::p_page->transmission_mode ==
         DeviceConfig::transmission_mode_t::MODE_2G) ||
        (DeviceConfig::p_page->server_url[0] == '\0') ||
        (DeviceConfig::p_page->server_post_route[0] == '\0'))
    {
        m_server_api_route_exists = false;

        // Send status over SMS immediately
        m_last_send_status_time.clear();
    }
    else
    {
        m_server_api_route_exists = true;

        strncpy(m_server_api_route,
                DeviceConfig::p_page->server_post_route,
                sizeof(m_server_api_route));
        m_http_content_type = DeviceConfig::p_page->http_content_type;

        // Try sending status over data first, then if that times out,
        // send over SMS. Give 10 seconds for status to be sent over
        // data before sending over SMS.
        w_hal_rtc_get_time(m_last_send_status_time);
        const uint32_t first_sms_timeout_seconds = 10;
        if ((DeviceConfig::p_page->sms_send_upload_frequency_seconds >
             first_sms_timeout_seconds) &&
            (m_last_send_status_time.time_seconds >
             DeviceConfig::p_page->sms_send_upload_frequency_seconds -
                 first_sms_timeout_seconds))
        {
            m_last_send_status_time.time_seconds -=
                DeviceConfig::p_page->sms_send_upload_frequency_seconds -
                first_sms_timeout_seconds;
        }
    }

    // Save pointer to message array
    m_p_comm_msgs              = queues_info.comm_msgs;
    m_p_comm_hp_msgs           = queues_info.comm_hp_msgs;

    // Save queue pointers
    m_p_comm_request_queue     = &queues_info.comm_request_queue;
    m_p_comm_response_queue    = &queues_info.comm_response_queue;
    m_p_comm_hp_request_queue  = &queues_info.comm_hp_request_queue;
    m_p_comm_hp_response_queue = &queues_info.comm_hp_response_queue;

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
    m_remaining_audio_buffers_to_write = 0;

    // Getting the value of data size in post request from max data cap
    // and time cap
    m_max_wav_data_size_in_post_request =
        (DeviceConfig::get_data_bit_rate() / CHAR_BIT) *
        MAX_WAV_SECONDS_PER_POST_REQUEST;
    if (m_max_wav_data_size_in_post_request > MAX_WAV_DATA_PER_POST_REQUEST)
    {
        m_max_wav_data_size_in_post_request = MAX_WAV_DATA_PER_POST_REQUEST;
    }

    m_file_name[0] = '\0';

    // Set up serial number
    strncpy(m_serial_number,
            DeviceConfig::p_page->serial_number,
            MAX_SERIAL_NUMBER_LENGTH);

    m_section_pointers[static_cast<uint32_t>(data_section_t::SERIAL_NUMBER)] =
        reinterpret_cast<uint8_t *>(m_serial_number);
    m_section_sizes[static_cast<uint32_t>(data_section_t::SERIAL_NUMBER)] =
        strlen(m_serial_number) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::SERIAL_NUMBER)] =
        bson_encode::type_t::STRING;

    // Set up sms keyword
    strncpy(m_sms_keyword,
            DeviceConfig::p_page->sms_keyword,
            MAX_SMS_KEYWORD_LENGTH);

    // Set up sms key
    strncpy(m_sms_key, DeviceConfig::p_page->sms_key, MAX_SMS_KEY_LENGTH);

    m_section_pointers[static_cast<uint32_t>(data_section_t::SMS_KEY)] =
        reinterpret_cast<uint8_t *>(m_sms_key);
    m_section_sizes[static_cast<uint32_t>(data_section_t::SMS_KEY)] =
        strlen(m_sms_key) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::SMS_KEY)] =
        bson_encode::type_t::STRING;

    // Set up sms upload frequency
    m_device_config.sms_send_upload_frequency_seconds =
        DeviceConfig::p_page->sms_send_upload_frequency_seconds;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::SMS_UPLOAD_FREQUENCY_SECONDS)] =
        reinterpret_cast<uint8_t *>(
            &m_device_config.sms_send_upload_frequency_seconds);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::SMS_UPLOAD_FREQUENCY_SECONDS)] =
        sizeof(m_device_config.sms_send_upload_frequency_seconds);
    m_section_types[static_cast<uint32_t>(
        data_section_t::SMS_UPLOAD_FREQUENCY_SECONDS)] =
        bson_encode::type_t::INT32;

    // Set up git sha
    m_section_pointers[static_cast<uint32_t>(data_section_t::GIT_SHA)] =
        const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(git_sha));
    m_section_sizes[static_cast<uint32_t>(data_section_t::GIT_SHA)] =
        strlen(git_sha) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::GIT_SHA)] =
        bson_encode::type_t::STRING;

    // Set up firmware version
    m_section_pointers[static_cast<uint32_t>(data_section_t::FW_VERSION)] =
        const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(fw_version));
    m_section_sizes[static_cast<uint32_t>(data_section_t::FW_VERSION)] =
        strlen(fw_version) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::FW_VERSION)] =
        bson_encode::type_t::STRING;

    const char **p_section_pointer; // section pointer to get gsm string
                                    // parameter information
    uint32_t *   p_size; // section pointer to get size of gsm information

    w_hal_system_get_device_info(m_device_id,
                                 sizeof(m_device_id) / sizeof(m_device_id[0]),
                                 m_device_unique_id,
                                 sizeof(m_device_unique_id) /
                                     sizeof(m_device_unique_id[0]));
    // Section pointers for device ID
    m_section_pointers[static_cast<uint32_t>(data_section_t::DEVICE_ID)] =
        reinterpret_cast<uint8_t *>(m_device_id);
    m_section_sizes[static_cast<uint32_t>(data_section_t::DEVICE_ID)] =
        strlen(m_device_id) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::DEVICE_ID)] =
        bson_encode::type_t::STRING;

    // Section pointers for device unique id
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::DEVICE_UNIQUE_ID)] =
        reinterpret_cast<uint8_t *>(m_device_unique_id);
    m_section_sizes[static_cast<uint32_t>(data_section_t::DEVICE_UNIQUE_ID)] =
        strlen(m_device_unique_id) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::DEVICE_UNIQUE_ID)] =
        bson_encode::type_t::STRING;

    // Section pointers for mic gain
    m_device_config.mic_gain =
        static_cast<double>(0.1F * (DeviceConfig::p_page->mic_gain_times_ten));
    m_section_pointers[static_cast<uint32_t>(data_section_t::MIC_GAIN)] =
        reinterpret_cast<uint8_t *>(&m_device_config.mic_gain);
    m_section_sizes[static_cast<uint32_t>(data_section_t::MIC_GAIN)] =
        sizeof(m_device_config.mic_gain);
    m_section_types[static_cast<uint32_t>(data_section_t::MIC_GAIN)] =
        bson_encode::type_t::DOUBLE;

    // Section pointers for sampling width
    m_device_config.sample_width = DeviceConfig::p_page->audio_sampling_width;
    m_section_pointers[static_cast<uint32_t>(data_section_t::SAMPLE_WIDTH)] =
        reinterpret_cast<uint8_t *>(&m_device_config.sample_width);
    m_section_sizes[static_cast<uint32_t>(data_section_t::SAMPLE_WIDTH)] =
        sizeof(m_device_config.sample_width);
    m_section_types[static_cast<uint32_t>(data_section_t::SAMPLE_WIDTH)] =
        bson_encode::type_t::INT32;

    // Section pointers for number of audio channels
    m_device_config.num_audio_channels =
        DeviceConfig::p_page->num_audio_channels;
    m_section_pointers[static_cast<uint32_t>(data_section_t::NUM_CHANNELS)] =
        reinterpret_cast<uint8_t *>(&m_device_config.num_audio_channels);
    m_section_sizes[static_cast<uint32_t>(data_section_t::NUM_CHANNELS)] =
        sizeof(m_device_config.num_audio_channels);
    m_section_types[static_cast<uint32_t>(data_section_t::NUM_CHANNELS)] =
        bson_encode::type_t::INT32;

    // Section pointers for sampling rate
    m_device_config.sampling_rate = DeviceConfig::p_page->sampling_rate;
    m_section_pointers[static_cast<uint32_t>(data_section_t::SAMPLING_RATE)] =
        reinterpret_cast<uint8_t *>(&m_device_config.sampling_rate);
    m_section_sizes[static_cast<uint32_t>(data_section_t::SAMPLING_RATE)] =
        sizeof(m_device_config.sampling_rate);
    m_section_types[static_cast<uint32_t>(data_section_t::SAMPLING_RATE)] =
        bson_encode::type_t::INT32;

    // Section pointers for cutoff frequencies
    int ret_val = snprintf(m_device_config.cutoff_freq_range,
                           sizeof(m_device_config.cutoff_freq_range),
                           "%d-%d",
                           DeviceConfig::p_page->lower_cutoff_frequency,
                           DeviceConfig::p_page->upper_cutoff_frequency);

    SYSTEM_ASSERT_GT(ret_val, 0);

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::CUTOFF_FREQUENCY)] =
        reinterpret_cast<uint8_t *>(m_device_config.cutoff_freq_range);
    m_section_sizes[static_cast<uint32_t>(data_section_t::CUTOFF_FREQUENCY)] =
        strlen(m_device_config.cutoff_freq_range) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::CUTOFF_FREQUENCY)] =
        bson_encode::type_t::STRING;

    // Section pointers for recording length
    m_device_config.recording_length = DeviceConfig::p_page->recording_length;
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::RECORDING_LENGTH)] =
        reinterpret_cast<uint8_t *>(&m_device_config.recording_length);
    m_section_sizes[static_cast<uint32_t>(data_section_t::RECORDING_LENGTH)] =
        sizeof(m_device_config.recording_length);
    m_section_types[static_cast<uint32_t>(data_section_t::RECORDING_LENGTH)] =
        bson_encode::type_t::INT32;

    // Section pointers for sleep pause interval
    m_device_config.sleep_pause_interval =
        DeviceConfig::p_page->sleep_pause_interval;
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::SLEEP_PAUSE_INTERVAL)] =
        reinterpret_cast<uint8_t *>(&m_device_config.sleep_pause_interval);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::SLEEP_PAUSE_INTERVAL)] =
        sizeof(m_device_config.sleep_pause_interval);
    m_section_types[static_cast<uint32_t>(
        data_section_t::SLEEP_PAUSE_INTERVAL)] = bson_encode::type_t::INT32;

    // Section pointers for start and end date
    DeviceConfig::get_start_end_date(
        m_device_config.start_end_date_string,
        sizeof(m_device_config.start_end_date_string));
    m_section_pointers[static_cast<uint32_t>(data_section_t::START_END_DATE)] =
        reinterpret_cast<uint8_t *>(m_device_config.start_end_date_string);
    m_section_sizes[static_cast<uint32_t>(data_section_t::START_END_DATE)] =
        strlen(m_device_config.start_end_date_string) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::START_END_DATE)] =
        bson_encode::type_t::STRING;

    // Section pointers for start and end time of day
    for (uint32_t index = 0; index < MAX_SCHEDULE_TIME_OF_DAY_RANGES; index++)
    {
        uint32_t section_index =
            static_cast<uint32_t>(data_section_t::START_END_TIME_OF_DAY_0) +
            index;

        DeviceConfig::get_start_end_time_of_day(
            index,
            m_device_config.start_end_time_string[index],
            sizeof(m_device_config.start_end_time_string[index]));
        m_section_pointers[section_index] = reinterpret_cast<uint8_t *>(
            m_device_config.start_end_time_string[index]);
        m_section_sizes[section_index] =
            strlen(m_device_config.start_end_time_string[index]) + 1;
        m_section_types[section_index] = bson_encode::type_t::STRING;
    }

    // Section pointers for day of week selection
    DeviceConfig::get_days_of_week(m_device_config.day_of_week_string,
                                   sizeof(m_device_config.day_of_week_string));
    m_section_pointers[static_cast<uint32_t>(data_section_t::DAYS_OF_WEEK)] =
        reinterpret_cast<uint8_t *>(m_device_config.day_of_week_string);
    m_section_sizes[static_cast<uint32_t>(data_section_t::DAYS_OF_WEEK)] =
        strlen(m_device_config.day_of_week_string) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::DAYS_OF_WEEK)] =
        bson_encode::type_t::STRING;

    // Section pointers for detector
    DeviceConfig::get_detector(m_device_config.detector_select_string,
                               sizeof(m_device_config.detector_select_string));
    m_section_pointers[static_cast<uint32_t>(data_section_t::DETECTOR_SELECT)] =
        reinterpret_cast<uint8_t *>(m_device_config.detector_select_string);
    m_section_sizes[static_cast<uint32_t>(data_section_t::DETECTOR_SELECT)] =
        strlen(m_device_config.detector_select_string) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::DETECTOR_SELECT)] =
        bson_encode::type_t::STRING;

    // Section pointers for AI sensitivity
    m_device_config.ai_sensitivity = DeviceConfig::p_page->ai_sensitivity;
    m_section_pointers[static_cast<uint32_t>(data_section_t::AI_SENSITIVITY)] =
        reinterpret_cast<uint8_t *>(&m_device_config.ai_sensitivity);
    m_section_sizes[static_cast<uint32_t>(data_section_t::AI_SENSITIVITY)] =
        sizeof(m_device_config.ai_sensitivity);
    m_section_types[static_cast<uint32_t>(data_section_t::AI_SENSITIVITY)] =
        bson_encode::type_t::INT32;

    // Section pointers for save only detected sounds
    m_device_config.save_only_detected_sounds =
        (DeviceConfig::p_page->flags.flags.save_only_detected_sounds == 1);
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::SAVE_ONLY_DETECTED_SOUNDS)] =
        reinterpret_cast<uint8_t *>(&m_device_config.save_only_detected_sounds);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::SAVE_ONLY_DETECTED_SOUNDS)] =
        sizeof(m_device_config.save_only_detected_sounds);
    m_section_types[static_cast<uint32_t>(
        data_section_t::SAVE_ONLY_DETECTED_SOUNDS)] =
        bson_encode::type_t::BOOLEAN;

    // Section pointers for transmit only detected sounds
    m_device_config.transmit_only_detected_sounds =
        (DeviceConfig::p_page->flags.flags.transmit_only_detected_sounds == 1);
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::TRANSMIT_ONLY_DETECTED_SOUNDS)] =
        reinterpret_cast<uint8_t *>(
            &m_device_config.transmit_only_detected_sounds);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::TRANSMIT_ONLY_DETECTED_SOUNDS)] =
        sizeof(m_device_config.transmit_only_detected_sounds);
    m_section_types[static_cast<uint32_t>(
        data_section_t::TRANSMIT_ONLY_DETECTED_SOUNDS)] =
        bson_encode::type_t::BOOLEAN;

    // Section pointers for daily folder structure configuration
    DeviceConfig::get_folder_structure(
        m_device_config.folder_structure_string,
        sizeof(m_device_config.folder_structure_string));
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::FOLDER_STRUCTURE)] =
        reinterpret_cast<uint8_t *>(m_device_config.folder_structure_string);
    m_section_sizes[static_cast<uint32_t>(data_section_t::FOLDER_STRUCTURE)] =
        strlen(m_device_config.folder_structure_string) + 1;
    m_section_types[static_cast<uint32_t>(data_section_t::FOLDER_STRUCTURE)] =
        bson_encode::type_t::STRING;

    // Section pointers for transmission mode
    DeviceConfig::get_transmission_mode(
        m_device_config.transmission_mode_string,
        sizeof(m_device_config.transmission_mode_string));
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::GSM_TRANSMISSION_MODE)] =
        reinterpret_cast<uint8_t *>(&m_device_config.transmission_mode_string);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::GSM_TRANSMISSION_MODE)] =
        strlen(m_device_config.transmission_mode_string) + 1;
    m_section_types[static_cast<uint32_t>(
        data_section_t::GSM_TRANSMISSION_MODE)] = bson_encode::type_t::STRING;

    // Section pointers for modem name
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_MODEM_NAME)]);
    p_size =
        &m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_MODEM_NAME)];
    w_hal_gsm_get_modem_name(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_MODEM_NAME)] =
        bson_encode::type_t::STRING;

    // Section pointers for modem firmware version
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_FIRMWARE_VERSION)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(
        data_section_t::GSM_FIRMWARE_VERSION)];
    w_hal_gsm_get_fw_version(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(
        data_section_t::GSM_FIRMWARE_VERSION)] = bson_encode::type_t::STRING;

    // Section pointers for imei number of gsm
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_IMEI_NUMBER)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(
        data_section_t::GSM_IMEI_NUMBER)];
    w_hal_gsm_get_imei(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_IMEI_NUMBER)] =
        bson_encode::type_t::STRING;

    // Section pointers for IMSI number of gsm
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_IMSI)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_IMSI)];
    w_hal_gsm_get_imsi(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_IMSI)] =
        bson_encode::type_t::STRING;

    // Section pointers for ICCID number of gsm
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_ICCID)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_ICCID)];
    w_hal_gsm_get_iccid(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_ICCID)] =
        bson_encode::type_t::STRING;

    // Section pointers for operator name for gsm
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_OPERATOR_NAME)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(
        data_section_t::GSM_OPERATOR_NAME)];
    w_hal_gsm_get_operator_name(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_OPERATOR_NAME)] =
        bson_encode::type_t::STRING;

    // Section for gprs status
    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_GPRS_ATTACH)] =
        &m_gsm_info.gprs_status;
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_GPRS_ATTACH)] =
        sizeof(m_gsm_info.gprs_status);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_GPRS_ATTACH)] =
        bson_encode::type_t::BOOLEAN;

    // Section for GSM power status
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::GSM_POWER_STATUS)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.power_status);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_POWER_STATUS)] =
        sizeof(m_gsm_info.power_status);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_POWER_STATUS)] =
        bson_encode::type_t::INT32;

    // Section for GSM SIM status
    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_SIM_STATUS)] =
        &m_gsm_info.sim_status;
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_SIM_STATUS)] =
        sizeof(m_gsm_info.sim_status);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_SIM_STATUS)] =
        bson_encode::type_t::BOOLEAN;

    // Section for GSM signal quality
    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_RSSI)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.rssi);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_RSSI)] =
        sizeof(m_gsm_info.rssi);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_RSSI)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_RXQUAL)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.rxqual);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_RXQUAL)] =
        sizeof(m_gsm_info.rxqual);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_RXQUAL)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_RSCP)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.rscp);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_RSCP)] =
        sizeof(m_gsm_info.rscp);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_RSCP)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_ECNO)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.ecno);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_ECNO)] =
        sizeof(m_gsm_info.ecno);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_ECNO)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_RSRP)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.rsrp);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_RSRP)] =
        sizeof(m_gsm_info.rsrp);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_RSRP)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_RSRQ)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.rsrq);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_RSRQ)] =
        sizeof(m_gsm_info.rsrq);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_RSRQ)] =
        bson_encode::type_t::INT32;

    // Setup for boot date and time
    m_gsm_info.boot_time_ms = w_hal_gsm_get_init_time() * MSEC_PER_SEC;
    m_section_pointers[static_cast<uint32_t>(data_section_t::GSM_BOOT_TIME)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.boot_time_ms);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_BOOT_TIME)] =
        sizeof(m_gsm_info.boot_time_ms);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_BOOT_TIME)] =
        bson_encode::type_t::UTC_DATETIME;

    // Setup for carrier date and time
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::GSM_CARRIER_TIME)] =
        reinterpret_cast<uint8_t *>(&m_gsm_info.carrier_time_ms);
    m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_CARRIER_TIME)] =
        sizeof(m_gsm_info.carrier_time_ms);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_CARRIER_TIME)] =
        bson_encode::type_t::UTC_DATETIME;

    // Section pointers for PDP address
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_PDP_ADDRESS)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(
        data_section_t::GSM_PDP_ADDRESS)];
    w_hal_gsm_get_pdp_address(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_PDP_ADDRESS)] =
        bson_encode::type_t::STRING;

    // Section pointers for location area code
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_LOCATION_AREA_CODE)]);
    p_size = &m_section_sizes[static_cast<uint32_t>(
        data_section_t::GSM_LOCATION_AREA_CODE)];
    w_hal_gsm_get_location_area_code(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(
        data_section_t::GSM_LOCATION_AREA_CODE)] = bson_encode::type_t::STRING;

    // Section pointers for gsm cell id
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_CELL_ID)]);
    p_size =
        &m_section_sizes[static_cast<uint32_t>(data_section_t::GSM_CELL_ID)];
    w_hal_gsm_get_cell_id(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::GSM_CELL_ID)] =
        bson_encode::type_t::STRING;

    // Section for phone number
    p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::PHONE_NUMBER)]);
    p_size =
        &m_section_sizes[static_cast<uint32_t>(data_section_t::PHONE_NUMBER)];
    w_hal_gsm_get_phone_number(*p_section_pointer, *p_size);
    m_section_types[static_cast<uint32_t>(data_section_t::PHONE_NUMBER)] =
        bson_encode::type_t::STRING;

    // Section pointer for storage task to get total_bytes_written and
    // total_capacity
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::STORAGE_TOTAL_BYTES_WRITTEN)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.storage_info.total_bytes_written);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::STORAGE_TOTAL_BYTES_WRITTEN)] =
        sizeof(shared_glbl.storage_info.total_bytes_written);
    m_section_types[static_cast<uint32_t>(
        data_section_t::STORAGE_TOTAL_BYTES_WRITTEN)] =
        bson_encode::type_t::INT64;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::STORAGE_TOTAL_CAPACITY)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.storage_info.total_sd_card_capacity);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::STORAGE_TOTAL_CAPACITY)] =
        sizeof(shared_glbl.storage_info.total_sd_card_capacity);
    m_section_types[static_cast<uint32_t>(
        data_section_t::STORAGE_TOTAL_CAPACITY)] = bson_encode::type_t::INT64;

    // Section pointer for admin task to get all the parameters related to power
    m_section_pointers[static_cast<uint32_t>(data_section_t::BATTERY_VOLTAGE)] =
        reinterpret_cast<uint8_t *>(&shared_glbl.power_mgmt_info.voltage);
    m_section_sizes[static_cast<uint32_t>(data_section_t::BATTERY_VOLTAGE)] =
        sizeof(shared_glbl.power_mgmt_info.voltage);
    m_section_types[static_cast<uint32_t>(data_section_t::BATTERY_VOLTAGE)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(data_section_t::BATTERY_CURRENT)] =
        reinterpret_cast<uint8_t *>(&shared_glbl.power_mgmt_info.current);
    m_section_sizes[static_cast<uint32_t>(data_section_t::BATTERY_CURRENT)] =
        sizeof(shared_glbl.power_mgmt_info.current);
    m_section_types[static_cast<uint32_t>(data_section_t::BATTERY_CURRENT)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::BATTERY_REMAINING_CAPACITY)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.power_mgmt_info.remaining_capacity);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::BATTERY_REMAINING_CAPACITY)] =
        sizeof(shared_glbl.power_mgmt_info.remaining_capacity);
    m_section_types[static_cast<uint32_t>(
        data_section_t::BATTERY_REMAINING_CAPACITY)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::BATTERY_FULL_CHARGE_CAPACITY)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.power_mgmt_info.full_charge_capacity);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::BATTERY_FULL_CHARGE_CAPACITY)] =
        sizeof(shared_glbl.power_mgmt_info.full_charge_capacity);
    m_section_types[static_cast<uint32_t>(
        data_section_t::BATTERY_FULL_CHARGE_CAPACITY)] =
        bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::BATTERY_AVERAGE_CURRENT)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.power_mgmt_info.average_current);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::BATTERY_AVERAGE_CURRENT)] =
        sizeof(shared_glbl.power_mgmt_info.average_current);
    m_section_types[static_cast<uint32_t>(
        data_section_t::BATTERY_AVERAGE_CURRENT)] = bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::BATTERY_AVERAGE_POWER)] =
        reinterpret_cast<uint8_t *>(&shared_glbl.power_mgmt_info.average_power);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::BATTERY_AVERAGE_POWER)] =
        sizeof(shared_glbl.power_mgmt_info.average_power);
    m_section_types[static_cast<uint32_t>(
        data_section_t::BATTERY_AVERAGE_POWER)] = bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::BATTERY_TIME_TO_EMPTY)] =
        reinterpret_cast<uint8_t *>(&shared_glbl.power_mgmt_info.time_to_empty);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::BATTERY_TIME_TO_EMPTY)] =
        sizeof(shared_glbl.power_mgmt_info.time_to_empty);
    m_section_types[static_cast<uint32_t>(
        data_section_t::BATTERY_TIME_TO_EMPTY)] = bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::BATTERY_TEMPERATURE)] =
        reinterpret_cast<uint8_t *>(&shared_glbl.power_mgmt_info.temperature);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::BATTERY_TEMPERATURE)] =
        sizeof(shared_glbl.power_mgmt_info.temperature);
    m_section_types[static_cast<uint32_t>(
        data_section_t::BATTERY_TEMPERATURE)] = bson_encode::type_t::INT32;

    m_section_pointers[static_cast<uint32_t>(
        data_section_t::NUMBER_OF_BATTERY_CELLS)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.device_config.num_battery_cells_used);
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::NUMBER_OF_BATTERY_CELLS)] =
        sizeof(shared_glbl.device_config.num_battery_cells_used);
    m_section_types[static_cast<uint32_t>(
        data_section_t::NUMBER_OF_BATTERY_CELLS)] = bson_encode::type_t::INT32;

    // Section pointer for sd_card file format
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::SD_CARD_FILE_SYSTEM_FORMAT)] =
        reinterpret_cast<uint8_t *>(
            &shared_glbl.storage_info.sd_card_file_format);
    m_section_types[static_cast<uint32_t>(
        data_section_t::SD_CARD_FILE_SYSTEM_FORMAT)] =
        bson_encode::type_t::STRING;

    // Section pointer for number of seconds recorded
    // Section size will be set when string is populated
    m_section_pointers[static_cast<uint32_t>(
        data_section_t::SECONDS_RECORDED)] =
        reinterpret_cast<uint8_t *>(&m_recent_seconds_recorded);
    m_section_types[static_cast<uint32_t>(data_section_t::SECONDS_RECORDED)] =
        bson_encode::type_t::STRING;

    // Section size is predetermined by bson format, so no need to set it up
    m_section_pointers[static_cast<uint32_t>(data_section_t::RECORD_TIME)] =
        reinterpret_cast<uint8_t *>(&m_comm_request_time_ms);
    m_section_sizes[static_cast<uint32_t>(data_section_t::RECORD_TIME)] =
        sizeof(m_comm_request_time_ms);
    m_section_types[static_cast<uint32_t>(data_section_t::RECORD_TIME)] =
        bson_encode::type_t::UTC_DATETIME;

    m_section_pointers[static_cast<uint32_t>(data_section_t::CREATED_AT)] =
        reinterpret_cast<uint8_t *>(&m_comm_request_transfer_time_ms);
    m_section_sizes[static_cast<uint32_t>(data_section_t::CREATED_AT)] =
        sizeof(m_comm_request_transfer_time_ms);
    m_section_types[static_cast<uint32_t>(data_section_t::CREATED_AT)] =
        bson_encode::type_t::UTC_DATETIME;

    // Special field for making sure the whole bson is aligned to 4 bytes
    m_section_pointers[static_cast<uint32_t>(data_section_t::DUMMY_ALIGN)] =
        m_dummy_align;
    m_section_sizes[static_cast<uint32_t>(data_section_t::DUMMY_ALIGN)] = 0;
    m_section_types[static_cast<uint32_t>(data_section_t::DUMMY_ALIGN)] =
        bson_encode::type_t::BINARY;

    // Section pointers for wavfile is never set up
    // Section size for wavfile binary will be set up later
    m_section_pointers[static_cast<uint32_t>(data_section_t::WAV)] = nullptr;
    m_section_sizes[static_cast<uint32_t>(data_section_t::WAV)]    = 0;
    m_section_types[static_cast<uint32_t>(data_section_t::WAV)] =
        bson_encode::type_t::BINARY;

    m_section_index               = 0;

    m_bson_header_populated       = false;
    m_bson_buffer_bytes_populated = 0;
    m_bson_document_size          = 0;

    m_abort_requests              = false;

    m_first_sms_sent              = false;
}

/**
 * @brief Compute the bson document size based on the data sections populated
 *
 * @return uint32_t bson document size
 */
uint32_t CommTask::compute_bson_document_size()
{
    // Clear the dummy align section but will be populated later
    // once the bson document size is computed
    m_section_sizes[static_cast<uint32_t>(data_section_t::DUMMY_ALIGN)] = 0;

    m_bson_document_size = bson_encode::get_header_size();
    for (uint32_t field_index = 0; field_index < NUM_DATA_SECTIONS;
         field_index++)
    {
        // Skip the field if it does not need to be populated
        if (!SECTION_FLAGS[field_index].bits.data_status)
        {
            continue;
        }

        const uint32_t key_length = strlen(SECTION_NAMES[field_index]);
        uint32_t       field_size = 0;
        bool           skip_field_size_check = false;

        switch (m_section_types[field_index])
        {
            case bson_encode::type_t::INT32:
            {
                field_size = bson_encode::get_int32_field_size(key_length);
                break;
            }
            case bson_encode::type_t::INT64:
            {
                field_size = bson_encode::get_int64_field_size(key_length);
                break;
            }
            case bson_encode::type_t::DOUBLE:
            {
                field_size = bson_encode::get_double_field_size(key_length);
                break;
            }
            case bson_encode::type_t::STRING:
            {
                field_size = bson_encode::get_string_field_size(
                    key_length,
                    m_section_sizes[field_index] - 1);
                break;
            }
            case bson_encode::type_t::BINARY:
            {
                if (field_index == static_cast<uint32_t>(data_section_t::WAV))
                {
                    skip_field_size_check = true;
                }

                field_size = bson_encode::get_binary_field_size(
                    key_length,
                    m_section_sizes[field_index]);
                break;
            }
            case bson_encode::type_t::BOOLEAN:
            {
                field_size = bson_encode::get_boolean_field_size(key_length);
                break;
            }
            case bson_encode::type_t::UTC_DATETIME:
            {
                field_size = bson_encode::get_datetime_field_size(key_length);
                break;
            }
            default:
            {
                SYSTEM_ASSERT_TRUE(false);
            }
        }

        if (!skip_field_size_check)
        {
            SYSTEM_ASSERT_LTEQ(field_size, BSON_BUFFER_SIZE);
        }

        m_bson_field_sizes[field_index] = field_size;
        m_bson_document_size += field_size;
    }

    // Ensure the bson document (minus end-of-document field) is aligned to
    // 4-byte boundary
    if (m_bson_document_size & DUMMY_ALIGN_MASK)
    {
        uint32_t pad_align =
            DUMMY_ALIGN_SIZE - (m_bson_document_size & DUMMY_ALIGN_MASK);
        m_section_sizes[static_cast<uint32_t>(data_section_t::DUMMY_ALIGN)] =
            pad_align;
        m_bson_field_sizes[static_cast<uint32_t>(
            data_section_t::DUMMY_ALIGN)] += pad_align;
        m_bson_document_size += pad_align;
    }

    // Add end of document field
    m_bson_document_size += bson_encode::get_end_of_document_field_size();

    return m_bson_document_size;
}

/**
 * @brief Collect section data from gsm
 */
void CommTask::prepare_gsm_status_info()
{
    // Setup gprs status
    m_gsm_info.gprs_status  = w_hal_gsm_get_gprs_status();

    // Setup power status
    m_gsm_info.power_status = w_hal_gsm_get_power_status();

    // Setup SIM status
    m_gsm_info.sim_status   = w_hal_gsm_get_sim_status();

    // Setup signal quality
    w_hal_gsm_get_signal_quality(m_gsm_info.rssi,
                                 m_gsm_info.rxqual,
                                 m_gsm_info.rscp,
                                 m_gsm_info.ecno,
                                 m_gsm_info.rsrp,
                                 m_gsm_info.rsrq);

    // Save signal quality info to shared memory also
    w_hal_gsm_get_signal_quality(shared_glbl.shared_gsm_signal_info.rssi,
                                 shared_glbl.shared_gsm_signal_info.rxqual,
                                 shared_glbl.shared_gsm_signal_info.rscp,
                                 shared_glbl.shared_gsm_signal_info.ecno,
                                 shared_glbl.shared_gsm_signal_info.rsrp,
                                 shared_glbl.shared_gsm_signal_info.rsrq);

    // Setup for carrier date and time
    m_gsm_info.carrier_time_ms = MSEC_PER_SEC * w_hal_gsm_get_carrier_time();

    // Setup for PDP address
    const char **p_section_pointer = reinterpret_cast<const char **>(
        &m_section_pointers[static_cast<uint32_t>(
            data_section_t::GSM_PDP_ADDRESS)]);
    w_hal_gsm_get_pdp_address(*p_section_pointer,
                              m_section_sizes[static_cast<uint32_t>(
                                  data_section_t::GSM_PDP_ADDRESS)]);
}

/**
 * @brief Update seconds recorded from last status
 */
void CommTask::update_seconds_recorded()
{
    uint32_t    string_length;
    const char *p_format;

    // Update the seconds recorded after last status
    if (shared_glbl.storage_info.sd_card_is_empty)
    {
        p_format = "%d";
    }
    else
    {
        p_format = "+%d";
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    string_length = static_cast<uint32_t>(
        snprintf(m_recent_seconds_recorded,
                 sizeof(m_recent_seconds_recorded),
                 p_format,
                 shared_glbl.storage_info.seconds_recorded_since_boot -
                     m_last_seconds_recorded));
#pragma GCC diagnostic pop

    SYSTEM_ASSERT_GT(string_length, 0);

    m_section_sizes[static_cast<uint32_t>(data_section_t::SECONDS_RECORDED)] =
        string_length + 1;

    m_last_seconds_recorded =
        shared_glbl.storage_info.seconds_recorded_since_boot;
}

/**
 * @brief Prepare source data to be transferred over SMS
 */
void CommTask::prepare_sms_source_data()
{
    // Update the seconds recorded since last status
    update_seconds_recorded();

    prepare_gsm_status_info();

    // Setup transfer time
    w_hal_rtc_time_st transfer_time;
    w_hal_rtc_get_time(transfer_time);
    m_comm_request_transfer_time_ms =
        static_cast<uint64_t>(transfer_time.time_seconds) * MSEC_PER_SEC +
        (transfer_time.time_nanoseconds / NSEC_PER_MSEC);

    // Setup sd card file format size.
    // This is done here because the StorageTask is not running when the
    // device is first powered on so CommTask does not know
    // the sd card file format at the time of the initialization.
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::SD_CARD_FILE_SYSTEM_FORMAT)] =
        strlen(shared_glbl.storage_info.sd_card_file_format) + 1;
}

/**
 * @brief Prepare a data header for transfer
 *
 * @return the total size of the binary data
 */
uint32_t CommTask::prepare_data_header()
{
    SYSTEM_ASSERT_NOT_EQUAL(m_request_id, INVALID_COMM_MSG_ID);
    CommRequestMsg *p_comm_request = &m_p_comm_msgs[m_request_id];

    // Get maximum post request size from the modem
    uint32_t max_post_request_size = w_hal_gsm_get_max_post_request_size();

    // Update the seconds recorded since last status
    update_seconds_recorded();

    prepare_gsm_status_info();

    // Setup record time
    w_hal_rtc_time_st record_time;
    p_comm_request->get_time(record_time);
    m_comm_request_time_ms =
        static_cast<uint64_t>(record_time.time_seconds) * MSEC_PER_SEC +
        (record_time.time_nanoseconds / NSEC_PER_MSEC);

    // Setup transfer time
    w_hal_rtc_time_st transfer_time;
    w_hal_rtc_get_time(transfer_time);
    m_comm_request_transfer_time_ms =
        static_cast<uint64_t>(transfer_time.time_seconds) * MSEC_PER_SEC +
        (transfer_time.time_nanoseconds / NSEC_PER_MSEC);

    // Setup sd card file format size.
    // This is done here because the StorageTask is not running when the
    // device is first powered on so CommTask does not know
    // the sd card file format at the time of the initialization.
    m_section_sizes[static_cast<uint32_t>(
        data_section_t::SD_CARD_FILE_SYSTEM_FORMAT)] =
        strlen(shared_glbl.storage_info.sd_card_file_format) + 1;

    tm tm_struct;
    gmtime_r(&transfer_time.time_seconds, &tm_struct);

    // Set up file name
    int retval = snprintf(
        m_file_name,
        FILE_NAME_MAX_LENGTH,
        FILE_FORMAT_STRING,
        TIME_BASE_YEAR + static_cast<uint32_t>(tm_struct.tm_year),
        TIME_BASE_MONTH + static_cast<uint32_t>(tm_struct.tm_mon),
        tm_struct.tm_mday,
        tm_struct.tm_hour,
        tm_struct.tm_min,
        tm_struct.tm_sec,
        static_cast<uint32_t>(transfer_time.time_nanoseconds / NSEC_PER_MSEC));

    SYSTEM_ASSERT_GT(retval, 0);
    SYSTEM_ASSERT_LT(static_cast<uint32_t>(retval), FILE_NAME_MAX_LENGTH);

    // Prepare wav header
    m_remaining_audio_buffers_to_write =
        p_comm_request->get_remaining_buffers() + 1;

    uint32_t wav_data_size = m_remaining_audio_buffers_to_write;
    wav_data_size <<= BUFFER_SIZE_LOG2;

    if (wav_data_size >= m_max_wav_data_size_in_post_request)
    {
        // Cap the max data size
        m_remaining_audio_buffers_to_write =
            m_max_wav_data_size_in_post_request >> BUFFER_SIZE_LOG2;
        wav_data_size = m_remaining_audio_buffers_to_write;
        wav_data_size <<= BUFFER_SIZE_LOG2;
    }

    wav_file_set_data_size(m_wav_header, wav_data_size);

    static_assert(
        static_cast<uint32_t>(data_section_t::WAV) == NUM_DATA_SECTIONS - 1,
        "WAV DATA should be last section. We have special handling for it.");

    m_section_sizes[static_cast<uint32_t>(data_section_t::WAV)] =
        sizeof(m_wav_header) + wav_data_size;

    // Compute bson document size for bson header
    uint32_t binary_size = compute_bson_document_size();

    // Ensure the binary data is aligned to 4-byte boundary
    if (binary_size & DUMMY_ALIGN_MASK)
    {
        // Pad end of document field to 4-byte boundary
        uint32_t pad_align =
            DUMMY_ALIGN_SIZE - (binary_size & DUMMY_ALIGN_MASK);
        binary_size += pad_align;
    }

    // Re computing to get the minimum size value for gsm upload
    if (max_post_request_size < binary_size)
    {
        wav_data_size -= (binary_size - max_post_request_size);
        m_remaining_audio_buffers_to_write = wav_data_size >> BUFFER_SIZE_LOG2;

        wav_data_size                      = m_remaining_audio_buffers_to_write;
        wav_data_size <<= BUFFER_SIZE_LOG2;
        wav_file_set_data_size(m_wav_header, wav_data_size);

        static_assert(static_cast<uint32_t>(data_section_t::WAV) ==
                          NUM_DATA_SECTIONS - 1,
                      "WAV DATA should be last section. We have special "
                      "handling for it.");

        m_section_sizes[static_cast<uint32_t>(data_section_t::WAV)] =
            sizeof(m_wav_header) + wav_data_size;

        // Compute bson document size for bson header
        binary_size = compute_bson_document_size();

        // Ensure the binary data is aligned to 4-byte boundary
        if (binary_size & DUMMY_ALIGN_MASK)
        {
            // Pad end of document field to 4-byte boundary
            uint32_t pad_align =
                DUMMY_ALIGN_SIZE - (binary_size & DUMMY_ALIGN_MASK);
            binary_size += pad_align;
        }
    }

    return binary_size;
}

/**
 * @brief Populate the data footer for transfer
 */
void CommTask::populate_end_of_document_field()
{
    // Setup end of document
    bson_encode::error_t error =
        bson_encode::setup_end_of_document_field(m_bson_buffer,
                                                 BSON_BUFFER_SIZE);

    // If buffer is too small, then this is a problem that
    // cannot be recovered from
    SYSTEM_ASSERT_EQUAL(error, bson_encode::error_t::NO_ERROR);

    m_bson_buffer_bytes_populated =
        bson_encode::get_end_of_document_field_size();

    if (m_bson_buffer_bytes_populated & DUMMY_ALIGN_MASK)
    {
        // Pad end of document field to 4-byte boundary
        uint32_t pad_align = DUMMY_ALIGN_SIZE -
                             (m_bson_buffer_bytes_populated & DUMMY_ALIGN_MASK);
        memset(&m_bson_buffer[m_bson_buffer_bytes_populated], 0, pad_align);
        m_bson_buffer_bytes_populated += pad_align;
    }
}

/**
 * @brief Populate the sms field in buffer for transfer
 *
 * @param p_buffer buffer to populate
 * @param buffer_size bytes available in buffer
 * @param p_section_name field name
 * @param p_section_pointer field pointer data
 * @param data_type data type of field
 * @param filled[out] : number of bytes filled in buffer
 * @return true if successful
 * @return false otherwise
 */
bool CommTask::fill_sms_field_in_buffer(char *              p_buffer,
                                        uint32_t            buffer_size,
                                        const char *        p_section_name,
                                        const char *        p_section_pointer,
                                        bson_encode::type_t data_type,
                                        uint32_t &          filled)
{
    filled      = 0;

    int ret_val = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    switch (data_type)
    {
        case bson_encode::type_t::INT32:
        {
            ret_val =
                snprintf(p_buffer,
                         buffer_size,
                         "\"%s\":%d,",
                         p_section_name,
                         *reinterpret_cast<const int32_t *>(p_section_pointer));
            break;
        }
        case bson_encode::type_t::INT64:
        {
            ret_val =
                snprintf(p_buffer,
                         buffer_size,
                         "\"%s\":%" PRId64 ",",
                         p_section_name,
                         *reinterpret_cast<const int64_t *>(p_section_pointer));
            break;
        }
        case bson_encode::type_t::DOUBLE:
        {
            ret_val =
                snprintf(p_buffer,
                         buffer_size,
                         "\"%s\":%g,",
                         p_section_name,
                         *reinterpret_cast<const double *>(p_section_pointer));
            break;
        }
        case bson_encode::type_t::STRING:
        {
            ret_val =
                snprintf(p_buffer,
                         buffer_size,
                         "\"%s\":\"%s\",",
                         p_section_name,
                         reinterpret_cast<const char *>(p_section_pointer));
            break;
        }
        case bson_encode::type_t::BOOLEAN:
        {
            if (*reinterpret_cast<const bool *>(p_section_pointer))
            {
                ret_val = snprintf(p_buffer,
                                   buffer_size,
                                   "\"%s\":true,",
                                   p_section_name);
            }
            else
            {
                ret_val = snprintf(p_buffer,
                                   buffer_size,
                                   "\"%s\":false,",
                                   p_section_name);
            }
            break;
        }
        case bson_encode::type_t::UTC_DATETIME:
        {
            ret_val = snprintf(
                p_buffer,
                buffer_size,
                "\"%s\":%" PRId64 ",",
                p_section_name,
                *reinterpret_cast<const uint64_t *>(p_section_pointer));
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
        }
    }
#pragma GCC diagnostic pop

    if (ret_val > 0)
    {
        filled = static_cast<uint32_t>(ret_val);
    }

    return (filled > 0) && (filled <= buffer_size);
}

/**
 * @brief Prepare the sms buffer for transfer
 */
void CommTask::prepare_sms_buffer()
{
    uint32_t offset = 0;

    static_assert(sizeof(m_bson_buffer) >= MAX_SMS_LENGTH,
                  "SMS buffer not big enough");
    char *const p_sms_buffer = reinterpret_cast<char *>(m_bson_buffer);

    uint32_t    field_index;

    // Add keyword to SMS. Every SMS must start with the keyword so that
    // service provider can identify the SMS as a message that should be
    // forwarded to the server.
    uint32_t    keyword_length = strlen(m_sms_keyword);
    SYSTEM_ASSERT_LTEQ(keyword_length, m_sms_chars_remaining);
    memcpy(p_sms_buffer + offset, m_sms_keyword, keyword_length);
    offset += keyword_length;
    m_sms_chars_remaining -= keyword_length;

    // Add space after sms keyword
    p_sms_buffer[offset++] = ' ';
    m_sms_chars_remaining--;

    // Every SMS must contain a few fields to use as an identifier
    for (field_index = 0; field_index < NUM_DATA_SECTIONS; field_index++)
    {
        if (!SECTION_FLAGS[field_index].bits.every_sms_message)
        {
            continue;
        }

        uint32_t bytes_filled = 0;
        bool     success      = fill_sms_field_in_buffer(
            p_sms_buffer + offset,
            m_sms_chars_remaining,
            SECTION_NAMES[field_index],
            reinterpret_cast<const char *>(m_section_pointers[field_index]),
            m_section_types[field_index],
            bytes_filled);
        SYSTEM_ASSERT_EQUAL(success, true);

        SYSTEM_ASSERT_LTEQ(static_cast<uint32_t>(bytes_filled),
                           m_sms_chars_remaining);
        offset += bytes_filled;
        m_sms_chars_remaining -= bytes_filled;
    }

    // Fill as many fields as possible into the SMS buffer
    for (field_index = m_section_index; field_index < NUM_DATA_SECTIONS;
         field_index++)
    {
        if (SECTION_FLAGS[field_index].bits.every_sms_message)
        {
            // Field has already been added to this SMS in previous loop
            continue;
        }

        // Check if section needs to be included in status
        if (!SECTION_FLAGS[field_index].bits.every_sms_status &&
            (m_first_sms_sent ||
             !SECTION_FLAGS[field_index].bits.first_sms_status))
        {
            continue;
        }

        uint32_t bytes_filled = 0;
        bool     success      = fill_sms_field_in_buffer(
            p_sms_buffer + offset,
            m_sms_chars_remaining,
            SECTION_NAMES[field_index],
            reinterpret_cast<const char *>(m_section_pointers[field_index]),
            m_section_types[field_index],
            bytes_filled);

        if (!success)
        {
            break;
        }

        SYSTEM_ASSERT_LTEQ(static_cast<uint32_t>(bytes_filled),
                           m_sms_chars_remaining);
        offset += bytes_filled;
        m_sms_chars_remaining -= bytes_filled;
    }

    if (p_sms_buffer[offset - 1] == ',')
    {
        // Remove the last comma
        offset--;
        m_sms_chars_remaining++;
    }

    m_section_index               = field_index;

    m_bson_buffer_bytes_populated = offset;
}

/**
 * @brief Prepare a data buffer for transfer
 */
void CommTask::prepare_data_buffer()
{
    uint32_t             offset = 0;

    bson_encode::error_t error  = bson_encode::error_t::NO_ERROR;

    if (m_bson_header_populated == false)
    {
        // Setup header
        error = bson_encode::setup_header(m_bson_buffer,
                                          BSON_BUFFER_SIZE,
                                          m_bson_document_size);
        SYSTEM_ASSERT_EQUAL(error, bson_encode::error_t::NO_ERROR);

        m_bson_header_populated       = true;
        m_section_index               = 0;
        m_bson_buffer_bytes_populated = offset = bson_encode::get_header_size();
    }

    uint32_t field_index;
    for (field_index = m_section_index; field_index < NUM_DATA_SECTIONS;
         field_index++)
    {
        // Skip the field if it does not need to be populated
        if (!SECTION_FLAGS[field_index].bits.data_status)
        {
            continue;
        }

        uint32_t key_length = strlen(SECTION_NAMES[field_index]);

// The data was initially cast to a uint8_t pointer, so we need to
// ignore the warning about casting to a pointer with a different alignment
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
        switch (m_section_types[field_index])
        {
            case bson_encode::type_t::INT32:
            {
                error = bson_encode::setup_int32_field(
                    &m_bson_buffer[offset],
                    BSON_BUFFER_SIZE - offset,
                    SECTION_NAMES[field_index],
                    key_length,
                    *reinterpret_cast<const int32_t *>(
                        m_section_pointers[field_index]));
                break;
            }
            case bson_encode::type_t::INT64:
            {
                error = bson_encode::setup_int64_field(
                    &m_bson_buffer[offset],
                    BSON_BUFFER_SIZE - offset,
                    SECTION_NAMES[field_index],
                    key_length,
                    *reinterpret_cast<const int64_t *>(
                        m_section_pointers[field_index]));
                break;
            }
            case bson_encode::type_t::DOUBLE:
            {
                error = bson_encode::setup_double_field(
                    &m_bson_buffer[offset],
                    BSON_BUFFER_SIZE - offset,
                    SECTION_NAMES[field_index],
                    key_length,
                    *reinterpret_cast<const double *>(
                        m_section_pointers[field_index]));
                break;
            }
            case bson_encode::type_t::STRING:
            {
                error = bson_encode::setup_string_field(
                    &m_bson_buffer[offset],
                    BSON_BUFFER_SIZE - offset,
                    SECTION_NAMES[field_index],
                    key_length,
                    reinterpret_cast<const char *>(
                        m_section_pointers[field_index]),
                    m_section_sizes[field_index] - 1);
                break;
            }
            case bson_encode::type_t::BINARY:
            {
                if (field_index == static_cast<uint32_t>(data_section_t::WAV))
                {
                    // Set up the field header and the wav header together

                    uint32_t field_header_size =
                        bson_encode::get_binary_field_header_size(key_length);

                    if ((field_header_size + sizeof(m_wav_header)) <=
                        (BSON_BUFFER_SIZE - offset))
                    {
                        // set up only the header
                        error = bson_encode::setup_binary_field_header(
                            &m_bson_buffer[offset],
                            BSON_BUFFER_SIZE - offset,
                            SECTION_NAMES[field_index],
                            key_length,
                            m_section_sizes[field_index],
                            bson_encode::binary_subtype_t::GENERIC);

                        SYSTEM_ASSERT_EQUAL(error,
                                            bson_encode::error_t::NO_ERROR);

                        offset += field_header_size;

                        // Special handling for wav data
                        memcpy(&m_bson_buffer[offset],
                               &m_wav_header,
                               sizeof(m_wav_header));

                        offset += sizeof(m_wav_header);

                        // We will upload the audio sample data directly
                        // from the audio buffer.
                        // Since the section size includes the size of the
                        // audio data, we need to subtract it from the offset
                        offset -= m_bson_field_sizes[field_index];
                    }
                    else
                    {
                        // It would be a problem if we can't fit both the bson
                        // header and wav header in the bson buffer
                        SYSTEM_ASSERT_NOT_EQUAL(offset, 0);

                        error = bson_encode::error_t::BUFFER_TOO_SMALL;
                    }
                }
                else
                {
                    // Set up the entire field in the bson buffer
                    error = bson_encode::setup_binary_field(
                        &m_bson_buffer[offset],
                        BSON_BUFFER_SIZE - offset,
                        SECTION_NAMES[field_index],
                        key_length,
                        m_section_pointers[field_index],
                        m_section_sizes[field_index],
                        bson_encode::binary_subtype_t::GENERIC);
                }

                break;
            }
            case bson_encode::type_t::BOOLEAN:
            {
                error = bson_encode::setup_boolean_field(
                    &m_bson_buffer[offset],
                    BSON_BUFFER_SIZE - offset,
                    SECTION_NAMES[field_index],
                    key_length,
                    *reinterpret_cast<const bool *>(
                        m_section_pointers[field_index]));
                break;
            }
            case bson_encode::type_t::UTC_DATETIME:
            {
                error = bson_encode::setup_datetime_field(
                    &m_bson_buffer[offset],
                    BSON_BUFFER_SIZE - offset,
                    SECTION_NAMES[field_index],
                    key_length,
                    *reinterpret_cast<const uint64_t *>(
                        m_section_pointers[field_index]));
                break;
            }
            default:
            {
                SYSTEM_ASSERT_TRUE(false);
            }
        }
// Restore the warning about casting to a pointer with a different
// alignment
#pragma GCC diagnostic pop

        if (error == bson_encode::error_t::BUFFER_TOO_SMALL)
        {
            break;
        }
        SYSTEM_ASSERT_EQUAL(error, bson_encode::error_t::NO_ERROR);

        offset += m_bson_field_sizes[field_index];
    }

    m_section_index               = field_index;

    m_bson_buffer_bytes_populated = offset;
}

/**
 * @brief Indicates whether comm requests should be aborted
 *
 * @return true if comm requests should be aborted
 * @return false if comm requests should not be aborted
 */
bool CommTask::should_abort_comm_requests()
{
    return !m_server_api_route_exists || m_abort_requests ||
           shared_glbl.power_mgmt_info.low_battery_flag;
}

/**
 * @brief
 * Handle aborting of comm requests if applicable
 *
 * @return bool true if request was aborted, false otherwise
 */
bool CommTask::abort_current_comm_request()
{
    if (m_request_id != INVALID_COMM_MSG_ID)
    {
        CommRequestMsg *p_comm_request = &m_p_comm_msgs[m_request_id];

        p_comm_request->set_status(MsgStatus::ABORT);
        m_p_comm_response_queue->push(m_request_id);
        m_request_id = INVALID_COMM_MSG_ID;

        // Flush remaining buffer with zeros
        w_hal_gsm_cleanup();

        return true;
    }

    return false;
}

/**
 * @brief
 * Handle aborting of comm requests if applicable
 *
 * @return bool true if request was observed, false otherwise
 */
bool CommTask::handle_abort_comm_requests()
{
    if (m_handle == INVALID_GSM_HANDLE)
    {
        if (should_abort_comm_requests())
        {
            comm_request_id_t request_id = INVALID_COMM_MSG_ID;

            if (m_p_comm_request_queue->pop(request_id))
            {
                SYSTEM_ASSERT_NOT_EQUAL(request_id, INVALID_COMM_MSG_ID);
                CommRequestMsg *p_comm_request = &m_p_comm_msgs[request_id];

                // Abort the request and send it back
                p_comm_request->set_status(MsgStatus::ABORT);
                m_p_comm_response_queue->push(request_id);

                return true;
            }
        }
    }

    return false;
}

/**
 * @brief
 * Handle high priority queue requests
 *
 * @return bool true if request was observed, false otherwise
 */
bool CommTask::handle_hp_comm_requests()
{
    // Check high priority queue for new requests
    comm_hp_request_id_t hp_request_id;
    if (m_p_comm_hp_request_queue->pop(hp_request_id))
    {
        SYSTEM_ASSERT_NOT_EQUAL(hp_request_id, INVALID_COMM_HP_MSG_ID);
        CommHpRequestMsg *p_comm_hp_request = &m_p_comm_hp_msgs[hp_request_id];

        switch (p_comm_hp_request->get_status())
        {
            case MsgStatus::BUFFER_SLOWDOWN:
            case MsgStatus::COMM_STATUS_QUERY:
            {
                // An open post request can take more buffers in
                if ((m_handle != INVALID_GSM_HANDLE) ||
                    (w_hal_gsm_check_can_open_post_request() ==
                     wHalErrorCode::NO_ERROR))
                {
                    p_comm_hp_request->set_status(MsgStatus::COMM_OK);

                    if (m_abort_requests)
                    {
                        m_abort_requests = false;
                    }
                }
                else
                {
                    // Don't repeat aborting requests if already in abort mode
                    if (!m_abort_requests)
                    {
                        // Do the same with the buffers in the queue
                        m_abort_requests = true;

                        // Return the buffer back to Audio Task
                        abort_current_comm_request();

                        while (true)
                        {
                            if (!handle_abort_comm_requests())
                            {
                                break;
                            }
                        }
                    }

                    // After clearing the queue, respond to the request that
                    // CommTask has been stalled.
                    p_comm_hp_request->set_status(MsgStatus::COMM_SUSPENDED);
                }

                break;
            }
            default:
            {
                break;
            }
        }

        // Send the request back to Audio Task
        m_p_comm_hp_response_queue->push(hp_request_id);

        return true;
    }

    return false;
}

/**
 * @brief Parse and process received SMS
 *
 * @param p_buffer SMS buffer
 * @param length length of SMS buffer
 */
void CommTask::parse_received_sms(char *p_buffer, uint32_t length)
{
    static_assert(COMM_CORE == CORE_0,
                  "COMM_CORE should be on the same core as console");

    // Parse the received SMS line by line
    const char *p_line = p_buffer;
    uint32_t    buffer_index;
    int         match_length = 0;
    int         match_index;

    match_index = re_match("\\(Msg\\s+\\d+\\s+of\\s+",
                           p_buffer,
                           static_cast<int>(length),
                           &match_length);

    if (match_index != RE_NO_MATCH_INDEX)
    {
        // Skip matched string
        p_buffer += match_index + match_length;
        length -= static_cast<uint32_t>(match_index + match_length);

        // Scan for SMS count
        match_index = re_match("\\d+)",
                               p_buffer,
                               static_cast<int>(length),
                               &match_length);

        if (match_index != RE_NO_MATCH_INDEX)
        {
            int total = atoi(&p_buffer[match_index]);

            if (m_number_of_msgs_total != total)
            {
                // Unexpected so reset
                m_number_of_msgs_received = 0;
            }

            m_number_of_msgs_total = total;

            if (++m_number_of_msgs_received == m_number_of_msgs_total)
            {
                m_number_of_msgs_received = 0;
                m_number_of_msgs_total    = 0;
            }

            p_buffer += (match_index + match_length) + 1;
            length -= static_cast<uint32_t>((match_index + match_length) + 1);
            p_line = p_buffer;
        }

        for (buffer_index = 0; buffer_index < length; buffer_index++)
        {
            if ((p_buffer[buffer_index] == '\r') ||
                (p_buffer[buffer_index] == '\n'))
            {
                int line_length = p_buffer + buffer_index - p_line;
                if (line_length > 0)
                {
                    // Replace line terminator with Null terminator
                    p_buffer[buffer_index] = '\0';

                    printf("CommTask: Sending command to console: %s\n",
                           p_line);

                    new_console_command_handler(
                        p_line,
                        static_cast<uint32_t>(line_length));
                }
                p_line = p_buffer + buffer_index + 1;
            }
        }

        // If there is a partial line left, then assume it is a new line
        if (p_line < (p_buffer + length))
        {
            uint32_t line_length = p_buffer + length - p_line;
            new_console_command_handler(p_line, line_length);
        }
    }
}

/**
 * @brief
 * Main routine for Comm Task
 *
 * @return TaskErrorCode NoError
 */
TaskErrorCode CommTask::run()
{
    wHalErrorCode error = w_hal_gsm_handler();

    // Handle high priority requests first
    handle_hp_comm_requests();

    // Handle aborting of requests if applicable
    handle_abort_comm_requests();

    const wHalErrorCode valid_errors[] = {wHalErrorCode::NO_ERROR,
                                          wHalErrorCode::BUSY};
    SYSTEM_ASSERT_IN_SET(error,
                         valid_errors,
                         sizeof(valid_errors) / sizeof(wHalErrorCode));

    switch (m_state)
    {
        case State::WAIT_FOR_COMM_REQUEST:
        {
            if (m_handle == INVALID_GSM_HANDLE)
            {
                // Can't send SMS if there is no phone number
                // to send it to
                w_hal_rtc_time_st current_time;
                w_hal_rtc_get_time(current_time);

                w_hal_rtc_time_delta_st send_delta;
                w_hal_rtc_time_delta_st receive_delta;

                // Check if it is time to check if there is an sms to
                // receive from the server
                if (m_a2p_phone_number_exists &&
                    w_hal_gsm_need_to_check_for_receive_sms())
                {
                    // Wake up from sleep mode
                    w_hal_gsm_sleep(false);
                    m_empty_sms_count_before_sleep = 0;
                    m_state = State::CHECK_FOR_RECEIVE_SMS;
                    break;
                }

                if (m_p2a_phone_number_exists)
                {
                    if (shared_glbl.send_status_now)
                    {
                        // Create timeout for sending status
                        m_last_send_status_time.time_seconds     = 0;
                        m_last_send_status_time.time_nanoseconds = 0;
                        shared_glbl.send_status_now              = false;
                    }

                    // Check if it is time to send status
                    current_time.subtract(m_last_send_status_time, send_delta);
                    if (send_delta.seconds >=
                        m_device_config.sms_send_upload_frequency_seconds)
                    {
                        // Wake up from sleep mode
                        w_hal_gsm_sleep(false);
                        m_state = State::PREPARE_SMS_SOURCE_DATA;
                        break;
                    }
                }

                current_time.subtract(m_last_check_receive_sms_time,
                                      receive_delta);

                // Check for empty sms at least twice consecutively before
                // going to sleep
                if (m_a2p_phone_number_exists &&
                    (m_empty_sms_count_before_sleep <
                     EMPTY_SMS_LIMIT_BEFORE_SLEEP))
                {
                    if ((receive_delta.seconds >
                         SMS_EMPTY_STATE_CHECK_TIMEOUT_SECONDS) &&
                        !w_hal_gsm_get_sleep_state())
                    {
                        m_state = State::CHECK_FOR_RECEIVE_SMS;
                        break;
                    }
                }
                else
                {
                    // Only put gsm to sleep if there is no sms to receive and
                    // no buffer needs to be sent to the server
                    if ((send_delta.seconds >=
                         IDLE_STATE_CHECK_TIMEOUT_SECONDS) &&
                        (receive_delta.seconds >=
                         IDLE_STATE_CHECK_TIMEOUT_SECONDS) &&
                        (should_abort_comm_requests() ||
                         m_p_comm_request_queue->isEmpty()))
                    {
                        // enter sleep mode
                        if (w_hal_gsm_sleep(true) == wHalErrorCode::NO_ERROR)
                        {
                            break;
                        }
                    }
                }

                if (!should_abort_comm_requests())
                {
                    if (w_hal_gsm_check_can_open_post_request() ==
                        wHalErrorCode::GSM_TRANSFER_NO_HANDLE_AVAILABLE)
                    {
                        // If there is no handle available, then CommTask is not
                        // ready to take requests
                        break;
                    }
                }
            }

            SYSTEM_ASSERT_EQUAL(m_request_id, INVALID_COMM_MSG_ID);

            if (!m_p_comm_request_queue->pop(m_request_id))
            {
                // Nothing new to process
                break;
            }

            SYSTEM_ASSERT_NOT_EQUAL(m_request_id, INVALID_COMM_MSG_ID);

            // If low battery or issue with comm, abort the request
            // and send it back.
            if (should_abort_comm_requests())
            {
                // Abort the request and send it back
                abort_current_comm_request();
                break;
            }

            // Check if post request is open
            if (m_handle != INVALID_GSM_HANDLE)
            {
                // If more data is available, then continue with preparing
                // data buffer with next data
                if (m_section_index < NUM_DATA_SECTIONS)
                {
                    m_state = State::PREPARE_DATA_BUFFER;
                }
                else
                {
                    SYSTEM_ASSERT_GT(m_remaining_audio_buffers_to_write, 0);
                    m_state = State::TRANSFER_WAV_DATA;
                }

                break;
            }

            m_state = State::OPEN_POST_REQUEST;
        }
        // FALL-THROUGH
        case State::OPEN_POST_REQUEST:
        {
            // Check if post request is open (if not, then open)
            SYSTEM_ASSERT_EQUAL(m_handle, INVALID_GSM_HANDLE);

            uint32_t total_size = prepare_data_header();

            error = w_hal_gsm_open_post_request(m_server_api_route,
                                                m_http_content_type,
                                                m_file_name,
                                                m_handle,
                                                static_cast<int>(total_size));

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            SYSTEM_ASSERT_NOT_EQUAL(m_handle, INVALID_GSM_HANDLE);

            m_bson_header_populated = false;
            m_state                 = State::PREPARE_DATA_BUFFER;
        }
        // FALL-THROUGH
        case State::PREPARE_DATA_BUFFER:
        {
            // Prepare data buffer
            prepare_data_buffer();

            m_state = State::TRANSFER_DATA_BUFFER;
        }
        // FALL-THROUGH
        case State::TRANSFER_DATA_BUFFER:
        {
            // Write data binary header out
            SYSTEM_ASSERT_NOT_EQUAL(m_handle, INVALID_GSM_HANDLE);
            error = w_hal_gsm_upload(m_bson_buffer,
                                     m_bson_buffer_bytes_populated,
                                     m_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }
            else
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            }

            m_state = State::WAIT_FOR_TRANSFER_DATA_BUFFER_RESPONSE;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_TRANSFER_DATA_BUFFER_RESPONSE:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_handle, INVALID_GSM_HANDLE);
            if (w_hal_gsm_check_active_upload(m_handle))
            {
                break;
            }

            // If more data is available, then continue with preparing data
            // buffer with next data
            if (m_section_index < NUM_DATA_SECTIONS)
            {
                m_state = State::PREPARE_DATA_BUFFER;
                break;
            }
            else if (m_remaining_audio_buffers_to_write == 0)
            {
                // This last transfer was for the end of the file, so close
                // the post request
                m_state = State::CLOSE_POST_REQUEST;
                break;
            }
            else
            {
                m_state = State::TRANSFER_WAV_DATA;
            }
        }
        // FALL-THROUGH
        case State::TRANSFER_WAV_DATA:
        {
            // Transfer data. If response status comes back as BUSY,
            // then save request and come back later. If response is
            // NO_ERROR, then break out and come back
            SYSTEM_ASSERT_EQUAL(m_section_index, NUM_DATA_SECTIONS);

            SYSTEM_ASSERT_NOT_EQUAL(m_request_id, INVALID_COMM_MSG_ID);
            CommRequestMsg *p_comm_request = &m_p_comm_msgs[m_request_id];

            buffer_id_t     current_buffer_id;
            uint16_t        current_buffer_length;

            p_comm_request->get_buffer(current_buffer_id,
                                       current_buffer_length);
            SYSTEM_ASSERT_NOT_EQUAL(current_buffer_id, INVALID_BUFFER_ID);

            const uint8_t *p_buffer = get_buffer_address(current_buffer_id);

            SYSTEM_ASSERT_NOT_EQUAL(m_handle, INVALID_GSM_HANDLE);
            error = w_hal_gsm_upload(p_buffer, current_buffer_length, m_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }
            else
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            }

            m_state = State::WAITING_FOR_TRANSFER_WAV_DATA_RESPONSE;
        }
        // FALL-THROUGH
        case State::WAITING_FOR_TRANSFER_WAV_DATA_RESPONSE:
        {
            // Check if current upload is done. If it is,
            // then send the response.
            SYSTEM_ASSERT_EQUAL(m_section_index, NUM_DATA_SECTIONS);

            SYSTEM_ASSERT_NOT_EQUAL(m_handle, INVALID_GSM_HANDLE);
            if (w_hal_gsm_check_active_upload(m_handle))
            {
                break;
            }

            SYSTEM_ASSERT_NOT_EQUAL(m_request_id, INVALID_COMM_MSG_ID);
            CommRequestMsg *p_comm_request = &m_p_comm_msgs[m_request_id];

            p_comm_request->set_status(MsgStatus::GOOD);
            m_p_comm_response_queue->push(m_request_id);
            m_request_id = INVALID_COMM_MSG_ID;

            // If this is the last in series of transfers for one
            // request, then proceed to close post request
            if (--m_remaining_audio_buffers_to_write == 0)
            {
                m_state = State::PREPARE_END_OF_DATA;
            }
            else
            {
                m_state = State::WAIT_FOR_COMM_REQUEST;
                break;
            }
        }
        // FALL-THROUGH
        case State::PREPARE_END_OF_DATA:
        {
            populate_end_of_document_field();

            m_state = State::TRANSFER_DATA_BUFFER;
            break;
        }
        // FALL-THROUGH
        case State::CLOSE_POST_REQUEST:
        {
            SYSTEM_ASSERT_NOT_EQUAL(m_handle, INVALID_GSM_HANDLE);
            error = w_hal_gsm_close_post_request(m_handle);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            // No longer need to worry about handle
            w_hal_rtc_get_time(m_last_send_status_time);
            m_handle                                  = INVALID_GSM_HANDLE;
            m_state                                   = State::FIRST;

            // Only want to send empty sd card indication once
            shared_glbl.storage_info.sd_card_is_empty = false;

            break;
        }
        case State::PREPARE_SMS_SOURCE_DATA:
        {
            prepare_sms_source_data();

            m_section_index = 0;
            m_state         = State::OPEN_SMS_REQUEST;
        }
        // FALL-THROUGH
        case State::OPEN_SMS_REQUEST:
        {
            error = w_hal_gsm_write_sms_request(m_sms_chars_remaining);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            m_state = State::PREPARE_SMS_BUFFER;
        }
        // FALL-THROUGH
        case State::PREPARE_SMS_BUFFER:
        {
            // Prepare data buffer
            prepare_sms_buffer();

            m_state = State::TRANSFER_SMS_BUFFER;
        }
        // FALL-THROUGH
        case State::TRANSFER_SMS_BUFFER:
        {
            // Write data binary header out
            error = w_hal_gsm_add_sms_content(
                reinterpret_cast<char *>(&m_bson_buffer),
                m_bson_buffer_bytes_populated,
                m_sms_chars_remaining);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }
            else
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            }

            m_state = State::WAIT_FOR_SMS_TRANSFER_RESPONSE;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_SMS_TRANSFER_RESPONSE:
        {
            if (w_hal_gsm_check_sms_content_uploading())
            {
                break;
            }

            m_state = State::CLOSE_SMS_REQUEST;
        }
        // FALL-THROUGH
        case State::CLOSE_SMS_REQUEST:
        {
            error = w_hal_gsm_close_sms_write_request();

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }
            else
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            }

            m_state = State::WAIT_FOR_SMS_CLOSE_RESPONSE;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_SMS_CLOSE_RESPONSE:
        {
            if (!w_hal_gsm_check_sms_request_closed())
            {
                break;
            }

            // More data to send means we need to send another SMS
            if (m_section_index < NUM_DATA_SECTIONS)
            {
                m_state = State::OPEN_SMS_REQUEST;
            }
            else
            {
                m_first_sms_sent = true;
                w_hal_rtc_get_time(m_last_send_status_time);
                m_state                                   = State::FIRST;

                // Only want to send empty sd card indication once
                shared_glbl.storage_info.sd_card_is_empty = false;
            }
            break;
        }
        case State::CHECK_FOR_RECEIVE_SMS:
        {
            error = w_hal_gsm_read_sms_request(
                reinterpret_cast<char *>(&m_bson_buffer),
                BSON_BUFFER_SIZE);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            m_state = State::WAIT_FOR_RECEIVE_SMS;
        }
        // FALL-THROUGH
        case State::WAIT_FOR_RECEIVE_SMS:
        {
            uint32_t sms_length = 0;
            error               = w_hal_gsm_check_sms_read_done(sms_length);

            if (error == wHalErrorCode::BUSY)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            // Process result message
            char *p_sms_buffer       = reinterpret_cast<char *>(&m_bson_buffer);
            p_sms_buffer[sms_length] = '\0';
            parse_received_sms(p_sms_buffer, sms_length);

            error = w_hal_gsm_clear_sms_read_done();
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            w_hal_rtc_get_time(m_last_check_receive_sms_time);

            if (sms_length != 0)
            {
                m_empty_sms_count_before_sleep = 0;
                m_state                        = State::CHECK_FOR_RECEIVE_SMS;
                break;
            }
            else
            {
                m_empty_sms_count_before_sleep++;
                m_state = State::FIRST;
                break;
            }
        }
        default:
            break;
    }

    // To log gsm parameters for every 2 minutes
    time_t updated_time = w_hal_rtc_get_time();
    if ((updated_time - m_last_gsm_info_logged_time) > 120)
    {
        LOG_MSG_WITH_ARGS_NO_PRINT(TraceFile::COMM_TASK_CPP,
                                   "GPRS_status: %d, SIM_status: %d, RSSI: %d, "
                                   "RXQUAL: %d, RSCP: %d, ECNO: "
                                   "%d, RSRP: %d, RSRQ: %d\n",
                                   m_gsm_info.gprs_status,
                                   m_gsm_info.sim_status,
                                   m_gsm_info.rssi,
                                   m_gsm_info.rxqual,
                                   m_gsm_info.rscp,
                                   m_gsm_info.ecno,
                                   m_gsm_info.rsrp,
                                   m_gsm_info.rsrq);
        m_last_gsm_info_logged_time = updated_time;
    }

    switch (shared_glbl.shutdown_stage)
    {
        case shutdown_stage_t::Stage_6:
        {
            w_hal_rtc_time_st       current_time;
            w_hal_rtc_time_delta_st receive_delta;

            w_hal_rtc_get_time(current_time);
            current_time.subtract(m_last_check_receive_sms_time, receive_delta);

            // Check if the device is expecting more messages
            if ((m_number_of_msgs_received != m_number_of_msgs_total) &&
                (receive_delta.seconds < A2P_MESSAGE_COUNT_TO))
            {
                break;
            }

            // Check for Comm task request queue and response queue to be empty
            // and go to sleep
            error = w_hal_gsm_handler();
            if (error != wHalErrorCode::NO_ERROR)
            {
                break;
            }

            if (!m_p_comm_request_queue->isEmpty() || (m_state != State::FIRST))
            {
                break;
            }

            w_hal_gsm_cleanup();
            m_handle = INVALID_GSM_HANDLE;

            SYSTEM_ASSERT_TRUE(m_p_comm_response_queue->isEmpty());

            shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                static_cast<int>(shared_glbl.shutdown_stage) + 1);
            return TaskErrorCode::Sleep;
        }
        default:
        {
            break;
        }
    }

    return TaskErrorCode::NoError;
}

#endif // __CORE__ == COMM_CORE
