#include "w_hal_gsm.hpp"

#include "WAVFile.hpp"
#include "assert.hpp"
#include "bson_encode.hpp"
#include "constants.hpp"
#include "shared.hpp"
#include "trace.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string.h>
#include <time.h>

static const uint32_t     MAX_OPEN_HANDLES = 2;

static bool               sms_request_open;
static uint32_t           remaining_bytes_in_sms;

static uint32_t           bytes_received;

// Handle counter which is incremented with every open post request
static wGsmTransferHandle handle_counter;

// Data counter incremented with every sample seen. It can and will overflow
// but this is not a problem.
static uint16_t           data_counter;

// List of open handles
static wGsmTransferHandle open_handles[MAX_OPEN_HANDLES];

// Expected size of data coming in
static int                remaining_bytes[MAX_OPEN_HANDLES];

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
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Initialize GSM module
 *
 * @param device_config device configuration
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_gsm_init(DeviceConfig::config_page_st &device_config)
{
    sms_request_open       = false;
    remaining_bytes_in_sms = 0;

    bytes_received         = 0;
    handle_counter         = 0;
    data_counter           = 0;

    for (uint32_t index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        open_handles[index] = INVALID_GSM_HANDLE;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get time value that was read during initialization
 *
 * @return time_t the time value in GMT
 */
time_t w_hal_gsm_get_init_time()
{
    return time(nullptr);
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
    // Write is going on?
    if (sms_request_open)
    {
        return wHalErrorCode::BUSY;
    }

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
    sms_size = 0;
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
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Open a handle to a SMS request
 *
 * @param sms_size remaining characters in SMS request
 * @return wHalErrorCode BUSY if SMS request is already
 * open, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_write_sms_request(uint32_t &sms_size)
{
    if (sms_request_open || (remaining_bytes_in_sms > 0))
    {
        return wHalErrorCode::BUSY;
    }

    sms_size = remaining_bytes_in_sms = MAX_SMS_LENGTH;
    sms_request_open                  = true;

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
    if (!sms_request_open)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_NOT_OPEN;
    }

    if (size > remaining_bytes_in_sms)
    {
        return wHalErrorCode::GSM_SMS_REQUEST_TOO_LONG;
    }

    remaining_bytes_in_sms -= size;
    remaining_size = remaining_bytes_in_sms;

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
    return false;
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
    if (remaining_bytes_in_sms != 0)
    {
        remaining_bytes_in_sms = 0;
    }
    sms_request_open = false;

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
    return (remaining_bytes_in_sms == 0);
}

/**
 * @brief Check if there are any available file handles
 *
 * @return uint32_t number of available file handles
 */
uint32_t w_hal_gsm_get_available_post_requests()
{
    uint32_t available_handles = 0;
    for (uint32_t index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        if (open_handles[index] == INVALID_GSM_HANDLE)
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
    uint32_t index;

    // Find open spot for handle
    for (index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        if (open_handles[index] == INVALID_GSM_HANDLE)
        {
            break;
        }
    }

    return index;
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
    if (index == MAX_OPEN_HANDLES)
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
    uint32_t index = get_available_handle_tracker();

    if (index == MAX_OPEN_HANDLES)
    {
        return wHalErrorCode::GSM_TRANSFER_NO_HANDLE_AVAILABLE;
    }

    handle                 = ++handle_counter;
    open_handles[index]    = handle;
    remaining_bytes[index] = total_size;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Add data to post request
 *
 * @param p_buffer buffer data to be transferred
 * @param size size of buffer data
 * @param handle handle for open post request
 * @return wHalErrorCode BUSY if request cannot be serviced now,
 * GSM_INVALID_HANDLE if handle is not open or is invalid, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_upload(const uint8_t *    p_buffer,
                               uint32_t           size,
                               wGsmTransferHandle handle)
{
    const uint32_t MAX_BIN_SIZE = 32 * 1024 * 1024;
    static uint8_t receive_buffer[MAX_BIN_SIZE];
    uint32_t       index;

    // Check if handle is open
    for (index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        if (open_handles[index] == handle)
        {
            break;
        }
    }

    if (index == MAX_OPEN_HANDLES)
    {
        return wHalErrorCode::GSM_INVALID_HANDLE;
    }

    memcpy(&receive_buffer[bytes_received], p_buffer, size);
    bytes_received += size;

    if (remaining_bytes[index] != INVALID_GSM_UPLOAD_SIZE)
    {
        SYSTEM_ASSERT_LTEQ(static_cast<int>(size), remaining_bytes[index]);
        remaining_bytes[index] -= static_cast<int>(size);
    }

    if (bytes_received >= sizeof(bson_encode::main_header_t))
    {
        bson_encode::main_header_t *p_header =
            reinterpret_cast<bson_encode::main_header_t *>(receive_buffer);

        if (bytes_received >= p_header->size)
        {
            uint32_t offset = bson_encode::get_header_size();

            while (offset < bytes_received)
            {
                bson_encode::key_field_common_t *p_key_field =
                    reinterpret_cast<bson_encode::key_field_common_t *>(
                        &receive_buffer[offset]);

                const uint32_t key_length = strlen(p_key_field->key);

                uint32_t       field_size = 0;
                switch (p_key_field->type)
                {
                    case bson_encode::type_t::INT32:
                    {
                        field_size =
                            bson_encode::get_int32_field_size(key_length);
                        break;
                    }
                    case bson_encode::type_t::INT64:
                    {
                        field_size =
                            bson_encode::get_int64_field_size(key_length);
                        break;
                    }
                    case bson_encode::type_t::DOUBLE:
                    {
                        field_size =
                            bson_encode::get_double_field_size(key_length);
                        break;
                    }
                    case bson_encode::type_t::STRING:
                    {
                        bson_encode::string_field_subheader_t *p_subheader =
                            reinterpret_cast<
                                bson_encode::string_field_subheader_t *>(
                                &receive_buffer
                                    [offset +
                                     sizeof(bson_encode::key_field_common_t) +
                                     key_length]);

                        field_size = bson_encode::get_string_field_size(
                            key_length,
                            p_subheader->length - 1);

                        break;
                    }
                    case bson_encode::type_t::BINARY:
                    {
                        bson_encode::binary_field_subheader_t *p_subheader =
                            reinterpret_cast<
                                bson_encode::binary_field_subheader_t *>(
                                &receive_buffer
                                    [offset +
                                     sizeof(bson_encode::key_field_common_t) +
                                     key_length]);

                        field_size = bson_encode::get_binary_field_size(
                            key_length,
                            p_subheader->length);

                        if (strcmp(p_key_field->key, "audio") == 0)
                        {
                            uint32_t wav_file_offset =
                                offset +
                                sizeof(bson_encode::key_field_common_t) +
                                key_length +
                                sizeof(bson_encode::binary_field_subheader_t);

                            wav_header_t *p_wav_header =
                                reinterpret_cast<wav_header_t *>(
                                    &receive_buffer[wav_file_offset]);

                            bool result = wav_file_verify_header(
                                *p_wav_header,
                                DeviceConfig::p_page->num_audio_channels,
                                DeviceConfig::p_page->sampling_rate,
                                DeviceConfig::p_page->audio_sampling_width,
                                m_wav_header.env_subchunk.temperature,
                                m_wav_header.env_subchunk.humidity,
                                m_wave_header.batt_subchunk.voltage,
                                m_wave_header.batt_subchunk.current,
                                m_wave_header.batt_subchunk.remaining_capacity,
                                m_wave_header.batt_subchunk.average_current,
                                m_wave_header.batt_subchunk.time_to_empty,
                                m_wav_header.loc_subchunk.latitude_e7,
                                m_wav_header.loc_subchunk.longitude_e7);

                            SYSTEM_ASSERT_TRUE(result);

                            uint32_t wav_data_end =
                                wav_file_offset + p_subheader->length;

                            const uint32_t aligned_eod_field_size =
                                sizeof(bson_encode::end_of_document_field_t) +
                                (sizeof(uint32_t) -
                                 (sizeof(uint32_t) %
                                  sizeof(
                                      bson_encode::end_of_document_field_t)));
                            SYSTEM_ASSERT_LTEQ(bytes_received,
                                               wav_data_end +
                                                   aligned_eod_field_size);

                            const uint32_t wav_data_size =
                                p_wav_header->data_subchunk_header.size;

                            SYSTEM_ASSERT_EQUAL(p_subheader->length,
                                                sizeof(wav_header_t) +
                                                    wav_data_size);

                            uint32_t wav_data_start =
                                wav_file_offset + sizeof(wav_header_t);

                            const uint16_t *p_sample =
                                reinterpret_cast<const uint16_t *>(
                                    &receive_buffer[wav_data_start]);
                            for (uint32_t byte_index = wav_data_start;
                                 byte_index < wav_data_end;
                                 byte_index += sizeof(uint16_t))
                            {
                                SYSTEM_ASSERT_EQUAL(*p_sample, data_counter);

                                p_sample++;
                                data_counter++;
                            }

                            // Clear receive buffer
                            memset(receive_buffer, 0, bytes_received);
                            bytes_received = 0;
                        }

                        break;
                    }
                    case bson_encode::type_t::BOOLEAN:
                    {
                        field_size =
                            bson_encode::get_boolean_field_size(key_length);
                        break;
                    }
                    case bson_encode::type_t::UTC_DATETIME:
                    {
                        field_size =
                            bson_encode::get_datetime_field_size(key_length);
                        break;
                    }
                    default:
                    {
                        SYSTEM_ASSERT_TRUE(false);
                    }
                }

                offset += field_size;
            }
        }
    }

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
    return false;
}

/**
 * @brief Close post request for handle
 *
 * @param handle handle for open post request
 * @return wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_close_post_request(wGsmTransferHandle handle)
{
    uint32_t index;

    // Check if handle is open
    for (index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        if (open_handles[index] == handle)
        {
            if (remaining_bytes[index] != INVALID_GSM_UPLOAD_SIZE)
            {
                SYSTEM_ASSERT_EQUAL(remaining_bytes[index], 0);
            }

            // clear from list
            open_handles[index] = INVALID_GSM_HANDLE;
        }
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
bool w_hal_gsm_check_post_request_closed(wGsmTransferHandle handle)
{
    // Check if handle is open
    for (uint32_t index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        if (open_handles[index] == handle)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Run background activities for GSM HAL
 *
 * @return wHalErrorCode BUSY if there is work to be done, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_gsm_handler()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get the gsm modem name string
 *
 * @param p_modem_name[out]: GSM modem name string
 * @param modem_size[out]: size of the modem name string
 */
void w_hal_gsm_get_modem_name(const char *&p_modem_name, uint32_t &modem_size)
{
    static const char *p_name = "Simulation GSM modem";
    p_modem_name              = p_name;
    modem_size                = strlen(p_name) + 1;
}

/**
 * @brief Get the gsm firmware version string
 *
 * @param p_fw_version[out]: GSM firmware version string
 * @param version_size[out]: size of firmware version string
 */
void w_hal_gsm_get_fw_version(const char *&p_fw_version, uint32_t &version_size)
{
    static const char *p_version = "1.0.0";
    p_fw_version                 = p_version;
    version_size                 = strlen(p_version) + 1;
}

/**
 * @brief Get the gsm imei number string
 *
 * @param p_imei[out]: imei number string
 * @param imei_size[out]: size of imei number string
 */
void w_hal_gsm_get_imei(const char *&p_imei, uint32_t &imei_size)
{
    static const char *p_id = "012345678912345";
    p_imei                  = p_id;
    imei_size               = strlen(p_id) + 1;
}

/**
 * @brief Get the gsm imsi number string
 *
 * @param p_imsi[out]: imsi number string
 * @param imsi_size[out]: size of imsi number string
 */
void w_hal_gsm_get_imsi(const char *&p_imsi, uint32_t &imsi_size)
{
    static const char *p_number = "123456789012345";
    p_imsi                      = p_number;
    imsi_size                   = strlen(p_number) + 1;
}

/**
 * @brief Get the gsm iccid string
 *
 * @param p_iccid[out]: iccid string
 * @param iccid_size[out]: size of iccid string
 */
void w_hal_gsm_get_iccid(const char *&p_iccid, uint32_t &iccid_size)
{
    static const char *p_id = "1234567890123456789012";
    p_iccid                 = p_id;
    iccid_size              = strlen(p_id) + 1;
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
    static const char *p_name = "Simulation operator";
    p_operator_name           = p_name;
    operator_size             = strlen(p_name) + 1;
}

/**
 * @brief Get the gsm gprs status
 *
 * @return uint8_t: Returns the gprs attachment status
 */
uint8_t w_hal_gsm_get_gprs_status()
{
    return 0;
}

/**
 * @brief Get the gsm power status
 *
 * @return uint8_t: Returns the gsm power status
 */
uint8_t w_hal_gsm_get_power_status()
{
    return 0;
}

/**
 * @brief Get the gsm sim status
 *
 * @return uint8_t: Returns SIM status
 */
uint8_t w_hal_gsm_get_sim_status()
{
    return 0;
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
    rssi   = 0;
    rxqual = 0;
    rscp   = 0;
    ecno   = 0;
    rsrp   = 0;
    rsrq   = 0;
}

/**
 * @brief Get the gsm carrier time
 *
 * @return time_t: Returns the Carrier time
 */
time_t w_hal_gsm_get_carrier_time()
{
    return time(nullptr);
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
    static const char *p_address = "111.222.333.444";
    p_pdp_address                = p_address;
    pdp_address_size             = strlen(p_address) + 1;
}

/**
 * @brief Get the gsm location area code string
 *
 * @param p_location_area_code[out]: location area code
 * @param location_area_code_size[out]: size of the location area code string
 */
void w_hal_gsm_get_location_area_code(const char *&p_location_area_code,
                                      uint32_t &   location_area_code_size)
{
    static const char *p_code = "A1B2";
    p_location_area_code      = p_code;
    location_area_code_size   = strlen(p_code) + 1;
}

/**
 * @brief Get the gsm cell id string
 *
 * @param p_cell_id[out]: cell id
 * @param cell_id_size[out]: size of the cell id string
 */
void w_hal_gsm_get_cell_id(const char *&p_cell_id, uint32_t &cell_id_size)
{
    static const char *p_id = "ABCDEF01";
    p_cell_id               = p_id;
    cell_id_size            = strlen(p_id) + 1;
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
    static const char *p_phone = "1234567890";
    p_phone_number             = p_phone;
    phone_number_size          = strlen(p_phone) + 1;
}

/**
 * @brief Print GSM Information obtained during Initialization
 */
void w_hal_gsm_print_info()
{
    LOG_MSG(TraceFile::W_HAL_GSM_CPP, "WHAL GSM INFO : SIM\n");
}

/**
 * @brief Teardown of the GSM
 *
 * @return wHalErrorCode BUSY if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_teardown()
{
    bytes_received      = 0;
    wHalErrorCode error = wHalErrorCode::BUSY;
    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_gsm_handler();
    }

    handle_counter = 0;
    data_counter   = 0;
    for (uint32_t index = 0; index < MAX_OPEN_HANDLES; index++)
    {
        open_handles[index] = '\0';
    }

    return error;
}

/**
 * @brief To enter to the sleep mode for gsm
 *
 * @param sleep flag to set for sleep mode
 * @return wHalErrorCode wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_sleep(bool sleep)
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief to check if gsm receives any SMS
 *
 * @return true: SMS received
 * @return false: SMS not received
 */
bool w_hal_gsm_need_to_check_for_receive_sms()
{
    return false;
}

/**
 * @brief To get the sleep state of an modem
 *
 * @return true: If it is in sleep
 * @return false: If it is in wakeup state
 */
bool w_hal_gsm_get_sleep_state()
{
    return false;
}

/**
 * @brief GSM file cleanup before shutting it down
 */
void w_hal_gsm_cleanup(void)
{
}

/**
 * @brief To get the max file system size for post request
 *
 * @return uint32_t: Returns the max supported post request size
 */
uint32_t w_hal_gsm_get_max_post_request_size()
{
    return 1024 * 1024;
}