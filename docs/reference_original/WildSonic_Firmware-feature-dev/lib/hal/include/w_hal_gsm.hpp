#ifndef LIB_HAL_W_HAL_GSM_HPP
#define LIB_HAL_W_HAL_GSM_HPP

#include "device_config.hpp"

#include "w_hal_error_code.hpp"
#include <time.h>

using wGsmTransferHandle                         = time_t;
const wGsmTransferHandle INVALID_GSM_HANDLE      = 0; // Invalid GSM handle
const int                INVALID_GSM_UPLOAD_SIZE = -1;

/**
 * @brief Validate GSM configuration
 *
 * @param device_config device configuration
 * @param fix indicates if the configuration should be fixed
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_gsm_validate_config(DeviceConfig::config_page_st &device_config,
                          bool                          fix);

/**
 * @brief Initialize GSM module
 *
 * @param device_config device configuration
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_gsm_init(DeviceConfig::config_page_st &device_config);

/**
 * @brief Get time value that was read during initialization
 *
 * @return time_t the time value in GMT
 */
time_t        w_hal_gsm_get_init_time();

/**
 * @brief Issue request to modem to check and read oldest SMS
 *
 * @param p_sms SMS buffer to hold content
 * @param sms_size SMS buffer size
 * @return wHalErrorCode GSM_SMS_NOT_ENABLED if not enabled,
 * BUSY if SMS read request is not open, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_read_sms_request(char *p_sms, uint32_t sms_size);

/**
 * @brief Check if SMS read is done
 *
 * @param sms_size[out] characters in received SMS
 * @return wHalErrorCode  GSM_SMS_NOT_ENABLED if not enabled,
 * GSM_SMS_READ_NOT_OPEN if SMS read request is not open,
 * BUSY if SMS read request is not open, NO_ERROR
 * otherwise.
 */
wHalErrorCode w_hal_gsm_check_sms_read_done(uint32_t &sms_size);

/**
 * @brief Clear SMS read done flag
 *
 * @return wHalErrorCode  GSM_SMS_NOT_ENABLED if not enabled,
 * GSM_SMS_READ_NOT_OPEN if SMS read request is not open,
 * BUSY if SMS read request is not open, NO_ERROR
 * otherwise.
 */
wHalErrorCode w_hal_gsm_clear_sms_read_done();

/**
 * @brief Open a handle to a SMS request
 *
 * @param sms_size remaining characters in SMS request
 * @return wHalErrorCode BUSY if SMS request is already
 * open, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_write_sms_request(uint32_t &sms_size);

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
                                        uint32_t &  remaining_size);

/**
 * @brief Check if sms content for active handle is still uploading
 *
 * @return true if sms content for active handle is still uploading
 * @return false otherwise
 */
bool          w_hal_gsm_check_sms_content_uploading();

/**
 * @brief Close SMS request
 *
 * @return wHalErrorCode BUSY if hal is busy, GSM_SMS_REQUEST_NOT_OPEN if SMS
 * request is not open, GSM_SMS_REQUEST_ALREADY_CLOSING if SMS request is
 * already closing, NO_ERROR otherwise.
 */
wHalErrorCode w_hal_gsm_close_sms_write_request();

/**
 * @brief Check if post request for handle is not open
 *
 * @param handle handle for open post request
 * @return true if post request is still active
 * @return false if post request does not exist or has closed
 */
bool          w_hal_gsm_check_sms_request_closed();

/**
 * @brief Check if there are any available file handles
 *
 * @return uint32_t number of available file handles
 */
uint32_t      w_hal_gsm_get_available_post_requests();

/**
 * @brief Check if a post request can be opened
 *
 * @return wHalErrorCode GSM_TRANSFER_NO_HANDLE_AVAILABLE if no handle,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_check_can_open_post_request();

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
wHalErrorCode
              w_hal_gsm_open_post_request(const char *        p_api_route,
                                          http_content_type_t content_type,
                                          const char *        p_file_name,
                                          wGsmTransferHandle &handle,
                                          int total_size = INVALID_GSM_UPLOAD_SIZE);

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
                               wGsmTransferHandle handle);

/**
 * @brief Check if post request for handle is actively uploading
 *
 * @param handle handle for open post request
 * @return true if post request is still open and upload is happening for
 * handle
 * @return false otherwise
 */
bool          w_hal_gsm_check_active_upload(wGsmTransferHandle handle);

/**
 * @brief Close post request for handle
 *
 * @param handle handle for open post request
 * @return wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_close_post_request(wGsmTransferHandle handle);

/**
 * @brief Check if post request for handle is not open
 *
 * @param handle handle for open post request
 * @return true if post request is still active
 * @return false if post request does not exist or has closed
 */
bool          w_hal_gsm_check_post_request_closed(wGsmTransferHandle handle);

/**
 * @brief Run background activities for GSM HAL
 *
 * @return wHalErrorCode BUSY if there is work to be done, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_handler();

/**
 * @brief Print GSM Information obtained during Initialization
 */
void          w_hal_gsm_print_info();

/**
 * @brief Teardown of the GSM
 *
 * @return wHalErrorCode BUSY if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_teardown();

/**
 * @brief GSM file cleanup before shutting it down
 */
void          w_hal_gsm_cleanup(void);

/**
 * @brief Get the gsm modem name string
 *
 * @param p_modem_name[out]: GSM modem name string
 * @param modem_size[out]: size of the modem name string
 */
void w_hal_gsm_get_modem_name(const char *&p_modem_name, uint32_t &modem_size);

/**
 * @brief Get the gsm firmware version string
 *
 * @param p_fw_version[out]: GSM firmware version string
 * @param version_size[out]: size of firmware version string
 */
void w_hal_gsm_get_fw_version(const char *&p_fw_version,
                              uint32_t &   version_size);

/**
 * @brief Get the gsm imei number string
 *
 * @param p_imei[out]: imei number string
 * @param imei_size[out]: size of imei number string
 */
void w_hal_gsm_get_imei(const char *&p_imei, uint32_t &imei_size);

/**
 * @brief Get the gsm imsi number string
 *
 * @param p_imsi[out]: imsi number string
 * @param imsi_size[out]: size of imsi number string
 */
void w_hal_gsm_get_imsi(const char *&p_imsi, uint32_t &imsi_size);

/**
 * @brief Get the gsm iccid string
 *
 * @param p_iccid[out]: iccid string
 * @param iccid_size[out]: size of iccid string
 */
void w_hal_gsm_get_iccid(const char *&p_iccid, uint32_t &iccid_size);

/**
 * @brief Get the gsm operator name string
 *
 * @param p_operator_name[out]: operator name string
 * @param operator_size[out]: size of operator name string
 */
void w_hal_gsm_get_operator_name(const char *&p_operator_name,
                                 uint32_t &   operator_size);

/**
 * @brief Get the gsm gprs status
 *
 * @return uint8_t: Returns the gprs attachment status
 */
uint8_t  w_hal_gsm_get_gprs_status();

/**
 * @brief Get the gsm power status
 *
 * @return uint8_t: Returns the gsm power status
 */
uint8_t  w_hal_gsm_get_power_status();

/**
 * @brief Get the gsm sim status
 *
 * @return uint8_t: Returns SIM status
 */
uint8_t  w_hal_gsm_get_sim_status();

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
void     w_hal_gsm_get_signal_quality(int32_t &rssi,
                                      int32_t &rxqual,
                                      int32_t &rscp,
                                      int32_t &ecno,
                                      int32_t &rsrp,
                                      int32_t &rsrq);

/**
 * @brief Get the gsm carrier time
 *
 * @return time_t: Returns the Carrier time
 */
time_t   w_hal_gsm_get_carrier_time();

/**
 * @brief Get the gsm pdp address string
 *
 * @param p_pdp_address[out]: PDP address
 * @param pdp_address_size[out]: size of the PDP address string
 */
void     w_hal_gsm_get_pdp_address(const char *&p_pdp_address,
                                   uint32_t &   pdp_address_size);

/**
 * @brief Get the gsm location area code string
 *
 * @param p_location_area_code[out]: location area code
 * @param location_area_code_size[out]: size of the location area code string
 */
void     w_hal_gsm_get_location_area_code(const char *&p_location_area_code,
                                          uint32_t &   location_area_code_size);

/**
 * @brief Get the gsm cell id string
 *
 * @param p_cell_id[out]: cell id
 * @param cell_id_size[out]: size of the cell id string
 */
void     w_hal_gsm_get_cell_id(const char *&p_cell_id, uint32_t &cell_id_size);

/**
 * @brief To get the phone number from the SIM
 *
 * @param p_phone_number[out]: phone number
 * @param phone_number_size[out]: size of the phone number string
 */
void     w_hal_gsm_get_phone_number(const char *&p_phone_number,
                                    uint32_t &   phone_number_size);

/**
 * @brief To get the max file system size for post request
 *
 * @return uint32_t: Returns the max supported post request size
 */
uint32_t w_hal_gsm_get_max_post_request_size();

/**
 * @brief To enter to the sleep mode for gsm
 *
 * @param sleep flag to set for sleep mode
 * @return wHalErrorCode wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_gsm_sleep(bool sleep);

/**
 * @brief to check if gsm receives any SMS
 *
 * @return true: SMS received
 * @return false: SMS not received
 */
bool          w_hal_gsm_need_to_check_for_receive_sms();

/**
 * @brief To get the sleep state of an modem
 *
 * @return true: If it is in sleep
 * @return false: If it is in wakeup state
 */
bool          w_hal_gsm_get_sleep_state();

#endif // LIB_HAL_W_HAL_GSM_HPP