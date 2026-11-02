#ifndef LIB_HAL_INCLUDE_W_HAL_SD_CARD_HPP
#define LIB_HAL_INCLUDE_W_HAL_SD_CARD_HPP

#include "device_config.hpp"

#include "w_hal_error_code.hpp"
#include "w_hal_rtc.hpp"

#include <stdint.h>

using wFileHandle = void *;

/**
 * @brief
 * File information object from hal
 */
struct w_hal_file_info_t
{
    char *   p_file_name;      // Pointer to filename
    uint32_t file_name_length; // Filename length (not
                               // including null-terminator)
    uint32_t date;             // Date value
    uint32_t time;             // Time value
    uint32_t size;             // File size
};

// Function pointers for callbacks
using sd_is_file_valid_func = bool (*)(const w_hal_file_info_t *p_file_info);
using sd_compare_files_func = bool (*)(const w_hal_file_info_t *p_file_info_1,
                                       const w_hal_file_info_t *p_file_info_2);

/**
 * @brief Validate sd card configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_sd_card_validate_config(DeviceConfig::config_page_st &device_config,
                              bool                          fix);

/**
 * @brief : Initialises the SD card, Mounts the SD card and formats if
 * filesystem doesn't match
 *
 * @param device_config device configuration
 * @param p_time time variable to update
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_init(DeviceConfig::config_page_st &device_config,
                                 w_hal_rtc_time_st *           p_time);

/**
 * @brief : Unmount the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_tear_down();

/**
 * @brief : Get the total size of SD card in bytes
 *
 * @return uint64_t : Return the size of total SD card memory in bytes. If ZERO
 * is returned, then error occurred while fetching the information.
 */
uint64_t      w_hal_sd_card_get_size();

/**
 * @brief : Get the available free bytes in SD card
 *
 * @return uint64_t : Return the bytes left in SD card. If ZERO is returned,
 * then error occurred while fetching the information.
 */
uint64_t      w_hal_sd_card_get_free_space();

/**
 * @brief : Check there is sufficient memory is available to write one file
 *
 * @return true : Sufficient memory is available
 * @return false : Sufficient memory is not available
 */
bool          w_hal_sd_card_check_space();

/**
 * @brief : Create folder in SD card
 *
 * @param p_folder_name : Pointer to name of the folder name
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 * */
wHalErrorCode w_hal_sd_card_create_folder(const char *p_folder_name,
                                          uint32_t    folder_name_length);

/**
 * @brief : Find the last folder created
 *
 * @param p_folder_name : Pointer to store folder name
 * @param folder_name_length : Length of the folder name string
 * @param is_file_valid_func : function to tell if folder is valid
 * @param compare_files_func : function to compare folder
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
              w_hal_sd_card_last_folder_created(char *                p_folder_name,
                                                uint32_t              folder_name_length,
                                                sd_is_file_valid_func is_folder_valid_func,
                                                sd_compare_files_func compare_folder_func);

/**
 * @brief : Checks if file or directory exists
 *
 * @param p_name : Pointer to file or directory name
 * @param name_length : Length of the name string
 * @return wHalErrorCode : NO_ERROR if file or folder exists,BUSY if cannot
 * service request, SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST otherwise
 */
wHalErrorCode w_hal_sd_card_check_exists(const char *p_name,
                                         uint32_t    name_length);

/**
 * @brief : Giving the absolute path as the input to the directory, then that
 * will be the present working directory
 *
 * @param p_folder_name : Pointer to name of the folder
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_goto_folder(const char *p_folder_name,
                                        uint32_t    folder_name_length);

/**
 * @brief : Deleting the folder in SD card
 *
 * @param p_folder_name : Pointer to name of the folder name
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_delete_folder(const char *p_folder_name,
                                          uint32_t    folder_name_length);

/**
 * @brief : creates and opens file.
 *
 * @param p_file_name : Pointer to file name
 * @param file_name_length : Length of the file name string
 * @param file_handle [out] : Stores the file handle
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_create_file(const char * p_file_name,
                                        uint32_t     file_name_length,
                                        wFileHandle &file_handle);

/**
 * @brief : opens file.
 *
 * @param p_file_name : Pointer to file name
 * @param file_name_length : Length of the file name string
 * @param file_handle [out] : Stores the file handle
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_open_file(const char * p_file_name,
                                      uint32_t     file_name_length,
                                      wFileHandle &file_handle);

/**
 * @brief : Get the last file created
 *
 * @param p_file_name [out]: Pointer to store file name
 * @param file_name_length [out]: Length of the file name string
 * @param file_size [out]: Size of the file
 * @param is_file_valid_func : function to tell if file is valid
 * @param compare_files_func : function to compare files
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
              w_hal_sd_card_last_file_created(char *                p_file_name,
                                              uint32_t              file_name_length,
                                              uint32_t &            file_size,
                                              sd_is_file_valid_func is_file_valid_func,
                                              sd_compare_files_func compare_files_func);

/**
 * @brief : Close the open file
 *
 * @param file_handle : Pointer to the file object.
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_close_file(wFileHandle file_handle);

/**
 * @brief : Write handler for queued asynchronous writes
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_handler();

/**
 * @brief : Write the data of the buffer into the file
 *
 * @param p_buffers : Pointer to data buffer
 * @param buffer_sizes : Size of the data buffer
 * @param number_of_buffers : Number of buffers
 * @param file_handle : Pointer to file object
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_write_to_file(const void * p_buffers[],
                                          unsigned int buffer_sizes[],
                                          unsigned int number_of_buffers,
                                          wFileHandle  file_handle);

/**
 * @brief : Get the status of the write buffer (whether it is still being used
 * or not)
 *
 * @param p_buffer : Pointer to data buffer
 * @param buffer_size : Size of the data buffer
 * @return wHalErrorCode : BUSY if buffer is still being used, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_sd_card_get_write_buffer_status(const void * p_buffer,
                                                    unsigned int buffer_size);

/**
 * @brief : Write the data of the buffer into the file
 *
 * @param p_buffer : Pointer to data buffer
 * @param buffer_size : Size of the data buffer
 * @param file_handle : Pointer to file object
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_write_to_file(const void * p_buffer,
                                          unsigned int buffer_size,
                                          wFileHandle  file_handle);

/**
 * @brief : Sync the data of the buffer into the file
 *
 * @param p_buffer : Pointer to data buffer
 * @param buffer_size : Size of the data buffer
 * @param file_handle : Pointer to file object
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_sync_file(wFileHandle file_handle);

/**
 * @brief : Function to read the data of the file and send the pointer of
 * the buffer
 *
 * @param p_buffer : Pointer to data buffer
 * @param buffer_size : Size of the data buffer
 * @param bytes_read [out] : Stores the number of bytes read
 * @param file_handle : Pointer to file object
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_read_from_file(void *      p_buffer,
                                           uint32_t    buffer_size,
                                           uint32_t &  bytes_read,
                                           wFileHandle file_handle);

/**
 * @brief : Function to move the file pointer to the given position in the file
 *
 * @param file_handle : Pointer to file object
 * @param offset : Offset from beginning of file to move the file pointer
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_seek(wFileHandle file_handle, uint32_t offset);

/**
 * @brief : Delete particular file in the given the absolute path.
 *
 * @param p_file_name : Pointer to file name
 * @param file_name_length : Length of the file name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_delete_file(const char *p_file_name,
                                        uint32_t    file_name_length);

/**
 * @brief Get the file system format of the sd card
 *
 * @param p_format[out] : This is the format of the file in string
 * @param format_size : Size of the file format string
 */
void w_hal_sd_card_get_file_system_format(char *p_format, uint32_t format_size);

/**
 * @brief : Format the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_format();

#endif // LIB_HAL_INCLUDE_W_HAL_SD_CARD_HPP
