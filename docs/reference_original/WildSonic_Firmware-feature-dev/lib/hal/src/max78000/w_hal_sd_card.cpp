#if STORAGE_CORE != INVALID_CORE_NUM

#include "w_hal_sd_card.hpp"
#include "w_hal_error_code.hpp" //Header file to enum of errors

#include "device_config.hpp"
#include "trace.hpp"

#if __CORE__ == STORAGE_CORE

#include "w_hal_spi_shared.hpp"

#include "assert.hpp"

#include "ff.h" //Header file for file handling

#include "board.h"
#include "diskio.h"
#include "gpio.h"
#include "mxc_delay.h"  //Header file for MAX78000 delay
#include "mxc_device.h" //Header file for MAX78000 device
#include "uart.h"

#include <stdint.h>
#include <string.h>
#include <string>

static wHalErrorCode w_hal_sd_open_file(const char * p_file_name,
                                        uint32_t     file_name_length,
                                        wFileHandle &file_handle,
                                        BYTE         mode);

static wHalErrorCode
w_hal_sd_card_last_folder_file(char *                p_found_name,
                               uint32_t              found_name_length,
                               sd_is_file_valid_func is_file_folder_valid_func,
                               sd_compare_files_func compare_file_folder_func,
                               bool                  find_folder,
                               uint32_t &            found_size);

// Structure to store the file pointer and whether the file is closed or opened
struct file_t
{
    FIL  file; // FFat File Object
    bool open; // Variable to check whether the file is created : true-Open,
               // false-Close.
};

/***** Globals *****/
static FATFS          file_system_object;
static FILINFO        file_info_object; // FFat File Information Object
static DIR            folder;           // FFat Directory Object

static const uint32_t MAXLEN           = 256; // Maximum length of string
static char           volume           = '0';
static char           cwd[MAXLEN]      = "/";
static const uint32_t MIN_MEMORY_SPACE = 1000000; // In bytes
static const int      NUMBER_OF_FILES  = 2;

static DWORD          clusters_free    = 0; // Variable to store clusters free
static uint64_t       free_memory      = 0; // Variable to store free memory
static uint64_t       total_memory     = 0; // Variable to store total memory
static BYTE           work[4096];

static file_t         file_instance[NUMBER_OF_FILES]; // Variable to store the
                                                      // file pointer

static DeviceConfig::config_page_st *p_device_config;

/**
 * @brief : Unmount the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
static wHalErrorCode                 w_hal_sd_card_unmount()
{
    FRESULT err;

    // Unmount the default drive from its mount point
    if ((err = f_unmount("")) != FR_OK)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                          "Failed to unmount with FRESULT %d\n",
                          err);

        return (wHalErrorCode::SD_CARD_UNMOUNT_FAILED);
    }

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Initialises the SD card, Mounts the SD card and formats if
 * filesystem doesn't match
 *
 * @param device_config device configuration
 * @param p_time time variable to update
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_init(DeviceConfig::config_page_st &device_config,
                                 w_hal_rtc_time_st *           p_time)
{
    p_device_config = &device_config;

    FRESULT err;
    FATFS * file_system; // FFat Filesystem Object
    // Create file system object
    file_system = &file_system_object;

    if (SD_Power(0) != E_NO_ERROR)
    {
        return wHalErrorCode::SD_CARD_POWER_OFF_FAILED;
    }

    // Scale SPI frequency based off data rate. Always provide at least 2x the
    // data rate.
    const uint32_t data_bit_rate =
        DeviceConfig::get_data_bit_rate(device_config);
    disk_set_min_spi_frequency(data_bit_rate * 2);
    if (p_time != nullptr)
    {
        int time_zone_offset = device_config.time_zone_offset;
        disk_set_time_pointer(&p_time->time_seconds, time_zone_offset);
    }

    // If mounting failed mount to default or unmount
    if ((err = f_mount(file_system, "", 1)) != FR_OK)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                          "Failed to mount with FRESULT %d\n",
                          err);

        // Unmount the default drive from its mount point
        w_hal_sd_card_unmount();

        return (wHalErrorCode::SD_CARD_MOUNT_FAILED);
    }
    // Set the Current working directory
    f_getcwd(cwd, sizeof(cwd));

    memset(file_instance, 0, sizeof(file_instance));

    wHalErrorCode error = w_hal_spi_shared_initialize();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "SD Card Init Done\n");

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Unmount the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_tear_down()
{
    wHalErrorCode error = wHalErrorCode::BUSY;

    error               = w_hal_spi_shared_request_baton_for_sd_card();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_sd_card_handler();
    }

    error = w_hal_sd_card_unmount();

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    if (SD_Power(0) != E_NO_ERROR)
    {
        return wHalErrorCode::SD_CARD_POWER_OFF_FAILED;
    }

    // Release SPI baton
    error = wHalErrorCode::BUSY;

    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_spi_shared_release_baton_for_sd_card();
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief : Get the total size of SD card in bytes
 *
 * @return uint64_t : Return the size of total SD card memory in bytes. If
 * ZERO is returned, then error occurred while fetching the information.
 */
uint64_t w_hal_sd_card_get_size()
{
    FATFS *file_system; // FFat Filesystem Object
    // Create file system object
    file_system  = &file_system_object;
    // Get the total memory
    total_memory = static_cast<uint64_t>(file_system->n_fatent - 2) *
                   file_system->csize * 512;
    return (total_memory);
}

/**
 * @brief : Get the available free bytes in SD card
 *
 * @return uint64_t : Return the bytes left in SD card. If ZERO is returned,
 * then error occurred while fetching the information.
 */
uint64_t w_hal_sd_card_get_free_space()
{
    FRESULT err;
    FATFS * file_system; // FFat Filesystem Object
    // Create file system object
    file_system         = &file_system_object;

    wHalErrorCode error = w_hal_spi_shared_request_baton_for_sd_card();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Get the free memory
    if ((err = f_getfree(&volume, &clusters_free, &file_system)) != FR_OK)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                          "Failed to get free memory with FRESULT %d\n",
                          err);

        free_memory = 0;
    }
    else
    {
        // Calculate the free memory from the free clusters
        free_memory =
            static_cast<uint64_t>(clusters_free) * file_system->csize * 512;
    }

    error = w_hal_spi_shared_release_baton_for_sd_card();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    return (free_memory);
}

/**
 * @brief : Check there is sufficient memory is available to write one file
 *
 * @return true : Sufficient memory is available
 * @return false : Sufficient memory is not available
 */
bool w_hal_sd_card_check_space()
{
    // Return true if the memory available is more
    // than minimum required memory space
    return (w_hal_sd_card_get_free_space() > MIN_MEMORY_SPACE);
}

/**
 * @brief : Create folder in SD card
 *
 * @param p_folder_name : Pointer to name of the folder name
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 * */
wHalErrorCode w_hal_sd_card_create_folder(const char *p_folder_name,
                                          uint32_t    folder_name_length)
{
    FRESULT       err; // FFat Result (enum)
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        // Check folder created successfully
        if ((err = f_mkdir(p_folder_name)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to make directory with FRESULT %d\n",
                              err);

            if (err == FR_EXIST)
            {
                return_error = wHalErrorCode::SD_CARD_FOLDER_EXIST;
            }
            else
            {
                // Unmount the default drive from its mount point
                w_hal_sd_card_unmount();

                return_error = wHalErrorCode::SD_CARD_FOLDER_CREATE_FAILED;
            }
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }
    return return_error;
}

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
                                  sd_compare_files_func compare_folder_func)
{
    wHalErrorCode error_code;    // Variable to store the
                                 // w_hal_error_code enum type
    bool     find_folder = true; // Flag to identify to get last folder created
    uint32_t found_size  = 0;
    // Store the last folder created
    error_code           = w_hal_sd_card_last_folder_file(p_folder_name,
                                                folder_name_length,
                                                is_folder_valid_func,
                                                compare_folder_func,
                                                find_folder,
                                                found_size);

    return (error_code);
}

/**
 * @brief : Checks if file or directory exists
 *
 * @param p_name : Pointer to file or directory name
 * @param name_length : Length of the name string
 * @return wHalErrorCode : NO_ERROR if file or folder exists,
 * SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST otherwise
 */
wHalErrorCode w_hal_sd_card_check_exists(const char *p_name,
                                         uint32_t    name_length)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err; // FFat Result (enum)

        // Check whether file or folder exists
        if ((err = f_stat(p_name, &file_info_object)) == FR_NO_FILE)
        {
            return_error = wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST;
        }
        else
        {
            return_error = wHalErrorCode::NO_ERROR;
        }
        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief Add root slash to cwd if it is not already there
 */
static void add_root_slash_to_cwd()
{
    if (cwd[0] != '/')
    {
        uint32_t length = strlen(cwd);
        SYSTEM_ASSERT_LT(length, MAXLEN);
        memmove(&cwd[1], &cwd[0], length + 1);
        cwd[0] = '/';
    }
}

/**
 * @brief Make folder name one folder level up
 *
 * @param p_folder_name path to the folder
 * @param folder_name_length length of the folder name
 * @return char the char that was replaced with null
 */
static char change_to_one_folder_level_up(char *   p_folder_name,
                                          uint32_t folder_name_length)
{
    // Check that the path starts with a slash
    SYSTEM_ASSERT_EQUAL(p_folder_name[0], '/');

    // Set p_folder_name to the current working directory up to the last slash
    int path_name_length = folder_name_length;

    // Remove trailing slash since we are going up one level
    // Traverse to first slash in set of contiguous slashes
    while ((path_name_length > 0) &&
           (p_folder_name[path_name_length - 1] == '/'))
    {
        path_name_length--;
    }

    // Find last forward-slash
    for (; path_name_length > 0; path_name_length--)
    {
        if (p_folder_name[path_name_length - 1] == '/')
        {
            break;
        }
    }

    // Traverse to first slash in set of contiguous slashes
    while ((path_name_length > 0) &&
           (p_folder_name[path_name_length - 1] == '/'))
    {
        path_name_length--;
    }

    SYSTEM_ASSERT_GTEQ(path_name_length, 0);

    if (path_name_length == 0)
    {
        // If the path is just root slash, then set the length to 1
        SYSTEM_ASSERT_EQUAL(p_folder_name[0], '/');
        path_name_length = 1;
    }

    char previous_char              = p_folder_name[path_name_length];
    p_folder_name[path_name_length] = '\0';

    return previous_char;
}

/**
 * @brief : Giving the absolute path as the input to the directory, then
 * that will be the present working directory
 *
 * @param p_folder_name : Pointer to name of the folder
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_goto_folder(const char *p_folder_name,
                                        uint32_t    folder_name_length)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        bool traverse_up                  = false;
        char replaced_char_in_traverse_up = '\0';
        if ((strcmp(p_folder_name, "..") == 0) ||
            (strcmp(p_folder_name, "../") == 0))
        {
            // Make sure cwd starts with a slash
            add_root_slash_to_cwd();

            // Set p_folder_name to the current working directory up to the
            // last slash
            int path_name_length = strlen(cwd);
            replaced_char_in_traverse_up =
                change_to_one_folder_level_up(cwd, path_name_length);

            p_folder_name = cwd;
            traverse_up   = true;
        }

        // Check change directory is successful
        FRESULT err = f_chdir(p_folder_name);

        if (traverse_up)
        {
            // Restore the previous character in place of the null
            cwd[strlen(cwd)] = replaced_char_in_traverse_up;
        }

        if (err != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to change directory with FRESULT %d\n",
                              err);

            // Unmount the default drive from its mount point
            w_hal_sd_card_unmount();

            return_error = wHalErrorCode::SD_CARD_FOLDER_CHANGE_FAILED;
        }
        else
        {
            if (traverse_up)
            {
                // Set the current working directory up to the
                // last slash
                int path_name_length = strlen(cwd);
                change_to_one_folder_level_up(cwd, path_name_length);
            }
            else if (p_folder_name[0] == '/')
            {
                // overwrite path
                strcpy(cwd, p_folder_name);
            }
            else if ((strcmp(p_folder_name, ".") == 0) ||
                     (strcmp(p_folder_name, "./") == 0))
            {
            }
            else
            {
                int path_name_length = strlen(cwd);
                if (path_name_length > 0)
                {
                    if (cwd[path_name_length - 1] != '/')
                    {
                        cwd[path_name_length++] = '/';
                        cwd[path_name_length]   = '\0';
                    }
                }
                strcpy(&cwd[path_name_length], p_folder_name);
            }
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief : Deleting the folder in SD card
 *
 * @param p_folder_name : Pointer to name of the folder name
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_delete_folder(const char *p_folder_name,
                                          uint32_t    folder_name_length)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err; // FFat Result (enum)

        // Check detach or delete the folder is successful
        if ((err = f_unlink(p_folder_name)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to unlink directory with FRESULT %d\n",
                              err);

            if (err == FR_NO_FILE)
            {
                LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                        "Cannot unlink directory that does not exist!\n");

                return_error = wHalErrorCode::SD_CARD_FOLDER_DOES_NOT_EXIST;
            }
            else
            {
                return_error = wHalErrorCode::SD_CARD_FOLDER_DELETE_FAILED;
            }
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

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
                                        wFileHandle &file_handle)
{
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type
    BYTE          mode = (FA_CREATE_ALWAYS | FA_WRITE); // Set type of mode
    // Call function to create file
    error_code =
        w_hal_sd_open_file(p_file_name, file_name_length, file_handle, mode);

    return (error_code);
}

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
                                      wFileHandle &file_handle)
{
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type
    BYTE          mode = (FA_READ | FA_WRITE | FA_OPEN_ALWAYS |
                 FA_OPEN_EXISTING); // Set type of mode
    // Call function to open file
    error_code =
        w_hal_sd_open_file(p_file_name, file_name_length, file_handle, mode);

    return (error_code);
}

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
                                sd_compare_files_func compare_files_func)
{
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type
    bool find_folder = false; // Flag to identify to get last file created
    file_size        = 0;
    // Store the last file created
    error_code       = w_hal_sd_card_last_folder_file(p_file_name,
                                                file_name_length,
                                                is_file_valid_func,
                                                compare_files_func,
                                                find_folder,
                                                file_size);

    return (error_code);
}

/**
 * @brief : Close the open file
 *
 * @param file_handle : Pointer to the file object.
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_close_file(wFileHandle file_handle)
{
    wHalErrorCode return_error = wHalErrorCode::SD_CARD_FILE_NOT_OPEN;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err; // FFat Result (enum)

        // Find the file pointer in the file instance and close the file
        // pointer along with setting 'open' to false.
        for (uint32_t iter = 0; iter < NUMBER_OF_FILES; iter++)
        {
            if (file_instance[iter].open &&
                (&file_instance[iter].file == static_cast<FIL *>(file_handle)))
            {
                // Close the file
                if ((err = f_close(static_cast<FIL *>(file_handle))) != FR_OK)
                {
                    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                                      "Failed to close file with FRESULT %d\n",
                                      err);

                    return_error = wHalErrorCode::SD_CARD_FILE_CLOSE_FAILED;
                    break;
                }
                // Set file pointer as closed
                file_instance[iter].open = false;
                return_error             = wHalErrorCode::NO_ERROR;
                break;
            }
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief : Write handler for queued asynchronous writes
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_handler()
{
    if (f_write_handler())
    {
        return wHalErrorCode::BUSY;
    }
    else
    {
        w_hal_spi_shared_release_baton_for_sd_card();
        return wHalErrorCode::NO_ERROR;
    }
}

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
                                          wFileHandle  file_handle)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        UINT    bytes_written = 0; // Variable to store number of bytes written
        FRESULT err;               // FFat Result (enum)
        // Write data from 'p_buffer' into file
        if ((err = f_write_multiple(static_cast<FIL *>(file_handle),
                                    p_buffers,
                                    static_cast<UINT *>(buffer_sizes),
                                    number_of_buffers,
                                    &bytes_written)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to write file with FRESULT %d\n",
                              err);

            return_error = wHalErrorCode::SD_CARD_FILE_WRITE_FAILED;
        }

#if MXC_DISK_WRITE_ASYNC == 0
        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
#endif
    }

    return return_error;
}

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
                                                    unsigned int buffer_size)
{
    FRESULT result = f_get_write_buffer_status(p_buffer, buffer_size);

    if (result == FR_OK)
    {
        return wHalErrorCode::BUSY;
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
}

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
                                          wFileHandle  file_handle)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        UINT    bytes_written = 0; // Variable to store number of bytes written
        FRESULT err;               // FFat Result (enum)
        // Write data from 'p_buffer' into file
        if ((err = f_write(static_cast<FIL *>(file_handle),
                           p_buffer,
                           buffer_size,
                           &bytes_written)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to write file with FRESULT %d\n",
                              err);

            return_error = wHalErrorCode::SD_CARD_FILE_WRITE_FAILED;
        }

#if MXC_DISK_WRITE_ASYNC == 0
        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
#endif
    }

    return return_error;
}

/**
 * @brief : Sync the data of the buffer into the file
 *
 * @param p_buffer : Pointer to data buffer
 * @param buffer_size : Size of the data buffer
 * @param file_handle : Pointer to file object
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_sync_file(wFileHandle file_handle)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err; // FFat Result (enum)
        if ((err = f_sync(static_cast<FIL *>(file_handle))) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to sync file with FRESULT %d\n",
                              err);

            return_error = wHalErrorCode::SD_CARD_FILE_SYNC_FAILED;
        }

#if MXC_DISK_WRITE_ASYNC == 0
        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
#endif
    }

    return return_error;
}

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
                                           wFileHandle file_handle)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        bytes_read   = 0;
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err;    // FFat Result (enum)
        UINT    br = 0; // Variable to store number of bytes read
        // Read from file and store it in 'p_buffer'
        if ((err = f_read(static_cast<FIL *>(file_handle),
                          p_buffer,
                          buffer_size,
                          &br)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to read file with FRESULT %d\n",
                              err);

            // Unmount the default drive from its mount point
            w_hal_sd_card_unmount();

            bytes_read   = 0;

            return_error = wHalErrorCode::SD_CARD_FILE_READ_FAILED;
        }
        else
        {
            bytes_read = br;
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief : Function to move the file pointer to the given position in the file
 *
 * @param file_handle : Pointer to file object
 * @param offset : Offset from beginning of file to move the file pointer
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_seek(wFileHandle file_handle, uint32_t offset)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();

    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err; // FFat Result (enum)
        // Move the file pointer to the given offset
        if ((err = f_lseek(static_cast<FIL *>(file_handle), offset)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to seek file with FRESULT %d\n",
                              err);

            return_error = wHalErrorCode::SD_CARD_FILE_SEEK_FAILED;
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief : Delete particular file in the given the absolute path.
 *
 * @param p_file_name : Pointer to file name
 * @param file_name_length : Length of the file name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_delete_file(const char *p_file_name,
                                        uint32_t    file_name_length)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT err; // FFat Result (enum)

        // Delete or detach the file
        if ((err = f_unlink(p_file_name)) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to unlink file with FRESULT %d\n",
                              err);

            if (err == FR_NO_FILE)
            {
                LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                        "Cannot unlink file that does not exist!\n");

                return_error = wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST;
            }
            else
            {
                return_error = wHalErrorCode::SD_CARD_FILE_DELETE_FAILED;
            }
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}
/**
 * @brief : Format the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_format()
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT   err; // FFat Result (enum)

        // SD_card formatting parameter
        MKFS_PARM param;
        param.fmt     = FM_EXFAT;
        param.n_fat   = 0;
        param.align   = 0;
        param.n_root  = 0;
        param.au_size = 0;

        // Format the SD_card
        if ((err = f_mkfs("", &param, work, sizeof(work))) != FR_OK)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                              "Failed to mkfs with FRESULT %d\n",
                              err);

            return_error = wHalErrorCode::SD_CARD_FORMAT_FAILED;
        }
        else
        {
            // Mount the SD card
            w_hal_sd_card_init(*p_device_config, nullptr);

            // Set the label for the SD card
            f_setlabel("WILDSONIC");

            // Unmount the SD card
            w_hal_sd_card_tear_down();
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief : Common function to open or create file on the basis of the mode
 * selected
 *
 * @param p_file_name : Pointer to file name
 * @param file_name_length : Length of the file name string
 * @param file_handle [out] : Stores the file handle
 * @param mode : Mode in which file needs to be opened
 * @return wHalErrorCode : return enum type error code value
 */
static wHalErrorCode w_hal_sd_open_file(const char * p_file_name,
                                        uint32_t     file_name_length,
                                        wFileHandle &file_handle,
                                        BYTE         mode)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error        = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT  err; // FFat Result (enum)

        // Check whether there is free file handle and
        // maximum number of files are not opened
        uint32_t file_iter;
        for (file_iter = 0; file_iter < NUMBER_OF_FILES; file_iter++)
        {
            if (!(file_instance[file_iter].open))
            {
                break;
            }
        }

        if (file_iter == NUMBER_OF_FILES)
        {
            // Return too many file opened if the file pointer are more than
            // maximum number of files to be opened
            return_error = wHalErrorCode::SD_CARD_TOO_MANY_FILES_OPEN;
        }
        else
        {
            // Open file
            if ((err = f_open(&file_instance[file_iter].file,
                              p_file_name,
                              mode)) != FR_OK)
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                                  "Failed to open file with FRESULT %d\n",
                                  err);
                return_error = wHalErrorCode::SD_CARD_FILE_NOT_CREATED;

                if (mode & (FA_CREATE_ALWAYS | FA_CREATE_NEW))
                {
                    if (err == FR_EXIST)
                    {
                        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                                          "Cannot create file %s that already "
                                          "exists! FRESULT %d\n",
                                          p_file_name,
                                          err);

                        return_error = wHalErrorCode::SD_CARD_FILE_EXIST;
                    }
                }
                else
                {
                    if (err == FR_NO_FILE)
                    {
                        LOG_MSG_WITH_ARGS(
                            TraceFile::W_HAL_SD_CARD_CPP,
                            "Cannot open file that does not exist!"
                            " FRESULT %d\n",
                            err);

                        return_error =
                            wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST;
                    }
                }

                if (return_error == wHalErrorCode::SD_CARD_FILE_NOT_CREATED)
                {
                    // Otherwise here is a fatal error
                    // Unmount the default drive from its mount point
                    w_hal_sd_card_unmount();
                }
            }
        }

        if (return_error == wHalErrorCode::NO_ERROR)
        {
            // Set the file pointer as used
            file_instance[file_iter].open = true;
            file_handle =
                &file_instance[file_iter].file; // Set the file handle value
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief : Function to get the last folder or file created
 *
 * @param p_found_name : Pointer to file or folder name
 * @param found_name_length : Length of the file or folder name string
 * @param is_file_folder_valid_func : function to tell if file/folder is
 * valid
 * @param compare_file_folder_func : function to compare files/folders
 * @param find_folder : If the value is true then find the 'folder' or else
 * find the 'file' created
 * @param found_size [out] : Stores the size of the file if find_folder is false
 * @return wHalErrorCode : return enum type error code value
 */
static wHalErrorCode
w_hal_sd_card_last_folder_file(char *                p_found_name,
                               uint32_t              found_name_length,
                               sd_is_file_valid_func is_file_folder_valid_func,
                               sd_compare_files_func compare_file_folder_func,
                               bool                  find_folder,
                               uint32_t &            found_size)
{
    wHalErrorCode return_error; // Variable to store the
    // w_hal_error_code enum type
    found_size          = 0; // Variable to store the size of the file

    wHalErrorCode error = w_hal_spi_shared_request_baton_for_sd_card();
    if (error == wHalErrorCode::BUSY)
    {
        return_error = wHalErrorCode::BUSY;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        FRESULT           err; // FFat Result (enum)
        bool              found           = false;
        w_hal_file_info_t whal_found_info = {0};

        // Check opening folder
        if (f_opendir(&folder, ".") != FR_OK)
        {
            return_error = wHalErrorCode::SD_CARD_FOLDER_OPEN_FAILED;
        }
        else
        {
            // Loop until error occur or last file name is fetched
            while (true)
            {
                // Read the file information
                err = f_readdir(&folder, &file_info_object);

                // Check whether there is error while fetching or last file
                // is fetched already
                if (err != FR_OK)
                {
                    f_closedir(&folder);

                    return_error = wHalErrorCode::SD_CARD_FOLDER_DOES_NOT_EXIST;
                    break;
                }

                // Close directory if already last file is fetched
                if (file_info_object.fname[0] == 0)
                {
                    f_closedir(&folder);

                    // If the file is found, then return no error or else
                    // return no file in folder
                    if (found)
                    {
                        return_error = wHalErrorCode::NO_ERROR;
                    }
                    else
                    {
                        return_error = wHalErrorCode::SD_CARD_FOLDER_EMPTY;
                    }
                    break;
                }

                uint32_t          fname_length = strlen(file_info_object.fname);

                w_hal_file_info_t whal_file_info;
                whal_file_info.p_file_name      = file_info_object.fname;
                whal_file_info.file_name_length = fname_length;
                whal_file_info.date             = file_info_object.fdate;
                whal_file_info.time             = file_info_object.ftime;
                whal_file_info.size             = file_info_object.fsize;

                // Check whether this folder should be inspected
                if (!is_file_folder_valid_func(&whal_file_info))
                {
                    continue;
                }

                if (!found ||
                    compare_file_folder_func(&whal_file_info, &whal_found_info))
                {
                    bool is_folder = (file_info_object.fattrib & AM_DIR);
                    if ((find_folder && is_folder) ||
                        (!find_folder && !is_folder))
                    {
                        // Account for nullptr when checking length
                        SYSTEM_ASSERT_LT(fname_length, found_name_length);

                        // save found information
                        strcpy(p_found_name, file_info_object.fname);
                        found_size = file_info_object.fsize;
                        memcpy(&whal_found_info,
                               &whal_file_info,
                               sizeof(whal_file_info));
                        whal_found_info.p_file_name = p_found_name;
                        found                       = true;
                    }
                }
            }
        }

        error = w_hal_spi_shared_release_baton_for_sd_card();
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return return_error;
}

/**
 * @brief Get the file format of the sd card
 *
 * @param p_format[out] : This is the format of the file in string
 * @param format_size : Size of the file format string
 */
void w_hal_sd_card_get_file_system_format(char *p_format, uint32_t format_size)
{
    switch (file_system_object.fs_type)
    {
        case FS_FAT12:
        {
            strncpy(p_format, "FAT12", format_size);
            break;
        }
        case FS_FAT16:
        {
            strncpy(p_format, "FAT16", format_size);
            break;
        }
        case FS_FAT32:
        {
            strncpy(p_format, "FAT32", format_size);
            break;
        }
        case FS_EXFAT:
        {
            strncpy(p_format, "EXFAT", format_size);
            break;
        }
        default:
        {
            break;
        }
    }
}

#endif // __CORE__ == STORAGE_CORE

/**
 * @brief Validate sd card configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_sd_card_validate_config(DeviceConfig::config_page_st &device_config,
                              bool                          fix)
{
    // Make sure file size does not exceed 4GB.
    uint64_t recording_length = device_config.recording_length;
    uint64_t max_file_duration =
        DeviceConfig::get_max_file_duration(device_config);

    if (recording_length > max_file_duration)
    {
        if (fix)
        {
            // Set the number of seconds per file to max file duration.
            device_config.recording_length = max_file_duration;
        }
        else
        {
            LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                    "File size exceeds 4GB limit.\n");

            return wHalErrorCode::SD_CARD_FILE_SIZE_EXCEEDS_4GB;
        }
    }

    return wHalErrorCode::NO_ERROR;
}

#endif // STORAGE_CORE != INVALID_CORE_NUM