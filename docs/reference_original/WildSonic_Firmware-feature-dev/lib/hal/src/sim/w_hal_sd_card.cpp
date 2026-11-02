#include "w_hal_sd_card.hpp"
#include "w_hal_error_code.hpp" //Header file to enum of errors

#include "WAVFile.hpp"
#include "assert.hpp"
#include "constants.hpp"
#include "shared.hpp"
#include "trace.hpp"

#include <bits/stdc++.h>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace std;

// Structure to store the file pointer and whether the file is closed or opened
struct file_t
{
    bool     open; // Variable to check whether the file is created : true-Open,
                   // false-Close.
    uint32_t remainingExpectedBytes; // Remaining sample bytes left in file
};

/***** Globals *****/

static ofstream trace_ofstream; // Variable to store the
// trace file pointer
static file_t   trace_file;
static file_t   file;         // Variable to store the
                              // file pointer
static uint16_t data_counter; // Counter to use to check
                              // sample value against

const uint32_t  DIR_LENGTH = 2048;
static char     root_folder[DIR_LENGTH];

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
    return wHalErrorCode::NO_ERROR;
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
    file.open                         = false;
    file.remainingExpectedBytes       = 0;

    trace_file.open                   = false;
    trace_file.remainingExpectedBytes = static_cast<uint32_t>(-1);

    data_counter                      = 0;

    getcwd(root_folder, DIR_LENGTH);

    int length = strlen(root_folder);

    umask(0);

    int err = chdir("build/out/SIM/");
    SYSTEM_ASSERT_EQUAL(err, 0);

    getcwd(root_folder, DIR_LENGTH);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP,
                      "Current working directory: %s\n",
                      root_folder);

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Unmount the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_tear_down()
{
    SYSTEM_ASSERT_FALSE(file.open);
    SYSTEM_ASSERT_EQUAL(file.remainingExpectedBytes, 0);

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Get the total size of SD card in bytes
 *
 * @return uint64_t : Return the size of total SD card memory in bytes. If ZERO
 * is returned, then error occurred while fetching the information.
 */
uint64_t w_hal_sd_card_get_size()
{
    return SIZE_1GB;
}

/**
 * @brief : Get the available free bytes in SD card
 *
 * @return uint64_t : Return the bytes left in SD card. If ZERO is returned,
 * then error occurred while fetching the information.
 */
uint64_t w_hal_sd_card_get_free_space()
{
    return w_hal_sd_card_get_size();
}

/**
 * @brief : Check there is sufficient memory is available to write one file
 *
 * @return true : Sufficient memory is available
 * @return false : Sufficient memory is not available
 */
bool w_hal_sd_card_check_space()
{
    return (true);
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
    int err = mkdir((std::string("./") + p_folder_name).c_str(),
                    S_IRWXU | S_IRWXG | S_IRWXO);

    if (err == -1)
    {
        int errsv = errno;
        if (errsv == EEXIST)
        {
            return wHalErrorCode::SD_CARD_FOLDER_EXIST;
        }
        else
        {
            return wHalErrorCode::SD_CARD_FOLDER_CREATE_FAILED;
        }
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
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
    bool              found = false;
    w_hal_file_info_t whal_found_info;
    found_size = 0;

    struct dirent *d;
    DIR *          dr;
    dr = opendir(".");
    if (dr != nullptr)
    {
        for (d = readdir(dr); d != nullptr; d = readdir(dr))
        {
            uint32_t          fname_length = strlen(d->d_name);

            w_hal_file_info_t whal_file_info;
            whal_file_info.p_file_name      = d->d_name;
            whal_file_info.file_name_length = strlen(d->d_name) + 1;
            whal_file_info.date             = 0;
            whal_file_info.time             = 0;
            whal_file_info.size             = 1;

            if (!is_file_folder_valid_func(&whal_file_info))
            {
                continue;
            }

            if (!found ||
                compare_file_folder_func(&whal_file_info, &whal_found_info))
            {
                bool is_folder = (d->d_type == DT_DIR);
                if ((find_folder && is_folder) || (!find_folder && !is_folder))
                {
                    // Account for nullptr when checking length
                    SYSTEM_ASSERT_LT(fname_length, found_name_length);

                    // save found information
                    strcpy(p_found_name, d->d_name);
                    memcpy(&whal_found_info,
                           &whal_file_info,
                           sizeof(whal_file_info));
                    whal_found_info.p_file_name = p_found_name;
                    found                       = true;
                }
            }
        }
        closedir(dr);
    }

    if (found)
    {
        return wHalErrorCode::NO_ERROR;
    }
    else
    {
        return wHalErrorCode::SD_CARD_FOLDER_EMPTY;
    }
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
    struct stat buf;
    int         err = stat(p_name, &buf);

    if (err != 0)
    {
        return wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST;
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
}

/**
 * @brief : Giving the absolute path as the input to the directory, then that
 * will be the present working directory
 *
 * @param p_folder_name : Pointer to name of the folder
 * @param folder_name_length : Length of the folder name string
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_goto_folder(const char *p_folder_name,
                                        uint32_t    folder_name_length)
{
    int err;
    if (strcmp(p_folder_name, "/") == 0)
    {
        err = chdir(root_folder);
    }
    else
    {
        err = chdir(p_folder_name);
    }

    if (err != 0)
    {
        return (wHalErrorCode::SD_CARD_FOLDER_CHANGE_FAILED);
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
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
    return (wHalErrorCode::NOT_IMPLEMENTED);
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
    if (strstr(p_file_name, "_log.bin"))
    {
        SYSTEM_ASSERT_FALSE(trace_file.open);
        SYSTEM_ASSERT_EQUAL(trace_file.remainingExpectedBytes,
                            static_cast<uint32_t>(-1));

        ifstream ifile;
        ifile.open(p_file_name, ios::binary | ios::in);
        if (ifile)
        {
            ifile.close();
            return wHalErrorCode::SD_CARD_FILE_EXIST;
        }

        trace_ofstream.open(p_file_name, ios::binary | ios::out | ios::ate);

        if (!trace_ofstream.is_open())
        {
            return wHalErrorCode::SD_CARD_FILE_NOT_CREATED;
        }

        trace_file.open = true;
        file_handle     = &trace_file;
    }
    else
    {
        SYSTEM_ASSERT_FALSE(file.open);
        SYSTEM_ASSERT_EQUAL(file.remainingExpectedBytes, 0);

        file.open   = true;

        // setup file handle
        file_handle = &file;
    }

    return wHalErrorCode::NO_ERROR;
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
    if (strstr(p_file_name, "_log.bin"))
    {
        SYSTEM_ASSERT_FALSE(trace_file.open);
        SYSTEM_ASSERT_EQUAL(trace_file.remainingExpectedBytes,
                            static_cast<uint32_t>(-1));

        ifstream ifile;
        ifile.open(p_file_name, ios::binary | ios::in);
        if (!ifile)
        {
            ifile.close();
            return wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST;
        }

        trace_ofstream.open(p_file_name, ios::binary | ios::out | ios::app);

        if (!trace_ofstream.is_open())
        {
            return wHalErrorCode::SD_CARD_FILE_NOT_OPEN;
        }
        else
        {
            trace_file.open = true;
            file_handle     = &trace_file;
            return wHalErrorCode::NO_ERROR;
        }
    }
    else
    {
        return wHalErrorCode::NOT_IMPLEMENTED;
    }
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
    SYSTEM_ASSERT_EQUAL(file_handle, static_cast<wFileHandle>(&file));
    SYSTEM_ASSERT_TRUE(file.open);
    SYSTEM_ASSERT_EQUAL(file.remainingExpectedBytes, 0);

    file.open = false;

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Write handler for queued asynchronous writes
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_handler()
{
    return wHalErrorCode::NO_ERROR;
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

    if (file_handle == &trace_file)
    {
        SYSTEM_ASSERT_TRUE(trace_file.open);

        for (uint32_t index = 0; index < number_of_buffers; index++)
        {
            trace_ofstream.write(
                reinterpret_cast<const char *>(p_buffers[index]),
                buffer_sizes[index]);
        }
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(file_handle, static_cast<wFileHandle>(&file));
        SYSTEM_ASSERT_TRUE(file.open);

        uint32_t data_offset = 0; // start of data in buffer

        if (file.remainingExpectedBytes == 0)
        {
            const wav_header_t *p_wav_header =
                static_cast<const wav_header_t *>(p_buffers[0]);

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

            result = wav_file_verify_data_size(
                *p_wav_header,
                DeviceConfig::get_wav_file_sample_data_size());

            SYSTEM_ASSERT_TRUE(result);

            file.remainingExpectedBytes =
                DeviceConfig::get_wav_file_sample_data_size();

            data_offset += sizeof(wav_header_t);
        }

        for (uint32_t index = 0; index < number_of_buffers; index++)
        {
            file.remainingExpectedBytes -= buffer_sizes[index] - data_offset;

            const uint16_t *p_sample = reinterpret_cast<const uint16_t *>(
                static_cast<const uint8_t *>(p_buffers[index]) + data_offset);

            for (; data_offset < buffer_sizes[index];
                 data_offset += sizeof(uint16_t))
            {
                SYSTEM_ASSERT_EQUAL(*p_sample, data_counter);

                p_sample++;
                data_counter++;
            }

            data_offset = 0;
        }
    }

    return (wHalErrorCode::NO_ERROR);
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
    const void * buffers[1];
    unsigned int buffer_sizes[1];

    buffers[0]      = p_buffer;
    buffer_sizes[0] = buffer_size;

    return w_hal_sd_card_write_to_file(buffers, buffer_sizes, 1, file_handle);
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
    if (file_handle == &trace_file)
    {
        SYSTEM_ASSERT_TRUE(trace_file.open);

        trace_ofstream.flush();
    }

    return wHalErrorCode::NO_ERROR;
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
    return (wHalErrorCode::NOT_IMPLEMENTED);
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
    return (wHalErrorCode::NOT_IMPLEMENTED);
}
/**
 * @brief : Format the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_format()
{
    return (wHalErrorCode::NOT_IMPLEMENTED);
}

/**
 * @brief Get the file format of the sd card
 *
 * @param p_format[out] : This is the format of the file in string
 * @param format_size : Size of the file format string
 */
void w_hal_sd_card_get_file_system_format(char *p_format, uint32_t format_size)
{
    strncpy(p_format, "EXFAT", format_size);
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
    return wHalErrorCode::NO_ERROR;
}