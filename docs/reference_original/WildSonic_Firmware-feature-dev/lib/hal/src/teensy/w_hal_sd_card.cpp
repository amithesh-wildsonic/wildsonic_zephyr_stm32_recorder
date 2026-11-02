#if STORAGE_CORE != INVALID_CORE_NUM

#include "w_hal_sd_card.hpp"
#include "w_hal_error_code.hpp" //Header file to enum of errors

#include "device_config.hpp"
#include "trace.hpp"

#if __CORE__ == STORAGE_CORE

#include "constants.hpp"

#include "SdFs.h" //SD card library

#include <cstdio>
#include <stdint.h>
#include <string.h>

static wHalErrorCode w_hal_sd_open_file(const char * p_file_name,
                                        uint32_t     file_name_length,
                                        wFileHandle &file_handle,
                                        uint8_t      mode);

static wHalErrorCode
w_hal_sd_card_last_folder_file(char *                p_found_name,
                               uint32_t              found_name_length,
                               sd_is_file_valid_func is_file_folder_valid_func,
                               sd_compare_files_func compare_file_folder_func,
                               bool                  find_folder,
                               uint32_t &            found_size);

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif // HAS_SDIO_CLASS

static SdFs   sd_card_inst;
static FsFile folder_inst;
static FsFile file_inst; // File Object

// Structure to store the file pointer and whether the file is closed or opened
struct file_t
{
    FsFile file; // File Object
    bool   open; // Variable to check whether the file
                 // is created : true-Open, false-Close.
};

static const uint32_t MAXLEN = 256;
static uint64_t       total_memory;
static uint64_t       free_memory;
static const uint32_t MIN_MEMORY_SPACE          = 1000000; // In bytes
static char           cwd[MAXLEN]               = "/";
static char           prev_wd[MAXLEN]           = "/";
static const uint32_t NUMBER_OF_FILES           = 3;
static const uint32_t SECTOR_SIZE               = SECTOR_SIZE_512;
static const uint32_t MIN_EXT_CARD_SECTOR_COUNT = 32 * SIZE_1GB / SECTOR_SIZE;
static uint8_t        sectorBuffer[512];
static file_t         file_instance[NUMBER_OF_FILES]; // Variable to store
                                                      // the file pointers

/**
 * @brief : Unmount the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
static wHalErrorCode  w_hal_sd_card_unmount()
{
    sd_card_inst.end();

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
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type
    // Mount the sd card with the following configuration
    if (!(sd_card_inst.begin(SD_CONFIG)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "Failed to mount!\n");

        // Unmount the default drive from its mount point
        w_hal_sd_card_unmount();

        return (wHalErrorCode::SD_CARD_MOUNT_FAILED);
    }

    error_code = w_hal_sd_card_goto_folder(cwd, MAXLEN);

    for (int iter = 0; iter < NUMBER_OF_FILES; iter++)
    {
        file_instance[iter].open = false;
    }

    LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "SD Card Init Done\n");

    return (error_code);
}

/**
 * @brief : Unmount the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_tear_down()
{
    return w_hal_sd_card_unmount();
}

/**
 * @brief : Get the total size of SD card in bytes
 *
 * @return uint64_t : Return the size of total SD card memory in bytes. If
 * ZERO is returned, then error occurred while fetching the information.
 */
uint64_t w_hal_sd_card_get_size()
{
    // Get the total clusters in the volume
    total_memory = sd_card_inst.clusterCount();

    // Get the total memory in the volume
    total_memory =
        SECTOR_SIZE * total_memory * sd_card_inst.sectorsPerCluster();

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
    // Get the free clusters in the volume
    free_memory = sd_card_inst.freeClusterCount();

    // Get the free memory in the volume
    free_memory = SECTOR_SIZE * free_memory * sd_card_inst.sectorsPerCluster();

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
    // Create folder
    if (!(sd_card_inst.mkdir(p_folder_name)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "Failed to make directory\n");

        // Unmount the default drive from its mount point
        w_hal_sd_card_unmount();

        return (wHalErrorCode::SD_CARD_FOLDER_CREATE_FAILED);
    }

    return (wHalErrorCode::NO_ERROR);
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
    FRESULT err; // FFat Result (enum)

    // Return error if file nor folder does not exists
    if (!sd_card_inst.exists(p_name))
    {
        return (wHalErrorCode::SD_CARD_FILE_OR_FOLDER_DOES_NOT_EXIST);
    }

    return wHalErrorCode::NO_ERROR;
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
    // Send error if change directory is not successful
    if (!(sd_card_inst.chdir(p_folder_name)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "Failed to change directory!\n");

        // Unmount the default drive from its mount point
        w_hal_sd_card_unmount();

        return (wHalErrorCode::SD_CARD_FOLDER_CHANGE_FAILED);
    }

    // Save previous working directory
    strcpy(prev_wd, cwd);

    if (p_folder_name[0] == '/')
    {
        // overwrite path
        strcpy(cwd, p_folder_name);
    }
    else if ((strcmp(p_folder_name, ".") == 0) ||
             (strcmp(p_folder_name, "./") == 0))
    {
    }
    else if ((strcmp(p_folder_name, "..") == 0) ||
             (strcmp(p_folder_name, "../") == 0))
    {
        int path_name_length = strlen(cwd);

        // Find last forward-slash
        for (; path_name_length > 0; path_name_length--)
        {
            if (cwd[path_name_length] == '/')
            {
                break;
            }
        }

        // Traverse to first slash in set of contiguous slashes
        while ((path_name_length > 0) && (cwd[path_name_length - 1] == '/'))
        {
            path_name_length--;
        }

        SYSTEM_ASSERT_GTEQ(path_name_length, 0);
        cwd[path_name_length] = '\0';
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

    return (wHalErrorCode::NO_ERROR);
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
    // Return error if folder does not exists
    if (!sd_card_inst.exists(p_folder_name))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                "Cannot remove directory that does not exist!\n");

        return (wHalErrorCode::SD_CARD_FOLDER_DOES_NOT_EXIST);
    }

    if (!(sd_card_inst.rmdir(p_folder_name)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "Failed to remove directory!\n");

        return (wHalErrorCode::SD_CARD_FOLDER_DELETE_FAILED);
    }

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : creates and opens file.
 *
 * @param p_file_name : Pointer to file name
 * @param file_name_length : Length of the file name string
 * @param file_handle [out] : Stores the file handle
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_create_file(const char * p_folder_name,
                                        uint32_t     folder_name_length,
                                        const char * p_file_name,
                                        uint32_t     file_name_length,
                                        wFileHandle &file_handle)
{
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type
    uint8_t       mode = (O_CREAT | O_TRUNC | O_RDWR); // Set type of mode
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
    wHalErrorCode error_code;                 // Variable to store the
                                              // w_hal_error_code enum type
    uint8_t       mode = (O_RDWR | O_AT_END); // Set type of mode
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
    // Find the file pointer in the file instance and close the file pointer
    // along with setting 'open' to false.
    for (uint32_t iter = 0; iter < NUMBER_OF_FILES; iter++)
    {
        if (file_instance[iter].open &&
            (&file_instance[iter].file == static_cast<FsFile *>(file_handle)))
        {
            // Close the file
            if (!(file_instance[iter].file.close()))
            {
                LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                        "Failed to close file!\n");

                return (wHalErrorCode::SD_CARD_FILE_CLOSE_FAILED);
            }
            // Set file pointer as closed
            file_instance[iter].open = false;
            return (wHalErrorCode::NO_ERROR);
        }
    }
    return (wHalErrorCode::SD_CARD_FILE_NOT_OPEN);
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
 * @param p_buffer : Pointer to data buffer
 * @param buffer_size : Size of the data buffer
 * @param file_handle : Pointer to file object
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_write_to_file(const void *p_buffer,
                                          uint32_t    buffer_size,
                                          wFileHandle file_handle)
{
    size_t  bytes_written = 0; // Variable to store
                               // number of bytes written
    // Convert void pointer to type "FsFile"
    FsFile *p_file        = static_cast<FsFile *>(file_handle);
    // Write to file
    bytes_written         = p_file->write(p_buffer, buffer_size);
    if (bytes_written == buffer_size)
    {
        return (wHalErrorCode::NO_ERROR);
    }
    return (wHalErrorCode::SD_CARD_FILE_WRITE_FAILED);
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
    // Convert void pointer to type "FsFile"
    FsFile *p_file = static_cast<FsFile *>(file_handle);
    // read from file
    bytes_read     = p_file->read(p_buffer, buffer_size);
    // Check if there is error while reading cause it needs to return '0' as
    // there was no error along end of line reached and less than '0' if error
    // occur
    if (bytes_read == (size_t)(-1))
    {
        // Unmount the default drive from its mount point
        w_hal_sd_card_unmount();

        return (wHalErrorCode::SD_CARD_FILE_READ_FAILED);
    }

    return (wHalErrorCode::NO_ERROR);
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
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type

    // Check file existence
    if (!(sd_card_inst.exists(p_file_name)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                "Cannot unlink file that does not exist!\n");

        return (wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST);
    }

    // Delete the file
    if (!(sd_card_inst.remove(p_file_name)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "Failed to remove file!\n");

        return (wHalErrorCode::SD_CARD_FILE_DELETE_FAILED);
    }

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Format the SD card
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_sd_card_format()
{
    uint32_t       cardSectorCount = 0;

    ExFatFormatter exFatFormatter;
    FatFormatter   fatFormatter;
    SdCard *       m_card = nullptr;

    // SdCardFactory constructs and initializes the appropriate card.
    SdCardFactory  cardFactory;
    // Select and initialize proper card driver.
    m_card = cardFactory.newCard(SD_CONFIG);
    if (!m_card || m_card->errorCode())
    {
        return (wHalErrorCode::SD_CARD_FORMAT_FAILED);
    }

    cardSectorCount = m_card->sectorCount();

    if (!cardSectorCount)
    {
        return (wHalErrorCode::SD_CARD_FORMAT_FAILED);
    }

    // Format exFAT if larger than 32GB.
    bool rtn = cardSectorCount > MIN_EXT_CARD_SECTOR_COUNT ?
                   exFatFormatter.format(m_card, sectorBuffer, &Serial) :
                   fatFormatter.format(m_card, sectorBuffer, &Serial);

    if (!rtn)
    {
        return (wHalErrorCode::SD_CARD_FORMAT_FAILED);
    }

    return (wHalErrorCode::NO_ERROR);
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
                                        uint8_t      mode)
{
    wHalErrorCode error_code; // Variable to store the
                              // w_hal_error_code enum type

    // Check whether there is free file handle and
    // maximum number of files are not opened
    uint32_t      file_iter;
    for (file_iter = 0; file_iter < NUMBER_OF_FILES; file_iter++)
    {
        if (!(file_instance[file_iter].open))
        {
            break;
        }
    }

    if (file_iter == NUMBER_OF_FILES)
    {
        // Return too many file opened if the file pointer are more than maximum
        // number of files to be opened
        return (wHalErrorCode::SD_CARD_TOO_MANY_FILES_OPEN);
    }

    // Check file existence
    bool exists = sd_card_inst.exists(p_file_name);

    if (mode & O_CREAT)
    {
        if (exists)
        {
            LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                    "Cannot create file that already exists!\n");

            return (wHalErrorCode::SD_CARD_FILE_EXIST);
        }
    }
    else
    {
        if (!exists)
        {
            LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP,
                    "Cannot open file that does not exist!\n");

            return (wHalErrorCode::SD_CARD_FILE_DOES_NOT_EXIST);
        }
    }

    // Open file
    if (!(file_instance[file_iter].file.open(&sd_card_inst, p_file_name, mode)))
    {
        LOG_MSG(TraceFile::W_HAL_SD_CARD_CPP, "Failed to open file\n");

        // Unmount the default drive from its mount point
        w_hal_sd_card_unmount();

        return (wHalErrorCode::SD_CARD_FILE_NOT_CREATED);
    }

    // Set the file pointer as used
    file_instance[file_iter].open = true;
    file_handle = &file_instance[file_iter].file; // Set the file handle value

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : Function to get the last folder or file created
 *
 * @param p_found_name : Pointer to file or folder name
 * @param found_name_length : Length of the file or folder name string
 * @param is_file_folder_valid_func : function to tell if file/folder is valid
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
    static char       fname[MAXLEN];
    bool              found           = false;
    w_hal_file_info_t whal_found_info = {0};

    dir_t             dir;
    wHalErrorCode     error_code; // Variable to store the
                                  // w_hal_error_code enum type

    // Open directory
    if (!folder_inst.open(p_search_folder_name))
    {
        return (wHalErrorCode::SD_CARD_FOLDER_OPEN_FAILED);
    }

    while (true)
    {
        if (!file_inst.openNext(&folder_inst, O_READ))
        {
            if (found)
            {
                error_code = wHalErrorCode::NO_ERROR;
            }
            else
            {
                error_code = wHalErrorCode::SD_CARD_FOLDER_EMPTY;
            }
            break;
        }

        // FIXME : Need to create wildsonic fork of FAT library to expose
        // dirEntry function

        // if (!dirEntry(&dir))
        // {
        //     return (wHalErrorCode::SD_CARD_FOLDER_OPEN_FAILED);
        // }

        file_inst.getName(fname, MAXLEN);
        uint32_t          fname_length = strlen(fname);

        w_hal_file_info_t whal_file_info;
        whal_file_info.p_file_name      = fname;
        whal_file_info.file_name_length = fname_length;
        whal_file_info.date =
            dir.modifyDate[0] | (dir.modifyDate[1] << CHAR_BIT);
        whal_file_info.time =
            dir.modifyTime[0] | (dir.modifyTime[1] << CHAR_BIT);
        whal_file_info.size = dir.fileSize[0] | (dir.fileSize[1] << CHAR_BIT) |
                              (dir.fileSize[2] << (2 * CHAR_BIT)) |
                              (dir.fileSize[3] << (3 * CHAR_BIT));

        // Check whether this folder should be inspected
        if (!is_file_folder_valid_func(&whal_file_info))
        {
            continue;
        }

        if (!found ||
            compare_file_folder_func(&whal_file_info, &whal_found_info))
        {
            bool is_folder = file_inst.isDir();

            if ((find_folder && is_folder) || (!find_folder && !is_folder))
            {
                // Account for nullptr when checking length
                SYSTEM_ASSERT_LT(fname_length, found_name_length);

                // save found information
                strcpy(p_found_name, fname);
                found_size = whal_file_info.size;
                memcpy(&whal_found_info,
                       &whal_file_info,
                       sizeof(whal_file_info));
                whal_found_info.p_file_name = p_found_name;
                found                       = true;
            }
        }
    }

    return error_code;
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
    // TODO: Implement and check
    return wHalErrorCode::NO_ERROR;
}

#endif // STORAGE_CORE != INVALID_CORE_NUM