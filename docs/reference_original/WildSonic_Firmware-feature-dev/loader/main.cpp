#include "assert.hpp"
#include "crc32.hpp"
#include "device_config.hpp"
#include "trace.hpp"
#include "version.hpp"

#include "w_hal_flash.hpp"
#include "w_hal_sd_card.hpp"
#include "w_hal_system.hpp"

#include "re.h"

#include <algorithm>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Magic number to identify the firmware binary
static const uint32_t FIRMWARE_BINARY_MAGIC               = 0x17D50F1C;
// Initial CRC32 value for the firmware binary and header
static const uint32_t FIRMWARE_INIITAL_CRC32              = 0xAABBCCDD;

// Must match the flash value in the linker scatter and Makefile.
static const uint32_t FLASH_BYTES_RESERVED_FOR_BOOTLOADER = 0x20000;
static const uint32_t FLASH_PAGES_RESERVED_FOR_CONFIG     = 2;
// Read buffer size Needs to be larger than the flash page size
static const uint32_t READ_BUFFER_SIZE                    = 4096 * 3;
static const uint32_t FILE_NAME_MAX_LENGTH                = 40;

// File name buffer for reading the firmware binary from the SD card
static char           m_file_name[FILE_NAME_MAX_LENGTH];
// File name buffer for identifying and deleting a older unused
// firmware binary from the SD card
static char           m_delete_file_name[FILE_NAME_MAX_LENGTH];

/**
 * @brief FW version structure
 */
union fw_version_st
{
    uint32_t dw;
    struct
    {
        uint8_t major;    // Major version number
        uint8_t minor;    // Minor version number
        uint8_t patch;    // Patch version number
        uint8_t reserved; // Reserved for future use
    } byte;
};

/**
 * @brief Header of the firmware binary stored in the SD card.
 */
struct release_binary_header_st
{
    uint32_t      magic;     // Magic number to identify the firmware binary
    fw_version_st version;   // Release Version of the firmware binary
    uint32_t config_version; // Version of the device configuration that the
                             // firmware binary is compatible with.
    uint32_t flags;          // Reserved for future use
    uint32_t offset;         // Offset of the binary from
                             // the end of the header to
                             // the beginning of the binary data
    uint32_t size;           // Size of the binary
    uint32_t crc32;          // CRC32 checksum of the binary
    uint32_t reserved[2];    // Reserved for future use
    uint32_t header_crc32;   // CRC32 checksum of the header
};

DeviceConfig::config_page_st    device_config;

// Structure to read the firmware binary header
static release_binary_header_st firmware_header;

// File handle for the firmware binary stored in the SD card
static wFileHandle              file_handle;

// Read buffer for reading the firmware binary from the SD card
static uint8_t                  read_buffer[READ_BUFFER_SIZE];

// RTC time structure - Not used. Only for keeping the HAL happy.
static uint32_t                 time_update;
static w_hal_rtc_time_st        rtc_time;

/**
 * @brief Align the value to the next multiple of the alignment.
 *
 * @param value the value to align
 * @param alignment the alignment value
 * @return uint32_t value aligned to the next multiple of the alignment
 */
static uint32_t align_ceiling_log2(uint32_t value, uint32_t alignment)
{
    return (value + alignment - 1) & ~(alignment - 1);
}

/**
 * @brief
 * Check if filename on file system is a valid bin file
 *
 * @param p_file_info file information from HAL
 * @return true if file is an expected wav file
 * @return false otherwise
 */
static bool is_bin_file_name_valid(const w_hal_file_info_t *p_file_info)
{
    // Check if it matches the form "wildsonic_vX.Y.Z.bin"
    // where X, Y, Z are integers
    int match_length = 0;
    int match_index;

    match_index = re_match("wildsonic_v[0-9]+\\.[0-9]+\\.[0-9]+\\.bin",
                           p_file_info->p_file_name,
                           static_cast<int>(p_file_info->file_name_length),
                           &match_length);

    return (match_index != RE_NO_MATCH_INDEX);
}

/**
 * @brief
 * Check if filename on file system is a valid bin file and needs deletion
 *
 * @param p_file_info file information from HAL
 * @return true if file is an expected wav file and needs deletion
 * @return false otherwise
 */
static bool
is_bin_file_name_valid_and_unused(const w_hal_file_info_t *p_file_info)
{
    return is_bin_file_name_valid(p_file_info) &&
           (strcmp(m_file_name, p_file_info->p_file_name) != 0);
}

/**
 * @brief
 * Compare bin file names to see which one is newer
 *
 * @param p_file_info_1 first file information from HAL
 * @param p_file_info_2 second file information from HAL
 * @return true if first file is newer than second one
 * @return false if second file is newer than first one
 */
static bool compare_bin_file_names(const w_hal_file_info_t *p_file_info_1,
                                   const w_hal_file_info_t *p_file_info_2)
{
    return (strcmp(p_file_info_1->p_file_name, p_file_info_2->p_file_name) > 0);
}

/**
 * @brief Get the flash start address of the main binary.
 *
 * @param flash_page_size size of the flash page
 * @return uint32_t flash start address of the main binary
 */
static uint32_t get_main_binary_flash_start_address(uint32_t flash_page_size)
{
    // The first page is reserved for the bootloader.
    return align_ceiling_log2(FLASH_BYTES_RESERVED_FOR_BOOTLOADER,
                              flash_page_size);
}

/**
 * @brief Get the flash end address of the main binary.
 *
 * @param flash_page_size size of the flash page
 * @param memory_size size of the flash memory
 * @return uint32_t the flash end address of the main binary
 */
static uint32_t get_main_binary_flash_end_address(uint32_t flash_page_size,
                                                  uint32_t memory_size)
{
    // The last pages are reserved for the stored configuration.
    SYSTEM_ASSERT_GTEQ(memory_size,
                       FLASH_PAGES_RESERVED_FOR_CONFIG * flash_page_size);
    return memory_size - FLASH_PAGES_RESERVED_FOR_CONFIG * flash_page_size;
}

/**
 * @brief Close the opened file.
 *
 * @return true if the file is successfully closed
 * @return false otherwise
 */
static bool close_file()
{
    wHalErrorCode error_code = w_hal_sd_card_close_file(file_handle);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        printf("Error closing firmware binary: %s with error: %d\n",
               m_file_name,
               error_code);

        return false;
    }

    return true;
}

/**
 * @brief Validate the firmware binary stored in the SD card.
 * Assumes SD card and flash memory are initialized.
 *
 * @param p_filename filename of the firmware binary. File should be stored in
 * root directory.
 * @param filename_length length of the filename, excluding the null terminator
 * @param binary_offset[out]: offset of the binary from the beginning of the
 * file
 * @param binary_size[out]: size of the binary
 * @param binary_crc32[out]: CRC32 checksum of the binary
 * @return true if the firmware binary is valid and compatible with the device
 * @return false otherwise
 */
static bool validate_firmware_binary_in_sd_card(const char *p_filename,
                                                uint32_t    filename_length,
                                                uint32_t &  binary_offset,
                                                uint32_t &  binary_size,
                                                uint32_t &  binary_crc32)
{
    // Read first the header of the firmware binary stored in the SD card
    // Open the file for reading, read the header.
    wHalErrorCode error_code =
        w_hal_sd_card_open_file(m_file_name, FILE_NAME_MAX_LENGTH, file_handle);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        printf("Error opening firmware binary: %s with error: %d\n",
               m_file_name,
               error_code);
        return false;
    }

    // Read the firmware binary header
    uint32_t bytes_read = 0;
    memset(&firmware_header, 0xFF, sizeof(firmware_header));
    error_code = w_hal_sd_card_read_from_file(&firmware_header,
                                              sizeof(firmware_header),
                                              bytes_read,
                                              file_handle);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        printf("Error reading firmware binary header: %d\n", error_code);
    }

    bool valid_binary = false;
    if ((error_code == wHalErrorCode::NO_ERROR) &&
        (bytes_read == sizeof(firmware_header)))
    {
        const uint32_t header_crc32 = compute_crc32(
            &firmware_header,
            (sizeof(firmware_header) - sizeof(firmware_header.header_crc32)),
            FIRMWARE_INIITAL_CRC32);

        bool valid_header = true;

        // Only validate the remaining of the firmware binary if:
        // 1. the header is valid
        // 2. There is enough space in flash to accommodate the executing code.
        //    There is 1 page reserved for the bootloader and 2 pages reserved
        //    for the configuration.
        // 3. The binary is compatible with the existing device configuration.
        if (header_crc32 != firmware_header.header_crc32)
        {
            printf("Invalid header CRC32. Calculated: 0x%08X, Actual: 0x%08X\n",
                   header_crc32,
                   firmware_header.header_crc32);
            valid_header = false;
        }

        if (firmware_header.magic != FIRMWARE_BINARY_MAGIC)
        {
            printf("Invalid magic number. Expected: 0x%08X, Actual: 0x%08X\n",
                   FIRMWARE_BINARY_MAGIC,
                   firmware_header.magic);
            valid_header = false;
        }

        if (firmware_header.size == 0)
        {
            printf("Invalid firmware size. Expected: > 0, Actual: %d\n",
                   firmware_header.size);
            valid_header = false;
        }

        // Check if the firmware binary can fit in the flash memory
        // Validate the firmware header and then the content of
        // the firmware binary. Both have crc32 checksums.
        uint32_t flash_page_size, memory_size;
        error_code = w_hal_flash_get_parameters(flash_page_size, memory_size);
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

        const uint32_t flash_start_address =
            get_main_binary_flash_start_address(flash_page_size);
        const uint32_t flash_end_address =
            get_main_binary_flash_end_address(flash_page_size, memory_size);
        SYSTEM_ASSERT_GTEQ(flash_end_address, flash_start_address);
        const uint32_t flash_bytes_available =
            flash_end_address - flash_start_address;

        if (firmware_header.size > flash_bytes_available)
        {
            printf("Firmware too large. Got: %d, Space: %d\n",
                   firmware_header.size,
                   flash_bytes_available);
            valid_header = false;
        }

        // Check if the firmware binary is compatible with the device
        // configuration
        if (device_config.version != firmware_header.config_version)
        {
            printf("Firmware incompatible with device config. Expected: %d, "
                   "Actual: %d\n",
                   device_config.version,
                   firmware_header.config_version);
            valid_header = false;
        }

        if (valid_header)
        {
            // Print the firmware version information
            printf("File %s - Firmware version: %d.%d.%d\n",
                   m_file_name,
                   firmware_header.version.byte.major,
                   firmware_header.version.byte.minor,
                   firmware_header.version.byte.patch);

            // Skip the offset of the binary
            error_code = w_hal_sd_card_seek(file_handle,
                                            sizeof(firmware_header) +
                                                firmware_header.offset);
            if (error_code != wHalErrorCode::NO_ERROR)
            {
                printf("Error seeking firmware binary: %s with error: %d\n",
                       m_file_name,
                       error_code);

                // Close the file before returning
                close_file();

                return false;
            }

            // Flash page should be able to be read in one go
            SYSTEM_ASSERT_LTEQ(flash_page_size, READ_BUFFER_SIZE);
            const uint32_t read_buffer_size    = flash_page_size;

            // Initialize the CRC32 value
            uint32_t       crc32               = FIRMWARE_INIITAL_CRC32;
            uint32_t       remaining_data_size = firmware_header.size;

            do
            {
                // Validate the firmware binary
                bytes_read = 0;
                error_code = w_hal_sd_card_read_from_file(
                    read_buffer,
                    std::min(read_buffer_size, remaining_data_size),
                    bytes_read,
                    file_handle);

                if ((error_code != wHalErrorCode::NO_ERROR) ||
                    (bytes_read == 0))
                {
                    break;
                }

                // Update the CRC32 checksum
                crc32 = compute_crc32(read_buffer, bytes_read, crc32);

                SYSTEM_ASSERT_GTEQ(remaining_data_size, bytes_read);
                remaining_data_size -= bytes_read;
            } while ((bytes_read == read_buffer_size) &&
                     (remaining_data_size > 0));

            // Only if the entire firmware binary is read and the CRC32
            // checksum is valid, then the binary is valid.
            if ((remaining_data_size == 0) && (crc32 == firmware_header.crc32))
            {
                valid_binary = true;
            }
            else
            {
                printf("Invalid firmware binary. CRC32 mismatch. Expected: "
                       "0x%08X, Actual: 0x%08X with %d bytes remaining.\n",
                       firmware_header.crc32,
                       crc32,
                       remaining_data_size);
            }
        }
    }

    // Close the file
    if (!close_file())
    {
        return false;
    }

    if (valid_binary)
    {
        binary_offset = sizeof(firmware_header) + firmware_header.offset;
        binary_size   = firmware_header.size;
        binary_crc32  = firmware_header.crc32;
    }

    return valid_binary;
}

/**
 * @brief Read the firmware binary from the SD card and write it to the flash
 * memory.
 *
 * It can be assumed that the firmware binary is valid.
 *
 * @param binary_offset offset of the binary from the beginning of the file
 * @param binary_size size of the binary
 * @param binary_crc32 CRC32 checksum of the binary
 *
 * @return true if the firmware binary is successfully written to the flash
 * memory
 * @return false otherwise
 */
static bool read_firmware_binary_and_write_to_flash(uint32_t binary_offset,
                                                    uint32_t binary_size,
                                                    uint32_t binary_crc32)
{
    // Open the file for reading
    wHalErrorCode error_code =
        w_hal_sd_card_open_file(m_file_name, FILE_NAME_MAX_LENGTH, file_handle);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        printf("Error opening firmware binary: %s with error: %d\n",
               m_file_name,
               error_code);
        return false;
    }

    // Skip the offset of the binary
    error_code = w_hal_sd_card_seek(file_handle, binary_offset);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        printf("Error seeking firmware binary: %s with error: %d\n",
               m_file_name,
               error_code);

        // Close the file before returning
        close_file();

        return false;
    }

    // Get page size and the size of the flash memory
    uint32_t flash_page_size, memory_size;
    error_code = w_hal_flash_get_parameters(flash_page_size, memory_size);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Flash page should be able to be read in one go
    SYSTEM_ASSERT_LTEQ(flash_page_size, READ_BUFFER_SIZE);
    const uint32_t read_buffer_size = flash_page_size;

    const uint32_t flash_start_address =
        get_main_binary_flash_start_address(flash_page_size);
    const uint32_t flash_end_address =
        get_main_binary_flash_end_address(flash_page_size, memory_size);
    SYSTEM_ASSERT_GTEQ(flash_end_address, flash_start_address);

    // Erase the flash memory before writing the firmware binary
    for (uint32_t address = flash_start_address; address < flash_end_address;
         address += flash_page_size)
    {
        error_code = w_hal_flash_page_erase(address);
        if (error_code != wHalErrorCode::NO_ERROR)
        {
            printf("Error erasing flash memory at address: 0x%08X with error: "
                   "%d\n",
                   address,
                   error_code);

            // Close the file before returning
            close_file();

            return false;
        }
    }

    // Initialize the CRC32 value
    uint32_t crc32 = FIRMWARE_INIITAL_CRC32;

    // Read the firmware binary and write it to the flash memory
    uint32_t bytes_read;
    uint32_t remaining_data_size = binary_size;
    uint32_t flash_address       = flash_start_address;
    do
    {
        // Read the firmware binary
        bytes_read = 0;
        error_code = w_hal_sd_card_read_from_file(
            read_buffer,
            std::min(read_buffer_size, remaining_data_size),
            bytes_read,
            file_handle);

        if ((error_code != wHalErrorCode::NO_ERROR) || (bytes_read == 0))
        {
            break;
        }

        // Write the firmware binary to the flash memory
        error_code =
            w_hal_flash_program(flash_address, read_buffer, bytes_read);
        if (error_code != wHalErrorCode::NO_ERROR)
        {
            printf("Error writing flash memory at address: 0x%08X with error: "
                   "%d\n",
                   flash_address,
                   error_code);
            break;
        }

        // Update the CRC32 checksum
        crc32 = compute_crc32(read_buffer, bytes_read, crc32);

        SYSTEM_ASSERT_GTEQ(remaining_data_size, bytes_read);
        remaining_data_size -= bytes_read;

        flash_address += bytes_read;

    } while ((bytes_read == read_buffer_size) && (remaining_data_size > 0) &&
             (flash_address < flash_end_address));

    // Verify the CRC32 checksum
    bool firmware_updated =
        (remaining_data_size == 0) && (crc32 == binary_crc32);

    if (!firmware_updated)
    {
        printf(
            "Firmware binary write failed. CRC32 mismatch. Expected: 0x%08X, "
            "Actual: 0x%08X with %d bytes remaining.\n",
            binary_crc32,
            crc32,
            remaining_data_size);
    }

    // Close the file
    if (!close_file())
    {
        return false;
    }

    return firmware_updated;
}

/**
 * @brief Read the firmware binary from the flash memory and validate it.
 *
 * @param size the number of bytes to read from the flash memory
 * @param expected_crc the expected CRC32 checksum of the firmware binary
 * @return true if the firmware binary is valid
 * @return false otherwise
 */
static bool validate_main_firmware_binary_in_flash(uint32_t size,
                                                   uint32_t expected_crc)
{
    // Get page size and the size of the flash memory
    uint32_t      flash_page_size, memory_size;
    wHalErrorCode error_code =
        w_hal_flash_get_parameters(flash_page_size, memory_size);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Flash page should be able to be read in one go
    SYSTEM_ASSERT_LTEQ(flash_page_size, READ_BUFFER_SIZE);
    const uint32_t read_buffer_size = flash_page_size;

    const uint32_t flash_start_address =
        get_main_binary_flash_start_address(flash_page_size);
    const uint32_t flash_end_address =
        get_main_binary_flash_end_address(flash_page_size, memory_size);
    SYSTEM_ASSERT_GTEQ(flash_end_address, flash_start_address);

    // Initialize the CRC32 value
    uint32_t crc32               = FIRMWARE_INIITAL_CRC32;

    // Read the firmware binary from flash memory and validate it
    uint32_t remaining_data_size = size;
    uint32_t flash_address       = flash_start_address;

    do
    {
        // Read the flash memory
        const uint32_t bytes_read =
            std::min(read_buffer_size, remaining_data_size);

        error_code = w_hal_flash_read(flash_address, read_buffer, bytes_read);

        if (error_code != wHalErrorCode::NO_ERROR)
        {
            printf("Error reading flash memory at address: 0x%08X with error: "
                   "%d\n",
                   flash_address,
                   error_code);
            break;
        }

        // Update the CRC32 checksum
        crc32 = compute_crc32(read_buffer, bytes_read, crc32);

        SYSTEM_ASSERT_GTEQ(remaining_data_size, bytes_read);
        remaining_data_size -= bytes_read;

        flash_address += bytes_read;

    } while ((remaining_data_size > 0) && (flash_address < flash_end_address));

    if ((remaining_data_size == 0) && (crc32 == expected_crc))
    {
        printf("Firmware binary in flash is valid.\n");
        return true;
    }
    else
    {
        printf("CRC32 mismatch. Expected: 0x%08X, Actual: 0x%08X with %d "
               "bytes remaining.\n",
               expected_crc,
               crc32,
               remaining_data_size);

        return false;
    }
}

/**
 * @brief
 * main entry point function
 *
 * @return int error code 0
 */
int main()
{
    print_fw_version_information();

    // Read the configuration page from the flash memory to get the
    // config version.
    DeviceConfig::initialize(device_config, true);

    // Initialize hal components
    rtc_time.clear();
    time_update = 0;
    wHalErrorCode error_code =
        w_hal_system_init(device_config, &rtc_time, &time_update);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    error_code = w_hal_flash_init();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    error_code = w_hal_sd_card_init(device_config, &rtc_time);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Scan the SD card root directory for the latest firmware binary
    // and update the firmware if necessary
    uint32_t file_size;
    error_code = w_hal_sd_card_last_file_created(m_file_name,
                                                 FILE_NAME_MAX_LENGTH,
                                                 file_size,
                                                 is_bin_file_name_valid,
                                                 compare_bin_file_names);

    const wHalErrorCode valid_errors[] = {wHalErrorCode::NO_ERROR,
                                          wHalErrorCode::SD_CARD_FOLDER_EMPTY};
    SYSTEM_ASSERT_IN_SET(error_code,
                         valid_errors,
                         sizeof(valid_errors) / sizeof(wHalErrorCode));

    bool firmware_updated = false;
    if (error_code == wHalErrorCode::NO_ERROR)
    {
        printf("Found firmware binary: %s\n", m_file_name);

        uint32_t binary_offset, binary_size, binary_crc32;
        bool     valid_binary =
            validate_firmware_binary_in_sd_card(m_file_name,
                                                strlen(m_file_name),
                                                binary_offset,
                                                binary_size,
                                                binary_crc32);

        // If the firmware binary is valid, update the firmware
        // by writing the binary to the flash starting from the
        // 2nd sector of the flash memory to the second to the last sector.
        // The first sector is reserved for the bootloader.
        // The last sector is reserved for the stored configuration.
        if (valid_binary)
        {
            // Power LED is already on. Turn on the activity and error LEDs.
            // When all LEDs are on, it indicates that the firmware is being
            // updated.
            w_hal_system_set_activity_led(true);
            w_hal_system_set_error_led(true);

            printf("Writing firmware binary to flash...\n");

            // Update the firmware
            firmware_updated =
                read_firmware_binary_and_write_to_flash(binary_offset,
                                                        binary_size,
                                                        binary_crc32);

            if (firmware_updated)
            {
                printf("Validating written firmware binary...\n");

                // Verify the firmware binary in the flash
                firmware_updated =
                    validate_main_firmware_binary_in_flash(binary_size,
                                                           binary_crc32);
            }

            if (firmware_updated)
            {
                // Keep scanning the SD card root directory for the any more
                // firmware binaries and delete them so that they are not
                // processed on the next boot.
                // Delete the old unused firmware binaries from the SD card
                // first, then delete the current firmware binary.
                do
                {
                    error_code = w_hal_sd_card_last_file_created(
                        m_delete_file_name,
                        FILE_NAME_MAX_LENGTH,
                        file_size,
                        is_bin_file_name_valid_and_unused,
                        compare_bin_file_names);

                    SYSTEM_ASSERT_IN_SET(error_code,
                                         valid_errors,
                                         sizeof(valid_errors) /
                                             sizeof(wHalErrorCode));

                    if (error_code != wHalErrorCode::NO_ERROR)
                    {
                        // No firmware binary found
                        break;
                    }

                    printf("Deleting firmware binary: %s\n",
                           m_delete_file_name);

                    error_code =
                        w_hal_sd_card_delete_file(m_delete_file_name,
                                                  FILE_NAME_MAX_LENGTH);

                    if (error_code != wHalErrorCode::NO_ERROR)
                    {
                        printf("Error deleting firmware binary: %s with "
                               "error: %d\n",
                               m_delete_file_name,
                               error_code);
                    }

                } while (error_code == wHalErrorCode::NO_ERROR);

                printf("Deleting firmware binary: %s\n", m_file_name);

                // Delete the firmware binary from the SD card
                wHalErrorCode error_code =
                    w_hal_sd_card_delete_file(m_file_name,
                                              FILE_NAME_MAX_LENGTH);

                if (error_code != wHalErrorCode::NO_ERROR)
                {
                    printf(
                        "Error deleting firmware binary: %s with error: %d\n",
                        m_file_name,
                        error_code);
                }
            }

            printf("Firmware update %s\n",
                   firmware_updated ? "successful" : "failed");

            // If the firmware is updated, blink all LEDs
            // twice to indicate success
            if (firmware_updated)
            {
                for (uint32_t index = 0; index < 2; index++)
                {
                    w_hal_system_set_activity_led(false);
                    w_hal_system_set_error_led(false);
                    w_hal_system_set_power_led(false);
                    w_hal_system_delay_ms(500);

                    w_hal_system_set_activity_led(true);
                    w_hal_system_set_error_led(true);
                    w_hal_system_set_power_led(true);
                    w_hal_system_delay_ms(500);
                }
            }
        }
        else
        {
            printf("Invalid firmware binary: %s\n", m_file_name);
        }
    }

    // Teardown
    error_code = w_hal_sd_card_tear_down();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    error_code = w_hal_flash_teardown();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    w_hal_system_teardown();

    // Jump to the firmware in the flash
    printf("Jumping to the main firmware...\n");
    printf("...\n");

    const uint32_t cpu_flash_base_address = w_hal_flash_get_cpu_base_address();
    uint32_t       flash_page_size, flash_memory_size;
    w_hal_flash_get_parameters(flash_page_size, flash_memory_size);

    error_code = w_hal_system_jump_to_new_entry_point(
        cpu_flash_base_address +
        get_main_binary_flash_start_address(flash_page_size));
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    return 0;
}