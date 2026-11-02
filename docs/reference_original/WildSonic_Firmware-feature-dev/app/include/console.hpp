#ifndef _CONSOLE_HPP
#define _CONSOLE_HPP

#include "i2c.h"
#include <stdint.h>

/**
 * @brief Initialization for console commands
 */
void           console_init();

/**
 * @brief Handling of full command string
 *
 * @param p_cmd command string
 * @param length length of the command string
 */
void           new_console_command_handler(const char *p_cmd, uint32_t length);

/* I2C console command Related constants and declaration*/
const uint32_t I2C_SCAN_FREQUENCY    = 100000; // I2C frequency
const uint8_t  SYSTEM_I2C1_BUS       = 1;      // I2C1
const uint8_t  SYSTEM_I2C2_BUS       = 2;      // I2C2
const uint8_t  I2C_BUS_COUNT         = 2;      // Total bus count
const uint32_t I2C_CMD_MAX_BYTES     = 4;      // Max bytes
const uint32_t I2C_CMD_MAX_RETRY     = 20;     // Max retries
const uint32_t I2C_CMD_RETRY_MS      = 10;     // Delay in ms
const uint8_t  I2C_CMD_ONE_BYTE      = 1;      // one byte
const uint8_t  I2C_CMD_TWO_BYTES     = 2;      // two byte

/* Addresses of known I2C devices on the system */
const uint16_t I2C_SGTL5000_ADDRESS  = 0x0A;
const uint16_t I2C_MAX9867_ADDRESS   = 0x18;
const uint16_t I2C_TLV320AIC_ADDRESS = 0x18;
const uint16_t I2C_SHT30_ADDRESS     = 0x44;
const uint16_t I2C_BQ34110_ADDRESS   = 0x55;
const uint16_t I2C_MCP7940_ADDRESS   = 0x6F;

/* structure to hold I2C bus information for scanning */
struct i2c_bus_info_t
{
    mxc_i2c_regs_t *p_i2c_regs; // I2C registers
    const char *    p_pin_info; // I2C pins
    bool            init_bus;   // Initialize bus before scanning
};

/* structure to hold known I2C device information for display during scanning */
struct i2c_device_info_t
{
    uint8_t     bus;     // I2C bus
    uint16_t    address; // I2C address
    const char *p_name;  // Device name
};

/* enum class for I2C command access types */
enum class i2c_cmd_access_t : uint8_t
{
    I2C_REGISTER, // Register address and data use MSB-first byte order
    SMBUS_WORD, // Register address is one byte, data uses LSB-first byte order
};

/* structure to hold I2C command page information for devices with paged
 * register maps */
struct i2c_cmd_page_info_t
{
    const char *p_name; // Page name
    uint8_t     page;   // Page number, reserved for paged register maps
};

/* structure to hold I2C command device information for devices accessible via
 * the I2C command
 */
struct i2c_cmd_device_info_t
{
    const char *               p_alias;                // Console alias
    const char *               p_name;                 // Device name
    uint8_t                    bus;                    // I2C bus
    uint16_t                   address;                // I2C address
    uint8_t                    reg_address_size_bytes; // Register address size
    uint8_t                    reg_data_size_bytes;    // Register data size
    i2c_cmd_access_t           access;                 // Access mode
    bool                       write_allowed;          // Write access allowed
    bool                       read_before_write;      // Read before write
    const i2c_cmd_page_info_t *p_page_info;            // Page lookup table
    uint8_t                    page_count;             // Number of pages
};

/* List of I2C buses available on the system for scanning, along with pin info
 */
const i2c_bus_info_t i2c_scan_bus_info[] = {
    {MXC_I2C1, "P0.16(SCL), P0.17(SDA)", false},
    {MXC_I2C2, "P0.30(SCL), P0.31(SDA)", true},
};

/* TLV320AIC Register Pages, rest of the devices default to page 0 since they
 * don't have paged register maps.
 *
 * Page 0 : Main control/status registers,
 *          clocking, audio routing,
 *          interface configuration, ADC/DAC control.
 *
 * Page 1 : Analog block configuration, microphone inputs,
 *          headphone/speaker drivers, bias and power settings.
 *
 * Page 8 : ADC Adaptive Filter configuration and
 *          ADC Coefficient Buffer-A (C0-C29).
 *
 * Pages 9-16 : Remaining ADC Coefficient Buffer-A
 *              coefficients (C30-C255).
 *
 * Pages 26-34 : ADC Coefficient Buffer-B (C0-C255).
 *
 * Page 44 : DAC Adaptive Filter configuration and
 *           DAC Coefficient Buffer-A (C0-C29).
 *
 * Pages 45-52 : Remaining DAC Coefficient Buffer-A
 *               coefficients (C30-C255).
 *
 * Pages 62-70 : DAC Coefficient Buffer-B (C0-C255)
 */
const i2c_cmd_page_info_t i2c_cmd_page_info[] = {
    {"PAGE0", 0}, // Main digital control/status registers
    {"PAGE1", 1}, // Analog configuration registers

    {"PAGE8", 8}, // ADC Adaptive Filter + ADC Coeff Buffer-A (C0-C29)
    {"PAGE9", 9}, // ADC Coeff Buffer-A continuation (C30-C255)

    {"PAGE16", 16}, // Last page of ADC Coeff Buffer-A region

    {"PAGE26", 26}, // ADC Coeff Buffer-B (start)
    {"PAGE34", 34}, // ADC Coeff Buffer-B (end)

    {"PAGE44", 44}, // DAC Adaptive Filter + DAC Coeff Buffer-A (C0-C29)
    {"PAGE45", 45}, // DAC Coeff Buffer-A continuation (start)
    {"PAGE52", 52}, // DAC Coeff Buffer-A continuation (end)

    {"PAGE62", 62}, // DAC Coeff Buffer-B (start)
    {"PAGE70", 70}, // DAC Coeff Buffer-B (end)
};

// List of known I2C devices on the system for display during scanning
const i2c_device_info_t i2c_device_info[] = {
    {SYSTEM_I2C1_BUS, I2C_SGTL5000_ADDRESS, "SGTL5000 audio codec"},
#if defined(I2S_PERIPHERAL) && (I2S_PERIPHERAL == max9867)
    {SYSTEM_I2C1_BUS, I2C_MAX9867_ADDRESS, "MAX9867 audio codec"},
#else
    {SYSTEM_I2C1_BUS, I2C_TLV320AIC_ADDRESS, "TLV320AIC audio codec"},
#endif
    {SYSTEM_I2C1_BUS, I2C_BQ34110_ADDRESS, "BQ34110 fuel gauge"},
    {SYSTEM_I2C1_BUS, I2C_MCP7940_ADDRESS, "MCP7940N RTC"},
    {SYSTEM_I2C2_BUS,
     I2C_SHT30_ADDRESS,
     "SHT30 temperature and humidity sensor"},
};

// List of Available I2C devices and their registers for the I2C command
const i2c_cmd_device_info_t i2c_cmd_device_info[] = {
    {"SGTL",
     "SGTL5000 audio codec",
     SYSTEM_I2C1_BUS,
     I2C_SGTL5000_ADDRESS,
     I2C_CMD_TWO_BYTES,
     I2C_CMD_TWO_BYTES,
     i2c_cmd_access_t::I2C_REGISTER,
     true,
     true,
     i2c_cmd_page_info,
     static_cast<uint8_t>(sizeof(i2c_cmd_page_info) /
                          sizeof(i2c_cmd_page_info[0]))},
#if defined(I2S_PERIPHERAL) && (I2S_PERIPHERAL == max9867)
    {"MAX9867",
     "MAX9867 audio codec",
     SYSTEM_I2C1_BUS,
     I2C_MAX9867_ADDRESS,
     I2C_CMD_ONE_BYTE,
     I2C_CMD_ONE_BYTE,
     i2c_cmd_access_t::I2C_REGISTER,
     true,
     false,
     i2c_cmd_page_info,
     static_cast<uint8_t>(sizeof(i2c_cmd_page_info) /
                          sizeof(i2c_cmd_page_info[0]))},
#else
    {"TLV320AIC",
     "TLV320AIC audio codec",
     SYSTEM_I2C1_BUS,
     I2C_TLV320AIC_ADDRESS,
     I2C_CMD_ONE_BYTE,
     I2C_CMD_ONE_BYTE,
     i2c_cmd_access_t::I2C_REGISTER,
     true,
     false,
     i2c_cmd_page_info,
     static_cast<uint8_t>(sizeof(i2c_cmd_page_info) /
                          sizeof(i2c_cmd_page_info[0]))},
#endif
    {"BQ34110",
     "BQ34110 fuel gauge",
     SYSTEM_I2C1_BUS,
     I2C_BQ34110_ADDRESS,
     I2C_CMD_ONE_BYTE,
     I2C_CMD_TWO_BYTES,
     i2c_cmd_access_t::SMBUS_WORD,
     false,
     false,
     i2c_cmd_page_info,
     static_cast<uint8_t>(sizeof(i2c_cmd_page_info) /
                          sizeof(i2c_cmd_page_info[0]))},
    {"RTC",
     "MCP7940N RTC",
     SYSTEM_I2C1_BUS,
     I2C_MCP7940_ADDRESS,
     I2C_CMD_ONE_BYTE,
     I2C_CMD_ONE_BYTE,
     i2c_cmd_access_t::I2C_REGISTER,
     true,
     false,
     i2c_cmd_page_info,
     static_cast<uint8_t>(sizeof(i2c_cmd_page_info) /
                          sizeof(i2c_cmd_page_info[0]))},

};

#endif // _CONSOLE_HPP
