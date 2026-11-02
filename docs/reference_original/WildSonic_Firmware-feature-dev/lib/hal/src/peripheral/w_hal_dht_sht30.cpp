#include "w_hal_dht_internal.hpp"

#include "w_hal_error_code.hpp"
#include "w_hal_i2c.hpp"
#include "w_hal_system.hpp"

#include "assert.hpp"
#include "trace.hpp"

static const uint32_t SHT30_I2C_FREQUENCY        = 100000; // Standard frequency
static const uint32_t SHT30_I2C_CHANNEL          = 2;      // I2C channel
static const uint32_t SHT30_I2C_ADDRESS          = 0x44;   // SHT30 I2C address
static const uint16_t SHT30_I2c_READ_BUFFER_SIZE = 6;      // Read buffer size
/*
 * @brief Error codes returned by SHT30
 */
enum class sht30_error_code_t : uint8_t
{
    SHT30_OK                    = 0x00,
    SHT30_ERR_WRITECMD          = 0x81,
    SHT30_ERR_READBYTES         = 0x82,
    SHT30_ERR_HEATER_OFF        = 0x83,
    SHT30_ERR_NOT_CONNECT       = 0x84,
    SHT30_ERR_CRC_TEMP          = 0x85,
    SHT30_ERR_CRC_HUM           = 0x86,
    SHT30_ERR_CRC_STATUS        = 0x87,
    SHT30_ERR_HEATER_COOLDOWN   = 0x88,
    SHT30_ERR_HEATER_ON         = 0x89,
    SHT30_ERR_SERIAL_NUMBER_CRC = 0x8A,
};

/*
 * @brief Structure to track SHT30 state
 */
struct sht30_tracker_st
{
    int32_t            rawTemperature = 0;
    int32_t            rawHumidity    = 0;
    sht30_error_code_t error          = sht30_error_code_t::SHT30_OK;
};

/*
 * Supported commands for SHT30 sensor
 */
static const uint16_t SHT30_READ_STATUS               = 0xF32D;
static const uint16_t SHT30_CLEAR_STATUS              = 0x3041;
static const uint16_t SHT30_SOFT_RESET                = 0x30A2;
static const uint16_t SHT30_HARD_RESET                = 0x0006;
static const uint16_t SHT30_FAST_CLOCKSTRETCH         = 0x2C06;
static const uint16_t SHT30_MEDIUM_CLOCKSTRETCH       = 0x2C0D;
static const uint16_t SHT30_SLOW_CLOCKSTRETCH         = 0x2C10;
static const uint16_t SHT30_MEASUREMENT_FAST          = 0x2416;
static const uint16_t SHT30_MEASUREMENT_MEDIUM        = 0x240B;
static const uint16_t SHT30_MEASUREMENT_SLOW          = 0x2400;
static const uint16_t SHT30_HEAT_ON                   = 0x306D;
static const uint16_t SHT30_HEAT_OFF                  = 0x3066;
static const uint32_t SHT30_HEATER_TIMEOUT            = 180000UL;
static const uint16_t SHT30_GET_SERIAL_NUMBER         = 0x3682;
static const uint8_t  SHT30_FAST_MEASUREMENT_DELAY_MS = 4;
static const uint8_t  SHT30_SLOW_MEASUREMENT_DELAY_MS = 15;
static const uint16_t SHT30_RAW_MAX_VALUE             = 65535;
static const int16_t  SHT30_TEMPERATURE_MIN_CELSIUS   = -45;
static const uint16_t SHT30_TEMPERATURE_SCALE         = 175;
static const uint16_t SHT30_HUMIDITY_SCALE            = 100;
static const uint16_t SHT30_PROCESSING_DELAY_MS =
    100; // Time to wait after writing data before next command
static sht30_tracker_st sht30_tracker;

/**
 * @brief Write data to the I2C peripheral at the specified address
 *
 * @param address address of the register to write
 * @param data data to write to the register
 */
static void             i2c_write(uint8_t *p_data, uint8_t size)
{
    // Assume I2C is already initialized
    wHalErrorCode error;
    do
    {
        error = w_hal_i2c_write_data(SHT30_I2C_CHANNEL,
                                     p_data,
                                     size,
                                     false,
                                     SHT30_I2C_ADDRESS,
                                     true);
    } while (error == wHalErrorCode::BUSY);

    while (error == wHalErrorCode::I2C_WRITE_IN_PROGRESS)
    {
        error = w_hal_i2c_check_status(SHT30_I2C_CHANNEL);
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
    w_hal_i2c_clear_completion_status(SHT30_I2C_CHANNEL);
}

/**
 * @brief Read a word from the I2C peripheral
 *
 * @param address address of the register to read
 * @return uint8_t data read from the register
 */
static void i2c_read(uint8_t *p_data, uint8_t size)
{
    wHalErrorCode error = w_hal_i2c_write_read_data(SHT30_I2C_CHANNEL,
                                                    nullptr,
                                                    0,
                                                    p_data,
                                                    size,
                                                    false,
                                                    SHT30_I2C_ADDRESS,
                                                    true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_COMPLETE);

    error = w_hal_i2c_clear_completion_status(SHT30_I2C_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
}

/**
 * @brief Soft reset the DHT sensor
 *
 */
static void dht_soft_reset()
{
    uint8_t cmd_buffer[2];

    cmd_buffer[0] = SHT30_SOFT_RESET >> 8;
    cmd_buffer[1] = SHT30_SOFT_RESET & 0xFF;

    i2c_write(cmd_buffer, sizeof(cmd_buffer));

    w_hal_system_delay_ms(1);
}

/**
 * @brief Write a command to the DHT sensor
 *
 * @param cmd command to write
 */
static void write_dht_command(uint16_t cmd)
{
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = cmd >> 8;
    cmd_buffer[1] = cmd & 0xFF;
    i2c_write(cmd_buffer, sizeof(cmd_buffer));
}

/**
 * @brief Calculate CRC8 for data validation
 *
 * @param data data to validate
 * @param length length of data
 * @return uint8_t CRC8 value
 */
static uint8_t calc_crc8(uint8_t *data, int length)
{
    const uint8_t POLY = 0x31; // Polynomial for CRC8 calculation
    uint8_t       crc  = 0xFF; // Initial value
    for (uint8_t j = length; j; j--)
    {
        crc ^= *data++;
        for (uint8_t i = 8; i; i--)
        {
            // If MSB is set, shift left and XOR with polynomial; otherwise just
            // shift
            crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Read data from the DHT sensor
 *
 * @param fast indicates if fast measurement mode is used
 * @return true If data read successfully
 * @return false If data read failed
 */
static bool read_dht_data(bool fast)
{
    w_hal_rtc_time_st current_time;
    uint8_t           read_buffer[SHT30_I2c_READ_BUFFER_SIZE];
    i2c_read(read_buffer, sizeof(read_buffer));

    if (!fast)
    {
        if (read_buffer[2] != calc_crc8(read_buffer, 2))
        {
            sht30_tracker.error = sht30_error_code_t::SHT30_ERR_CRC_TEMP;
            return false;
        }
        if (read_buffer[5] != calc_crc8(&read_buffer[3], 2))
        {
            sht30_tracker.error = sht30_error_code_t::SHT30_ERR_CRC_HUM;
            return false;
        }
    }

    // Process the read data
    sht30_tracker.rawTemperature =
        (static_cast<int32_t>(read_buffer[0]) << 8) | read_buffer[1];
    sht30_tracker.rawHumidity =
        (static_cast<int32_t>(read_buffer[3]) << 8) | read_buffer[4];

    w_hal_rtc_get_time(current_time);

    return true;
}

/**
 * @brief Update DHT data
 *
 * @param fast indicates if fast measurement mode is used
 */
static void update_dht_data(bool fast)
{
    write_dht_command(fast ? SHT30_FAST_CLOCKSTRETCH : SHT30_SLOW_CLOCKSTRETCH);
    w_hal_system_delay_ms(SHT30_PROCESSING_DELAY_MS);

    read_dht_data(fast);
}

/**
 * @brief Get temperature in Celsius
 *
 * @return float temperature in Celsius
 */
float get_temperature_celsius()
{
    return (static_cast<float>(SHT30_TEMPERATURE_MIN_CELSIUS) +
            (static_cast<float>(SHT30_TEMPERATURE_SCALE) *
             (static_cast<float>(sht30_tracker.rawTemperature) /
              static_cast<float>(SHT30_RAW_MAX_VALUE))));
}

/**
 * @brief Get relative humidity in percentage
 *
 * @return float relative humidity in percentage
 */
float get_relative_humidity()
{
    return static_cast<float>(SHT30_HUMIDITY_SCALE) *
           (static_cast<float>(sht30_tracker.rawHumidity) /
            static_cast<float>(SHT30_RAW_MAX_VALUE));
}

/**
 * @brief Initialize the DHT peripheral
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_init_peripheral(void)
{
    wHalErrorCode error;
    // initialize I2C Communication
    error = w_hal_i2c_init(SHT30_I2C_CHANNEL,
                           whal_i2c_mode::MASTER,
                           SHT30_I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Do soft reset of the DHT sensor
    dht_soft_reset();

    // Acquire temperature and humidity
    // false: SHT30_MEASUREMENT_SLOW
    // true: SHT30_MEASUREMENT_FAST
    update_dht_data(true);

    // Teardown I2C Communication
    error = w_hal_i2c_teardown(SHT30_I2C_CHANNEL);

    return error;
}