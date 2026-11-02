#if __CORE__ == PM_CORE

#include "w_hal_pm.hpp"
#include "shared.hpp"
#include "w_hal_error_code.hpp"
#include "w_hal_i2c.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_smbus.hpp"
#include "w_hal_system.hpp"

#include "assert.hpp"
#include "trace.hpp"

#include <stdio.h>
#include <string.h>
#include <time.h>

#define PMIC_EN                0
#define FUEL_GAUGE_EN          1

#define PMIC_DEBUG_PRINT       0
#define FUEL_GAUGE_DEBUG_PRINT 0

#if PMIC_EN
static const uint8_t PMIC_SMBUS_CHANNEL     = 0x01; // Use I2C1
static const uint8_t BQ25700A_SLAVE_ADDRESS = 0x09; // (0x12h / 2) is
                                                    // the slave address
#endif

#if FUEL_GAUGE_EN
static const uint8_t  FUEL_GAUGE_I2C_CHANNEL   = 0x01; // Use I2C1
static const uint16_t BQ34110PWR_SLAVE_ADDRESS = 0x55;
#endif
#if PMIC_EN
static const uint16_t BQ25700A_MANUFACTURER_ID = 0x40; // Manufacturer ID
static const uint16_t BQ25700A_DEVICE_ID       = 0x79; // Device ID
#endif
static const uint32_t I2C_FREQUENCY  = 100000; // Standard frequency
static const uint32_t WAIT_IDLE_TIME = 60;     // 1 minute

/**
 * @brief State machine for background pm handling
 */
enum class pm_handler_state_t : uint8_t
{
    INIT,
    FIRST,
    PMIC_FIRST = FIRST,
#if PMIC_EN
    WRITE_ADC_OPTION = PMIC_FIRST, // Enable ADC options
    WAIT_WRITE_ADC_OPTION,         // Wait for write to be done
    WAIT_ADC_CONVERSION,           // Wait for ADC conversion
    READ_VBAT_VSYS,                // Read VBAT/VSYS register
    WAIT_READ_VBAT_VSYS,           // Wait for read result
    READ_ICHG_ADC,                 // Read Input Charge Register
    WAIT_READ_ICHG_ADC,            // Wait for read result
    READ_IIN_ADC,                  // Read Input Current register
    WAIT_READ_IIN_ADC,             // Wait for read result
    READ_VBUS_ADC,                 // Read VBUS ADC register
    WAIT_READ_VBUS_ADC,            // Wait for read result
    PMIC_END,
#else
    PMIC_END       = PMIC_FIRST,
#endif
    FUEL_GAUGE_FIRST = PMIC_END,
#if FUEL_GAUGE_EN
    READ_FUEL_GAUGE_DATA = FUEL_GAUGE_FIRST,
    WAIT_READ_FUEL_GAUGE_DATA,
    FUEL_GAUGE_END,
#else
    FUEL_GAUGE_END = FUEL_GAUGE_FIRST,
#endif
    START_IDLE = FUEL_GAUGE_END, // Start to wait until next time to poll ADC
    WAIT_IDLE,                   // Wait until next time to poll ADC
};

#if PMIC_EN
/**
 * @brief Register addresses for TI BQ25700A
 */
enum class BQ25700A_REG_ADDRESS : uint8_t
{
    CHARGE_OPTION_0        = 0x12,
    CHARGE_CURRENT         = 0x14,
    MAX_CHARGE_VOLTAGE     = 0x15,
    CHARGE_STATUS          = 0x20,
    PROCHOT_STATUS         = 0x21,
    IIN_DPN                = 0x23,
    VBUS_PSYS_ADC          = 0x23,
    ICHG_IDCHG_ADC         = 0x24,
    IIN_CMPIN_ADC          = 0x25,
    VSYS_VBAT_ADC          = 0x26,
    CHARGE_OPTION_1        = 0x30,
    CHARGE_OPTION_2        = 0x31,
    CHARGE_OPTION_3        = 0x32,
    PROCHOT_OPTION_0       = 0x33,
    PROCHOT_OPTION_1       = 0x34,
    ADC_OPTION             = 0x35,
    OTG_VOLTAGE            = 0x3B,
    OTG_CURRENT            = 0x3C,
    INPUT_VOLTAGE          = 0x3D,
    MINIMUM_SYSTEM_VOLTAGE = 0x3E,
    IIN_HOST               = 0x3F,
    MANUFACTURER_ID        = 0xFE,
    DEVICE_ID              = 0xFF,
};
#endif

#if FUEL_GAUGE_EN
/**
 * @brief Register addresses for TI BQ34110PWR
 */
enum class BQ34110_CMD : uint8_t
{
    CONTROL                     = 0x00,
    VOLTAGE                     = 0x08,
    CURRENT                     = 0x0C,
    REMAINING_CAPACITY          = 0x10,
    FULL_CHARGE_CAPACITY        = 0x12,
    AVERAGE_CURRENT             = 0x14,
    TIME_TO_EMPTY               = 0x16,
    AVERAGE_POWER               = 0x24,
    INTERNAL_TEMPERATURE        = 0x28,
    OPERATION_STATUS            = 0x3A,
    FULL_CHARGE_CAPACITY_DIRECT = 0x12,
    DESIGN_CAPACITY             = 0x3C,
};
#endif

#if FUEL_GAUGE_EN
static constexpr uint8_t  BQ34110_MAC_CONTROL                     = 0x3E;
static constexpr uint8_t  BQ34110_MAC_DATA                        = 0x40;
static constexpr uint8_t  BQ34110_MAC_DATA_SUM                    = 0x60;
static constexpr uint8_t  BQ34110_MAC_DATA_LEN                    = 0x61;
static constexpr uint8_t  BQ34110_MAC_BLOCK_LENGTH                = 32;
static constexpr uint8_t  BQ34110_MAC_RESPONSE_TRAILER_LENGTH     = 2;
static constexpr uint16_t BQ34110_MAC_SUBCMD_RESET                = 0x0041;
static constexpr uint16_t BQ34110_DF_LEARNED_FULL_CHARGE_CAPACITY = 0x40C0;
static constexpr uint32_t BQ34110_DF_TRANSFER_DELAY_MS            = 2;
static constexpr uint32_t BQ34110_DF_COMMIT_DELAY_MS              = 100;
static constexpr uint32_t BQ34110_RESET_SETTLE_DELAY_MS           = 2000;
static constexpr uint32_t BQ34110_RESET_RETRY_COUNT               = 5;
static constexpr uint8_t  BQ34110_MAC_OVERHEAD_LENGTH             = 4;
#endif

static constexpr uint32_t BITS_PER_BYTE = 8;

#if PMIC_EN
/**
 * @brief PMIC management state
 */
struct pmic_state_st
{
    uint8_t  vsys_voltage;   // Digital output of system voltage
    uint8_t  vbat_voltage;   // Digital output of battery voltage
    uint16_t charge_current; // Charge current register value
    uint16_t input_current;  // Input current register value
    uint16_t vbus_voltage;   // Digital output of input voltage
};
#endif

#if FUEL_GAUGE_EN
/**
 * @brief Fuel Gauge management state
 */
struct fuel_gauge_state_st
{
    uint16_t voltage;              // mV
    int16_t  current;              // mA
    uint16_t remaining_capacity;   // mAH
    uint16_t full_charge_capacity; // mAH
    int16_t  average_current;      // mA
    uint16_t time_to_empty;        // minutes
    int16_t  average_power;        // 10 mW
    uint16_t temperature;          // 0.1 Kelvins

    uint8_t  sequence_index; // sequencing index for reading fuel gauge
};
#endif

/**
 * @brief HAL PM management state
 */
struct pm_state_st
{
    pm_handler_state_t handler_state;   // handler state
    time_t             idle_start_time; // time when idle started
    bool               i2c_initialized; // true when PM currently owns I2C1
#if PMIC_EN
    pmic_state_st pmic;
#endif
#if FUEL_GAUGE_EN
    fuel_gauge_state_st fuel_gauge;
#endif
};

#if FUEL_GAUGE_EN
/**
 * @brief Sequence entry for reading values from fuel gauge
 */
struct fuel_gauge_sequence_entry_st
{
    BQ34110_CMD command; // Command
    uint16_t *  p_value; // Value to populate
};
#endif

pm_state_st pm_state;

#if FUEL_GAUGE_EN
/**
 * @brief Sequence index for reading fuel gauge
 */
static fuel_gauge_sequence_entry_st sequence[] = {
    {BQ34110_CMD::VOLTAGE, &pm_state.fuel_gauge.voltage},
    {BQ34110_CMD::CURRENT,
     reinterpret_cast<uint16_t *>(&pm_state.fuel_gauge.current)},
    {BQ34110_CMD::REMAINING_CAPACITY, &pm_state.fuel_gauge.remaining_capacity},
    {BQ34110_CMD::FULL_CHARGE_CAPACITY,
     &pm_state.fuel_gauge.full_charge_capacity},
    {BQ34110_CMD::AVERAGE_CURRENT,
     reinterpret_cast<uint16_t *>(&pm_state.fuel_gauge.average_current)},
    {BQ34110_CMD::TIME_TO_EMPTY, &pm_state.fuel_gauge.time_to_empty},
    {BQ34110_CMD::AVERAGE_POWER,
     reinterpret_cast<uint16_t *>(&pm_state.fuel_gauge.average_power)},
    {BQ34110_CMD::INTERNAL_TEMPERATURE, &pm_state.fuel_gauge.temperature}};
#endif

#if PMIC_EN
/**
 * @brief Read register value
 *
 * @param reg_address register address
 * @return uint16_t register value
 */
static uint16_t
w_hal_pm_read_word_data_non_async(BQ25700A_REG_ADDRESS reg_address)
{
    wHalErrorCode error =
        w_hal_smbus_read_word_data(PMIC_SMBUS_CHANNEL,
                                   BQ25700A_SLAVE_ADDRESS,
                                   static_cast<uint8_t>(reg_address));

    if (error != wHalErrorCode::I2C_READ_COMPLETE)
    {
        while (w_hal_smbus_check_status(PMIC_SMBUS_CHANNEL) !=
               wHalErrorCode::I2C_READ_COMPLETE)
        {
        }
    }

    uint16_t reg_value = w_hal_smbus_get_read_result(PMIC_SMBUS_CHANNEL);

    w_hal_smbus_clear_completion_status(PMIC_SMBUS_CHANNEL);

#if PMIC_DEBUG_PRINT
    printf("PMIC Address 0x%08x Value 0x%08x\n", reg_address, reg_value);
#endif

    return reg_value;
}

/**
 * @brief Write register value
 *
 * @param reg_address register address
 * @param reg_value register value
 */
static void w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS reg_address,
                                               uint16_t             reg_value)
{
    wHalErrorCode error =
        w_hal_smbus_write_word_data(PMIC_SMBUS_CHANNEL,
                                    BQ25700A_SLAVE_ADDRESS,
                                    static_cast<uint8_t>(reg_address),
                                    reg_value);

    if (error != wHalErrorCode::I2C_WRITE_COMPLETE)
    {
        while (w_hal_smbus_check_status(PMIC_SMBUS_CHANNEL) !=
               wHalErrorCode::I2C_WRITE_COMPLETE)
        {
        }
    }

    w_hal_smbus_clear_completion_status(PMIC_SMBUS_CHANNEL);

#if PMIC_DEBUG_PRINT
    printf("PMIC Wrote Address 0x%08x Value 0x%08x\n", reg_address, reg_value);
#endif
}
#endif

/**
 * @brief Initialize I2C interface(s)
 *
 * @return wHalErrorCode NO_ERROR if initialized, error code otherwise
 */
static wHalErrorCode w_hal_pm_i2c_init()
{
    if (pm_state.i2c_initialized)
    {
        return wHalErrorCode::NO_ERROR;
    }

#if PMIC_EN || FUEL_GAUGE_EN
    wHalErrorCode error = wHalErrorCode::NO_ERROR;
#endif

#if PMIC_EN
    error = w_hal_smbus_init(PMIC_SMBUS_CHANNEL, I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
#endif

#if FUEL_GAUGE_EN
#if PMIC_EN
    if (FUEL_GAUGE_I2C_CHANNEL != PMIC_PMIC_SMBUS_CHANNEL)
#endif
    {
        error = w_hal_i2c_init(FUEL_GAUGE_I2C_CHANNEL,
                               whal_i2c_mode::MASTER,
                               I2C_FREQUENCY);
        if (error != wHalErrorCode::NO_ERROR)
        {
            return error;
        }
    }
#endif

    pm_state.i2c_initialized = true;
    return wHalErrorCode::NO_ERROR;
}

#if PMIC_EN || FUEL_GAUGE_EN
/**
 * @brief Disconnect from I2C interface(s)
 *
 * @return wHalErrorCode NO_ERROR if torn down, error code otherwise
 */
static wHalErrorCode w_hal_pm_i2c_teardown()
{
    if (!pm_state.i2c_initialized)
    {
        return wHalErrorCode::NO_ERROR;
    }

    wHalErrorCode error = wHalErrorCode::NO_ERROR;

#if PMIC_EN
    error = w_hal_smbus_teardown(PMIC_SMBUS_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
#endif

#if FUEL_GAUGE_EN
#if PMIC_EN
    if (FUEL_GAUGE_I2C_CHANNEL != PMIC_PMIC_SMBUS_CHANNEL)
#endif
    {
        error = w_hal_i2c_teardown(FUEL_GAUGE_I2C_CHANNEL);
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }
#endif

    pm_state.i2c_initialized = false;
    return error;
}
#endif

/**
 * @brief Initialization for Power Management
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_pm_init()
{
    memset(&pm_state, 0, sizeof(pm_state));
    pm_state.handler_state   = pm_handler_state_t::INIT;
    pm_state.i2c_initialized = false;
#if FUEL_GAUGE_EN
    pm_state.fuel_gauge.sequence_index = 0;
#endif

#if PMIC_EN
    wHalErrorCode error = w_hal_pm_i2c_init();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    uint16_t manufacturer_id = w_hal_pm_read_word_data_non_async(
        BQ25700A_REG_ADDRESS::MANUFACTURER_ID);
    SYSTEM_ASSERT_EQUAL(manufacturer_id, BQ25700A_MANUFACTURER_ID);

    uint16_t device_id =
        w_hal_pm_read_word_data_non_async(BQ25700A_REG_ADDRESS::DEVICE_ID);
    SYSTEM_ASSERT_EQUAL(device_id, BQ25700A_DEVICE_ID);

    w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS::IIN_HOST, 0x0100);

    w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS::IIN_DPN, 0x0100);

    w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS::ADC_OPTION,
                                       0xe0ff);

    w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS::CHARGE_OPTION_0,
                                       0x020e);

    w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS::CHARGE_OPTION_1,
                                       0x1211);

    w_hal_pm_write_word_data_non_async(BQ25700A_REG_ADDRESS::CHARGE_OPTION_2,
                                       0x0237);

    w_hal_pm_write_word_data_non_async(
        BQ25700A_REG_ADDRESS::MINIMUM_SYSTEM_VOLTAGE,
        0x0e00);

    // Teardown and reinitialize at the beginning of runtime
    w_hal_pm_i2c_teardown();
#endif

    return wHalErrorCode::NO_ERROR;
}

#if PMIC_EN
/**
 * @brief Handle an issuing of read for pm handler issue read state
 *
 * @param address address of register
 * @param next_read_state next state to set if read is done
 * @param wait_state next state to set if read is in progress
 * @param read_result [out] register value if read is done, unchanged otherwise
 * @return true if read is done
 * @return false if read is still in progress
 */
static bool w_hal_pm_start_read_state(BQ25700A_REG_ADDRESS address,
                                      pm_handler_state_t   next_read_state,
                                      pm_handler_state_t   wait_state,
                                      uint16_t             read_result)
{
    wHalErrorCode error =
        w_hal_smbus_read_word_data(PMIC_SMBUS_CHANNEL,
                                   BQ25700A_SLAVE_ADDRESS,
                                   static_cast<uint8_t>(address));

    if (error == wHalErrorCode::I2C_READ_COMPLETE)
    {
        // Get read result
        read_result = w_hal_smbus_get_read_result(PMIC_SMBUS_CHANNEL);
        w_hal_smbus_clear_completion_status(PMIC_SMBUS_CHANNEL);

        pm_state.handler_state = next_read_state;
        return true;
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_IN_PROGRESS);
    pm_state.handler_state = wait_state;

    return false;
}

/**
 * @brief Handle a waiting for read done pm handler wait state
 *
 * @param channel i2c channel
 * @param next_read_state next state to set if read is done
 * @param read_result [out] register value if read is done, unchanged otherwise
 * @return true if read is done
 * @return false if read is still in progress
 */
static bool w_hal_pm_wait_read_state(uint8_t            channel,
                                     pm_handler_state_t next_read_state,
                                     uint16_t &         read_result)
{
    wHalErrorCode error = w_hal_smbus_check_status(channel);
    if (error == wHalErrorCode::I2C_READ_IN_PROGRESS)
    {
        return false;
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_COMPLETE);
    read_result = w_hal_smbus_get_read_result(channel);

    w_hal_smbus_clear_completion_status(channel);
    pm_state.handler_state = next_read_state;
    return true;
}
#endif

#if FUEL_GAUGE_EN
/**
 * @brief Get the fuel gauge result after checking status
 */
static void w_hal_pm_get_fuel_gauge_result()
{
    uint16_t read_result = w_hal_smbus_get_read_result(FUEL_GAUGE_I2C_CHANNEL);
    w_hal_smbus_clear_completion_status(FUEL_GAUGE_I2C_CHANNEL);

    *(sequence[pm_state.fuel_gauge.sequence_index].p_value) = read_result;

#if FUEL_GAUGE_DEBUG_PRINT
    printf(
        "Fuel Gauge Read Reg:0x%x Value:0x%x\n",
        static_cast<int>(sequence[pm_state.fuel_gauge.sequence_index].command),
        read_result);
#endif

    if (++pm_state.fuel_gauge.sequence_index ==
        sizeof(sequence) / sizeof(sequence[0]))
    {
        pm_state.fuel_gauge.sequence_index = 0;
        pm_state.handler_state             = pm_handler_state_t::FUEL_GAUGE_END;
    }
    else
    {
        pm_state.handler_state = pm_handler_state_t::READ_FUEL_GAUGE_DATA;
    }
}
#endif

#if FUEL_GAUGE_EN
/**
 * @brief Start a read of fuel gauge data based on the sequencing index
 *
 * @param p_data buffer to store data to be written
 * @param length length of buffer to be written
 * @return true if read is done, false if read is in progress
 */
static wHalErrorCode w_hal_pm_i2c_write_blocking(const uint8_t *p_data,
                                                 uint32_t       length)
{
    wHalErrorCode error = w_hal_i2c_write_data(FUEL_GAUGE_I2C_CHANNEL,
                                               p_data,
                                               length,
                                               false,
                                               BQ34110PWR_SLAVE_ADDRESS,
                                               true);

    if (error == wHalErrorCode::I2C_WRITE_COMPLETE)
    {
        w_hal_i2c_clear_completion_status(FUEL_GAUGE_I2C_CHANNEL);
        return wHalErrorCode::NO_ERROR;
    }

    return error;
}

/**
 * @brief Start a blocking write read transaction to read fuel gauge data
 *
 * @param p_write_data data to write
 * @param write_length length of data to write
 * @param p_read_data buffer to store read data
 * @param read_length length of data to read
 * @return wHalErrorCode::NO_ERROR if read is done,
 * wHalErrorCode::I2C_READ_IN_PROGRESS if read is in progress, other error code
 * otherwise
 */
static wHalErrorCode
w_hal_pm_i2c_write_read_blocking(const uint8_t *p_write_data,
                                 uint32_t       write_length,
                                 uint8_t *      p_read_data,
                                 uint32_t       read_length)
{
    wHalErrorCode error = w_hal_i2c_write_read_data(FUEL_GAUGE_I2C_CHANNEL,
                                                    p_write_data,
                                                    write_length,
                                                    p_read_data,
                                                    read_length,
                                                    false,
                                                    BQ34110PWR_SLAVE_ADDRESS,
                                                    true);

    if (error == wHalErrorCode::I2C_READ_COMPLETE)
    {
        w_hal_i2c_clear_completion_status(FUEL_GAUGE_I2C_CHANNEL);
        return wHalErrorCode::NO_ERROR;
    }

    return error;
}

/**
 * @brief Recover fuel gauge I2C by issuing teardown and init
 */
static void w_hal_pm_recover_fuel_gauge_i2c()
{
    wHalErrorCode error = w_hal_pm_i2c_teardown();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    error = w_hal_pm_i2c_init();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
}

/**
 * @brief Read bytes from fuel gauge starting at a register
 *
 * @param start_register register to start read
 * @param p_read_data buffer to store read data
 * @param read_length length of data to read
 * @return wHalErrorCode::NO_ERROR if read is done, other error code otherwise
 */
static wHalErrorCode w_hal_pm_read_fuel_gauge_bytes(uint8_t  start_register,
                                                    uint8_t *p_read_data,
                                                    uint32_t read_length)
{
    return w_hal_pm_i2c_write_read_blocking(&start_register,
                                            sizeof(start_register),
                                            p_read_data,
                                            read_length);
}

/**
 * @brief Read a word from fuel gauge starting at a register
 *
 * @param command register to read
 * @param value [out] value read if read is done, unchanged otherwise
 * @return wHalErrorCode::NO_ERROR if read is done, other error code otherwise
 */
static wHalErrorCode w_hal_pm_read_fuel_gauge_word(uint8_t   command,
                                                   uint16_t &value)
{
    uint8_t       read_data[2];
    wHalErrorCode error = w_hal_pm_i2c_write_read_blocking(&command,
                                                           sizeof(command),
                                                           read_data,
                                                           sizeof(read_data));

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    value = static_cast<uint16_t>(read_data[0]) |
            (static_cast<uint16_t>(read_data[1]) << BITS_PER_BYTE);
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Read a double word from fuel gauge starting at a register
 *
 * @param df_address address of the double word to read
 * @param value [out] value read if read is done, unchanged otherwise
 * @return wHalErrorCode::NO_ERROR if read is done, other error code otherwise
 */
static wHalErrorCode w_hal_pm_read_fuel_gauge_df_word(uint16_t  df_address,
                                                      uint16_t &value)
{
    const uint8_t mac_control_payload[] = {
        BQ34110_MAC_CONTROL,
        static_cast<uint8_t>(df_address),
        static_cast<uint8_t>(df_address >> BITS_PER_BYTE)};
    wHalErrorCode error =
        w_hal_pm_i2c_write_blocking(mac_control_payload,
                                    sizeof(mac_control_payload));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    uint8_t control_echo[2];
    error = w_hal_pm_read_fuel_gauge_bytes(BQ34110_MAC_CONTROL,
                                           control_echo,
                                           sizeof(control_echo));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    if ((control_echo[0] != static_cast<uint8_t>(df_address)) ||
        (control_echo[1] != static_cast<uint8_t>(df_address >> BITS_PER_BYTE)))
    {
        return wHalErrorCode::I2C_READ_FAILED;
    }

    uint8_t mac_data[BQ34110_MAC_BLOCK_LENGTH];
    error = w_hal_pm_read_fuel_gauge_bytes(BQ34110_MAC_DATA,
                                           mac_data,
                                           sizeof(mac_data));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    uint8_t mac_meta[BQ34110_MAC_RESPONSE_TRAILER_LENGTH];
    error = w_hal_pm_read_fuel_gauge_bytes(BQ34110_MAC_DATA_SUM,
                                           mac_meta,
                                           sizeof(mac_meta));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    const uint8_t mac_checksum = mac_meta[0];
    const uint8_t mac_length   = mac_meta[1];
    if ((mac_length < BQ34110_MAC_OVERHEAD_LENGTH) ||
        (static_cast<uint32_t>(mac_length - BQ34110_MAC_OVERHEAD_LENGTH) >
         BQ34110_MAC_BLOCK_LENGTH))
    {
        return wHalErrorCode::I2C_READ_FAILED;
    }

    const uint8_t data_length =
        static_cast<uint8_t>(mac_length - BQ34110_MAC_OVERHEAD_LENGTH);
    if (data_length < sizeof(uint16_t))
    {
        return wHalErrorCode::I2C_READ_FAILED;
    }

    uint8_t checksum =
        static_cast<uint8_t>(static_cast<uint8_t>(df_address) +
                             static_cast<uint8_t>(df_address >> BITS_PER_BYTE));
    for (uint32_t index = 0; index < data_length; index++)
    {
        checksum = static_cast<uint8_t>(checksum + mac_data[index]);
    }

    checksum = static_cast<uint8_t>(~checksum);
    if (checksum != mac_checksum)
    {
        return wHalErrorCode::I2C_READ_FAILED;
    }

    value = static_cast<uint16_t>(mac_data[0] << BITS_PER_BYTE) |
            static_cast<uint16_t>(mac_data[1]);
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Write a double word to fuel gauge starting at a register
 *
 * @param df_address address of the double word to write
 * @param value value to write
 * @return wHalErrorCode::NO_ERROR if write is done, other error code otherwise
 */
static wHalErrorCode w_hal_pm_write_fuel_gauge_df_word(uint16_t df_address,
                                                       uint16_t value)
{
    // write DF address (little-endian) and payload bytes
    // contiguously starting at ManufacturerAccessControl (0x3E).
    const uint8_t write_df_payload[] = {
        BQ34110_MAC_CONTROL,
        static_cast<uint8_t>(df_address),
        static_cast<uint8_t>(df_address >> BITS_PER_BYTE),
        static_cast<uint8_t>(value >> BITS_PER_BYTE),
        static_cast<uint8_t>(value)};
    wHalErrorCode error =
        w_hal_pm_i2c_write_blocking(write_df_payload, sizeof(write_df_payload));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    const uint8_t checksum = static_cast<uint8_t>(
        ~(static_cast<uint8_t>(write_df_payload[1] + write_df_payload[2] +
                               write_df_payload[3] + write_df_payload[4])));
    static constexpr uint8_t df_write_payload_size =
        4; // 2-byte address + 2-byte value
    static constexpr uint8_t mac_data_length = df_write_payload_size + 2;

    const uint8_t checksum_write[]           = {BQ34110_MAC_DATA_SUM, checksum};
    error = w_hal_pm_i2c_write_blocking(checksum_write, sizeof(checksum_write));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    const uint8_t length_write[] = {BQ34110_MAC_DATA_LEN, mac_data_length};
    error = w_hal_pm_i2c_write_blocking(length_write, sizeof(length_write));
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    w_hal_system_delay_ms(BQ34110_DF_TRANSFER_DELAY_MS);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Issue fuel gauge reset command
 *
 * @return wHalErrorCode::NO_ERROR if reset is done, other error code otherwise
 */
static wHalErrorCode w_hal_pm_fuel_gauge_reset()
{
    const uint8_t reset_subcommand[] = {
        BQ34110_MAC_CONTROL,
        static_cast<uint8_t>(BQ34110_MAC_SUBCMD_RESET),
        static_cast<uint8_t>(BQ34110_MAC_SUBCMD_RESET >> BITS_PER_BYTE)};
    return w_hal_pm_i2c_write_blocking(reset_subcommand,
                                       sizeof(reset_subcommand));
}

/**
 * @brief Set fuel gauge full charge capacity by writing learned full charge
 * capacity DF and issuing reset
 *
 * @param full_charge_capacity_mah full charge capacity in mAh to set
 * @return wHalErrorCode::NO_ERROR if update is successful, other error code
 * otherwise
 */
wHalErrorCode
w_hal_pm_set_fuel_gauge_full_charge_capacity(uint16_t full_charge_capacity_mah)
{
    while ((pm_state.handler_state != pm_handler_state_t::INIT) &&
           (pm_state.handler_state != pm_handler_state_t::WAIT_IDLE))
    {
        w_hal_pm_handler();
    }

    wHalErrorCode init_error = w_hal_pm_i2c_init();
    if (init_error != wHalErrorCode::NO_ERROR)
    {
        return init_error;
    }

    uint16_t      full_charge_capacity_before    = 0;
    uint16_t      full_charge_capacity_after     = 0;
    uint16_t      df_full_charge_capacity_before = 0;
    uint16_t      df_full_charge_capacity_after  = 0;
    wHalErrorCode before_error = w_hal_pm_read_fuel_gauge_word(
        static_cast<uint8_t>(BQ34110_CMD::FULL_CHARGE_CAPACITY_DIRECT),
        full_charge_capacity_before);
    wHalErrorCode df_before_error = w_hal_pm_read_fuel_gauge_df_word(
        BQ34110_DF_LEARNED_FULL_CHARGE_CAPACITY,
        df_full_charge_capacity_before);
    wHalErrorCode error = w_hal_pm_write_fuel_gauge_df_word(
        BQ34110_DF_LEARNED_FULL_CHARGE_CAPACITY,
        full_charge_capacity_mah);
    if (error == wHalErrorCode::NO_ERROR)
    {
        w_hal_system_delay_ms(BQ34110_DF_COMMIT_DELAY_MS);
    }
    wHalErrorCode df_after_error = w_hal_pm_read_fuel_gauge_df_word(
        BQ34110_DF_LEARNED_FULL_CHARGE_CAPACITY,
        df_full_charge_capacity_after);
    wHalErrorCode reset_error = wHalErrorCode::NO_ERROR;

    if (error == wHalErrorCode::NO_ERROR)
    {
        w_hal_system_delay_ms(BQ34110_DF_COMMIT_DELAY_MS);
        for (uint32_t retry_index = 0; retry_index < BQ34110_RESET_RETRY_COUNT;
             retry_index++)
        {
            reset_error = w_hal_pm_fuel_gauge_reset();
            if (reset_error == wHalErrorCode::NO_ERROR)
            {
                break;
            }

            w_hal_pm_recover_fuel_gauge_i2c();
            w_hal_system_delay_ms(BQ34110_DF_COMMIT_DELAY_MS);
        }
    }

    if ((df_after_error != wHalErrorCode::NO_ERROR) ||
        ((reset_error != wHalErrorCode::NO_ERROR) &&
         (reset_error != wHalErrorCode::BUSY)))
    {
        w_hal_pm_recover_fuel_gauge_i2c();
    }

    if (reset_error == wHalErrorCode::NO_ERROR)
    {
        w_hal_system_delay_ms(BQ34110_RESET_SETTLE_DELAY_MS);
        w_hal_pm_recover_fuel_gauge_i2c();
    }

    wHalErrorCode after_error = w_hal_pm_read_fuel_gauge_word(
        static_cast<uint8_t>(BQ34110_CMD::FULL_CHARGE_CAPACITY_DIRECT),
        full_charge_capacity_after);

    wHalErrorCode teardown_error = w_hal_pm_i2c_teardown();

    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_PM_CPP,
                      "BQ34110 FCC update: Runtime FCC before=%u mAh "
                      "(err=%d)\n, Data Flash FCC before=%u mAh (err=%d)\n, "
                      "Target FCC=%u mAh, Write err=%d\n, Data Flash FCC "
                      "after=%u mAh (err=%d)\n, Reset err=%d\n, Runtime FCC "
                      "after=%u mAh (err=%d)\n",
                      full_charge_capacity_before,
                      before_error,
                      df_full_charge_capacity_before,
                      df_before_error,
                      full_charge_capacity_mah,
                      error,
                      df_full_charge_capacity_after,
                      df_after_error,
                      reset_error,
                      full_charge_capacity_after,
                      after_error);

    if (teardown_error != wHalErrorCode::NO_ERROR)
    {
        return teardown_error;
    }
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
    if (df_after_error != wHalErrorCode::NO_ERROR)
    {
        return df_after_error;
    }
    if (reset_error != wHalErrorCode::NO_ERROR)
    {
        return reset_error;
    }
    if (after_error != wHalErrorCode::NO_ERROR)
    {
        return after_error;
    }
    if (full_charge_capacity_after != full_charge_capacity_mah)
    {
        return wHalErrorCode::I2C_WRITE_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}
#endif

/**
 * @brief Background handling for all low-level power management
 *
 * @return wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_pm_handler()
{
#if PMIC_EN
    uint16_t read_result;
#endif

    switch (pm_state.handler_state)
    {
        case pm_handler_state_t::INIT:
        {
            if (shared_glbl.reboot_flag ||
                (shared_glbl.shutdown_stage != shutdown_stage_t::NONE))
            {
                pm_state.handler_state = pm_handler_state_t::WAIT_IDLE;
                break;
            }

            wHalErrorCode error = w_hal_pm_i2c_init();
            if (error == wHalErrorCode::BUSY)
            {
                break;
            }
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

            pm_state.handler_state = pm_handler_state_t::FIRST;
        }
        // FALL-THROUGH
#if PMIC_EN
        case pm_handler_state_t::WRITE_ADC_OPTION:
        {
            wHalErrorCode error = w_hal_smbus_write_word_data(
                PMIC_SMBUS_CHANNEL,
                BQ25700A_SLAVE_ADDRESS,
                static_cast<uint8_t>(BQ25700A_REG_ADDRESS::ADC_OPTION),
                0x60FF);

            if (error == wHalErrorCode::I2C_WRITE_COMPLETE)
            {
                w_hal_smbus_clear_completion_status(PMIC_SMBUS_CHANNEL);
                pm_state.handler_state =
                    pm_handler_state_t::WAIT_ADC_CONVERSION;
                pm_state.idle_start_time = w_hal_rtc_get_time();
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_IN_PROGRESS);
            pm_state.handler_state = pm_handler_state_t::WAIT_WRITE_ADC_OPTION;
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_WRITE_ADC_OPTION:
        {
            wHalErrorCode error = w_hal_smbus_check_status(PMIC_SMBUS_CHANNEL);
            if (error == wHalErrorCode::I2C_WRITE_IN_PROGRESS)
            {
                break;
            }

            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
            w_hal_smbus_clear_completion_status(PMIC_SMBUS_CHANNEL);
            pm_state.handler_state   = pm_handler_state_t::WAIT_ADC_CONVERSION;
            pm_state.idle_start_time = w_hal_rtc_get_time();
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_ADC_CONVERSION:
        {
            // Wait 1 second
            if ((w_hal_rtc_get_time() - pm_state.idle_start_time) < 1)
            {
                break;
            }

            pm_state.handler_state = pm_handler_state_t::READ_VBAT_VSYS;
        }
        // FALL-THROUGH
        case pm_handler_state_t::READ_VBAT_VSYS:
        {
            if (w_hal_pm_start_read_state(
                    BQ25700A_REG_ADDRESS::VSYS_VBAT_ADC,
                    pm_handler_state_t::READ_ICHG_ADC,
                    pm_handler_state_t::WAIT_READ_VBAT_VSYS,
                    read_result))
            {
                pm_state.pmic.vbat_voltage = read_result >> 16;
                pm_state.pmic.vsys_voltage = static_cast<uint8_t>(read_result);
                break;
            }
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_READ_VBAT_VSYS:
        {
            if (!w_hal_pm_wait_read_state(PMIC_SMBUS_CHANNEL,
                                          pm_handler_state_t::READ_ICHG_ADC,
                                          read_result))
            {
                break;
            }

            pm_state.pmic.vbat_voltage = read_result >> 16;
            pm_state.pmic.vsys_voltage = static_cast<uint8_t>(read_result);
        }
        // FALL-THROUGH
        case pm_handler_state_t::READ_ICHG_ADC:
        {
            if (w_hal_pm_start_read_state(
                    BQ25700A_REG_ADDRESS::ICHG_IDCHG_ADC,
                    pm_handler_state_t::READ_IIN_ADC,
                    pm_handler_state_t::WAIT_READ_ICHG_ADC,
                    read_result))
            {
                pm_state.pmic.charge_current = read_result;
                break;
            }
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_READ_ICHG_ADC:
        {
            if (!w_hal_pm_wait_read_state(PMIC_SMBUS_CHANNEL,
                                          pm_handler_state_t::READ_IIN_ADC,
                                          read_result))
            {
                break;
            }

            pm_state.pmic.charge_current = read_result;
        }
        // FALL-THROUGH
        case pm_handler_state_t::READ_IIN_ADC:
        {
            if (w_hal_pm_start_read_state(BQ25700A_REG_ADDRESS::IIN_CMPIN_ADC,
                                          pm_handler_state_t::READ_VBUS_ADC,
                                          pm_handler_state_t::WAIT_READ_IIN_ADC,
                                          read_result))
            {
                pm_state.pmic.input_current = read_result;
                break;
            }
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_READ_IIN_ADC:
        {
            if (!w_hal_pm_wait_read_state(PMIC_SMBUS_CHANNEL,
                                          pm_handler_state_t::READ_VBUS_ADC,
                                          read_result))
            {
                break;
            }

            pm_state.pmic.input_current = read_result;
        }
        // FALL-THROUGH
        case pm_handler_state_t::READ_VBUS_ADC:
        {
            if (w_hal_pm_start_read_state(
                    BQ25700A_REG_ADDRESS::VBUS_PSYS_ADC,
                    pm_handler_state_t::PMIC_END,
                    pm_handler_state_t::WAIT_READ_VBUS_ADC,
                    read_result))
            {
                pm_state.pmic.vbus_voltage = read_result;
                break;
            }
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_READ_VBUS_ADC:
        {
            if (!w_hal_pm_wait_read_state(PMIC_SMBUS_CHANNEL,
                                          pm_handler_state_t::PMIC_END,
                                          read_result))
            {
                break;
            }

            pm_state.pmic.vbus_voltage = read_result;
        }
        // FALL-THROUGH
#endif
#if FUEL_GAUGE_EN
        case pm_handler_state_t::READ_FUEL_GAUGE_DATA:
        {
#if FUEL_GAUGE_DEBUG_PRINT
            printf("Fuel Gauge Reading Reg:0x%x\n",
                   static_cast<int>(
                       sequence[pm_state.fuel_gauge.sequence_index].command));
#endif
            // Interface Format is the same as smbus, so it is
            // fine to use smbus api
            wHalErrorCode error = w_hal_smbus_read_word_data(
                FUEL_GAUGE_I2C_CHANNEL,
                BQ34110PWR_SLAVE_ADDRESS,
                static_cast<uint8_t>(
                    sequence[pm_state.fuel_gauge.sequence_index].command));

            if (error == wHalErrorCode::I2C_READ_COMPLETE)
            {
                w_hal_pm_get_fuel_gauge_result();
                if (pm_state.handler_state !=
                    pm_handler_state_t::FUEL_GAUGE_END)
                {
                    break;
                }
            }
            else
            {
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_IN_PROGRESS);
                pm_state.handler_state =
                    pm_handler_state_t::WAIT_READ_FUEL_GAUGE_DATA;
            }
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_READ_FUEL_GAUGE_DATA:
        {
            if (pm_state.handler_state ==
                pm_handler_state_t::WAIT_READ_FUEL_GAUGE_DATA)
            {
                wHalErrorCode error =
                    w_hal_smbus_check_status(FUEL_GAUGE_I2C_CHANNEL);
                if (error == wHalErrorCode::I2C_READ_IN_PROGRESS)
                {
                    break;
                }

                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_COMPLETE);

                w_hal_pm_get_fuel_gauge_result();

                if (pm_state.handler_state ==
                    pm_handler_state_t::READ_FUEL_GAUGE_DATA)
                {
                    wHalErrorCode next_error = w_hal_smbus_read_word_data(
                        FUEL_GAUGE_I2C_CHANNEL,
                        BQ34110PWR_SLAVE_ADDRESS,
                        static_cast<uint8_t>(
                            sequence[pm_state.fuel_gauge.sequence_index]
                                .command));
                    if (next_error == wHalErrorCode::I2C_READ_COMPLETE)
                    {
                        w_hal_pm_get_fuel_gauge_result();
                    }
                    else
                    {
                        SYSTEM_ASSERT_EQUAL(
                            next_error,
                            wHalErrorCode::I2C_READ_IN_PROGRESS);
                        pm_state.handler_state =
                            pm_handler_state_t::WAIT_READ_FUEL_GAUGE_DATA;
                    }
                }

                if (pm_state.handler_state !=
                    pm_handler_state_t::FUEL_GAUGE_END)
                {
                    break;
                }
            }
        }
#endif
        // FALL-THROUGH
        case pm_handler_state_t::START_IDLE:
        {
#if PMIC_EN || FUEL_GAUGE_EN
            w_hal_pm_i2c_teardown();
#endif
            pm_state.idle_start_time = w_hal_rtc_get_time();
            pm_state.handler_state   = pm_handler_state_t::WAIT_IDLE;
        }
        // FALL-THROUGH
        case pm_handler_state_t::WAIT_IDLE:
        {
            if (shared_glbl.reboot_flag ||
                (shared_glbl.shutdown_stage != shutdown_stage_t::NONE))
            {
                break;
            }

            // Wait 1 minute
            if ((w_hal_rtc_get_time() - pm_state.idle_start_time) >=
                WAIT_IDLE_TIME)
            {
                // Go back to beginning
                pm_state.handler_state = pm_handler_state_t::INIT;
            }

            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    if (pm_state.handler_state == pm_handler_state_t::START_IDLE ||
        pm_state.handler_state == pm_handler_state_t::WAIT_IDLE)
    {
        return wHalErrorCode::NO_ERROR;
    }
    else
    {
        return wHalErrorCode::BUSY;
    }
}

/**
 * @brief To get Fuel gauge information
 *
 * @param voltage[out]: in mV
 * @param current[out]: in mA
 * @param remaining_capacity[out]: in mAh
 * @param full_charge_capacity[out]:in mAh
 * @param average_current[out]:in mA
 * @param time_to_empty[out]:in seconds
 * @param average_power[out]:in mW
 * @param temperature[out]: in Centigrade
 * @return wHalErrorCode PM_INVALID_FUEL_GAUGE_DATA if no valid fuel gauge data
 * is available, otherwise NO_ERROR
 */
wHalErrorCode w_hal_pm_get_fuel_gauge_info(int32_t &voltage,
                                           int32_t &current,
                                           int32_t &remaining_capacity,
                                           int32_t &full_charge_capacity,
                                           int32_t &average_current,
                                           int32_t &time_to_empty,
                                           int32_t &average_power,
                                           int32_t &temperature)
{
    if (pm_state.fuel_gauge.voltage == 0)
    {
        return wHalErrorCode::PM_INVALID_FUEL_GAUGE_DATA;
    }

    voltage            = pm_state.fuel_gauge.voltage;
    current            = pm_state.fuel_gauge.current;
    remaining_capacity = shared_glbl.device_config.num_battery_cells_used *
                         pm_state.fuel_gauge.remaining_capacity;
    full_charge_capacity = shared_glbl.device_config.num_battery_cells_used *
                           pm_state.fuel_gauge.full_charge_capacity;
    average_current = pm_state.fuel_gauge.average_current;
    time_to_empty   = SECS_PER_MIN *
                    shared_glbl.device_config.num_battery_cells_used *
                    pm_state.fuel_gauge.time_to_empty;
    average_power =
        pm_state.fuel_gauge.average_power * 10; // To convert from 10mW to mW

    // To convert from 0.1Kelvin to Centigrade
    temperature = ((pm_state.fuel_gauge.temperature - 2732) / 10);
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief This will print the power management parameters on boot up
 *
 */
void w_hal_pm_print_info()
{
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_PM_CPP,
                      "Battery_voltage: %d mV, "
                      "Battery_current: %d mA, "
                      "Battery_remaining_capacity: %d mAH, "
                      "Battery_full_charge_capacity: %d mAH, "
                      "Battery_average_current: %d mA, "
                      "Battery_time_to_empty: %d sec, "
                      "Battery_average_power: %d mW, "
                      "Battery_temperature: %d C\n",
                      pm_state.fuel_gauge.voltage,
                      pm_state.fuel_gauge.current,
                      shared_glbl.device_config.num_battery_cells_used *
                          pm_state.fuel_gauge.remaining_capacity,
                      shared_glbl.device_config.num_battery_cells_used *
                          pm_state.fuel_gauge.full_charge_capacity,
                      pm_state.fuel_gauge.average_current,
                      SECS_PER_MIN *
                          shared_glbl.device_config.num_battery_cells_used *
                          pm_state.fuel_gauge.time_to_empty,
                      pm_state.fuel_gauge.average_power * 10,
                      ((pm_state.fuel_gauge.temperature - 2732) / 10));
}

/**
 * @brief Teardown the PM
 *
 * @return wHalErrorCode I2C_SHUTDOWN_FAILED if teardown cannot be
 * complete, NO_ERROR otherwise
 */
wHalErrorCode w_hal_pm_teardown()
{
    wHalErrorCode error = wHalErrorCode::NO_ERROR;

    if ((pm_state.handler_state != pm_handler_state_t::INIT) &&
        (pm_state.handler_state != pm_handler_state_t::START_IDLE) &&
        (pm_state.handler_state != pm_handler_state_t::WAIT_IDLE))
    {
        error = wHalErrorCode::BUSY;
        while (error == wHalErrorCode::BUSY)
        {
            error = w_hal_pm_handler();
        }
    }

#if PMIC_EN || FUEL_GAUGE_EN
    error = w_hal_pm_i2c_teardown();
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }
#endif

    pm_state.handler_state = pm_handler_state_t::WAIT_IDLE;

    return wHalErrorCode::NO_ERROR;
}
#endif // __CORE__ == PM_CORE
