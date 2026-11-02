#if __CORE__ == RTC_CORE

#include "w_hal_error_code.hpp"
#include "w_hal_i2c.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_rtc_internal.hpp"
#include "w_hal_system.hpp"

#include "assert.hpp"
#include "shared.hpp"
#include "trace.hpp"

#include <time.h>

static const uint32_t I2C_FREQUENCY              = 100000; // Standard frequency
static const uint32_t I2C_CHANNEL                = 1;      // I2C channel
static const uint32_t I2C_ADDRESS                = 0x6F;   // RTC I2C address
static const uint8_t  I2C_REG_ADDRESS_SIZE_BYTES = 1;
static const uint8_t  I2C_REG_DATA_SIZE_BYTES    = 1;

// Trim Error PPM for RTC.
static const int8_t   TRIM_ERROR_PPM             = -20; // Oscillator is
                                                        // slower than expected.

/**
 * @brief Register addresses for MCP7940N RTC
 */
enum class RTC_I2C_ADDR : uint8_t
{
    RTC_SEC     = 0x00, // RTC seconds register
    RTC_MIN     = 0x01, // RTC minutes register
    RTC_HOUR    = 0x02, // RTC hours register
    RTC_WKDAY   = 0x03, // RTC week day register
    RTC_DATE    = 0x04, // RTC date register
    RTC_MTH     = 0x05, // RTC month register
    RTC_YEAR    = 0x06, // RTC year register
    RTC_CTRL    = 0x07, // RTC control register
    RTC_OSCTRIM = 0x08, // RTC oscillator trim register
};

// Register definitions

/**
 * @brief Register structure for RTC_SEC
 */
union rtc_sec_st
{
    struct
    {
        uint8_t secone : 4;
        uint8_t secten : 3;
        uint8_t st     : 1;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_MIN
 */
union rtc_min_st
{
    struct
    {
        uint8_t minone : 4;
        uint8_t minten : 3;
        uint8_t rsvd   : 1;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_HOUR
 */
union rtc_hour_st
{
    struct
    {
        uint8_t hrone    : 4;
        uint8_t hrten    : 1;
        uint8_t ampm     : 1;
        uint8_t hrtwelve : 1;
        uint8_t rsvd     : 1;
    } twelvehour;
    struct
    {
        uint8_t hrone    : 4;
        uint8_t hrten    : 2;
        uint8_t hrtwelve : 1;
        uint8_t rsvd     : 1;
    } twentyfourhour;
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_WKDAY
 */
union rtc_wkday_st
{
    struct
    {
        uint8_t wkday   : 3;
        uint8_t vbaten  : 1;
        uint8_t pwrfail : 1;
        uint8_t oscrun  : 1;
        uint8_t rsvd    : 2;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_DATE
 */
union rtc_date_st
{
    struct
    {
        uint8_t dateone : 4;
        uint8_t dateten : 2;
        uint8_t rsvd    : 2;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_MTH
 */
union rtc_mth_st
{
    struct
    {
        uint8_t mthone : 4;
        uint8_t mthten : 1;
        uint8_t lp     : 1;
        uint8_t rsvd   : 2;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_YEAR
 */
union rtc_year_st
{
    struct
    {
        uint8_t yearone : 4;
        uint8_t yearten : 4;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_CTRL
 */
union rtc_ctrl_st
{
    struct
    {
        uint8_t sqwfs   : 2;
        uint8_t crstrim : 1;
        uint8_t extosc  : 1;
        uint8_t alm0en  : 1;
        uint8_t alm1en  : 1;
        uint8_t sqwen   : 1;
        uint8_t out     : 1;
    };
    uint8_t reg;
};

/**
 * @brief Register structure for RTC_OSCTRIM
 */
union rtc_osctrim_st
{
    struct
    {
        uint8_t trimval : 7;
        uint8_t sign    : 1;
    };
    uint8_t reg;
};

/**
 * @brief Read a word from the I2C peripheral
 *
 * @param address address of the register to read
 * @return uint8_t data read from the register
 */
static uint8_t i2c_read(uint8_t address)
{
    uint8_t read_word = 0;
    uint8_t read_bytes[I2C_REG_ADDRESS_SIZE_BYTES + I2C_REG_DATA_SIZE_BYTES] = {
        0};

    for (uint32_t index = 0; index < I2C_REG_ADDRESS_SIZE_BYTES; index++)
    {
        read_bytes[index] =
            (uint8_t)(static_cast<uint8_t>(address) >>
                      (CHAR_BIT * (I2C_REG_ADDRESS_SIZE_BYTES - 1 - index)));
    }

    // Fill the rest of the buffer with dummy data. If read is a success, the
    // data will be overwritten.
    for (uint32_t index = 0; index < I2C_REG_DATA_SIZE_BYTES; index++)
    {
        read_bytes[index + I2C_REG_ADDRESS_SIZE_BYTES] = 0xFF;
    }

    wHalErrorCode error =
        w_hal_i2c_write_read_data(I2C_CHANNEL,
                                  read_bytes,
                                  I2C_REG_ADDRESS_SIZE_BYTES,
                                  &read_bytes[I2C_REG_ADDRESS_SIZE_BYTES],
                                  I2C_REG_DATA_SIZE_BYTES,
                                  false,
                                  I2C_ADDRESS,
                                  true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_COMPLETE);

    error = w_hal_i2c_clear_completion_status(I2C_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    for (uint32_t index = 0; index < I2C_REG_DATA_SIZE_BYTES; index++)
    {
        read_word |= read_bytes[index + I2C_REG_ADDRESS_SIZE_BYTES]
                     << (CHAR_BIT * (I2C_REG_DATA_SIZE_BYTES - 1 - index));
    }

    return read_word;
}

/**
 * @brief Read multiple bytes from the I2C peripheral
 *
 * @param address address of the first register to read
 * @param data single byte array to store the read data
 * @param size size of the data array. Number of registers to read will be size
 * of the data array divided by the size of a single register
 */
static void i2c_read_multiple(uint8_t address, uint8_t *data, uint8_t size)
{
    uint8_t address_bytes[I2C_REG_ADDRESS_SIZE_BYTES] = {0};
    for (uint32_t index = 0; index < I2C_REG_ADDRESS_SIZE_BYTES; index++)
    {
        address_bytes[index] =
            (uint8_t)(static_cast<uint8_t>(address) >>
                      (CHAR_BIT * (I2C_REG_ADDRESS_SIZE_BYTES - 1 - index)));
    }

    wHalErrorCode error = w_hal_i2c_write_read_data(I2C_CHANNEL,
                                                    address_bytes,
                                                    I2C_REG_ADDRESS_SIZE_BYTES,
                                                    data,
                                                    size,
                                                    false,
                                                    I2C_ADDRESS,
                                                    true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_COMPLETE);

    error = w_hal_i2c_clear_completion_status(I2C_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
}

/**
 * @brief Write data to the I2C peripheral at the specified address
 *
 * @param address address of the register to write
 * @param data data to write to the register
 */
static void i2c_write(uint8_t address, uint8_t data)
{
    // Assume I2C is already initialized
    wHalErrorCode error =
        w_hal_i2c_write_word<I2C_REG_ADDRESS_SIZE_BYTES,
                             I2C_REG_DATA_SIZE_BYTES>(I2C_CHANNEL,
                                                      address,
                                                      data,
                                                      false,
                                                      I2C_ADDRESS,
                                                      true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
}

/**
 * @brief Get the time from the RTC peripheral
 *
 * @param time [out]: time structure to populate
 */
static void peripheral_get_time(w_hal_rtc_time_st &time)
{
    // Assume I2C is already initialized
    rtc_sec_st   rtc_sec;
    rtc_min_st   rtc_min;
    rtc_hour_st  rtc_hour;
    rtc_wkday_st rtc_wkday;
    rtc_date_st  rtc_date;
    rtc_mth_st   rtc_mth;
    rtc_year_st  rtc_year;

    // The timekeeping registers should be read in a single
    // operation to utilize the on-board buffers and avoid
    // rollover issues.
    uint8_t      rtc_time_data[7 * I2C_REG_DATA_SIZE_BYTES] = {0};
    for (uint32_t index = 0; index < sizeof(rtc_time_data); index++)
    {
        rtc_time_data[index] = 0xFF;
    }

    i2c_read_multiple(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC),
                      rtc_time_data,
                      sizeof(rtc_time_data));

    rtc_sec.reg  = rtc_time_data[0];
    rtc_min.reg  = rtc_time_data[static_cast<uint8_t>(RTC_I2C_ADDR::RTC_MIN) -
                                static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)];
    rtc_hour.reg = rtc_time_data[static_cast<uint8_t>(RTC_I2C_ADDR::RTC_HOUR) -
                                 static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)];
    rtc_wkday.reg =
        rtc_time_data[static_cast<uint8_t>(RTC_I2C_ADDR::RTC_WKDAY) -
                      static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)];
    rtc_date.reg = rtc_time_data[static_cast<uint8_t>(RTC_I2C_ADDR::RTC_DATE) -
                                 static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)];
    rtc_mth.reg  = rtc_time_data[static_cast<uint8_t>(RTC_I2C_ADDR::RTC_MTH) -
                                static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)];
    rtc_year.reg = rtc_time_data[static_cast<uint8_t>(RTC_I2C_ADDR::RTC_YEAR) -
                                 static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)];

    tm time_struct     = {0};

    time_struct.tm_sec = rtc_sec.secone + rtc_sec.secten * 10;
    time_struct.tm_min = rtc_min.minone + rtc_min.minten * 10;
    if (rtc_hour.twelvehour.hrtwelve == 1)
    {
        time_struct.tm_hour = rtc_hour.twelvehour.hrone +
                              rtc_hour.twelvehour.hrten * 10 +
                              rtc_hour.twelvehour.ampm * 12;
    }
    else
    {
        time_struct.tm_hour =
            rtc_hour.twentyfourhour.hrone + rtc_hour.twentyfourhour.hrten * 10;
    }

    // rtc_wkday should be a value between 1 to 7 inclusive
    // tm_wday is a value between 0 to 6 inclusive
    time_struct.tm_wday   = rtc_wkday.wkday - 1;
    // tm_mday is the day of the month, a value between 1 and 31
    // rtc_date is the day of the month, a value between 1 and 31
    time_struct.tm_mday   = rtc_date.dateone + rtc_date.dateten * 10;
    // tm_mon is the month number, a value between 0 and 11
    // rtc_mth is the month number, a value between 1 and 12
    time_struct.tm_mon    = rtc_mth.mthone + rtc_mth.mthten * 10 - 1;
    // tm_year is the number of years since 1900
    // rtc_year is the number of years since 2000
    time_struct.tm_year   = rtc_year.yearone + rtc_year.yearten * 10 + 100;

    time.time_seconds     = w_hal_rtc_timegm(&time_struct);
    time.time_nanoseconds = 0;
}

#if RTC_WAKE != INVALID_CORE_NUM
/**
 * @brief Read Alarm 0 match time from MCP7940N and convert to epoch timestamp
 *
 * @param alarm_time [out]: populates with Alarm 0 epoch time, or invalidates if
 * disabled
 * @param current_time [in]: reference current time (to resolve the year)
 */
static void peripheral_get_alarm0_time(w_hal_rtc_time_st &      alarm_time,
                                       const w_hal_rtc_time_st &current_time)
{
    // Check if ALM0 is enabled in CONTROL register (0x07, bit 4)
    const uint8_t REG_CONTROL = 0x07;
    uint8_t       control_reg = i2c_read(REG_CONTROL);
    if ((control_reg & 0x10) == 0)
    {
        // Alarm 0 is not enabled
        alarm_time.invalidate();
        return;
    }

    // Read Alarm 0 match registers
    uint8_t alm_sec  = i2c_read(0x0A); // REG_ALM0SEC
    uint8_t alm_min  = i2c_read(0x0B); // REG_ALM0MIN
    uint8_t alm_hour = i2c_read(0x0C); // REG_ALM0HOUR
    uint8_t alm_date = i2c_read(0x0E); // REG_ALM0DATE
    uint8_t alm_mth  = i2c_read(0x0F); // REG_ALM0MTH

    // Convert current RTC time to struct tm to extract the current year
    tm      current_tm;
    gmtime_r(&current_time.time_seconds, &current_tm);

    tm alarm_tm      = {0};
    alarm_tm.tm_sec  = (alm_sec & 0x0F) + ((alm_sec >> 4) & 0x07) * 10;
    alarm_tm.tm_min  = (alm_min & 0x0F) + ((alm_min >> 4) & 0x07) * 10;
    alarm_tm.tm_hour = (alm_hour & 0x0F) + ((alm_hour >> 4) & 0x03) * 10;
    alarm_tm.tm_mday = (alm_date & 0x0F) + ((alm_date >> 4) & 0x03) * 10;
    alarm_tm.tm_mon =
        ((alm_mth & 0x0F) + ((alm_mth >> 4) & 0x01) * 10) - 1; // 0-indexed
    alarm_tm.tm_year = current_tm.tm_year; // Years since 1900

    // Handle year rollover (if current month is Dec and alarm month is Jan)
    if (alarm_tm.tm_mon < current_tm.tm_mon)
    {
        alarm_tm.tm_year += 1;
    }

    // Sanity-check month (0-11) and date (1-31)
    if ((alarm_tm.tm_mday >= 1) && (alarm_tm.tm_mday <= 31) &&
        (alarm_tm.tm_mon >= 0) && (alarm_tm.tm_mon <= 11))
    {
        alarm_time.time_seconds     = w_hal_rtc_timegm(&alarm_tm);
        alarm_time.time_nanoseconds = 0;
        if (alarm_time <= current_time)
        {
            alarm_time.invalidate();
        }
    }
    else
    {
        alarm_time.invalidate();
    }
}
#endif

/**
 * @brief Set the time in the RTC peripheral
 *
 * @param time[in]: time structure to set
 */
static void peripheral_set_time(w_hal_rtc_time_st &time)
{
    // Assume I2C is already initialized
    rtc_sec_st   rtc_sec;
    rtc_min_st   rtc_min;
    rtc_hour_st  rtc_hour;
    rtc_wkday_st rtc_wkday;
    rtc_date_st  rtc_date;
    rtc_mth_st   rtc_mth;
    rtc_year_st  rtc_year;

    tm           time_struct = {0};
    gmtime_r(&time.time_seconds, &time_struct);

    // Keep the oscillator running.
    rtc_sec.reg                      = 0;
    rtc_sec.st                       = 1;
    rtc_sec.secone                   = time_struct.tm_sec % 10;
    rtc_sec.secten                   = time_struct.tm_sec / 10;

    rtc_min.reg                      = 0;
    rtc_min.minone                   = time_struct.tm_min % 10;
    rtc_min.minten                   = time_struct.tm_min / 10;

    // Silicon Errata Issue 1: The MCP7940N has a bug in 12-hour mode where
    // the hour register is not updated correctly when the hour is set to 11.
    // Don't use 12-hour mode to avoid this issue.
    rtc_hour.reg                     = 0;
    rtc_hour.twentyfourhour.hrtwelve = 0;
    rtc_hour.twentyfourhour.hrone    = time_struct.tm_hour % 10;
    rtc_hour.twentyfourhour.hrten    = time_struct.tm_hour / 10;

    rtc_wkday.reg                    = 0;
    // Keep battery backup running
    rtc_wkday.vbaten                 = 1;
    // wkday should be a value between 1 to 7 inclusive
    // tm_wday is a value between 0 to 6 inclusive
    rtc_wkday.wkday                  = time_struct.tm_wday + 1;

    // tm_mday is the day of the month, a value between 1 and 31
    // rtc_date is the day of the month, a value between 1 and 31
    rtc_date.reg                     = 0;
    rtc_date.dateone                 = time_struct.tm_mday % 10;
    rtc_date.dateten                 = time_struct.tm_mday / 10;

    // tm_mon is the month number, a value between 0 and 11
    // rtc_mth is the month number, a value between 1 and 12
    rtc_mth.reg                      = 0;
    // To be a leap year, the year number must be divisible by 4,
    // except for end-of-century years, which must be divisible by 400.
    if ((((time_struct.tm_year % 4) == 0) &&
         ((time_struct.tm_year % 100) != 0)) ||
        ((time_struct.tm_year % 400) == 0))
    {
        rtc_mth.lp = 1;
    }
    else
    {
        rtc_mth.lp = 0;
    }
    rtc_mth.mthone   = (time_struct.tm_mon + 1) % 10;
    rtc_mth.mthten   = (time_struct.tm_mon + 1) / 10;

    // tm_year is the number of years since 1900
    // rtc_year is the number of years since 2000
    rtc_year.reg     = 0;
    rtc_year.yearone = (time_struct.tm_year - 100) % 10;
    rtc_year.yearten = (time_struct.tm_year - 100) / 10;

    // First byte is the address of the register to write.
    // The rest of the bytes are the data to write to consecutive registers.
    uint8_t rtc_time_data[I2C_REG_ADDRESS_SIZE_BYTES +
                          (7 * I2C_REG_DATA_SIZE_BYTES)] = {0};

    rtc_time_data[0] = static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC);
    rtc_time_data[1] = rtc_sec.reg;
    rtc_time_data[1 + static_cast<uint8_t>(RTC_I2C_ADDR::RTC_MIN) -
                  static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)] = rtc_min.reg;
    rtc_time_data[1 + static_cast<uint8_t>(RTC_I2C_ADDR::RTC_HOUR) -
                  static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)] = rtc_hour.reg;
    rtc_time_data[1 + static_cast<uint8_t>(RTC_I2C_ADDR::RTC_WKDAY) -
                  static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)] = rtc_wkday.reg;
    rtc_time_data[1 + static_cast<uint8_t>(RTC_I2C_ADDR::RTC_DATE) -
                  static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)] = rtc_date.reg;
    rtc_time_data[1 + static_cast<uint8_t>(RTC_I2C_ADDR::RTC_MTH) -
                  static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)] = rtc_mth.reg;
    rtc_time_data[1 + static_cast<uint8_t>(RTC_I2C_ADDR::RTC_YEAR) -
                  static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC)] = rtc_year.reg;

    // Assume I2C is already initialized
    wHalErrorCode error = w_hal_i2c_write_data(I2C_CHANNEL,
                                               rtc_time_data,
                                               sizeof(rtc_time_data),
                                               false,
                                               I2C_ADDRESS,
                                               true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);

    error = w_hal_i2c_clear_completion_status(I2C_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Silicon Errata Issue 3: The MCP7940N has a bug when writing a new value
    // in the Year, Month or Date registers. The Date register value may change
    // unexpectedly. To avoid this issue, always write the Date, month, and year
    // value in that order. Then write the date value again.
    i2c_write(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_DATE), rtc_date.reg);

    // Silicon Errata Issue 4: The MCP7940N has a bug when the RTCWKDAY register
    // is written when the oscillator is stopped, it is possible that the value
    // will read back as a different value. To avoid this issue, read the
    // back the value to confirm it is correct and if not, write the value
    // again.
    uint8_t wkday_read =
        i2c_read(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_WKDAY));
    if (wkday_read != rtc_wkday.reg)
    {
        i2c_write(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_WKDAY), rtc_wkday.reg);
    }
}

/**
 * @brief Setup the RTC peripheral trim value
 */
static void peripheral_setup_trim()
{
    // Disable coarse trim mode so that the fine trim value can be set
    rtc_ctrl_st rtc_ctrl;
    rtc_ctrl.reg     = i2c_read(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_CTRL));
    rtc_ctrl.crstrim = 0;
    i2c_write(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_CTRL), rtc_ctrl.reg);

    // Set the fine trim value for the RTC
    rtc_osctrim_st rtc_osctrim;
    rtc_osctrim.reg = 0;

    int trim_value  = (TRIM_ERROR_PPM * 32768 * 60) / 1000000 / 2;
    trim_value      = std::min(trim_value, 127);
    trim_value      = std::max(trim_value, -127);
    if (trim_value < 0)
    {
        rtc_osctrim.trimval = -trim_value;
        rtc_osctrim.sign    = 1;
    }
    else
    {
        rtc_osctrim.trimval = trim_value;
        rtc_osctrim.sign    = 0;
    }
    i2c_write(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_OSCTRIM), rtc_osctrim.reg);
}

/**
 * @brief Initialize the RTC peripheral
 *
 * @param time[in/out] : reference to the time structure. The RTC time will
 * be updated to the time in the structure or the time in the structure will
 * be updated to the RTC time.
 * @return wHalErrorCode NO_ERROR if successful, RTC_TIME_UPDATED if the
 * argument time was updated with the peripheral, error code otherwise
 */
wHalErrorCode w_hal_rtc_init_peripheral(w_hal_rtc_time_st &time)
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;

    wHalErrorCode error =
        w_hal_i2c_init(I2C_CHANNEL, whal_i2c_mode::MASTER, I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Check if the RTC is running
    bool       osc_running = true;
    rtc_sec_st rtc_sec;
    rtc_sec.reg = i2c_read(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC));
    if (rtc_sec.st)
    {
        osc_running = true;
    }
    else
    {
        osc_running = false;

        for (uint32_t attempt = 0; attempt < 10; attempt++)
        {
            // Enable the oscillator
            rtc_sec.st = 1;
            i2c_write(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC), rtc_sec.reg);

            // Wait for the oscillator to stabilize
            w_hal_system_delay_ms(1);

            // Read the oscillator enable bit
            rtc_sec.reg = i2c_read(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_SEC));

            // Poll the oscillator run bit to check if the oscillator is running
            for (uint32_t index = 0; index < 256; index++)
            {
                rtc_wkday_st rtc_wkday = {0};
                rtc_wkday.reg =
                    i2c_read(static_cast<uint8_t>(RTC_I2C_ADDR::RTC_WKDAY));

                if (rtc_wkday.oscrun == 1)
                {
                    osc_running = true;
                    break;
                }

                w_hal_system_delay_ms(1);
            }

            if (osc_running)
            {
                break;
            }
        }
    }

    // Check if the oscillator was started successfully
    if (osc_running)
    {
        peripheral_setup_trim();

#if RTC_WAKE != INVALID_CORE_NUM
        const uint8_t REG_ALM0WKDAY = 0x0D;
        uint8_t       alm0_wkday    = i2c_read(REG_ALM0WKDAY);
        bool          alarm_fired   = (alm0_wkday & 0x08) != 0; // Bit 3: ALM0IF
        // Store in shared global custom flags
        shared_glbl.custom_flags.rtc_mfp_wake_flag = alarm_fired ? 1 : 0;

        const uint8_t REG_CONTROL                  = 0x07;
        uint8_t       control_reg                  = i2c_read(REG_CONTROL);
        bool          alm0_en                      = (control_reg & 0x10) != 0;
        bool          out_bit                      = (control_reg & 0x80) != 0;
#endif

        // Either set the RTC time to the argument time or
        // read the time from the RTC and update the internal RTC time.
        w_hal_rtc_time_st peripheral_time;
        peripheral_get_time(peripheral_time);

#if RTC_WAKE != INVALID_CORE_NUM
        // Power-retention and alarm servicing:
        // When woken by Alarm 0, ALM0IF in REG_ALM0WKDAY must be cleared so it
        // does not persist as a stale interrupt flag on subsequent software
        // resets/boots. Additionally, REG_CONTROL is set to 0x80 (OUT=1,
        // ALM0EN=0) to keep the MFP pin asserted HIGH for hardware power
        // retention (holding Quadbuck enable) during runtime and across future
        // reboots.
        if (alarm_fired)
        {
            // Clear ALM0IF by writing 0 to bit 3 of ALM0WKDAY (preserving
            // polarity and mask)
            uint8_t clear_alm0if_val = static_cast<uint8_t>(alm0_wkday & ~0x08);
            i2c_write(REG_ALM0WKDAY, clear_alm0if_val);
            // Disable Alarm 0 and assert OUT=1 (MFP HIGH) for power retention
            uint8_t out_high_val = 0x80;
            i2c_write(REG_CONTROL, out_high_val);
        }
        else if (!alm0_en)
        {
            // If Alarm 0 is not enabled, ensure OUT=1 (0x80) so MFP is HIGH
            // (power retention)
            if (!out_bit)
            {
                uint8_t out_high_val = 0x80;
                i2c_write(REG_CONTROL, out_high_val);
            }
            // Also ensure any stale ALM0IF is cleared
            if (alm0_wkday & 0x08)
            {
                uint8_t clear_alm0if_val =
                    static_cast<uint8_t>(alm0_wkday & ~0x08);
                i2c_write(REG_ALM0WKDAY, clear_alm0if_val);
            }
        }
        // If the RTC time is less than the argument time, update the RTC time
        // to the argument time.
        peripheral_get_alarm0_time(shared_glbl.next_wakeup_time_utc,
                                   peripheral_time);
#endif

        // Compare the time from the RTC with the time in the argument
        // and update the RTC time if necessary.
        if (peripheral_time > time)
        {
            time         = peripheral_time;
            return_error = wHalErrorCode::RTC_TIME_UPDATED;
        }
        else if (peripheral_time < time)
        {
            // Set the time in the RTC peripheral
            peripheral_set_time(time);
        }
    }
    else
    {
        return_error = wHalErrorCode::RTC_OSCILLATOR_NOT_RUNNING;
    }

    error = w_hal_i2c_teardown(I2C_CHANNEL);
    if (error != wHalErrorCode::NO_ERROR)
    {
        if (return_error == wHalErrorCode::NO_ERROR)
        {
            return_error = error;
        }
    }

    return return_error;
}

/**
 * @brief Set the time in the RTC peripheral
 *
 * @param time [in]: time structure to set
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_rtc_peripheral_set_time(w_hal_rtc_time_st &time)
{
    wHalErrorCode error =
        w_hal_i2c_init(I2C_CHANNEL, whal_i2c_mode::MASTER, I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Set the time in the RTC peripheral
    peripheral_set_time(time);

    error = w_hal_i2c_teardown(I2C_CHANNEL);

    return error;
}

#endif // __CORE__ == RTC_CORE