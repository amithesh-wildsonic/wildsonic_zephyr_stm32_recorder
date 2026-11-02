#include "BleComm.hpp"
#include "OnBootConfig.hpp"
#include "assert.hpp"
#include "console.hpp"
#include "device_config.hpp"
#include "globals.hpp"
#include "shared.hpp"
#include "trace.hpp"
#include "version.hpp"

#if __CORE__ == COMM_CORE
#include "w_hal_gsm.hpp"
#endif
#if __CORE__ == AUDIO_CORE
#include "w_hal_i2s.hpp"
#endif
#if __CORE__ == OFF_CHIP_MEMORY_CORE
#include "w_hal_ocm.hpp"
#endif
#if __CORE__ == PM_CORE
#include "w_hal_pm.hpp"
#endif
#include "w_hal_rtc.hpp"
#if __CORE__ == STORAGE_CORE
#include "w_hal_sd_card.hpp"
#endif
#include "w_hal_system.hpp"
#if __CORE__ == CORE_0
#include "mxc_delay.h"
#include "w_hal_i2c.hpp"
#endif

globals_t                      glbl;

SHARED shared_t                shared_glbl;
SHARED_BUFFER shared_buffers_t shared_buffers_glbl;

#if __CORE__ == COMM_CORE
/**
 * @brief Initialize GSM and set the time
 */
static void initialize_gsm_and_set_time()
{
    wHalErrorCode error_code = w_hal_gsm_init(*DeviceConfig::p_page);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    time_t            current_time = w_hal_gsm_get_init_time();

    w_hal_rtc_time_st rtc_time;
    rtc_time.time_seconds     = current_time;
    rtc_time.time_nanoseconds = 0;
    error_code                = w_hal_rtc_set_time(rtc_time);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    w_hal_gsm_print_info();
}
#endif // __CORE__ == COMM_CORE

#if __CORE__ == CORE_0
/**
 * @brief Write a value to a register in the RTC over I2C
 */
static constexpr uint8_t  RTC_I2C_CHANNEL       = 1;
static constexpr uint16_t RTC_I2C_ADDRESS       = 0x6F;
static constexpr uint32_t RTC_I2C_SPEED_HZ      = 100000;
static constexpr uint32_t RTC_WRITE_DELAY_MS    = 5;
static constexpr uint32_t SHUTDOWN_DELAY_MS     = 100;

// MCP7940 Register addresses
static constexpr uint8_t  MCP7940_REG_CONTROL   = 0x07;
static constexpr uint8_t  MCP7940_REG_ALM0SEC   = 0x0A;
static constexpr uint8_t  MCP7940_REG_ALM0MIN   = 0x0B;
static constexpr uint8_t  MCP7940_REG_ALM0HOUR  = 0x0C;
static constexpr uint8_t  MCP7940_REG_ALM0WKDAY = 0x0D;
static constexpr uint8_t  MCP7940_REG_ALM0DATE  = 0x0E;
static constexpr uint8_t  MCP7940_REG_ALM0MTH   = 0x0F;

// MCP7940 Control & Alarm Bit definitions
static constexpr uint8_t  MCP7940_OUT_HIGH =
    0x80; // OUT=1 (MFP HIGH: power retention hold)
static constexpr uint8_t MCP7940_ALM0_ENABLE =
    0x10; // ALM0EN=1 (MFP driven by Alarm 0)
static constexpr uint8_t MCP7940_ALMPOL_HIGH =
    0x80; // ALMPOL=1 (asserted HIGH on match, unasserted LOW)
static constexpr uint8_t MCP7940_ALM0MSK_ALL =
    0x70; // ALM0MSK=111 (match all: sec, min, hour, wkday, date, mth)

static inline uint8_t dec_to_bcd(uint8_t val)
{
    return static_cast<uint8_t>(((val / 10) << 4) | (val % 10));
}

static bool write_rtc_register(uint8_t reg, uint8_t val)
{
    uint8_t       data[2] = {reg, val};
    wHalErrorCode error   = w_hal_i2c_write_data(RTC_I2C_CHANNEL,
                                               data,
                                               sizeof(data),
                                               false,
                                               RTC_I2C_ADDRESS,
                                               true);
    if (error != wHalErrorCode::I2C_WRITE_COMPLETE)
    {
        return false;
    }
    w_hal_i2c_clear_completion_status(RTC_I2C_CHANNEL);
    MXC_Delay(MXC_DELAY_MSEC(RTC_WRITE_DELAY_MS));
    return true;
}

static bool read_rtc_register(uint8_t reg, uint8_t &val)
{
    wHalErrorCode error = w_hal_i2c_write_read_data(RTC_I2C_CHANNEL,
                                                    &reg,
                                                    1,
                                                    &val,
                                                    1,
                                                    false,
                                                    RTC_I2C_ADDRESS,
                                                    true);
    if (error != wHalErrorCode::I2C_READ_COMPLETE)
    {
        return false;
    }
    w_hal_i2c_clear_completion_status(RTC_I2C_CHANNEL);
    return true;
}
#endif // __CORE__ == CORE_0

#if __CORE__ == CORE_0
/**
 * @brief
 * Initialize hal components to boot and apply device configurations
 */
static void hal_boot_init()
{
    wHalErrorCode error_code;

    error_code = w_hal_system_init(*DeviceConfig::p_page,
                                   &shared_glbl.time,
                                   &shared_glbl.time_updated);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

#if __CORE__ == COMM_CORE
    initialize_gsm_and_set_time();
#endif // __CORE__ == COMM_CORE

#if __CORE__ == STORAGE_CORE
    error_code = w_hal_sd_card_init(*DeviceConfig::p_page, &shared_glbl.time);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

    // Device configuration by SD_card config.ini file
    bool new_configuration = read_config_from_sd_card();

    if (new_configuration)
    {
        // Teardown the SD card which was initialized
        error_code = w_hal_sd_card_tear_down();
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);

#if __CORE__ == COMM_CORE
        // Teardown the GSM which was initialized
        error_code = w_hal_gsm_teardown();
        SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == COMM_CORE

        // Update the device config parameter
        DeviceConfig::write_config_page_to_flash();

#if RTC_CORE != INVALID_CORE_NUM
        // Reset RTC Alarm 0 so the new SD configuration starts fresh.
        // Maintain OUT=1 (0x80) in REG_CONTROL so MFP stays HIGH (power
        // retention) across the reset! Also ensure P1.7 (EN_uC_CTL) is driven
        // HIGH as output to hold regulator enable.
        mxc_gpio_cfg_t gpio_pwr_hold;
        gpio_pwr_hold.port  = MXC_GPIO1;
        gpio_pwr_hold.mask  = MXC_GPIO_PIN_7;
        gpio_pwr_hold.func  = MXC_GPIO_FUNC_OUT;
        gpio_pwr_hold.pad   = MXC_GPIO_PAD_NONE;
        gpio_pwr_hold.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&gpio_pwr_hold);
        MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_7);
#endif

        w_hal_system_teardown();

        // Reboot the complete system
        w_hal_system_reset();
    }
#endif // __CORE__ == STORAGE_CORE
}
#endif // __CORE__ == CORE_0

/**
 * @brief
 * Initialize hal components
 */
static void hal_init()
{
#if (__CORE__ != CORE_0) || (__CORE__ == OFF_CHIP_MEMORY_CORE) ||              \
    (__CORE__ == AUDIO_CORE)
    wHalErrorCode error_code;
#endif

    // This function is called for Core 0 in function hal_boot_init()
#if __CORE__ != CORE_0
    error_code = w_hal_system_init(*DeviceConfig::p_page,
                                   &shared_glbl.time,
                                   &shared_glbl.time_updated);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif

#if __CORE__ == COMM_CORE && (__CORE__ != CORE_0)
    initialize_gsm_and_set_time();
#endif // __CORE__ == COMM_CORE

#if __CORE__ == OFF_CHIP_MEMORY_CORE
    error_code = w_hal_ocm_init(*DeviceConfig::p_page);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
    shared_glbl.external_memory_size = w_hal_ocm_get_size();
#endif

#if (__CORE__ == STORAGE_CORE) && (__CORE__ != CORE_0)
    error_code = w_hal_sd_card_init(*DeviceConfig::p_page, &shared_glbl.time);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == STORAGE_CORE

#if __CORE__ == AUDIO_CORE
    error_code = w_hal_i2s_init(*DeviceConfig::p_page);
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == AUDIO_CORE
}

/**
 * @brief
 * Terminate hal components
 */
static void hal_terminate()
{
#if (__CORE__ == PM_CORE) || (__CORE__ == AUDIO_CORE) ||                       \
    (__CORE__ == STORAGE_CORE) || (__CORE__ == OFF_CHIP_MEMORY_CORE) ||        \
    (__CORE__ == COMM_CORE)
    wHalErrorCode error_code;
#endif

#if __CORE__ == PM_CORE
    error_code = w_hal_pm_teardown();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == PM_CORE

#if __CORE__ == AUDIO_CORE
    error_code = w_hal_i2s_teardown();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == AUDIO_CORE

#if __CORE__ == STORAGE_CORE
    error_code = w_hal_sd_card_tear_down();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == STORAGE_CORE

#if __CORE__ == OFF_CHIP_MEMORY_CORE
    error_code = w_hal_ocm_teardown();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == OFF_CHIP_MEMORY_CORE

#if __CORE__ == COMM_CORE
    error_code = w_hal_gsm_teardown();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
#endif // __CORE__ == COMM_CORE

#if __CORE__ == CORE_0
    // Check for apply flag before system teardown which will be set after ARBT
    // command is sent and write the parameters to flash
    if (shared_glbl.save_config_flag)
    {
        DeviceConfig::write_config_page_to_flash();
    }
#endif

    w_hal_system_teardown();

    printf("Core %d HAL termination is done\n", __CORE__);
}

/**
 * @brief
 * Perform second stage initialization of hal components
 */
static void hal_second_init()
{
    wHalErrorCode error_code;

    error_code = w_hal_system_second_init();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
}

/**
 * @brief
 * Finish remaining initialization of hal components
 */
static void hal_last_init()
{
    wHalErrorCode error_code;

    error_code = w_hal_system_last_init();
    SYSTEM_ASSERT_EQUAL(error_code, wHalErrorCode::NO_ERROR);
}

/**
 * @brief
 * Structure initialization and object binding
 */
static void initialize()
{
#if __CORE__ == CORE_0
    // This needs to be done before HAL initialization since RTC needs an
    // initial time value
    shared_glbl.time                = DeviceConfig::get_minimum_start_time();
    shared_glbl.time_updated        = 0;
    shared_glbl.shutdown_stage      = shutdown_stage_t::NONE;
    shared_glbl.reboot_flag         = false;
    shared_glbl.save_config_flag    = false;
    shared_glbl.format_sd_card_flag = false;
    shared_glbl.power_down_flag     = false;
    shared_glbl.custom_flags.reg    = 0;
    shared_glbl.next_wakeup_time_utc.invalidate();
    memset(&shared_glbl.power_mgmt_info,
           0,
           sizeof(shared_glbl.power_mgmt_info));
    shared_glbl.power_mgmt_info.low_battery_flag         = false;
    shared_glbl.power_mgmt_info.temperature              = INT32_MIN;
    shared_glbl.power_mgmt_info.time_to_empty            = INT32_MIN;
    shared_glbl.shared_assert_info.asserted              = false;
    shared_glbl.storage_info.seconds_recorded_since_boot = 0;
    shared_glbl.update_config_status_flag                = false;
    DeviceConfig::p_page->latitude_e7  = INVALID_LOCATION_COORDINATE;
    DeviceConfig::p_page->longitude_e7 = INVALID_LOCATION_COORDINATE;
    shared_glbl.ble_pairing_window_request_pending = 0;
    shared_glbl.ble_pairing_window_request_seconds = 0;
    memset(shared_glbl.core_init_done, 0, sizeof(shared_glbl.core_init_done));
    memset(shared_glbl.core_tasks_done, 0, sizeof(shared_glbl.core_tasks_done));
    memset(shared_glbl.core_shut_done, 0, sizeof(shared_glbl.core_shut_done));

    // HAL initialization will add to the trace log.
    // Trace must be initialized before here.
    Trace::initialize();

    DeviceConfig::initialize(shared_glbl.device_config, false);
#else
    DeviceConfig::set_config_page(shared_glbl.device_config);
#endif

#if __CORE__ == CORE_0
    hal_boot_init();
#endif

    hal_init();

#if __CORE__ == CONSOLE_CORE
    console_init();
#endif

#if __CORE__ == BLE_CORE
    ble_init();
#endif

#if __CORE__ == CORE_0
    shared_glbl.queues_info.detect_request_queue.initialize();
    shared_glbl.queues_info.storage_request_queue.initialize();
    shared_glbl.queues_info.comm_request_queue.initialize();
    shared_glbl.queues_info.ocm_read_request_queue.initialize();
    shared_glbl.queues_info.ocm_write_request_queue.initialize();

    shared_glbl.queues_info.detect_response_queue.initialize();
    shared_glbl.queues_info.storage_response_queue.initialize();
    shared_glbl.queues_info.comm_response_queue.initialize();
    shared_glbl.queues_info.ocm_response_queue.initialize();

    shared_glbl.queues_info.comm_hp_request_queue.initialize();
    shared_glbl.queues_info.comm_hp_response_queue.initialize();
#endif

    glbl.task_manager.initialize();

    glbl.admin_task.initialize();
#if __CORE__ == AUDIO_CORE
    glbl.audio_task.initialize(shared_glbl.queues_info);
#endif
#if __CORE__ == COMM_CORE
    glbl.comm_task.initialize(shared_glbl.queues_info);
#endif
#if __CORE__ == DETECT_CORE
    glbl.detect_task.initialize(shared_glbl.queues_info);
#endif
#if __CORE__ == OFF_CHIP_MEMORY_CORE
    glbl.ocm_task.initialize(shared_glbl.queues_info);
#endif
#if __CORE__ == STORAGE_CORE
    glbl.storage_task.initialize(shared_glbl.queues_info);
#endif

    // Add tasks to task manager
    glbl.task_manager.add_task(glbl.admin_task);
#if __CORE__ == AUDIO_CORE
    glbl.task_manager.add_task(glbl.audio_task);
#endif
#if __CORE__ == COMM_CORE
    glbl.task_manager.add_task(glbl.comm_task);
#endif
#if __CORE__ == DETECT_CORE
    glbl.task_manager.add_task(glbl.detect_task);
#endif
#if __CORE__ == OFF_CHIP_MEMORY_CORE
    glbl.task_manager.add_task(glbl.ocm_task);
#endif
#if __CORE__ == STORAGE_CORE
    glbl.task_manager.add_task(glbl.storage_task, 2);
#endif

    hal_second_init();

    // Core initialization is done
    shared_glbl.core_init_done[__CORE__] = 1;
}

/**
 * @brief
 * Wait for the other cores to come up
 */
static void wait_for_other_cores_initialized()
{
    for (uint32_t core_index = 0; core_index < NUM_CORES; core_index++)
    {
        volatile int *p_core_init_done = static_cast<volatile int *>(
            &shared_glbl.core_init_done[core_index]);
        while (*p_core_init_done == 0)
        {
            // DO NOTHING
        }
    }

    hal_last_init();
}

/**
 * @brief Wait for all the tasks on other cores to go to sleep
 *
 */
static void wait_for_other_cores_tasks_sleep()
{
    for (uint32_t core_index = 0; core_index < NUM_CORES; core_index++)
    {
        volatile int *p_core_tasks_done = static_cast<volatile int *>(
            &shared_glbl.core_tasks_done[core_index]);
        while (*p_core_tasks_done == 0)
        {
            // DO NOTHING
        }
    }
}

#if __CORE__ == CORE_0

/**
 * @brief Wait for other core tasks to terminate
 *
 */
static void wait_for_other_cores_terminate()
{
    for (uint32_t core_index = 0; core_index < NUM_CORES; core_index++)
    {
        if (core_index == __CORE__)
        {
            continue;
        }

        volatile int *p_core_shut_done = static_cast<volatile int *>(
            &shared_glbl.core_shut_done[core_index]);
        while (*p_core_shut_done == 0)
        {
            // DO NOTHING
        }
    }
}
#endif

/**
 * @brief
 * main entry point function
 *
 * @return int error code 0
 */
int main()
{
#if __CORE__ == CORE_0
    print_fw_version_information();
#endif

    // Perform setup
    initialize();
    LOG_MSG(TraceFile::MAIN_CPP, "Initialization Complete\n");

    wait_for_other_cores_initialized();

    // Function will never return
    glbl.task_manager.run_loop();

    shared_glbl.core_tasks_done[__CORE__] = 1;
    wait_for_other_cores_tasks_sleep();

// If core is CORE_0 wait for the other core tasks to terminate else directly
// perform shutdown of the core without waiting
#if __CORE__ == CORE_0
    wait_for_other_cores_terminate();
#endif

#if __CORE__ == CORE_0
    if (shared_glbl.power_down_flag)
    {
        // Configure Pin as output to Hold Quadbuck while setting RTC MFP pin
        // polarity
        mxc_gpio_cfg_t gpio_1_7;
        gpio_1_7.port  = MXC_GPIO1;
        gpio_1_7.mask  = MXC_GPIO_PIN_7;
        gpio_1_7.func  = MXC_GPIO_FUNC_OUT;
        gpio_1_7.pad   = MXC_GPIO_PAD_NONE;
        gpio_1_7.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&gpio_1_7);

        MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_7);
        printf(
            "Latch GPIO set HIGH (securing SoC power during configuration)\n");
        shared_glbl.custom_flags.mcu_wake_flag =
            1; // Set the flag to indicate MCU holding the power latch

        w_hal_rtc_time_st cur_time;
        w_hal_rtc_get_time(cur_time);

        if (!shared_glbl.next_wakeup_time_utc.is_valid() ||
            (shared_glbl.next_wakeup_time_utc <= cur_time))
        {
            printf("Abort power down: Invalid alarm target time. Maintaining "
                   "power.\n");
            shared_glbl.power_down_flag = false;
        }
        else
        {
            // Initialize I2C channel
            wHalErrorCode i2c_error = w_hal_i2c_init(RTC_I2C_CHANNEL,
                                                     whal_i2c_mode::MASTER,
                                                     RTC_I2C_SPEED_HZ);
            if (i2c_error == wHalErrorCode::NO_ERROR)
            {
                struct tm alarm_tm;
                gmtime_r(&shared_glbl.next_wakeup_time_utc.time_seconds,
                         &alarm_tm);
                // Prepare Alarm 0 register values in BCD format
                uint8_t alarm_sec   = dec_to_bcd(alarm_tm.tm_sec);
                uint8_t alarm_min   = dec_to_bcd(alarm_tm.tm_min);
                uint8_t alarm_hour  = dec_to_bcd(alarm_tm.tm_hour);
                uint8_t alarm_date  = dec_to_bcd(alarm_tm.tm_mday);
                uint8_t alarm_mth   = dec_to_bcd(alarm_tm.tm_mon + 1);
                uint8_t alarm_wkday = static_cast<uint8_t>(
                    MCP7940_ALMPOL_HIGH | MCP7940_ALM0MSK_ALL |
                    ((alarm_tm.tm_wday + 1) & 0x07));

                // Step 1: Temporarily disable Alarm 0 while writing match
                // registers
                bool write_ok = write_rtc_register(MCP7940_REG_CONTROL, 0x00);

                // Step 2: Write all Alarm 0 match registers
                if (write_ok)
                    write_ok =
                        write_rtc_register(MCP7940_REG_ALM0SEC, alarm_sec);
                if (write_ok)
                    write_ok =
                        write_rtc_register(MCP7940_REG_ALM0MIN, alarm_min);
                if (write_ok)
                    write_ok =
                        write_rtc_register(MCP7940_REG_ALM0HOUR, alarm_hour);
                if (write_ok)
                    write_ok =
                        write_rtc_register(MCP7940_REG_ALM0DATE, alarm_date);
                if (write_ok)
                    write_ok =
                        write_rtc_register(MCP7940_REG_ALM0MTH, alarm_mth);
                if (write_ok)
                    write_ok =
                        write_rtc_register(MCP7940_REG_ALM0WKDAY, alarm_wkday);

                // Step 3: Enable Alarm 0 in CONTROL register (ALM0EN=1, OUT=0)
                if (write_ok)
                    write_ok = write_rtc_register(MCP7940_REG_CONTROL,
                                                  MCP7940_ALM0_ENABLE);

                // Step 4: Read back and verify all registers
                uint8_t rb_sec = 0, rb_min = 0, rb_hour = 0, rb_date = 0,
                        rb_mth = 0, rb_wkday = 0, rb_ctrl = 0;
                bool read_ok = true;
                read_ok &= read_rtc_register(MCP7940_REG_ALM0SEC, rb_sec);
                read_ok &= read_rtc_register(MCP7940_REG_ALM0MIN, rb_min);
                read_ok &= read_rtc_register(MCP7940_REG_ALM0HOUR, rb_hour);
                read_ok &= read_rtc_register(MCP7940_REG_ALM0DATE, rb_date);
                read_ok &= read_rtc_register(MCP7940_REG_ALM0MTH, rb_mth);
                read_ok &= read_rtc_register(MCP7940_REG_ALM0WKDAY, rb_wkday);
                read_ok &= read_rtc_register(MCP7940_REG_CONTROL, rb_ctrl);

                bool verify_ok =
                    write_ok && read_ok && (rb_sec == alarm_sec) &&
                    (rb_min == alarm_min) && (rb_hour == alarm_hour) &&
                    (rb_date == alarm_date) && (rb_mth == alarm_mth) &&
                    (rb_wkday == alarm_wkday) &&
                    ((rb_ctrl & MCP7940_ALM0_ENABLE) != 0);

                if (verify_ok)
                {
                    printf("RTC Alarm configured for deep sleep.\n");
                }
                else
                {
                    printf(
                        "RTC Alarm configuration verification failed. "
                        "Restoring power retention and aborting shutdown.\n");
                    write_rtc_register(
                        MCP7940_REG_CONTROL,
                        MCP7940_OUT_HIGH); // Ensure OUT=1 so MFP is HIGH
                    shared_glbl.power_down_flag = false;
                }

                // Teardown I2C channel
                w_hal_i2c_teardown(RTC_I2C_CHANNEL);
            }
            else
            {
                printf("Failed to acquire I2C for RTC alarm programming (err "
                       "%d). Aborting shutdown.\n",
                       static_cast<int>(i2c_error));
                shared_glbl.power_down_flag = false;
            }
        }
    }
    else if (shared_glbl.save_config_flag)
    {
        // ARBT / config save reboot path:
        // Must maintain power retention across reset!
        // 1. Explicitly drive P1.7 (EN_uC_CTL) HIGH to hold Quadbuck enable
        mxc_gpio_cfg_t gpio_pwr_hold;
        gpio_pwr_hold.port  = MXC_GPIO1;
        gpio_pwr_hold.mask  = MXC_GPIO_PIN_7;
        gpio_pwr_hold.func  = MXC_GPIO_FUNC_OUT;
        gpio_pwr_hold.pad   = MXC_GPIO_PAD_NONE;
        gpio_pwr_hold.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&gpio_pwr_hold);
        MXC_GPIO_OutSet(MXC_GPIO1, MXC_GPIO_PIN_7);

        // 2. Put RTC in power-retention mode (OUT=1, ALM0EN=0) so MFP stays
        // HIGH during NVIC_SystemReset()
        wHalErrorCode rtc_init_err = w_hal_i2c_init(RTC_I2C_CHANNEL,
                                                    whal_i2c_mode::MASTER,
                                                    RTC_I2C_SPEED_HZ);
        if (rtc_init_err == wHalErrorCode::NO_ERROR)
        {
            // Write 0x80: OUT=1 (MFP HIGH: power retention), ALM0EN=0 (alarm
            // disabled)
            bool w1 = write_rtc_register(MCP7940_REG_CONTROL, MCP7940_OUT_HIGH);
            // Write 0x00: Clear ALM0IF flag
            bool w2 = write_rtc_register(MCP7940_REG_ALM0WKDAY, 0x00);
            if (!w1 || !w2)
            {
                printf("[ARBT/save_config] Failed to set RTC power retention: "
                       "reg0x07=%s reg0x0D=%s\n",
                       w1 ? "OK" : "FAIL",
                       w2 ? "OK" : "FAIL");
            }

            w_hal_i2c_teardown(RTC_I2C_CHANNEL);
        }
        else
        {
            printf("[ARBT/save_config] Failed to acquire I2C for RTC power "
                   "retention (err %d)\n",
                   static_cast<int>(rtc_init_err));
        }
    }
#endif

    // Perform Shutdown
    hal_terminate();

    // Core termination is done
    shared_glbl.core_shut_done[__CORE__] = 1;
#if __CORE__ == CORE_0
    if (shared_glbl.power_down_flag)
    {
        printf("Cutting power...\n");
        shared_glbl.custom_flags.mcu_wake_flag =
            0; // Clear the flag to indicate MCU releasing the power latch
        MXC_Delay(MXC_DELAY_MSEC(SHUTDOWN_DELAY_MS)); // Delay for stability
        MXC_GPIO_OutClr(MXC_GPIO1, MXC_GPIO_PIN_7);   // Drive latch Pin LOW
        while (true)
        {
            // Wait for physical power shutdown
        }
    }
#endif

// If core is CORE_0 reset the system otherwise stay in a busy loop
#if __CORE__ == CORE_0
    if (shared_glbl.reboot_flag)
    {
        w_hal_system_reset();
    }
#else
    while (true)
    {
        // Wait for system reset
    }
#endif

    return 0;
}