#if __CORE__ == BLE_CORE

#include "BleComm.hpp"
#include "console.hpp"
#include "constants.hpp"
#include "device_config.hpp"
#include "shared.hpp"
#include "trace.hpp"
#include "version.hpp"

#include "w_hal_common.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_uart.hpp"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const uint32_t MAX_UART_CMD_LEN = 80; // Maximum length of command
                                             // received over BLE UART
static const size_t   MAX_BLE_ADVERTISED_NAME_LENGTH = 12;
static const char     BLE_NAME_PREFIX[]              = "WS:";
static const char     BLE_NAME_FALLBACK_NAME[]       = "XXX";
static const char     BLE_NAME_FALLBACK_SN[]         = "00";
static const char     BLE_PAIRING_PASSWORD[]         = "123456";
static const uint32_t BLE_INIT_MAX_ATTEMPTS =
    10; // maximum AT handshake retries
static const uint32_t BLE_SLEEP_MAX_ATTEMPTS = 10;  // maximum sleep retries
static const uint32_t BLE_CMD_DELAY_MS       = 100; // delay between AT commands
static const uint32_t BLE_CMD_RESET_DELAY_MS =
    500; // delay after reset command to allow BLE module to reboot
#if !defined(BLE_RESPONSE_DELAY_MS)
static const uint32_t BLE_RESPONSE_DELAY_MS =
    100; // delay between response lines (pacing for module FIFO)
#endif
static const uint32_t    BLE_WAKE_MAX_ATTEMPTS       = 5;
static const uint32_t    BLE_PAIR_WINDOW_MAX_SECONDS = 600;
static const uint32_t    BLE_CMD_TIMEOUT_MS          = 150;
static const uint32_t    BLE_RESET_TIMEOUT_MS        = 700;
static const uint32_t    BLE_COOLDOWN_MS             = 250;

static volatile uint8_t  ble_init_complete           = 0;

extern const char *const fw_version;
extern const char *const git_branch;
extern const char *const git_sha;
extern const char *const build_datetime;

static char              uart_ble_cmd[MAX_UART_CMD_LEN] =
    ""; // Buffer to accumulate incoming
        // characters for command formation
static char ble_response_buffer[MAX_UART_CMD_LEN]; // Buffer to format response
                                                   // to be sent over BLE UART
static volatile uint8_t ble_wake_flag =
    0; // Flag to indicate if BLE module is in sleep mode
static volatile uint8_t  ble_lock_request_pending  = 0;
static uint8_t           ble_pairing_window_active = 0;
static w_hal_rtc_time_st ble_pairing_window_end_time;
static volatile uint8_t  ble_evt_ok    = 0;
static volatile uint8_t  ble_evt_wake  = 0;
static volatile uint8_t  ble_evt_sleep = 0;
static uint16_t ble_uart_char_cnt = 0; // Counter for tracking command length
static uint16_t ble_uart_cmd_len  = 0; // Variable to track command length
static volatile uint8_t ble_get_device_info_pending = 0;
static volatile uint8_t ble_get_status_info_pending = 0;

enum class BleSupervisorState : uint8_t
{
    IDLE,
    WAIT_WAKE,
    WAIT_ADTY_ACK,
    WAIT_RESET_WAKE,
    WAIT_SLEEP_ACK,
    COOLDOWN
};

enum class BleConnectabilityTarget : uint8_t
{
    LOCKED,
    UNLOCKED
};

struct BleSupervisor
{
    BleSupervisorState      state;
    BleConnectabilityTarget target;
    uint8_t                 target_dirty;
    uint8_t                 retry_count;
    uint64_t                deadline_ms;
    uint64_t                cooldown_until_ms;
};

static BleSupervisor ble_supervisor =
    {BleSupervisorState::IDLE, BleConnectabilityTarget::LOCKED, 0, 0, 0, 0};

/**
 * @brief Func to handle incoming character
 *
 * @param c is the character returned from terminal
 */
static void new_ble_character_handler(char c);

/**
 * @brief Get device information
 */
static void get_device_info();
static void get_status_info();
static void set_ble_connectability(bool connectable);
static void lock_ble_connectability();
#if !defined(BLE_CORE_CUSTOM)
static void lock_ble_connectability_blocking_init();
#endif // !defined(BLE_CORE_CUSTOM)
static void     process_pairing_window_request();
static void     process_pairing_window_timeout();
static uint64_t get_ble_ms_tick();
static void     service_ble_supervisor(bool     event_ok,
                                       bool     event_wake,
                                       bool     event_sleep,
                                       uint64_t now_ms);

/**
 * @brief Printf-style function for BLE UART output
 * Format and send data to BLE UART like printf
 *
 * @param format format string (same as printf)
 * @param ... variable arguments
 * @return number of characters sent (or 0 on error)
 */
static int      ble_printf(const char *format, ...);
#if !defined(BLE_CORE_CUSTOM)
/**
 * @brief Wake BLE module by sending "AT" and waiting for an "OK" response.
 *
 * @param attempts maximum retries
 * @return true if module wakes
 * @return false on timeout
 */
static bool wake_ble_module(uint32_t attempts);
#endif // !defined(BLE_CORE_CUSTOM)

/**
 * @brief Reset procedure for console commands: clear counter and command buffer
 *
 */
static void reset_ble_cmd()
{
    ble_uart_char_cnt = 0;
    ble_uart_cmd_len  = 0;
    memset(uart_ble_cmd, 0, (sizeof(char) * MAX_UART_CMD_LEN));
}

/**
 * @brief Monotonic milliseconds used by non-blocking BLE FSM timeouts.
 */
static uint64_t get_ble_ms_tick()
{
    w_hal_rtc_time_st now;
    w_hal_rtc_get_time(now);
    return (static_cast<uint64_t>(now.time_seconds) * 1000ULL) +
           static_cast<uint64_t>(now.time_nanoseconds / 1000000ULL);
}

/**
 * @brief Non-blocking BLE connectability supervisor.
 */
static void service_ble_supervisor(bool     event_ok,
                                   bool     event_wake,
                                   bool     event_sleep,
                                   uint64_t now_ms)
{
    if ((shared_glbl.custom_flags.ble_connection_flag == 1) &&
        (ble_supervisor.state != BleSupervisorState::IDLE))
    {
        ble_supervisor.state        = BleSupervisorState::IDLE;
        ble_supervisor.retry_count  = 0;
        ble_supervisor.target_dirty = 0;
        return;
    }

    switch (ble_supervisor.state)
    {
        case BleSupervisorState::IDLE:
            if (shared_glbl.custom_flags.ble_connection_flag == 1 ||
                ble_supervisor.target_dirty == 0 ||
                now_ms < ble_supervisor.cooldown_until_ms)
            {
                return;
            }

            ble_supervisor.retry_count = 0;
            if (ble_wake_flag == 0)
            {
                ble_printf("AT\r\n");
                ble_supervisor.state       = BleSupervisorState::WAIT_WAKE;
                ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
            }
            else
            {
                ble_printf("AT+ADTY0\r\n");
                ble_supervisor.state       = BleSupervisorState::WAIT_ADTY_ACK;
                ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
            }
            break;

        case BleSupervisorState::WAIT_WAKE:
            if ((ble_wake_flag == 1) || event_ok || event_wake)
            {
                ble_printf("AT+ADTY0\r\n");
                ble_supervisor.state       = BleSupervisorState::WAIT_ADTY_ACK;
                ble_supervisor.retry_count = 0;
                ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
            }
            else if (now_ms >= ble_supervisor.deadline_ms)
            {
                if (ble_supervisor.retry_count < BLE_WAKE_MAX_ATTEMPTS)
                {
                    ble_supervisor.retry_count++;
                    ble_printf("AT\r\n");
                    ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
                }
                else
                {
                    ble_supervisor.state = BleSupervisorState::COOLDOWN;
                    ble_supervisor.cooldown_until_ms = now_ms + BLE_COOLDOWN_MS;
                }
            }
            break;

        case BleSupervisorState::WAIT_ADTY_ACK:
            if (event_ok || event_wake)
            {
                ble_printf("AT+RESET\r\n");
                ble_wake_flag        = 0;
                ble_supervisor.state = BleSupervisorState::WAIT_RESET_WAKE;
                ble_supervisor.retry_count = 0;
                ble_supervisor.deadline_ms = now_ms + BLE_RESET_TIMEOUT_MS;
            }
            else if (now_ms >= ble_supervisor.deadline_ms)
            {
                if (ble_supervisor.retry_count < BLE_WAKE_MAX_ATTEMPTS)
                {
                    ble_supervisor.retry_count++;
                    ble_printf("AT+ADTY0\r\n");
                    ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
                }
                else
                {
                    ble_supervisor.state = BleSupervisorState::COOLDOWN;
                    ble_supervisor.cooldown_until_ms = now_ms + BLE_COOLDOWN_MS;
                }
            }
            break;

        case BleSupervisorState::WAIT_RESET_WAKE:
            if ((ble_wake_flag == 1) || event_wake)
            {
                if (ble_supervisor.target == BleConnectabilityTarget::LOCKED)
                {
                    ble_printf("AT+SLEEP\r\n");
                    ble_supervisor.state = BleSupervisorState::WAIT_SLEEP_ACK;
                    ble_supervisor.retry_count = 0;
                    ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
                }
                else
                {
                    ble_supervisor.target_dirty = 0;
                    ble_supervisor.state        = BleSupervisorState::IDLE;
                }
            }
            else if (now_ms >= ble_supervisor.deadline_ms)
            {
                if (ble_supervisor.retry_count < BLE_WAKE_MAX_ATTEMPTS)
                {
                    ble_supervisor.retry_count++;
                    ble_printf("AT\r\n");
                    ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
                }
                else
                {
                    ble_supervisor.state = BleSupervisorState::COOLDOWN;
                    ble_supervisor.cooldown_until_ms = now_ms + BLE_COOLDOWN_MS;
                }
            }
            break;

        case BleSupervisorState::WAIT_SLEEP_ACK:
            if ((ble_wake_flag == 0) || event_sleep)
            {
                ble_supervisor.target_dirty = 0;
                ble_supervisor.state        = BleSupervisorState::IDLE;
            }
            else if (now_ms >= ble_supervisor.deadline_ms)
            {
                if (ble_supervisor.retry_count < BLE_SLEEP_MAX_ATTEMPTS)
                {
                    ble_supervisor.retry_count++;
                    ble_printf("AT+SLEEP\r\n");
                    ble_supervisor.deadline_ms = now_ms + BLE_CMD_TIMEOUT_MS;
                }
                else
                {
                    ble_supervisor.state = BleSupervisorState::COOLDOWN;
                    ble_supervisor.cooldown_until_ms = now_ms + BLE_COOLDOWN_MS;
                }
            }
            break;

        case BleSupervisorState::COOLDOWN:
            if (now_ms >= ble_supervisor.cooldown_until_ms)
            {
                ble_supervisor.state        = BleSupervisorState::IDLE;
                ble_supervisor.retry_count  = 0;
                ble_supervisor.target_dirty = 0;
            }
            break;
    }
}

/**
 * @brief Set target BLE connectability mode for background FSM service.
 *
 * @param connectable true to allow connections, false to lock
 */
static void set_ble_connectability(bool connectable)
{
    if (shared_glbl.custom_flags.ble_connection_flag == 1)
    {
        return;
    }

    ble_supervisor.target = connectable ? BleConnectabilityTarget::UNLOCKED :
                                          BleConnectabilityTarget::LOCKED;
    ble_supervisor.target_dirty = 1;
    if (ble_supervisor.state != BleSupervisorState::IDLE)
    {
        ble_supervisor.state       = BleSupervisorState::IDLE;
        ble_supervisor.retry_count = 0;
    }
}

/**
 * @brief Lock BLE connectability and clear open pairing window state.
 * Non-blocking: schedules lock for background FSM servicing.
 */
static void lock_ble_connectability()
{
    ble_pairing_window_active = 0;
    set_ble_connectability(false);
    printf("BLE Access: LOCKED\n");
}

#if !defined(BLE_CORE_CUSTOM)
/**
 * @brief Blocking lock helper used during boot-time init only.
 */
static void lock_ble_connectability_blocking_init()
{
    ble_pairing_window_active        = 0;
    const uint32_t LOCK_MAX_ATTEMPTS = 5;

    for (uint32_t attempt = 0; attempt < LOCK_MAX_ATTEMPTS; attempt++)
    {
        if (shared_glbl.custom_flags.ble_connection_flag == 1)
        {
            break;
        }

        if (ble_wake_flag == 0)
        {
            for (uint32_t wake_attempt = 0;
                 wake_attempt < BLE_WAKE_MAX_ATTEMPTS;
                 wake_attempt++)
            {
                ble_printf("AT\r\n");
                w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
                if (ble_wake_flag == 1)
                {
                    break;
                }
            }
        }

        if (ble_wake_flag == 0)
        {
            w_hal_system_delay_ms(BLE_CMD_RESET_DELAY_MS);
            continue;
        }

        ble_printf("AT+ADTY0\r\n");
        w_hal_system_delay_ms(BLE_CMD_DELAY_MS);

        ble_printf("AT+RESET\r\n");
        ble_wake_flag = 0;
        w_hal_system_delay_ms(BLE_CMD_RESET_DELAY_MS);

        for (uint32_t wake_attempt = 0; wake_attempt < BLE_WAKE_MAX_ATTEMPTS;
             wake_attempt++)
        {
            if (ble_wake_flag == 1)
            {
                break;
            }
            ble_printf("AT\r\n");
            w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
        }

        if (ble_wake_flag == 1)
        {
            break;
        }
    }
}
#endif // !defined(BLE_CORE_CUSTOM)

/**
 * @brief Process requested pairing window from console core.
 */
static void process_pairing_window_request()
{
    if (ble_init_complete == 0) // ADD: ignore requests during init
    {
        shared_glbl.ble_pairing_window_request_pending = 0;
        return;
    }
    if (shared_glbl.ble_pairing_window_request_pending == 0)
    {
        return;
    }

    uint32_t window_seconds = shared_glbl.ble_pairing_window_request_seconds;
    shared_glbl.ble_pairing_window_request_pending = 0;

    if (window_seconds == 0)
    {
        window_seconds = BLE_PAIR_WINDOW_DEFAULT_SECONDS;
    }
    else if (window_seconds > BLE_PAIR_WINDOW_MAX_SECONDS)
    {
        window_seconds = BLE_PAIR_WINDOW_MAX_SECONDS;
    }

    if (shared_glbl.custom_flags.ble_connection_flag == 1)
    {
        ble_pairing_window_active = 0;
        return;
    }

    // If a relock is pending from a prior disconnect event, this explicit
    // open request should take precedence.
    ble_lock_request_pending = 0;
    set_ble_connectability(true);
    if (shared_glbl.custom_flags.ble_connection_flag == 1)
    {
        ble_pairing_window_active = 0;
        return;
    }

    w_hal_rtc_time_st now;
    w_hal_rtc_get_time(now);
    ble_pairing_window_end_time = now;
    ble_pairing_window_end_time += w_hal_rtc_time_delta_st(window_seconds, 0);
    ble_pairing_window_active = 1;
    printf("BLE Access: UNLOCKED (%u s)\n", window_seconds);
}

/**
 * @brief Auto-lock BLE when pairing window timeout has elapsed.
 */
static void process_pairing_window_timeout()
{
    if ((ble_pairing_window_active == 0) ||
        (shared_glbl.custom_flags.ble_connection_flag == 1))
    {
        return;
    }

    w_hal_rtc_time_st now;
    w_hal_rtc_get_time(now);
    if (now >= ble_pairing_window_end_time)
    {
        lock_ble_connectability();
    }
}

/**
 * @brief Initialize BLE communication
 */
void ble_init()
{
    reset_ble_cmd();

    // Initialize UART_0 for BLE communication and register the callback for
    // incoming characters
    wHalErrorCode error = w_hal_uart_ble_init(new_ble_character_handler);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

#if !defined(BLE_CORE_CUSTOM)
    // Stock HM10 only: full AT handshake, name set, ADTY2, SLEEP
    char ble_advertised_name[MAX_BLE_ADVERTISED_NAME_LENGTH + 1] = "";

    // 1. First wake up if it is in sleep by sending "AT" and wait for "OK".
    //    Do not block forever, use bounded retries.
    ble_printf("AT\r\n");
    ble_printf("WS+SYNC\r\n");
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);

    uint32_t handshake_attempts = 0;
    while ((ble_wake_flag == 0) && (handshake_attempts < BLE_INIT_MAX_ATTEMPTS))
    {
        ble_printf("AT\r\n");
        w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
        handshake_attempts++;
    }

    if (ble_wake_flag == 0)
    {
        printf("BLE init: no AT response after %u attempts; continuing without "
               "BLE initialization\n",
               handshake_attempts);
        ble_init_complete =
            1; // Mark init complete to avoid blocking pairing window requests
        return;
    }

    // 2. Set device name once BLE module acknowledges handshake.
    {
        const char *p_name   = DeviceConfig::p_page->device_name;
        const char *p_sn     = DeviceConfig::p_page->serial_number;
        size_t      name_len = strlen(p_name);
        size_t      sn_len   = strlen(p_sn);
        bool        name_valid =
            (name_len > 0) && (name_len <= MAX_DEVICE_NAME_LENGTH);
        bool sn_valid = (sn_len > 0) && (sn_len <= MAX_SERIAL_NUMBER_LENGTH);

        for (size_t i = 0; name_valid && (i < name_len); i++)
        {
            if (!isalnum(static_cast<unsigned char>(p_name[i])))
            {
                name_valid = false;
            }
        }

        for (size_t i = 0; sn_valid && (i < sn_len); i++)
        {
            if (!isalnum(static_cast<unsigned char>(p_sn[i])))
            {
                sn_valid = false;
            }
        }

        if (!name_valid)
        {
            p_name   = BLE_NAME_FALLBACK_NAME;
            name_len = strlen(p_name);
        }

        if (!sn_valid)
        {
            p_sn   = BLE_NAME_FALLBACK_SN;
            sn_len = strlen(p_sn);
        }

        if ((name_len == 0) && (sn_len == 0))
        {
            snprintf(ble_advertised_name,
                     sizeof(ble_advertised_name),
                     "%s%s-%s",
                     BLE_NAME_PREFIX,
                     BLE_NAME_FALLBACK_NAME,
                     BLE_NAME_FALLBACK_SN);
        }
        else if (name_len == 0)
        {
            snprintf(ble_advertised_name,
                     sizeof(ble_advertised_name),
                     "%s%s-%s",
                     BLE_NAME_PREFIX,
                     BLE_NAME_FALLBACK_NAME,
                     p_sn);
        }
        else if (sn_len == 0)
        {
            snprintf(ble_advertised_name,
                     sizeof(ble_advertised_name),
                     "%s%s-%s",
                     BLE_NAME_PREFIX,
                     p_name,
                     BLE_NAME_FALLBACK_SN);
        }
        else
        {
            snprintf(ble_advertised_name,
                     sizeof(ble_advertised_name),
                     "%s%s-%s",
                     BLE_NAME_PREFIX,
                     p_name,
                     p_sn);
        }
    }

    if (strlen(ble_advertised_name) > MAX_BLE_ADVERTISED_NAME_LENGTH)
    {
        printf("BLE init: failed to build BLE name\n");
        return;
    }

    if (ble_printf("AT+NAME%s\r\n", ble_advertised_name) < 0)
    {
        printf("BLE init: failed to send BLE name command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);

    if (ble_printf("AT+TYPE3\r\n") < 0)
    {
        printf("BLE init: failed to send BLE type3 command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);

    if (ble_printf("AT+PASS%s\r\n", BLE_PAIRING_PASSWORD) < 0)
    {
        printf("BLE init: failed to send BLE password command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
#if RTC_WAKE != INVALID_CORE_NUM
    if (ble_printf("AT+UART1\r\n") < 0)
    {
        printf("BLE init: failed to send BLE UART into sleep command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
#endif
    if (ble_printf("AT+PWRM0\r\n") < 0)
    {
        printf("BLE init: failed to send PWRM command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
    if (ble_printf("AT+MODE2\r\n") < 0)
    {
        printf("BLE init: failed to send MODE command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_DELAY_MS);

    if (ble_printf("AT+RESET\r\n") < 0)
    {
        printf("BLE init: failed to send reset command\n");
        return;
    }
    w_hal_system_delay_ms(BLE_CMD_RESET_DELAY_MS);

    // After reset, force a fresh wake handshake before entering sleep.
    ble_wake_flag = 0;
    if (!wake_ble_module(BLE_INIT_MAX_ATTEMPTS))
    {
        printf(
            "BLE init: no AT response after reset; skipping sleep command\n");
        return;
    }

    // 3. Start in locked mode: visible, not connectable.
    lock_ble_connectability_blocking_init();
    // 4. Put device in sleep mode after configuration.
    uint32_t sleep_attempts = 0;
    while ((ble_wake_flag == 1) && (sleep_attempts < BLE_SLEEP_MAX_ATTEMPTS))
    {
        ble_printf("AT+SLEEP\r\n");
        w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
        sleep_attempts++;
    }

    if (ble_wake_flag == 1)
    {
        printf("BLE init: sleep handshake not received after %u attempts; "
               "continuing in best-effort mode\n",
               sleep_attempts);
    }

    // Add an extra settling delay before marking init complete, so any
    // late OK+WAKE from the AT+RESET in lock_ble_connectability_blocking_init()
    // already arrived and been absorbed before the TaskManager::run_loop()
    // starts processing pairing window requests.
    w_hal_system_delay_ms(BLE_CMD_RESET_DELAY_MS);
#endif // !defined(BLE_CORE_CUSTOM)

    ble_init_complete = 1;
}

#if !defined(BLE_CORE_CUSTOM)
static bool wake_ble_module(uint32_t attempts)
{
    uint32_t handshake_attempts = 0;
    while ((ble_wake_flag == 0) && (handshake_attempts < attempts))
    {
        if (ble_printf("AT\r\n") < 0)
        {
            return false;
        }
        w_hal_system_delay_ms(BLE_CMD_DELAY_MS);
        handshake_attempts++;
    }

    return (ble_wake_flag == 1);
}
#endif // !defined(BLE_CORE_CUSTOM)

/**
 * @brief Update BLE status
 * If BLE module is awake and disconnected, send a sleep command to put it to
 * sleep
 */
void update_ble_status()
{
#if defined(BLE_CORE_CUSTOM)
    return;
#endif
    const uint64_t now_ms      = get_ble_ms_tick();
    const bool     event_ok    = (ble_evt_ok != 0);
    const bool     event_wake  = (ble_evt_wake != 0);
    const bool     event_sleep = (ble_evt_sleep != 0);
    ble_evt_ok                 = 0;
    ble_evt_wake               = 0;
    ble_evt_sleep              = 0;

    // If an unsolicited wake event occurs while locked and idle, trigger a
    // relock as a one-shot event rather than a continuous level condition.
    if (event_wake && (shared_glbl.custom_flags.ble_connection_flag == 0) &&
        (ble_pairing_window_active == 0) &&
        (ble_supervisor.state == BleSupervisorState::IDLE) &&
        (ble_supervisor.target_dirty == 0))
    {
        ble_lock_request_pending = 1;
    }

    if ((ble_lock_request_pending == 1) &&
        (shared_glbl.custom_flags.ble_connection_flag == 0))
    {
        ble_lock_request_pending = 0;
        lock_ble_connectability();
    }

    process_pairing_window_request();
    process_pairing_window_timeout();

    service_ble_supervisor(event_ok, event_wake, event_sleep, now_ms);
}

/**
 * @brief Process pending BLE commands outside UART callback
 */
void process_ble_pending_commands()
{
    if (ble_get_device_info_pending == 1)
    {
        ble_get_device_info_pending = 0;

        w_hal_system_delay_ms(
            10); // Brief delay after the app device info request to allow BLE
                 // UART RX processing to settle before sending the device_info
                 // response.
        get_device_info();

        w_hal_uart_ble_drain_fifo();

        if (strncmp(uart_ble_cmd, "OK+LOST", 7) == 0)
        {
            shared_glbl.custom_flags.ble_connection_flag   = 0;
            ble_pairing_window_active                      = 0;
            ble_lock_request_pending                       = 1;
            shared_glbl.ble_pairing_window_request_pending = 0;
            reset_ble_cmd();
        }
    }

    if (ble_get_status_info_pending == 1)
    {
        ble_get_status_info_pending = 0;

        w_hal_system_delay_ms(
            10); // Brief delay after the app status request to allow BLE UART
                 // RX processing to settle before sending the status response.
        get_status_info();

        w_hal_uart_ble_drain_fifo();

        if (strncmp(uart_ble_cmd, "OK+LOST", 7) == 0)
        {
            shared_glbl.custom_flags.ble_connection_flag   = 0;
            ble_pairing_window_active                      = 0;
            ble_lock_request_pending                       = 1;
            shared_glbl.ble_pairing_window_request_pending = 0;
            reset_ble_cmd();
        }
    }
}

/**
 * @brief Get device information
 */
static void get_device_info()
{
    char              folder_structure[10];
    struct tm         tm_struct;
    w_hal_rtc_time_st current_time;
    w_hal_rtc_get_time(current_time);
    int      time_zone_offset = DeviceConfig::p_page->time_zone_offset;
    time_t   new_current_time = current_time.time_seconds + time_zone_offset;
    char     time_zone_sign;
    uint16_t tz_hour;
    uint16_t tz_min;

    // Audio config status
    ble_printf("SAMF %d\r\n", DeviceConfig::p_page->sampling_rate);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("BTPS %d\r\n", DeviceConfig::p_page->audio_sampling_width);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("NUMCH %d\r\n", DeviceConfig::p_page->num_audio_channels);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("GAIN %d\r\n", DeviceConfig::p_page->mic_gain_times_ten);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("LCUTOFF %d\r\n", DeviceConfig::p_page->lower_cutoff_frequency);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("UCUTOFF %d\r\n", DeviceConfig::p_page->upper_cutoff_frequency);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("RECT %d\r\n", DeviceConfig::p_page->recording_length);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("SLEEP %d\r\n", DeviceConfig::p_page->sleep_pause_interval);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Version info
    ble_printf("VER %s, SHA %s, DT %s\r\n",
               fw_version,
               git_sha,
               build_datetime);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Storage Status
    ble_printf("SDUSED %lld, SDCAP %lld\r\n",
               shared_glbl.storage_info.total_bytes_written,
               shared_glbl.storage_info.total_sd_card_capacity);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    DeviceConfig::get_folder_structure(folder_structure,
                                       sizeof(folder_structure));
    ble_printf("FLDR %s\r\n", folder_structure);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Power management status
    ble_printf("VBATT %d, CBATT %d, REMBATT %d\r\n",
               shared_glbl.power_mgmt_info.voltage,
               shared_glbl.power_mgmt_info.current,
               shared_glbl.power_mgmt_info.remaining_capacity);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("FULLBATT %d, REMTIME %d, AVGPWR %d\r\n",
               shared_glbl.power_mgmt_info.full_charge_capacity,
               shared_glbl.power_mgmt_info.time_to_empty,
               shared_glbl.power_mgmt_info.average_power);

    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("NBAT %d\r\n", DeviceConfig::p_page->num_battery_cells_used);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Time and schedule status
    gmtime_r(&new_current_time, &tm_struct);
    w_hal_rtc_get_time_zone_components(time_zone_offset,
                                       time_zone_sign,
                                       tz_hour,
                                       tz_min);

    uint32_t milliseconds = current_time.time_nanoseconds / 1000000;
    ble_printf("DATETIME %04d-%02d-%02d %02d:%02d:%02d.%03d %c%02d:%02d\r\n",
               TIME_BASE_YEAR + static_cast<uint32_t>(tm_struct.tm_year),
               TIME_BASE_MONTH + static_cast<uint32_t>(tm_struct.tm_mon),
               tm_struct.tm_mday,
               tm_struct.tm_hour,
               tm_struct.tm_min,
               tm_struct.tm_sec,
               milliseconds,
               time_zone_sign,
               tz_hour,
               tz_min);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("TIMEZONE %c%02d:%02d\r\n", time_zone_sign, tz_hour, tz_min);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    ble_printf("DATERANGE %04d/%02d/%02d-%04d/%02d/%02d\r\n",
               TIME_BASE_YEAR +
                   static_cast<uint32_t>(DeviceConfig::p_page->start_date.year),
               TIME_BASE_MONTH + static_cast<uint32_t>(
                                     DeviceConfig::p_page->start_date.month),
               DeviceConfig::p_page->start_date.day,
               TIME_BASE_YEAR +
                   static_cast<uint32_t>(DeviceConfig::p_page->end_date.year),
               TIME_BASE_MONTH +
                   static_cast<uint32_t>(DeviceConfig::p_page->end_date.month),
               DeviceConfig::p_page->end_date.day);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    if (DeviceConfig::p_page->sun_recording_mode !=
        DeviceConfig::SUN_MODE_DISABLED)
    {
        if (DeviceConfig::p_page->sun_recording_mode ==
            DeviceConfig::SUN_MODE_SUNRISE)
        {
            ble_printf("SUNMODE Sunrise\r\n");
        }
        else if (DeviceConfig::p_page->sun_recording_mode ==
                 DeviceConfig::SUN_MODE_SUNSET)
        {
            ble_printf("SUNMODE Sunset\r\n");
        }
        else if (DeviceConfig::p_page->sun_recording_mode ==
                 DeviceConfig::SUN_MODE_SUNRISE_AND_SUNSET)
        {
            ble_printf("SUNMODE Sunrise and Sunset\r\n");
        }
        else if (DeviceConfig::p_page->sun_recording_mode ==
                 DeviceConfig::SUN_MODE_SUNRISE_TO_SUNSET)
        {
            ble_printf("SUNMODE Sunrise to Sunset\r\n");
        }
        else if (DeviceConfig::p_page->sun_recording_mode ==
                 DeviceConfig::SUN_MODE_SUNSET_TO_SUNRISE)
        {
            ble_printf("SUNMODE Sunset to Sunrise\r\n");
        }
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
        ble_printf("SUNRISEAFT %d\r\n",
                   DeviceConfig::p_page->sunrise_interval_after);
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
        ble_printf("SUNSETBEF %d\r\n",
                   DeviceConfig::p_page->sunset_interval_before);
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
        ble_printf("SUNSETAFT %d\r\n",
                   DeviceConfig::p_page->sunset_interval_after);
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
        ble_printf("SUNROUND %d\r\n",
                   DeviceConfig::p_page->sunrise_sunset_rounding);
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    }
    else
    {
        ble_printf("SUNMODE Disabled\r\n");
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    }

    for (uint32_t index = 0; index < MAX_SCHEDULE_TIME_OF_DAY_RANGES; index++)
    {
        const DeviceConfig::time_of_day_range_st &time_of_day_range =
            DeviceConfig::p_page->time_of_day_range[index];

        ble_printf("TIMEOFDAY %d %02d:%02d:%02d-%02d:%02d:%02d\r\n",
                   index,
                   time_of_day_range.start_second / SECS_PER_HR,
                   (time_of_day_range.start_second / SECS_PER_MIN) %
                       MINS_PER_HR,
                   time_of_day_range.start_second % SECS_PER_MIN,
                   time_of_day_range.end_second / SECS_PER_HR,
                   (time_of_day_range.end_second / SECS_PER_MIN) % MINS_PER_HR,
                   time_of_day_range.end_second % SECS_PER_MIN);
        w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    }

    uint8_t str_index           = 0;
    char    days_of_week_str[8] = "";
    if (DeviceConfig::p_page->day_of_week_mask & 0x01)
    {
        days_of_week_str[str_index] = 'M';
        str_index++;
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x02)
    {
        days_of_week_str[str_index] = 'T';
        str_index++;
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x04)
    {
        days_of_week_str[str_index] = 'W';
        str_index++;
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x08)
    {
        days_of_week_str[str_index] = 'R';
        str_index++;
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x10)
    {
        days_of_week_str[str_index] = 'F';
        str_index++;
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x20)
    {
        days_of_week_str[str_index] = 'S';
        str_index++;
    }
    if (DeviceConfig::p_page->day_of_week_mask & 0x40)
    {
        days_of_week_str[str_index] = 'U';
        str_index++;
    }
    days_of_week_str[str_index] = '\0';
    ble_printf("DAYSOFWEEK %s\r\n", days_of_week_str);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Environmental status
    w_hal_dht_init();
    w_hal_dht_get_env_info(shared_glbl.environment_info.temperature_celsius,
                           shared_glbl.environment_info.relative_humidity);

    ble_printf("TEMP %.2f, HUMID %.2f\r\n",
               shared_glbl.environment_info.temperature_celsius,
               shared_glbl.environment_info.relative_humidity);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
}

/**
 * @brief Get Status information
 */
static void get_status_info()
{
    // Storage Status
    ble_printf("SDUSED %lld, SDCAP %lld\r\n",
               shared_glbl.storage_info.total_bytes_written,
               shared_glbl.storage_info.total_sd_card_capacity);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Power management status
    ble_printf("VBATT %d, CBATT %d, REMBATT %d\r\n",
               shared_glbl.power_mgmt_info.voltage,
               shared_glbl.power_mgmt_info.current,
               shared_glbl.power_mgmt_info.remaining_capacity);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
    ble_printf("FULLBATT %d, REMTIME %d, AVGPWR %d\r\n",
               shared_glbl.power_mgmt_info.full_charge_capacity,
               shared_glbl.power_mgmt_info.time_to_empty,
               shared_glbl.power_mgmt_info.average_power);

    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);

    // Environmental status
    w_hal_dht_init();
    w_hal_dht_get_env_info(shared_glbl.environment_info.temperature_celsius,
                           shared_glbl.environment_info.relative_humidity);

    ble_printf("TEMP %.2f, HUMID %.2f\r\n",
               shared_glbl.environment_info.temperature_celsius,
               shared_glbl.environment_info.relative_humidity);
    w_hal_system_delay_ms(BLE_RESPONSE_DELAY_MS);
}
/**
 * @brief Printf-style output for BLE UART
 * Formats a string like printf and sends it over BLE UART
 *
 * @param format format string (same as printf)
 * @param ... variable arguments
 * @return int number of characters sent (or -1 on error)
 */
static int ble_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // Format the string into ble_response_buffer
    int num_chars =
        vsnprintf(ble_response_buffer, MAX_UART_CMD_LEN, format, args);
    va_end(args);

    // Check for error or buffer overflow
    if (num_chars < 0 || num_chars >= static_cast<int>(MAX_UART_CMD_LEN))
    {
        return -1;
    }

    // Send over BLE UART
    wHalErrorCode error =
        w_hal_uart_ble_write_blocking((uint8_t *)ble_response_buffer,
                                      static_cast<uint32_t>(num_chars));

    if (error != wHalErrorCode::NO_ERROR)
    {
        return -1;
    }

    return num_chars;
}

/**
 * @brief Routine to handle the character returned from the ble uart, form a
 * command and process
 *
 * @param c is the character returned from terminal
 */
static void new_ble_character_handler(char c)
{
#if defined(BLE_CORE_CUSTOM)
    if (c == '\n')
    {
        return;
    }
#endif
    // Backspace clears the last entered character
    if (c == '\b')
    {
        if (ble_uart_char_cnt != 0)
        {
            printf("\b \b");
            ble_uart_char_cnt--;

            // Backspace into the command name means we are back to making
            // characters upper case. Set the command length back to zero
            // since the command name is being modified.
            if (ble_uart_char_cnt <= ble_uart_cmd_len)
            {
                ble_uart_cmd_len = 0;
            }
        }
    }
    // Clear console buffer & counter if 'Escape' key is pressed
    else if ((c == 0x1B))
    {
        printf("\nClearing command cache of %d chars\n", ble_uart_char_cnt);
        reset_ble_cmd();
        printf(">"); // Move to new line
    }
    // Process command once character limit is reached or return character
    // is entered
    else if ((c == '\r') || (ble_uart_char_cnt == MAX_UART_CMD_LEN - 1))
    {
        uart_ble_cmd[ble_uart_char_cnt] = '\0';
        const bool is_ok_prefix         = (strncmp(uart_ble_cmd, "OK", 2) == 0);

        // HM-10 returns several command-specific OK variants (e.g. OK+Set:0).
        // Treat any "OK..." line as a generic command acknowledgment for the
        // FSM.
        if (is_ok_prefix)
        {
            ble_evt_ok = 1;
        }

        if (strcmp(uart_ble_cmd, "OK") == 0)
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received OK response from BLE module\n");
            ble_wake_flag = 1;
            ble_evt_ok    = 1;
        }
        else if ((strcmp(uart_ble_cmd, "OK+SLEEP") == 0))
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received OK+SLEEP response from BLE module, module is "
                    "entering sleep\n");
            ble_wake_flag = 0;
            ble_evt_sleep = 1;
        }
        else if (strcmp(uart_ble_cmd, "OK+WAKE") == 0)
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received OK+WAKE response from BLE module, module is "
                    "waking up\n");
            ble_wake_flag = 1;
            ble_evt_wake  = 1;
        }
        else if (strcmp(uart_ble_cmd, "OK+LOST") == 0)
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received OK+LOST response from BLE module, module is "
                    "disconnected\n");
            shared_glbl.custom_flags.ble_connection_flag   = 0;
            ble_pairing_window_active                      = 0;
            ble_lock_request_pending                       = 1;
            shared_glbl.ble_pairing_window_request_pending = 0;
        }
        else if (strcmp(uart_ble_cmd, "OK+CONN") == 0)
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received OK+CONN response from BLE module, module is "
                    "connected\n");
            ble_wake_flag                                  = 1;
            shared_glbl.custom_flags.ble_connection_flag   = 1;
            ble_pairing_window_active                      = 0;
            shared_glbl.ble_pairing_window_request_pending = 0;
        }
        else if (strcmp(uart_ble_cmd, "WS+ALIVE") == 0)
        {
            ble_wake_flag                                  = 1;
            shared_glbl.custom_flags.ble_connection_flag   = 1;
            ble_pairing_window_active                      = 1;
            shared_glbl.ble_pairing_window_request_pending = 0;
        }
        else if (strcmp(uart_ble_cmd, "GET_DEV_INFO") == 0)
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received GET_DEV_INFO command over BLE UART\n");
            ble_get_device_info_pending                  = 1;
            shared_glbl.custom_flags.ble_connection_flag = 1;
            ble_wake_flag                                = 1;
        }
        else if (strcmp(uart_ble_cmd, "GET_STATUS_INFO") == 0)
        {
            LOG_MSG(TraceFile::BLE_COMM_CPP,
                    "Received GET_STATUS command over BLE UART\n");
            ble_get_status_info_pending                  = 1;
            shared_glbl.custom_flags.ble_connection_flag = 1;
            ble_wake_flag                                = 1;
        }
        else if (shared_glbl.custom_flags.ble_connection_flag == 1)
        {
            new_console_command_handler(uart_ble_cmd, ble_uart_char_cnt);
        }
#if defined(BLE_CORE_CUSTOM)
        else
        {
            new_console_command_handler(uart_ble_cmd, ble_uart_char_cnt);
        }
#endif
        // Reset character count and buffer
        reset_ble_cmd();
    }
    // Accumulate characters input to form the command
    else if ((ble_uart_char_cnt < MAX_UART_CMD_LEN))
    {
        // Command length is initialized to zero. When a space is observed,
        // it means that the command name has been entered. So, we set the
        // command length to the current character count.
        if (isspace(c) && (ble_uart_cmd_len == 0))
        {
            ble_uart_cmd_len = ble_uart_char_cnt;
        }

        // Ignore spaces at the beginning of the command
        if ((ble_uart_char_cnt > 0) || !isspace(c))
        {
            // Only command should be forced to upper case. Command length
            // of zero means we are still entering the command name, in
            // which case convert the input character to upper case.
            if (ble_uart_cmd_len == 0)
            {
                c = toupper(c);
            }

            uart_ble_cmd[ble_uart_char_cnt++] = c;
        }
    }
}
#endif // __CORE__ == BLE_CORE
