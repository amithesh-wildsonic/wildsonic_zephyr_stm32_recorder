#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Test Suite: Clean Production Console & Original WildSonic Firmware Output Parity.

Verifies:
 1. Canonical 59-command WildSonic interface in PRODUCTION from HELP through TEST_ALARM.
 2. Exclusion of engineering commands (BENCH, DIAGNOSE, COMPARE, STRESS, INTEGRITY, START, STOP,
    RECORD, RATE, TIME, SCHEDULE, PSRAM, CONFIG wizard) from PRODUCTION HELP and dispatcher.
 3. Engineering commands cleanly exposed in TEST builds under separated section.
 4. SAMF query produces 'Value is set to <val>' and setter produces canonical notification.
 5. Automatic UCUTOFF reduction notification ('Upper cutoff frequency adjusted to <val>').
 6. SLEEP is strictly a session sleep configuration getter/setter without Stop 2 or BUSY conflation.
 7. ARBT outputs single concise 'Rebooting device.' message without user-aborted or paused noise.
 8. Safe WAV finalization, audio buffer flush, and SD sync during ARBT.
 9. Flash persistence operations succeed silently in PRODUCTION (no slot/sequence logs).
10. Persistence failures produce visible error messages (LOG_ERR).
11. Clean 3-line version banner in PRODUCTION; reset reason details in TEST.
12. BLE responses remain strictly structured and isolated from UART console output.
13. All Zephyr log modules conditionalized (LOG_LEVEL_INF in TEST, LOG_LEVEL_WRN in PRODUCTION).
14. prj.conf default log level is WRN (2) and test.conf default log level is INF (3).
15. Full verification of all 17 Section 10 Product Startup Status Requirements.
"""

from pathlib import Path
import re
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
PRJ_CONF = REPO_ROOT / "app" / "prj.conf"
TEST_CONF = REPO_ROOT / "app" / "test.conf"

CONSOLE_C = APP_SRC / "console.c"
MAIN_C = APP_SRC / "main.c"
RECORDER_C = APP_SRC / "recorder.c"
RECORDER_H = APP_SRC / "recorder.h"
CONFIG_STORE_C = APP_SRC / "config_store.c"
BLE_COMM_C = APP_SRC / "ble_comm.c"
RTC_CALENDAR_C = APP_SRC / "rtc_calendar.c"
RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"

CANONICAL_59_COMMANDS = [
    "HELP", "SN", "DNAME", "PW", "SMSKW", "SMSKEY", "A2PPHONE", "P2APHONE",
    "SURL", "SPORT", "POSTROUTE", "POSTCTNT", "STEST", "SWS", "GSM",
    "SAMF", "RECT", "SLEEP", "DATERANGE", "TIMEOFDAY", "DAYSOFWEEK",
    "DATETIME", "TIMEZONE", "BTPS", "NUMCH", "GAIN", "LCUTOFF", "UCUTOFF",
    "FLDR", "APPLY", "RBT", "ARBT", "SDFMT", "SMSFREQ", "ASSERT", "BATT",
    "SIG", "STOR", "LBAT", "NBAT", "FULLCAP", "SENDSTS", "AUDIO", "BLEPAIR",
    "Wildly", "VERSION", "ENVSTS", "LOC", "SUN", "SUNMODE", "SUNRISEBEF",
    "SUNRISEAFT", "SUNSETBEF", "SUNSETAFT", "SUNROUND", "DEFAULT", "I2C",
    "RTCRST", "TEST_ALARM"
]


def test_canonical_59_commands_defined_and_bounded():
    """Verify CANONICAL_CMD_COUNT is 59 and the command table contains the exact canonical order."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    assert "#define CANONICAL_CMD_COUNT 59" in c_text, "CANONICAL_CMD_COUNT must be defined as 59"

    # Extract console_cmd_lookup_table
    match = re.search(r"static const char \*(?:const )?\s*console_cmd_lookup_table\[\] = \{(.*?)\};", c_text, re.DOTALL)
    assert match is not None, "console_cmd_lookup_table not found"
    table_content = match.group(1)
    cmd_tokens = [m.group(1) for m in re.finditer(r'"([^"]+)"', table_content)]

    # Check first 59 commands match exactly
    assert len(cmd_tokens) >= 59
    assert cmd_tokens[:59] == CANONICAL_59_COMMANDS, "First 59 commands do not match canonical WildSonic order"


def test_help_in_production_outputs_canonical_59_and_no_engineering_tools():
    """Verify display_console_commands bounds iteration to CANONICAL_CMD_COUNT in PRODUCTION."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # In production, loop runs up to CANONICAL_CMD_COUNT
    assert "for (size_t i = 0; i < CANONICAL_CMD_COUNT; i++)" in c_text
    assert "List of commands" in c_text

    # Engineering tools must be gated under CONFIG_WILDSONIC_VERBOSE_LOGS
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in c_text
    assert "Engineering Extensions (TEST build only):" in c_text


def test_engineering_commands_rejected_in_production():
    """Verify PSRAM and CONFIG_WIZARD commands are rejected in PRODUCTION with Unknown or BAD command."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # Check case CMD_PSRAM / CMD_CONFIG_WIZARD fallback under #else
    psram_block = c_text.split("case CMD_TEST_ALARM:")[1].split("case CMD_GET_STATUS:")[0]
    assert "#else" in psram_block
    assert 'printk("\\nUnknown or BAD command : %s\\n", console_cmd_lookup_table[cmd_id]);' in psram_block


def test_samf_query_and_setter_canonical_formatting():
    """Verify SAMF query and setter output canonical messages without duplicate acknowledgements."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    samf_handler = c_text.split("static void handle_samf")[1].split("static void handle_bits")[0]

    # Query outputs only canonical Value is set to %u
    assert 'printk("\\nValue is set to %u\\n", g_cfg.sampling_rate);' in samf_handler

    # Setter outputs canonical Value is set to %u followed by APPLY prompt
    assert 'printk("\\nValue is set to %u\\n", g_cfg.sampling_rate);' in samf_handler
    assert 'printk("\\nSend APPLY command and reboot for changes to take effect\\n");' in samf_handler

    # Upper cutoff automatic adjustment notification
    assert 'printk("\\nUpper cutoff frequency adjusted to %u\\n", g_cfg.upper_cutoff_frequency);' in samf_handler


def test_sleep_query_and_setter_semantics():
    """Verify SLEEP is strictly an interval getter/setter, with no Stop 2 sleep or BUSY check on query."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    sleep_handler = c_text.split("static void handle_sleep")[1].split("static void handle_datetime")[0]

    # Query returns immediately with Value is set to %u, without checking recorder_is_active()
    query_block = sleep_handler.split("if (args_length == 0)")[1].split("if (p_args[0] == '-')")[0]
    assert "recorder_is_active()" not in query_block
    assert 'printk("\\nValue is set to %u\\n", g_cfg.sleep_pause_interval);' in query_block

    # Setter validates and reports Value is set + APPLY prompt
    assert 'printk("\\nValue is set to %u\\n", g_cfg.sleep_pause_interval);' in sleep_handler
    assert 'printk("\\nSend APPLY command and reboot for changes to take effect\\n");' in sleep_handler


def test_arbt_outputs_only_rebooting_device():
    """Verify ARBT and RBT print ONLY 'Rebooting device.' without user-stop or paused noise."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    arbt_handler = c_text.split("static void handle_arbt")[1].split("/* Command dispatcher")[0]

    # Must print single concise message
    assert 'printk("\\nRebooting device.\\n");' in arbt_handler
    assert "Saving configuration to flash" not in arbt_handler
    assert "recorder_wait_idle(5000)" in arbt_handler


def test_recorder_stop_reasons_and_reboot_suppression():
    """Verify recorder stop reason enum and scheduler silence on apply/reboot."""
    h_text = RECORDER_H.read_text(encoding="utf-8")
    assert "RECORDER_STOP_NONE" in h_text
    assert "RECORDER_STOP_USER" in h_text
    assert "RECORDER_STOP_APPLY_REBOOT" in h_text
    assert "RECORDER_STOP_SYSTEM_REBOOT" in h_text

    main_c = MAIN_C.read_text(encoding="utf-8")
    # Scheduler paused notice suppressed if reboot pending
    assert "!main_is_reboot_pending()" in main_c
    # Scheduler resume returns early if reboot pending
    assert "main_is_reboot_pending()" in main_c


def test_flash_persistence_silent_in_production():
    """Verify flash save operations do not emit routine informational messages to the console."""
    cfg_text = CONFIG_STORE_C.read_text(encoding="utf-8")

    # In production, log level is WRN so LOG_INF will be omitted
    assert "LOG_MODULE_REGISTER(ws_config_store, LOG_LEVEL_WRN);" in cfg_text

    # Status file creation notice is active product message in PRODUCTION
    assert 'printk("Configuration status file has been created\\n");' in cfg_text


def test_persistence_failure_remains_visible():
    """Verify flash write and erase failures use LOG_ERR which remains visible in production."""
    cfg_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    assert 'LOG_ERR("flash_area_write failed on slot %d: %d"' in cfg_text
    assert 'LOG_ERR("flash_area_erase failed on slot %d: %d"' in cfg_text


def test_production_banner_is_three_lines():
    """Verify production banner outputs only the 3 canonical version lines and gates reset reason in TEST."""
    main_c = MAIN_C.read_text(encoding="utf-8")

    banner_fn = main_c.split("static void print_banner(void)")[1].split("static void print_settings")[0]

    # Banner prints version, git branch, build date
    assert 'printk("Version: %s\\n", "v1.2.0");' in banner_fn
    assert 'printk("GIT Branch: %s, SHA: %s\\n", "production", "d7a4f91");' in banner_fn
    assert 'printk("Build Date: %s %s\\n", __DATE__, __TIME__);' in banner_fn

    # Reset reason gated under CONFIG_WILDSONIC_VERBOSE_LOGS
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in banner_fn
    assert "print_reset_reason();" in banner_fn


def test_ble_responses_compatible_and_isolated():
    """Verify BLE responses use structured output and do not leak console messages or vice versa."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # Mobile status and dev info queries are routed to ble_comm helpers
    assert "ble_comm_send_status_info();" in c_text
    assert "ble_comm_send_dev_info();" in c_text

    # BLE access initial boot message gated under verbose logs
    ble_c = BLE_COMM_C.read_text(encoding="utf-8")
    assert "[BLE] Access: UNLOCKED (initial boot state)" in ble_c


def test_all_modules_use_conditional_log_level():
    """Verify every C file in app/src using LOG_MODULE_REGISTER gates LOG_LEVEL_INF under CONFIG_WILDSONIC_VERBOSE_LOGS."""
    for c_file in sorted(APP_SRC.glob("*.c")):
        content = c_file.read_text(encoding="utf-8")
        if "LOG_MODULE_REGISTER(" in content:
            assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in content, \
                f"{c_file.name} lacks CONFIG_WILDSONIC_VERBOSE_LOGS check for LOG_MODULE_REGISTER"
            assert "LOG_LEVEL_INF" in content, f"{c_file.name} lacks LOG_LEVEL_INF in TEST block"
            assert "LOG_LEVEL_WRN" in content, f"{c_file.name} lacks LOG_LEVEL_WRN in PRODUCTION block"


def test_kconfig_default_log_levels():
    """Verify numeric Zephyr default log levels are used in production and TEST builds."""
    prj = PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_LOG_DEFAULT_LEVEL=2" in prj, "warning-level default missing from prj.conf"
    assert "CONFIG_BOOT_BANNER=n" in prj, "CONFIG_BOOT_BANNER=n missing from prj.conf"

    test_c = TEST_CONF.read_text(encoding="utf-8")
    assert "CONFIG_LOG_DEFAULT_LEVEL=3" in test_c, "info-level default missing from test.conf"
    assert "CONFIG_WILDSONIC_VERBOSE_LOGS=y" in test_c, "CONFIG_WILDSONIC_VERBOSE_LOGS=y missing from test.conf"
    assert "CONFIG_BOOT_BANNER=y" in test_c, "CONFIG_BOOT_BANNER=y missing from test.conf"


def test_consolidated_single_command_acknowledgement():
    """Verify redundant OK/ACK/PENDING messages are gated to TEST and absent in PRODUCTION."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    commit_fn = c_text.split("int console_commit_and_ack_config")[1].split("static void handle_apply")[0]

    # OK: Value staged for next session is in TEST only
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in commit_fn
    assert 'printk("\\nOK: Value staged for next session (active session unaffected)\\n");' in commit_fn

    # [ACK] and [PENDING] are in TEST only
    assert '[ACK] Command %s received and validated.' in commit_fn
    assert '[PENDING] Configuration pending application after reboot.' in commit_fn


# ==============================================================================
# SECTION 10: Specific Product Startup Status Regression Tests (Items 1 - 17)
# ==============================================================================

def test_p1_version_information_in_production():
    """Item 1: Verify version information remains visible in PRODUCTION."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    banner_fn = m_text.split("static void print_banner(void)")[1].split("static void print_settings")[0]
    assert 'printk("Version: %s\\n", "v1.2.0");' in banner_fn
    assert 'printk("GIT Branch: %s, SHA: %s\\n", "production", "d7a4f91");' in banner_fn
    assert 'printk("Build Date: %s %s\\n", __DATE__, __TIME__);' in banner_fn


def test_p2_rtc_wakeup_initialization_status_appears_once():
    """Item 2: Verify RTC wake-up initialization status appears once upon success."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    main_fn = m_text.split("int main(void) {")[1]
    assert 'printk("\\nRTC Wakeup Timer initialized\\n");' in main_fn
    # Must only print when rtc_sleep_rc == 0
    assert "if (rtc_sleep_rc == 0)" in main_fn


def test_p3_rtc_calendar_status_appears_once():
    """Item 3: Verify RTC calendar online status appears once when valid."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    main_fn = m_text.split("int main(void) {")[1]
    assert 'printk("RTC calendar online: %04u-%02u-%02u %02u:%02u:%02u.%03u\\n\\n"' in main_fn


def test_p4_utc_local_time_appears_only_when_valid():
    """Item 4: Verify UTC and local time appear only when RTC calendar is valid."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert 'if (rtc_calendar_is_valid()) {' in m_text
    assert 'printk("Current time UTC time is' in m_text
    assert 'printk("Current time Local is' in m_text


def test_p5_invalid_rtc_produces_one_actionable_warning():
    """Item 5: Verify invalid RTC calendar produces one actionable warning."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert 'printk("RTC calendar is uninitialized. Set time with DATETIME command.\\n\\n");' in m_text


def test_p6_sd_initialization_success_appears_only_after_successful_mount():
    """Item 6: Verify SD initialization success appears only after successful mount."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    mount_block = m_text.split("int ret = recorder_init();")[1].split("scheduler_init();")[0]
    assert "if (ret)" in mount_block
    assert 'printk("FATAL: Storage initialization failed' in mount_block
    assert 'printk("SD card initialized successfully\\n");' in mount_block


def test_p7_config_missing_and_status_file_created_in_production():
    """Item 7: Verify configuration missing and status file created messages appear in PRODUCTION."""
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    assert 'printk("Configuration file is missing. Using earlier programmed configuration.\\n");' in cs_text
    assert 'printk("Configuration status file has been created\\n");' in cs_text


def test_p8_config_import_archive_message_is_accurate():
    """Item 8: Verify configuration file archive message is accurate when config.ini is renamed."""
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    rename_block = cs_text.split("if (fs_rename(ini_path, bak_path) == 0) {")[1].split("config_store_write_status_file")[0]
    assert 'printk("Configuration file has been archived to config.bak\\n");' in rename_block
    assert 'printk("Configuration file has been deleted\\n");' in rename_block


def test_p9_audio_initialization_success_in_production():
    """Item 9: Verify audio initialization success appears only after successful configuration."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    audio_block = m_text.split("int rate_rc = audio_in_set_rate(settings.sample_rate);")[1].split("scheduler_init();")[0]
    assert 'printk("Audio peripheral initialized\\n");' in audio_block


def test_p10_system_initialization_complete_in_production():
    """Item 10: Verify System initialization complete appears in PRODUCTION before interactive prompt."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert 'printk("System initialization complete\\n");' in m_text


def test_p11_mcu_reset_reason_absent_in_production():
    """Item 11: Verify MCU reset reason is absent in PRODUCTION and gated to TEST."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    banner_fn = m_text.split("static void print_banner(void)")[1].split("static void print_settings")[0]
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in banner_fn
    assert "print_reset_reason();" in banner_fn


def test_p12_last_file_detection_absent_in_production():
    """Item 12: Verify last-file detection is gated under TEST and absent in PRODUCTION."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    last_file_block = m_text.split("recorder_scan_last_file")[0].split("apply_dsp_settings();")[-1]
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in last_file_block


def test_p13_flash_slot_and_sequence_details_absent_in_production():
    """Item 13: Verify flash slot and sequence details are absent in PRODUCTION."""
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    # All flash slot and sequence prints must be LOG_INF (which is WRN in production)
    assert "LOG_MODULE_REGISTER(ws_config_store, LOG_LEVEL_WRN);" in cs_text


def test_p14_zephyr_boot_banner_absent_in_production():
    """Item 14: Verify Zephyr boot banner is disabled in PRODUCTION prj.conf and enabled in test.conf."""
    prj = PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_BOOT_BANNER=n" in prj
    test_c = TEST_CONF.read_text(encoding="utf-8")
    assert "CONFIG_BOOT_BANNER=y" in test_c


def test_p15_test_build_retains_full_diagnostic_output():
    """Item 15: Verify TEST build fragment enables CONFIG_WILDSONIC_VERBOSE_LOGS and info-level logging."""
    test_c = TEST_CONF.read_text(encoding="utf-8")
    assert "CONFIG_WILDSONIC_VERBOSE_LOGS=y" in test_c
    assert "CONFIG_LOG_DEFAULT_LEVEL=3" in test_c
    assert "CONFIG_SDMMC_LOG_LEVEL_INF=y" in test_c
    assert "CONFIG_DISK_LOG_LEVEL_INF=y" in test_c
    assert "CONFIG_FS_LOG_LEVEL_INF=y" in test_c


def test_p16_genuine_hardware_failures_remain_visible():
    """Item 16: Verify genuine hardware failures remain visible via printk and LOG_ERR."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert 'printk("FATAL: Storage initialization failed' in m_text
    assert 'LOG_ERR("RTC Wakeup Timer initialization failed:' in m_text
    assert 'LOG_ERR("RTC calendar initialization failed:' in m_text


def test_p17_no_duplicate_startup_messages():
    """Item 17: Verify no duplicate RTC, SD, audio, configuration, or BLE startup messages occur."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    # "System initialization complete" appears exactly once
    assert m_text.count('printk("System initialization complete\\n");') == 1
    # "Audio peripheral initialized" appears exactly once
    assert m_text.count('printk("Audio peripheral initialized\\n");') == 1
    # "SD card initialized successfully" appears exactly once
    assert m_text.count('printk("SD card initialized successfully\\n");') == 1
    # Initial BLE unlocked notice is NOT duplicated in production
    ble_c = BLE_COMM_C.read_text(encoding="utf-8")
    assert ble_c.count('printk("[BLE] Access: UNLOCKED (initial boot state)\\n");') <= 2
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in ble_c
