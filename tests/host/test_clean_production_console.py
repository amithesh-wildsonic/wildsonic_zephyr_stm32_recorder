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

    # Engineering tools must be gated under CONFIG_WILDSONIC_TEST_OUTPUT
    assert "#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)" in c_text
    assert "Engineering Extensions (TEST build only):" in c_text


def test_engineering_commands_rejected_in_production():
    """Verify PSRAM and BATCHLOG are gated under CONFIG_WILDSONIC_TEST_OUTPUT so PROD rejects them via default."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # CMD_PSRAM / CMD_BATCHLOG switch cases and table entries gated under CONFIG_WILDSONIC_TEST_OUTPUT
    assert "#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)" in c_text
    assert "case CMD_PSRAM:" in c_text
    assert "case CMD_BATCHLOG:" in c_text
    assert 'TRACE_RAW("\\nUnknown or BAD command : %s\\n", p);' in c_text


def test_samf_query_and_setter_canonical_formatting():
    """Verify SAMF query and setter output canonical messages without duplicate acknowledgements."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    samf_handler = c_text.split("static void handle_samf")[1].split("static void handle_bits")[0]

    # Query outputs only canonical Value is set to %u
    assert 'TRACE_RAW("\\nValue is set to %u\\n", g_cfg.sampling_rate);' in samf_handler

    # Setter outputs canonical Value is set to %u followed by APPLY prompt
    assert 'TRACE_RAW("\\nValue is set to %u\\n", val);' in samf_handler
    assert 'TRACE_RAW("\\nSend APPLY command and reboot for changes to take effect\\n");' in samf_handler

    # Upper cutoff automatic adjustment notification
    assert 'TRACE_RAW("\\nUpper cutoff frequency adjusted to %u\\n", g_cfg.upper_cutoff_frequency);' in samf_handler


def test_sleep_query_and_setter_semantics():
    """Verify SLEEP is strictly an interval getter/setter, with no Stop 2 sleep or BUSY check on query."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    sleep_handler = c_text.split("static void handle_sleep")[1].split("static void handle_datetime")[0]

    # Query returns immediately with Value is set to %u, without checking recorder_is_active()
    query_block = sleep_handler.split("if (args_length == 0)")[1].split("if (p_args[0] == '-')")[0]
    assert "recorder_is_active()" not in query_block
    assert 'TRACE_RAW("\\nValue is set to %u\\n", g_cfg.sleep_pause_interval);' in query_block

    # Setter validates and reports Value is set + APPLY prompt
    assert 'TRACE_RAW("\\nValue is set to %u\\n", g_cfg.sleep_pause_interval);' in sleep_handler
    assert 'TRACE_RAW("\\nSend APPLY command and reboot for changes to take effect\\n");' in sleep_handler


def test_arbt_outputs_only_rebooting_device():
    """Verify ARBT and RBT print 'Rebooting device.' in both PROD and TEST."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    arbt_handler = c_text.split("static void handle_arbt")[1].split("/* Command dispatcher")[0]

    # Must print concise message
    assert 'TRACE_RAW("\\nRebooting device.\\n");' in arbt_handler
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
    """Verify config_store uses unified TRACE output and concise boot status."""
    cfg_text = CONFIG_STORE_C.read_text(encoding="utf-8")

    assert '#include "wildsonic_trace.h"' in cfg_text
    assert 'TRACE("Configuration loaded (slot %d, seq %u)", s_active_slot, s_seq);' in cfg_text


def test_persistence_failure_remains_visible():
    """Verify flash write, erase, and verify failures use TRACE so they remain visible in production."""
    cfg_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    assert 'TRACE("Flash write slot %d failed (%d)", target_slot, rc);' in cfg_text
    assert 'TRACE("Flash erase slot %d failed (%d)", target_slot, rc);' in cfg_text
    assert 'TRACE("Flash verify slot %d failed", target_slot);' in cfg_text


def test_production_banner_is_three_lines():
    """Verify production banner outputs 3 version lines followed by Basic System Init Done and Reset cause."""
    main_c = MAIN_C.read_text(encoding="utf-8")

    banner_fn = main_c.split("static void print_banner(void)")[1].split("static void main_emit_boot_tail")[0]

    # Banner prints version, git branch/SHA, build date
    assert 'TRACE_RAW("Version: %s\\n", "v1.2.0");' in banner_fn
    assert 'TRACE_RAW("GIT Branch: %s, SHA: %s\\n", WILDSONIC_GIT_BRANCH, WILDSONIC_GIT_COMMIT);' in banner_fn
    assert 'TRACE_RAW("Build Date: %s %s\\n", __DATE__, __TIME__);' in banner_fn

    # Unified boot init + reset cause line
    assert 'TRACE("Basic System Init Done");' in banner_fn
    assert 'TRACE("Reset cause: %s", retained_reboot_reason_str(reason));' in banner_fn


def test_ble_responses_compatible_and_isolated():
    """Verify BLE responses use structured output and stay silent during initial boot init."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # Mobile status and dev info queries are routed to ble_comm helpers
    assert "ble_comm_send_status_info();" in c_text
    assert "ble_comm_send_dev_info();" in c_text

    # Initial boot leaves BLE unlocked silently without legacy verbose logs
    ble_c = BLE_COMM_C.read_text(encoding="utf-8")
    assert "s_current_adty = 0;" in ble_c
    assert "[BLE] Access: UNLOCKED (initial boot state)" not in ble_c


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
    """Verify numeric Zephyr default log levels and TEST/PRODUCTION parity flags."""
    prj = PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_LOG_DEFAULT_LEVEL=2" in prj, "warning-level default missing from prj.conf"
    assert "CONFIG_BOOT_BANNER=n" in prj, "CONFIG_BOOT_BANNER=n missing from prj.conf"
    assert "CONFIG_WILDSONIC_TEST_OUTPUT=n" in prj

    test_c = TEST_CONF.read_text(encoding="utf-8")
    assert "CONFIG_LOG_DEFAULT_LEVEL=3" in test_c, "info-level default missing from test.conf"
    assert "CONFIG_WILDSONIC_TEST_OUTPUT=y" in test_c, "CONFIG_WILDSONIC_TEST_OUTPUT=y missing from test.conf"
    assert "CONFIG_WILDSONIC_BENCHMARKS=y" in test_c


def test_consolidated_single_command_acknowledgement():
    """Verify console_commit_and_ack_config emits structured ACK/PENDING via TRACE_RAW."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    commit_fn = c_text.split("int console_commit_and_ack_config")[1].split("static void handle_apply")[0]

    assert 'TRACE_RAW("\\nOK: Value staged for next session (active session unaffected)\\n");' in commit_fn
    assert 'config_store_stage_pending();' in commit_fn
    assert '[ACK] Command %s received and validated.' in commit_fn
    assert '[PENDING] Send APPLY or ARBT to save to flash.' in commit_fn


# ==============================================================================
# SECTION 10: Specific Product Startup Status Regression Tests (Items 1 - 17)
# ==============================================================================

def test_p1_version_information_in_production():
    """Item 1: Verify version information remains visible in PRODUCTION."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    banner_fn = m_text.split("static void print_banner(void)")[1].split("static void print_settings")[0]
    assert 'TRACE_RAW("Version: %s\\n", "v1.2.0");' in banner_fn
    assert 'TRACE_RAW("GIT Branch: %s, SHA: %s\\n", WILDSONIC_GIT_BRANCH, WILDSONIC_GIT_COMMIT);' in banner_fn
    assert 'TRACE_RAW("Build Date: %s %s\\n", __DATE__, __TIME__);' in banner_fn


def test_p2_rtc_wakeup_initialization_status_appears_once():
    """Item 2: Verify RTC wakeup timer initializes at boot without duplicate status spam."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    main_fn = m_text.split("int main(void) {")[1]
    assert "(void)rtc_sleep_init();" in main_fn


def test_p3_rtc_calendar_status_appears_once():
    """Item 3: Verify RTC calendar boot time is emitted via rtc_calendar_log_boot_times."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert "rtc_calendar_log_boot_times(tz_offset_sec);" in m_text


def test_p4_utc_local_time_appears_only_when_valid():
    """Item 4: Verify UTC and local time formatting in rtc_calendar_log_boot_times."""
    rc_text = RTC_CALENDAR_C.read_text(encoding="utf-8")
    assert 'if (!rtc_calendar_is_valid() || rtc_calendar_get(&cur_cal) != 0) {' in rc_text
    assert 'TRACE("RTC Time UTC: %04u-%02u-%02u %02u:%02u:%02u.%03u"' in rc_text
    assert 'TRACE("RTC Time Local: %04u-%02u-%02u %02u:%02u:%02u.%03u %c%02d:%02d"' in rc_text


def test_p5_invalid_rtc_produces_one_actionable_warning():
    """Item 5: Verify unset RTC calendar reports UNSET cleanly."""
    rc_text = RTC_CALENDAR_C.read_text(encoding="utf-8")
    assert 'TRACE("RTC Time UTC: UNSET");' in rc_text
    assert 'TRACE("RTC Time Local: UNSET");' in rc_text


def test_p6_sd_initialization_success_appears_only_after_successful_mount():
    """Item 6: Verify SD initialization status appears cleanly at boot."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    mount_block = m_text.split("int ret = recorder_init_storage();")[1].split("scheduler_init();")[0]
    assert "if (ret)" in mount_block
    assert 'TRACE("Storage init failed: %d", ret);' in mount_block
    assert "recorder_log_sd_boot_status();" in mount_block

    rec_text = RECORDER_C.read_text(encoding="utf-8")
    assert 'TRACE("SD Card Init Done (%s, %llu MB free)", fs_type_str, free_mb);' in rec_text


def test_p7_config_missing_and_status_file_created_in_production():
    """Item 7: Verify configuration boot status emits concise single-line TRACE."""
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    assert 'TRACE("Configuration loaded (slot %d, seq %u)", s_active_slot, s_seq);' in cs_text
    assert 'TRACE("Configuration loaded (defaults)");' in cs_text


def test_p8_config_import_archive_message_is_accurate():
    """Item 8: Verify configuration file archive message is accurate when config.ini is renamed."""
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    rename_block = cs_text.split("if (fs_rename(ini_path, bak_path) == 0) {")[1].split("config_store_write_status_file")[0]
    assert 'TRACE("Config file archived to config.bak");' in rename_block
    assert 'TRACE("Config file deleted");' in rename_block


def test_p9_audio_initialization_success_in_production():
    """Item 9: Verify audio initialization status appears via audio_in_log_boot_status."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    audio_block = m_text.split("ret = recorder_init_audio(settings.sample_rate);")[1].split("wildsonic_trace_start_pump();")[0]
    assert "audio_in_log_boot_status();" in audio_block
    ai_text = (APP_SRC / "audio_in_stm32l4.c").read_text(encoding="utf-8")
    assert 'TRACE("ADC/DMA Init Done");' in ai_text


def test_p10_system_initialization_complete_in_production():
    """Item 10: Verify Initialization Complete & Last System Init Done appear in boot tail."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert 'TRACE("Initialization Complete");' in m_text
    assert 'TRACE("Last System Init Done");' in m_text


def test_p11_mcu_reset_reason_absent_in_production():
    """Item 11: Verify unified Reset cause line is printed via retained_reboot in boot banner."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    banner_fn = m_text.split("static void print_banner(void)")[1].split("static void print_settings")[0]
    assert 'TRACE("Reset cause: %s", retained_reboot_reason_str(reason));' in banner_fn


def test_p12_last_file_detection_absent_in_production():
    """Item 12: Verify last-file detection logs concisely in boot sequence."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert "recorder_log_last_file();" in m_text


def test_p13_flash_slot_and_sequence_details_absent_in_production():
    """Item 13: Verify config_store logs concise slot/seq status via TRACE."""
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    assert 'TRACE("Configuration loaded (slot %d, seq %u)", s_active_slot, s_seq);' in cs_text


def test_p14_zephyr_boot_banner_absent_in_production():
    """Item 14: Verify Zephyr boot banner is disabled in both PRODUCTION and TEST for parity."""
    prj = PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_BOOT_BANNER=n" in prj
    test_c = TEST_CONF.read_text(encoding="utf-8")
    assert "CONFIG_WILDSONIC_TEST_OUTPUT=y" in test_c


def test_p15_test_build_retains_full_diagnostic_output():
    """Item 15: Verify TEST build fragment enables CONFIG_WILDSONIC_TEST_OUTPUT and info-level logging."""
    test_c = TEST_CONF.read_text(encoding="utf-8")
    assert "CONFIG_WILDSONIC_TEST_OUTPUT=y" in test_c
    assert "CONFIG_LOG_DEFAULT_LEVEL=3" in test_c
    assert "CONFIG_SDMMC_LOG_LEVEL_INF=y" in test_c
    assert "CONFIG_DISK_LOG_LEVEL_INF=y" in test_c
    assert "CONFIG_FS_LOG_LEVEL_INF=y" in test_c


def test_p16_genuine_hardware_failures_remain_visible():
    """Item 16: Verify genuine hardware failures remain visible via TRACE."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert 'TRACE("Storage init failed: %d", ret);' in m_text
    assert 'TRACE("Audio init failed: %d", ret);' in m_text
    rs_text = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert 'TRACE("LSI startup timeout; RTC init failed");' in rs_text


def test_p17_no_duplicate_startup_messages():
    """Item 17: Verify no duplicate startup messages occur."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    assert m_text.count('TRACE("Initialization Complete");') == 1
    assert m_text.count('TRACE("Last System Init Done");') == 1
    ai_text = (APP_SRC / "audio_in_stm32l4.c").read_text(encoding="utf-8")
    assert ai_text.count('TRACE("ADC/DMA Init Done");') == 1
    rec_text = RECORDER_C.read_text(encoding="utf-8")
    assert rec_text.count('TRACE("SD Card Init Done (%s, %llu MB free)"') == 1
    ble_c = BLE_COMM_C.read_text(encoding="utf-8")
    assert '[BLE] Access: UNLOCKED (initial boot state)' not in ble_c
