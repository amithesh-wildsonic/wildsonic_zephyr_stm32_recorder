# SPDX-License-Identifier: Apache-2.0
"""
Host tests for WildSonic Quiet Production Behavior, Recording LED Indicator,
and SRAM1/SRAM2 Memory Optimization.
"""

import os
import re
from pathlib import Path
import pytest

APP_DIR = Path(__file__).resolve().parent.parent.parent / "app"
SRC_DIR = APP_DIR / "src"
PRJ_CONF = APP_DIR / "prj.conf"


def test_production_logging_guards_and_levels():
    """Verify production logging policy in drivers and prj.conf."""
    # 1. prj.conf checks
    with open(PRJ_CONF, "r", encoding="utf-8") as f:
        prj = f.read()

    assert "CONFIG_SDMMC_LOG_LEVEL_WRN=y" in prj
    assert "CONFIG_DISK_LOG_LEVEL_WRN=y" in prj
    assert "CONFIG_FS_LOG_LEVEL_WRN=y" in prj
    assert "CONFIG_SDMMC_LOG_LEVEL_INF=y" not in prj

    # 2. Source module log levels guard checks
    modules_to_check = [
        "psram_is66wvs.c",
        "psram_staging.c",
        "dsp.c",
        "audio_in_stm32l4.c",
        "recorder.c",
    ]

    for mod in modules_to_check:
        path = SRC_DIR / mod
        with open(path, "r", encoding="utf-8") as f:
            code = f.read()

        assert "CONFIG_WILDSONIC_VERBOSE_LOGS" in code, f"{mod} missing verbose logs guard"
        assert "LOG_LEVEL_WRN" in code, f"{mod} missing LOG_LEVEL_WRN default for quiet production"


def test_led_indicator_architecture_and_states():
    """Verify LED indicator header and source implementation contracts."""
    h_path = SRC_DIR / "led_indicator.h"
    c_path = SRC_DIR / "led_indicator.c"

    assert h_path.exists(), "led_indicator.h must exist"
    assert c_path.exists(), "led_indicator.c must exist"

    with open(h_path, "r", encoding="utf-8") as f:
        h_code = f.read()

    assert "LED_IND_OFF" in h_code
    assert "LED_IND_RECORDING" in h_code
    assert "LED_IND_ERROR" in h_code
    assert "int led_indicator_init(void);" in h_code
    assert "void led_indicator_set(enum led_indicator_state state);" in h_code
    assert "enum led_indicator_state led_indicator_get(void);" in h_code

    with open(c_path, "r", encoding="utf-8") as f:
        c_code = f.read()

    # Verify non-blocking work queue usage
    assert "k_work_delayable" in c_code or "k_work_reschedule" in c_code
    assert "gpio_pin_toggle_dt" in c_code or "gpio_pin_set_dt" in c_code
    # Verify 1 Hz (500 ms) and 5 Hz (100 ms) intervals
    assert "500" in c_code
    assert "100" in c_code


def test_led_indicator_integration_in_recorder_and_main():
    """Verify LED indicator calls across recording lifecycle."""
    rec_path = SRC_DIR / "recorder.c"
    main_path = SRC_DIR / "main.c"
    console_path = SRC_DIR / "console.c"

    with open(rec_path, "r", encoding="utf-8") as f:
        rec_code = f.read()

    assert '#include "led_indicator.h"' in rec_code
    assert "led_indicator_set(LED_IND_RECORDING)" in rec_code
    assert "led_indicator_set(LED_IND_ERROR)" in rec_code
    assert "led_indicator_set(LED_IND_OFF)" in rec_code

    with open(main_path, "r", encoding="utf-8") as f:
        main_code = f.read()

    assert '#include "led_indicator.h"' in main_code
    assert "led_indicator_init();" in main_code

    with open(console_path, "r", encoding="utf-8") as f:
        console_code = f.read()

    assert '#include "led_indicator.h"' in console_code
    assert "led_indicator_set(LED_IND_OFF);" in console_code


def test_sram1_vs_sram2_memory_optimization():
    """Verify internal SRAM1 (96 KB) and SRAM2 (32 KB) section assignments and stack tuning."""
    rec_path = SRC_DIR / "recorder.c"
    with open(rec_path, "r", encoding="utf-8") as f:
        rec_code = f.read()

    # Audio block pool placed in SRAM1 section (which maps to STM32L4 SRAM2 at 0x10000000)
    assert 'STM32_SRAM2_RETENTION_SECTION' in rec_code
    assert 'section("SRAM1")' in rec_code or 'SRAM1' in rec_code
    assert "_k_mem_slab_buf_pool" in rec_code

    # SD writer stack tuned to 4096 bytes
    assert "SD_WRITER_STACK_SIZE 4096" in rec_code

    # Kernel stacks in prj.conf tuned to 4096 bytes
    with open(PRJ_CONF, "r", encoding="utf-8") as f:
        prj = f.read()

    assert "CONFIG_MAIN_STACK_SIZE=4096" in prj
    assert "CONFIG_ISR_STACK_SIZE=4096" in prj


def test_sd_write_failure_auto_recovery():
    """Verify SD write failure triggers clean error reporting and auto-recovery."""
    rec_path = SRC_DIR / "recorder.c"
    with open(rec_path, "r", encoding="utf-8") as f:
        rec_code = f.read()

    # Recovery function definition and caller at session end
    assert "int recorder_storage_recover(void)" in rec_code
    assert "recorder_storage_recover();" in rec_code
    assert "[ERROR] Recording session failed:" in rec_code


def test_test_build_configuration_fragment():
    """Verify test.conf exists and configures verbose development logging."""
    test_conf = APP_DIR / "test.conf"
    assert test_conf.exists(), "app/test.conf must exist for clean test builds"
    with open(test_conf, "r", encoding="utf-8") as f:
        conf = f.read()
    assert "CONFIG_WILDSONIC_VERBOSE_LOGS=y" in conf
    assert "CONFIG_SDMMC_LOG_LEVEL_INF=y" in conf
    assert "CONFIG_DISK_LOG_LEVEL_INF=y" in conf
    assert "CONFIG_FS_LOG_LEVEL_INF=y" in conf


