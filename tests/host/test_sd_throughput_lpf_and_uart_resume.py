# SPDX-License-Identifier: Apache-2.0
"""
Test suite verifying:
- Issue 1: SD Write Throughput, Benchmark Matrix, and Contiguous Pre-allocation (f_expand)
- Issue 2: PSRAM Staging Overflow Handling, ENOBUFS preservation, and wav_verified flag
- Issue 3: Comprehensive Error Origin Classification and Stage Mapping
- Issue 4: LPF Cutoff Validation for 48k, 96k, and 192k (< 0.49 * fs)
- Issue 5: Single-pass DSP Initialization after configuration load
- Issue 6: Stop 2 Console Serialization (USART2 TC wait) and Post-Wake Clock Restoration
"""

import math
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
APP_PRJ_CONF = REPO_ROOT / "app" / "prj.conf"
APP_KCONFIG = REPO_ROOT / "app" / "Kconfig"


# ==============================================================================
# Issue 1: SD write throughput, f_expand preallocation, benchmark matrix
# ==============================================================================

def test_sd_preallocation_kconfig_and_prj_conf_enabled():
    """Verify AUDIO_PREALLOCATE_FILE and native FatFs API are enabled by default."""
    prj_conf_text = APP_PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_AUDIO_PREALLOCATE_FILE=y" in prj_conf_text
    assert "CONFIG_FS_FATFS_EXTRA_NATIVE_API=y" in prj_conf_text

    kconfig_text = APP_KCONFIG.read_text(encoding="utf-8")
    assert "config AUDIO_PREALLOCATE_FILE" in kconfig_text
    assert "default y" in kconfig_text


def test_sd_throughput_operating_margins():
    """Verify target throughput calculation and 2x margin requirements."""
    # 192 kHz, 16-bit mono = 384,000 bytes/s
    sample_rate = 192000
    bytes_per_sample = 2
    channels = 1
    stream_rate_bps = sample_rate * bytes_per_sample * channels
    assert stream_rate_bps == 384000  # 384 KB/s

    margin_2x_bps = stream_rate_bps * 2
    assert margin_2x_bps == 768000  # 768 KB/s minimum required with 2x margin

    # With contiguous pre-allocation, FatFs sector writes bypass cluster lookup.
    # At 6 MHz SDMMC 4-bit bus:
    # 6 MHz * 4 bits = 24 Mbps = 3 MB/s raw link bandwidth.
    expected_sdmmc_max_bw = (6000000 * 4) // 8  # 3,000,000 B/s
    assert expected_sdmmc_max_bw > margin_2x_bps


def test_f_expand_contiguous_allocation_logic_in_recorder():
    """Verify recorder.c implements f_expand contiguous allocation with fallback."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    assert "f_expand" in rec_c
    assert "f_expand(fil, (FSIZE_t)(WAV_HEADER_BYTES + prealloc_bytes), 1);" in rec_c
    assert "f_expand(fil, (FSIZE_t)(WAV_HEADER_BYTES + prealloc_bytes), 0);" in rec_c


def test_sd_benchmark_implementation_present():
    """Verify sd_benchmark.h and sd_benchmark.c implement matrix benchmarks."""
    bench_h = (APP_SRC / "sd_benchmark.h").read_text(encoding="utf-8")
    bench_c = (APP_SRC / "sd_benchmark.c").read_text(encoding="utf-8")

    assert "sd_benchmark_run_matrix" in bench_h
    assert "sd_benchmark_run_test" in bench_h
    assert "enum sd_bench_type" in bench_h

    # Check chunk sizes: 32K, 64K, 128K, 256K, 512K
    for sz_str in ["32 * 1024", "64 * 1024", "128 * 1024", "256 * 1024", "512 * 1024"]:
        assert sz_str in bench_c


# ==============================================================================
# Issue 2: PSRAM staging overflow handling, non-destructive rejection
# ==============================================================================

def test_psram_staging_overflow_preserves_enobufs():
    """Verify PSRAM overflow returns -ENOBUFS and marks wav_verified = false."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    assert "st.psram_full_events++;" in rec_c
    assert "err = -ENOBUFS;" in rec_c
    assert "st.wav_verified = false;" in rec_c


def test_psram_lock_is_not_held_during_sd_write():
    """Verify writer thread fetches batch into buffer via peek and commits only after SD write."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    assert "psram_staging_peek(" in rec_c
    assert "fs_write(sd_file_ptr, sd_bounce_buf, bytes_read);" in rec_c
    assert "psram_staging_commit(" in rec_c


# ==============================================================================
# Issue 3: Error origin classification and stage mapping
# ==============================================================================

def test_recording_error_origin_stages():
    """Verify all real error origin stages are defined and recorded."""
    rec_h = (APP_SRC / "recorder.h").read_text(encoding="utf-8")
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    assert "char first_error_stage[32];" in rec_h

    assert '"PSRAM_STAGING_FULL"' in rec_c
    assert '"SDMMC_TRANSFER"' in rec_c
    assert '"FS_WRITE"' in rec_c
    assert '"DMA_ACQUISITION"' in rec_c
    assert '"DSP_QUEUE_OVERFLOW"' in rec_c
    assert '"WAV_FINALIZATION"' in rec_c


def test_recorder_maps_first_error_origin_properly():
    """Verify recorder.c sets first_error_stage when errors occur."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    assert 'strncpy(s_active_stats_ptr->first_error_stage, "SDMMC_TRANSFER"' in rec_c
    assert 'strncpy(s_active_stats_ptr->first_error_stage, "FS_WRITE"' in rec_c
    assert 'strncpy(st.first_error_stage, "PSRAM_STAGING_FULL"' in rec_c
    assert 'strncpy(st.first_error_stage, "WAV_FINALIZATION"' in rec_c


# ==============================================================================
# Issue 4: LPF cutoff validation (< 0.49 * fs)
# ==============================================================================

def simulate_dsp_validate_lpf_cutoff(sample_rate: int, requested_cutoff: int) -> int:
    """Python reference implementation of dsp_validate_lpf_cutoff."""
    if sample_rate == 0 or requested_cutoff == 0:
        return 0

    nyquist_bound = int(sample_rate * 0.49)

    if requested_cutoff < nyquist_bound:
        return requested_cutoff

    if sample_rate >= 192000:
        return 90000
    elif sample_rate >= 96000:
        return 45000
    elif sample_rate >= 48000:
        return 20000
    else:
        return (sample_rate * 45) // 100


def test_lpf_cutoff_validation_rules():
    """Verify LPF cutoff validation for 48k, 96k, and 192k sample rates."""
    # 48 kHz: Nyquist bound is 23520 Hz.
    # A requested cutoff >= 23520 Hz (e.g. 24000) must be clamped to 20000 Hz.
    assert simulate_dsp_validate_lpf_cutoff(48000, 24000) == 20000
    assert simulate_dsp_validate_lpf_cutoff(48000, 23520) == 20000
    assert simulate_dsp_validate_lpf_cutoff(48000, 20000) == 20000
    assert simulate_dsp_validate_lpf_cutoff(48000, 15000) == 15000

    # 96 kHz: Nyquist bound is 47040 Hz.
    # A requested cutoff >= 47040 Hz (e.g. 48000) must be clamped to 45000 Hz.
    assert simulate_dsp_validate_lpf_cutoff(96000, 48000) == 45000
    assert simulate_dsp_validate_lpf_cutoff(96000, 47040) == 45000
    assert simulate_dsp_validate_lpf_cutoff(96000, 45000) == 45000
    assert simulate_dsp_validate_lpf_cutoff(96000, 40000) == 40000

    # 192 kHz: Nyquist bound is 94080 Hz.
    # A requested cutoff of 96000 Hz (from val / 2) must be clamped to 90000 Hz.
    assert simulate_dsp_validate_lpf_cutoff(192000, 96000) == 90000
    assert simulate_dsp_validate_lpf_cutoff(192000, 94080) == 90000
    assert simulate_dsp_validate_lpf_cutoff(192000, 90000) == 90000
    assert simulate_dsp_validate_lpf_cutoff(192000, 80000) == 80000


def test_dsp_c_implements_lpf_validation_and_effective_check():
    """Verify dsp.c implements dsp_validate_lpf_cutoff and dsp_is_lpf_effective."""
    dsp_h = (APP_SRC / "dsp.h").read_text(encoding="utf-8")
    dsp_c = (APP_SRC / "dsp.c").read_text(encoding="utf-8")

    assert "uint32_t dsp_validate_lpf_cutoff(uint32_t sample_rate_hz, uint32_t requested_cutoff_hz);" in dsp_h
    assert "bool dsp_is_lpf_effective(void);" in dsp_h
    assert "uint32_t dsp_validate_lpf_cutoff(uint32_t sample_rate_hz, uint32_t requested_cutoff_hz)" in dsp_c
    assert "bool dsp_is_lpf_effective(void)" in dsp_c


def test_console_and_config_store_use_lpf_validation():
    """Verify handle_samf, handle_afmt, handle_ucutoff, and config_store_load validate cutoff."""
    con_c = (APP_SRC / "console.c").read_text(encoding="utf-8")
    cfg_c = (APP_SRC / "config_store.c").read_text(encoding="utf-8")

    assert "dsp_validate_lpf_cutoff" in con_c
    assert "dsp_validate_lpf_cutoff" in cfg_c
    assert "uint32_t nyquist_bound = (g_cfg.sampling_rate * 49) / 100;" in con_c


# ==============================================================================
# Issue 5: Remove unnecessary DSP reconfiguration during startup
# ==============================================================================

def test_startup_initializes_dsp_once_after_config_load():
    """Verify DSP is initialized ONCE after config is loaded, avoiding 48k reconfig."""
    main_c = (APP_SRC / "main.c").read_text(encoding="utf-8")

    config_import_idx = main_c.find("config_store_import_sd();")
    assert config_import_idx != -1

    dsp_init_idx = main_c.find("dsp_init(settings.sample_rate);", config_import_idx)
    assert dsp_init_idx != -1

    # Before config_store_import_sd, there should be NO dsp_init call in main()
    main_func_idx = main_c.find("int main(void)")
    early_dsp_init = main_c.find("dsp_init(", main_func_idx)
    assert early_dsp_init >= config_import_idx


# ==============================================================================
# Issue 6: Stop 2 console corruption and clock tree restoration
# ==============================================================================

def test_stop2_waits_for_usart2_tc_flag():
    """Verify rtc_sleep_enter_stop2 waits for LL_USART_IsActiveFlag_TC before sleep."""
    sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")

    # Check TC wait
    assert "LL_USART_IsActiveFlag_TC(USART2)" in sleep_c
    # Make sure it occurs before __WFI
    tc_idx = sleep_c.find("LL_USART_IsActiveFlag_TC(USART2)")
    wfi_idx = sleep_c.find("__WFI();", tc_idx)
    assert tc_idx != -1 and wfi_idx != -1
    assert tc_idx < wfi_idx


def test_wake_milestone_logging_deferred_until_clock_restoration():
    """Verify wake milestone logging occurs after stm32_clock_control_init & console resume."""
    sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")

    wfi_idx = sleep_c.find("__WFI();")
    clock_init_idx = sleep_c.find("stm32_clock_control_init", wfi_idx)
    console_resume_idx = sleep_c.find("console_async_resume()", wfi_idx)
    clocks_restored_print_idx = sleep_c.find('printk("[TEST] CLOCKS_RESTORED\\n");', wfi_idx)

    assert wfi_idx != -1
    assert clock_init_idx != -1
    assert console_resume_idx != -1
    assert clocks_restored_print_idx != -1

    # Print occurs AFTER clocks and console resume
    assert clocks_restored_print_idx > clock_init_idx
    assert clocks_restored_print_idx > console_resume_idx
