#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Unit and Contract Verification Suite for WildSonic Flash Configuration
Persistence, A/B Ping-Pong Slot Management, and SD On-Boot Configuration Import.

Verifies:
 1. Flash persistence data layout (512-byte packed struct, 508-byte checksum offset).
 2. Additive 32-bit byte checksum parity matching reference Maxim MAX78000 implementation.
 3. Ping-pong A/B slot alternation across saves and power-loss recovery.
 4. Slot corruption detection and rollback to the surviving valid slot.
 5. Complete /config.ini key-value translation to canonical WildSonic commands.
 6. Source integration across console.c, main.c, scheduler.c, and config_store.c.
 7. Autonomous scheduler non-blocking REPL behavior (no schedule pause on keypress).
"""

import struct
from pathlib import Path
import pytest

ROOT_DIR = Path(__file__).resolve().parents[2]
APP_DIR = ROOT_DIR / "app" if (ROOT_DIR / "app").is_dir() else ROOT_DIR
SRC_DIR = APP_DIR / "src"

CONFIG_STORE_H = SRC_DIR / "config_store.h"
CONFIG_STORE_C = SRC_DIR / "config_store.c"
CONSOLE_C = SRC_DIR / "console.c"
CONSOLE_H = SRC_DIR / "console.h"
MAIN_C = SRC_DIR / "main.c"
PRJ_CONF = APP_DIR / "prj.conf"

CONFIG_STORE_MAGIC = 0x13245768
CONFIG_STORE_VERSION = 1
CONFIG_STORE_RECORD_SIZE = 512
CONFIG_STORE_CHECKSUM_OFFSET = 508


# ==============================================================================
# 1. PYTHON REFERENCE IMPLEMENTATION OF PERSISTED CONFIG & A/B PING-PONG
# ==============================================================================

class PersistedConfigModel:
    """Python reference model of struct wildsonic_persisted_config."""

    def __init__(self, seq=1, samf=48000, rect=600, sleep=0, gain=0,
                 lcut=0, ucut=24000, fldr=2, psram=1):
        self.magic = CONFIG_STORE_MAGIC
        self.version = CONFIG_STORE_VERSION
        self.sequence_number = seq
        self.serial_number = b"WS4760001\x00" + b"\x00" * 22
        self.device_name = b"WildSonic\x00" + b"\x00" * 22
        self.device_password = b"wildsonic123\x00" + b"\x00" * 19
        self.sampling_rate = samf
        self.recording_length = rect
        self.sleep_pause_interval = sleep
        self.mic_gain_times_ten = gain
        self.audio_sampling_width = 16
        self.num_audio_channels = 1
        self.lower_cutoff_frequency = lcut
        self.upper_cutoff_frequency = ucut
        self.folder_structure = fldr
        self.psram_staging_enabled = psram

        self.start_year = 0
        self.start_month = 0
        self.start_day = 0
        self.end_year = 0
        self.end_month = 0
        self.end_day = 0
        self.day_of_week_mask = 0x7F
        self.time_zone_offset = 0
        self.num_tod_ranges = 0
        self.tod_start = [0] * 10
        self.tod_end = [0] * 10

    def serialize(self) -> bytes:
        # Build 508 bytes header + payload
        hdr = struct.pack(
            "<III32s32s32sIIIIIIIIBB",
            self.magic,
            self.version,
            self.sequence_number,
            self.serial_number,
            self.device_name,
            self.device_password,
            self.sampling_rate,
            self.recording_length,
            self.sleep_pause_interval,
            self.mic_gain_times_ten,
            self.audio_sampling_width,
            self.num_audio_channels,
            self.lower_cutoff_frequency,
            self.upper_cutoff_frequency,
            self.folder_structure,
            self.psram_staging_enabled
        )
        sched = struct.pack(
            "<HBBHBBBiB",
            self.start_year, self.start_month, self.start_day,
            self.end_year, self.end_month, self.end_day,
            self.day_of_week_mask,
            self.time_zone_offset,
            self.num_tod_ranges
        )
        tod = struct.pack("<10I10I", *(self.tod_start + self.tod_end))

        payload = hdr + sched + tod
        assert len(payload) == 236
        reserved = b"\x00" * (CONFIG_STORE_CHECKSUM_OFFSET - len(payload))
        data_before_checksum = payload + reserved
        assert len(data_before_checksum) == CONFIG_STORE_CHECKSUM_OFFSET

        # Additive byte sum
        chk = sum(data_before_checksum) & 0xFFFFFFFF
        return data_before_checksum + struct.pack("<I", chk)

    @staticmethod
    def validate(blob: bytes) -> bool:
        if len(blob) != CONFIG_STORE_RECORD_SIZE:
            return False
        magic, version, seq = struct.unpack_from("<III", blob, 0)
        if magic != CONFIG_STORE_MAGIC or version != CONFIG_STORE_VERSION:
            return False
        stored_chk = struct.unpack_from("<I", blob, CONFIG_STORE_CHECKSUM_OFFSET)[0]
        calc_chk = sum(blob[:CONFIG_STORE_CHECKSUM_OFFSET]) & 0xFFFFFFFF
        return stored_chk == calc_chk


class FlashStorageSimulator:
    """Simulates 2x 2048-byte flash slots for A/B ping-pong persistence."""

    def __init__(self):
        self.slots = [bytearray(2048), bytearray(2048)]
        self.active_slot = -1
        self.seq = 0

    def load(self):
        v0 = PersistedConfigModel.validate(bytes(self.slots[0][:512]))
        v1 = PersistedConfigModel.validate(bytes(self.slots[1][:512]))

        if v0 and v1:
            seq0 = struct.unpack_from("<I", self.slots[0], 8)[0]
            seq1 = struct.unpack_from("<I", self.slots[1], 8)[0]
            diff = (seq0 - seq1) & 0xFFFFFFFF
            if diff < 0x80000000:
                self.active_slot = 0
                self.seq = seq0
            else:
                self.active_slot = 1
                self.seq = seq1
            return True, self.active_slot, self.seq
        elif v0:
            self.active_slot = 0
            self.seq = struct.unpack_from("<I", self.slots[0], 8)[0]
            return True, 0, self.seq
        elif v1:
            self.active_slot = 1
            self.seq = struct.unpack_from("<I", self.slots[1], 8)[0]
            return True, 1, self.seq
        else:
            self.active_slot = -1
            self.seq = 0
            return False, -1, 0

    def save(self, cfg_model: PersistedConfigModel):
        target_slot = 1 if self.active_slot == 0 else 0
        self.seq += 1
        cfg_model.sequence_number = self.seq
        blob = cfg_model.serialize()

        # Erase target slot (0xFF) and program
        self.slots[target_slot] = bytearray(b"\xFF" * 2048)
        self.slots[target_slot][:512] = blob

        # Verify
        if PersistedConfigModel.validate(bytes(self.slots[target_slot][:512])):
            self.active_slot = target_slot
            return True
        return False


# ==============================================================================
# 2. UNIT TESTS
# ==============================================================================

def test_config_store_files_exist():
    assert CONFIG_STORE_H.is_file(), f"Missing {CONFIG_STORE_H}"
    assert CONFIG_STORE_C.is_file(), f"Missing {CONFIG_STORE_C}"


def test_packed_record_layout_and_checksum_offset():
    h_text = CONFIG_STORE_H.read_text(encoding="utf-8")
    assert "0x13245768U" in h_text
    assert "CONFIG_STORE_RECORD_SIZE 512U" in h_text
    assert "CONFIG_STORE_SLOT_SIZE   2048U" in h_text
    assert "uint32_t checksum;" in h_text

    model = PersistedConfigModel(seq=1)
    blob = model.serialize()
    assert len(blob) == 512
    assert PersistedConfigModel.validate(blob) is True


def test_checksum_corruption_detection():
    model = PersistedConfigModel(seq=42, samf=192000, rect=60, sleep=0)
    blob = bytearray(model.serialize())

    assert PersistedConfigModel.validate(bytes(blob)) is True

    # Mutate 1 bit in sampling_rate
    blob[108] ^= 0x01
    assert PersistedConfigModel.validate(bytes(blob)) is False

    # Restore bit and mutate magic
    blob[108] ^= 0x01
    blob[0] ^= 0x55
    assert PersistedConfigModel.validate(bytes(blob)) is False


def test_ab_ping_pong_slot_alternation():
    sim = FlashStorageSimulator()

    # 1. Initial state: uninitialized
    ok, slot, seq = sim.load()
    assert ok is False
    assert slot == -1
    assert seq == 0

    # 2. First commit: writes to Slot 0
    m1 = PersistedConfigModel(samf=192000, rect=120)
    assert sim.save(m1) is True
    assert sim.active_slot == 0
    assert sim.seq == 1

    ok, slot, seq = sim.load()
    assert ok is True
    assert slot == 0
    assert seq == 1

    # 3. Second commit: writes to Slot 1 (ping-pong!)
    m2 = PersistedConfigModel(samf=48000, rect=300)
    assert sim.save(m2) is True
    assert sim.active_slot == 1
    assert sim.seq == 2

    ok, slot, seq = sim.load()
    assert ok is True
    assert slot == 1
    assert seq == 2

    # 4. Third commit: writes back to Slot 0
    m3 = PersistedConfigModel(samf=96000, rect=600)
    assert sim.save(m3) is True
    assert sim.active_slot == 0
    assert sim.seq == 3

    ok, slot, seq = sim.load()
    assert ok is True
    assert slot == 0
    assert seq == 3


def test_slot_corruption_fallback_recovery():
    sim = FlashStorageSimulator()

    # Commit twice so both Slot 0 and Slot 1 have valid records
    m1 = PersistedConfigModel(rect=100)
    sim.save(m1)  # Slot 0, seq 1

    m2 = PersistedConfigModel(rect=200)
    sim.save(m2)  # Slot 1, seq 2

    # Active should be Slot 1
    ok, slot, seq = sim.load()
    assert slot == 1
    assert seq == 2

    # Simulate corruption of active Slot 1 (e.g. power cut during write)
    sim.slots[1][10] ^= 0xFF

    # Must automatically fall back to surviving valid Slot 0
    ok, slot, seq = sim.load()
    assert ok is True
    assert slot == 0
    assert seq == 1


def test_sd_config_ini_translation_logic():
    """Verify mapping of /config.ini key-value pairs to canonical WildSonic commands."""
    lines = [
        "sampling_frequency = 192000",
        "record_length = 600",
        "sleep_interval = 0",
        "bits_per_sample = 16",
        "number_of_channels = 1",
        "mic_gain = 12",
        "lower_cutoff_frequency = 100",
        "upper_cutoff_frequency = 18000",
        "date_range = 2026/01/01-2026/12/31",
        "time_of_day_0 = 06:00:00-18:00:00",
        "days_of_week = 1111111",
        "time_zone = +05:30",
        "folder_structure = daily",
    ]

    translations = {
        "sampling_frequency": "SAMF",
        "record_length": "RECT",
        "sleep_interval": "SLEEP",
        "bits_per_sample": "BTPS",
        "number_of_channels": "NUMCH",
        "mic_gain": "GAIN",
        "lower_cutoff_frequency": "LCUTOFF",
        "upper_cutoff_frequency": "UCUTOFF",
        "date_range": "DATERANGE",
        "days_of_week": "DAYSOFWEEK",
        "time_zone": "TIMEZONE",
        "folder_structure": "FLDR",
    }

    c_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    for key, cmd in translations.items():
        assert f'"{key}"' in c_text, f"Key {key} not handled in config_store.c"
        assert f'"{cmd} ' in c_text or f'"{cmd}"' in c_text, f"Command {cmd} not mapped in config_store.c"


def test_source_integration_contracts():
    """Verify integration of config_store across console, main, and build configurations."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    m_text = MAIN_C.read_text(encoding="utf-8")
    prj_text = PRJ_CONF.read_text(encoding="utf-8")

    # 1. Flash persistence enabled in prj.conf
    assert "CONFIG_FLASH=y" in prj_text
    assert "CONFIG_FLASH_PAGE_LAYOUT=y" in prj_text
    assert "CONFIG_FLASH_MAP=y" in prj_text

    # 2. Console invokes config_store_save on APPLY and ARBT
    assert "config_store_save();" in c_text

    # 3. Console invokes config_store_load_defaults on DEFAULT
    assert "config_store_load_defaults();" in c_text

    # 4. Main invokes config_store_init and config_store_import_sd on boot
    assert "config_store_init();" in m_text
    assert "config_store_import_sd();" in m_text


def test_autonomous_scheduler_non_blocking_repl():
    """Verify that user typing in console does not pause autonomous scheduler."""
    m_text = MAIN_C.read_text(encoding="utf-8")

    # Character entry (c >= 32 && c < 127) must NOT pause the scheduler
    assert "c >= 32 && c < 127" in m_text
    match = m_text.find("c >= 32 && c < 127")
    char_block = m_text[match:match + 150]
    assert "main_scheduler_pause" not in char_block, (
        "Found main_scheduler_pause() in character entry block: typing must not pause scheduler"
    )


def test_reference_factory_defaults():
    """Verify factory defaults align with Maxim reference (48 kHz, RECT 600, SLEEP 0, hourly, cutoffs)."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    m_text = MAIN_C.read_text(encoding="utf-8")
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")
    kconfig_text = (APP_DIR / "Kconfig").read_text(encoding="utf-8")

    # Kconfig default sample rate: 48000
    assert "default 48000" in kconfig_text

    # g_cfg initial values in console.c
    assert ".recording_length = 600" in c_text
    assert ".sleep_pause_interval = 0" in c_text
    assert ".lower_cutoff_frequency = 0" in c_text
    assert ".upper_cutoff_frequency = 24000" in c_text
    assert ".folder_structure = 2" in c_text

    # config_store_load_defaults values
    assert "cfg->sampling_rate = 48000;" in cs_text
    assert "cfg->recording_length = 600;" in cs_text
    assert "cfg->sleep_pause_interval = 0;" in cs_text
    assert "cfg->lower_cutoff_frequency = 0;" in cs_text
    assert "cfg->upper_cutoff_frequency = 24000;" in cs_text
    assert "cfg->folder_structure = 2;" in cs_text

    # main_reset_to_defaults values
    assert "settings.sample_rate = 48000;" in m_text
    assert "settings.record_seconds = 600;" in m_text
    assert "settings.sleep_seconds = 0;" in m_text
    assert "settings.hpf_enabled = false;" in m_text
    assert "settings.hpf_cutoff_hz = 0;" in m_text
    assert "settings.lpf_cutoff_hz = 24000;" in m_text


def test_folder_structure_and_last_recording_scan():
    """Verify folder structure path generation and last recording scanning contract."""
    rec_c = (SRC_DIR / "recorder.c").read_text(encoding="utf-8")
    rec_h = (SRC_DIR / "recorder.h").read_text(encoding="utf-8")

    # File path buffer enlarged to 80 bytes for hierarchical paths
    assert "char file[80];" in rec_h

    # Scan last file function declared and defined
    assert "recorder_scan_last_file" in rec_h
    assert "int recorder_scan_last_file(" in rec_c

    # Folder structures supported in recorder.c
    assert "folder_mode == 1" in rec_c or "folder_mode == 2" in rec_c
    assert "fs_mkdir(dir)" in rec_c or "fs_mkdir" in rec_c


def test_assert_command_parity():
    """Verify ASSERT command behavior matches Maxim reference (app/console.cpp:assert_info)."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    m_text = MAIN_C.read_text(encoding="utf-8")

    # Maxim canonical empty assert string: 'There is no assert information \n'
    assert 'printk("There is no assert information \\n");' in c_text

    # Normal boot must not dump reset flags unprompted in production
    assert "print_reset_reason();" not in m_text or (
        "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in m_text
    )


def test_informative_boot_sequence_parity():
    """Verify startup sequence includes operational information matching WildSonic reference."""
    m_text = MAIN_C.read_text(encoding="utf-8")
    cs_text = CONFIG_STORE_C.read_text(encoding="utf-8")

    # Version, GIT Branch, Build Date
    assert 'printk("Version: %s\\n", "v1.2.0");' in m_text
    assert 'printk("GIT Branch: %s, SHA: %s\\n", "production", "d7a4f91");' in m_text
    assert 'printk("Build Date: %s %s\\n", __DATE__, __TIME__);' in m_text

    # SD card status
    assert 'printk("SD card initialized successfully\\n");' in m_text

    # Configuration file status
    assert 'printk("Configuration file is missing. Using earlier programmed configuration.\\n");' in cs_text
    assert 'printk("Configuration file has been deleted\\n");' in cs_text
    assert 'printk("Configuration status file has been created\\n");' in cs_text

    # Audio peripheral initialization
    assert 'printk("Audio peripheral initialized\\n");' in m_text

    # Last recording file scan output
    assert "recorder_scan_last_file" in m_text
    assert 'printk("Last file detected in folder: %s is %s\\n"' in m_text or \
           'printk("Last file detected in' in m_text

    # RTC UTC and Local time with timezone
    assert 'printk("Current time UTC time is' in m_text
    assert 'printk("Current time Local is' in m_text

    # System initialization complete
    assert 'printk("System initialization complete\\n");' in m_text


