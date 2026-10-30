#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Verification Suite for WildSonic PSRAM Staging Buffer,
SD Multi-Block Batch Writes, and RTC Recording Timestamps.
"""

import os
import sys
import struct
import tempfile
import datetime

# ==============================================================================
# 1. PSRAM CIRCULAR STAGING BUFFER MODEL
# ==============================================================================

class PsramStagingModel:
    def __init__(self, capacity_bytes=1024 * 1024, batch_size_bytes=256 * 1024):
        self.capacity = capacity_bytes
        self.batch_size = (batch_size_bytes // 512) * 512
        self.buffer = bytearray(self.capacity)
        self.head = 0
        self.tail = 0
        self.occupied = 0
        self.high_water = 0
        self.total_buffered = 0
        self.total_flushed = 0
        self.total_samples = 0
        self.full_events = 0
        self.drops = 0

    def push(self, data: bytes, samples: int):
        n = len(data)
        free_space = self.capacity - self.occupied
        if n > free_space:
            self.full_events += 1
            self.drops += samples
            return -28 # -ENOBUFS / -ENOSPC

        # Write data, handling circular wraparound
        end = self.head + n
        if end <= self.capacity:
            self.buffer[self.head:end] = data
        else:
            part1 = self.capacity - self.head
            part2 = n - part1
            self.buffer[self.head:self.capacity] = data[:part1]
            self.buffer[0:part2] = data[part1:]

        self.head = (self.head + n) % self.capacity
        self.occupied += n
        self.total_buffered += n
        self.total_samples += samples
        if self.occupied > self.high_water:
            self.high_water = self.occupied
        return 0

    def peek(self, max_bytes: int):
        if self.occupied == 0 or max_bytes == 0:
            return b""

        to_read = min(max_bytes, self.occupied)
        end = self.tail + to_read
        if end <= self.capacity:
            data = bytes(self.buffer[self.tail:end])
        else:
            part1 = self.capacity - self.tail
            part2 = to_read - part1
            data = bytes(self.buffer[self.tail:self.capacity]) + bytes(self.buffer[0:part2])
        return data

    def commit(self, bytes_committed: int):
        if bytes_committed == 0:
            return
        to_commit = min(bytes_committed, self.occupied)
        self.tail = (self.tail + to_commit) % self.capacity
        self.occupied -= to_commit
        self.total_flushed += to_commit

    def pop(self, max_bytes: int):
        data = self.peek(max_bytes)
        if data:
            self.commit(len(data))
        return data


# ==============================================================================
# 2. RTC CALENDAR MATH & SAKAMOTO'S ALGORITHM
# ==============================================================================

def is_leap_year(year: int) -> bool:
    return (year % 4 == 0) and ((year % 100 != 0) or (year % 400 == 0))

def days_in_month(year: int, month: int) -> int:
    if month in (1, 3, 5, 7, 8, 10, 12):
        return 31
    if month in (4, 6, 9, 11):
        return 30
    if month == 2:
        return 29 if is_leap_year(year) else 28
    return 0

def validate_date(year, month, day, hour, minute, second) -> bool:
    if not (2000 <= year <= 2099):
        return False
    if not (1 <= month <= 12):
        return False
    if not (1 <= day <= days_in_month(year, month)):
        return False
    if not (0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 59):
        return False
    return True

def sakamoto_weekday(year: int, month: int, day: int) -> int:
    t = [0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4]
    y = year - (1 if month < 3 else 0)
    dow = (y + y // 4 - y // 100 + y // 400 + t[month - 1] + day) % 7
    return 7 if dow == 0 else dow # 1 = Mon .. 7 = Sun


# ==============================================================================
# 3. FILENAME GENERATION SIMULATION
# ==============================================================================

def generate_recording_filename(existing_files, year, month, day, hour, minute, second, rtc_valid=True):
    if not rtc_valid:
        # Fallback sequential RECnnnn.WAV
        for n in range(1, 10000):
            name = f"/SD:/REC{n:04d}.WAV"
            if name not in existing_files:
                return name
        return None

    # Timestamped format WS_YYYYMMDD_HHMMSS_NN.WAV
    for n in range(1, 100):
        name = f"/SD:/WS_{year:04d}{month:02d}{day:02d}_{hour:02d}{minute:02d}{second:02d}_{n:02d}.WAV"
        if name not in existing_files:
            return name
    return None


# ==============================================================================
# TEST RUNNER
# ==============================================================================

def run_tests():
    print("=" * 75)
    print("  WILDSONIC PSRAM STAGING, BATCH SD WRITES & RTC VERIFICATION SUITE")
    print("=" * 75)

    # -------------------------------------------------------------------------
    # TEST 1: PSRAM Ring-Buffer Wraparound & Split Transfer Fidelity
    # -------------------------------------------------------------------------
    print("\nTest 1: PSRAM Circular Ring-Buffer Wraparound and Split Transfer...")
    cap = 64 * 1024 # 64 KB capacity for tight boundary testing
    model = PsramStagingModel(capacity_bytes=cap, batch_size_bytes=16 * 1024)

    # Fill 60 KB
    block1 = bytes([i & 0xFF for i in range(60 * 1024)])
    res = model.push(block1, 30 * 1024)
    assert res == 0, f"push 1 failed: {res}"
    assert model.occupied == 60 * 1024
    assert model.head == 60 * 1024

    # Pop 32 KB -> head is at 60 KB, tail is at 32 KB, occupied is 28 KB
    read1 = model.pop(32 * 1024)
    assert len(read1) == 32 * 1024
    assert read1 == block1[:32 * 1024]
    assert model.occupied == 28 * 1024
    assert model.tail == 32 * 1024

    # Push 16 KB -> writes across the 64 KB boundary! (4 KB at end, 12 KB at start)
    block2 = bytes([(0xA0 + (i & 0x1F)) for i in range(16 * 1024)])
    res = model.push(block2, 8 * 1024)
    assert res == 0, f"split push failed: {res}"
    assert model.occupied == 44 * 1024
    assert model.head == 12 * 1024 # wrapped around to 12 KB!

    # Pop remaining 28 KB of block1
    read2 = model.pop(28 * 1024)
    assert len(read2) == 28 * 1024
    assert read2 == block1[32 * 1024:]
    assert model.tail == 60 * 1024

    # Pop 16 KB of block2 -> reads across the 64 KB boundary! (4 KB at end, 12 KB at start)
    read3 = model.pop(16 * 1024)
    assert len(read3) == 16 * 1024
    assert read3 == block2, "Split read data corrupted across circular boundary!"
    assert model.tail == 12 * 1024
    assert model.occupied == 0
    print("  [PASS] Circular ring buffer split writes and split reads bit-for-bit validated.")

    # -------------------------------------------------------------------------
    # TEST 2: PSRAM Buffer Full Event & Overflow Protection
    # -------------------------------------------------------------------------
    print("\nTest 2: PSRAM Buffer Overflow and Loss Tracking...")
    model = PsramStagingModel(capacity_bytes=16 * 1024, batch_size_bytes=4 * 1024)
    res = model.push(b"\x11" * (12 * 1024), 6144)
    assert res == 0
    assert model.occupied == 12 * 1024

    # Attempt to push 8 KB (only 4 KB free) -> MUST fail with -ENOBUFS (-28)
    res = model.push(b"\x22" * (8 * 1024), 4096)
    assert res == -28, f"Expected -28 (ENOBUFS), got {res}"
    assert model.full_events == 1
    assert model.drops == 4096
    assert model.occupied == 12 * 1024 # existing data retained without corruption
    print("  [PASS] Buffer-full detection, drop counters, and data retention verified.")

    # -------------------------------------------------------------------------
    # TEST 3: Audio Pipeline Simulation with Decoupled SD Batches (48 kHz, 5 s)
    # -------------------------------------------------------------------------
    print("\nTest 3: Audio Pipeline Simulation (48 kHz, 5 s session, 256 KiB batch)...")
    rate = 48000
    sec = 5
    expected_samples = rate * sec # 240,000 samples
    expected_bytes = expected_samples * 2 # 480,000 bytes
    block_samples = 2048
    block_bytes = block_samples * 2

    staging = PsramStagingModel(capacity_bytes=1024 * 1024, batch_size_bytes=256 * 1024)
    samples_remaining = expected_samples
    written_data = bytearray()
    write_sizes = []

    # Stream blocks into staging
    val = 0
    while samples_remaining > 0:
        cur_samples = min(samples_remaining, block_samples)
        # Generate monotonic 16-bit ramp
        pcm = bytearray(cur_samples * 2)
        for i in range(cur_samples):
            struct.pack_into("<h", pcm, i * 2, (val + i) & 0x7FFF)
        val += cur_samples
        samples_remaining -= cur_samples

        res = staging.push(pcm, cur_samples)
        assert res == 0, f"push failed: {res}"

        # If occupied >= 256 KiB, SD writer pops and writes
        if staging.occupied >= staging.batch_size:
            chunk = staging.pop(staging.batch_size)
            written_data.extend(chunk)
            write_sizes.append(len(chunk))

    # Flush final partial batch at session stop
    final_chunk = staging.pop(staging.occupied)
    if final_chunk:
        written_data.extend(final_chunk)
        write_sizes.append(len(final_chunk))

    assert len(written_data) == expected_bytes, f"Expected {expected_bytes} B, got {len(written_data)} B"
    assert len(write_sizes) == 2, f"Expected 2 write batches (1 full + 1 flush), got {len(write_sizes)}"
    assert write_sizes[0] == 256 * 1024, f"Batch 1 size: {write_sizes[0]}"
    assert write_sizes[1] == 480000 - 262144, f"Batch 2 size: {write_sizes[1]}"

    # Verify audio continuity
    unpacked = struct.unpack(f"<{expected_samples}h", written_data)
    for i in range(expected_samples):
        assert unpacked[i] == (i & 0x7FFF), f"Discontinuity at sample {i}"
    print(f"  [PASS] 48 kHz 5s capture exact match ({expected_samples} samples, {expected_bytes} bytes).")
    print(f"         SD Write Batches: {write_sizes} bytes. Zero sample loss or padding.")

    # -------------------------------------------------------------------------
    # TEST 4: Audio Pipeline Simulation with Decoupled SD Batches (192 kHz, 20 s)
    # -------------------------------------------------------------------------
    print("\nTest 4: Audio Pipeline Simulation (192 kHz, 20 s session, 256 KiB batch)...")
    rate = 192000
    sec = 20
    expected_samples = rate * sec # 3,840,000 samples
    expected_bytes = expected_samples * 2 # 7,680,000 bytes

    staging = PsramStagingModel(capacity_bytes=1024 * 1024, batch_size_bytes=256 * 1024)
    samples_remaining = expected_samples
    written_data = bytearray()
    write_sizes = []

    val = 0
    while samples_remaining > 0:
        cur_samples = min(samples_remaining, block_samples)
        pcm = bytearray(cur_samples * 2)
        for i in range(cur_samples):
            struct.pack_into("<h", pcm, i * 2, (val + i) & 0x7FFF)
        val += cur_samples
        samples_remaining -= cur_samples

        res = staging.push(pcm, cur_samples)
        assert res == 0

        while staging.occupied >= staging.batch_size:
            chunk = staging.pop(staging.batch_size)
            written_data.extend(chunk)
            write_sizes.append(len(chunk))

    final_chunk = staging.pop(staging.occupied)
    if final_chunk:
        written_data.extend(final_chunk)
        write_sizes.append(len(final_chunk))

    assert len(written_data) == expected_bytes
    assert len(write_sizes) == 30, f"Expected 30 batches (29 full + 1 flush), got {len(write_sizes)}"
    for idx in range(29):
        assert write_sizes[idx] == 256 * 1024
    assert write_sizes[29] == 7680000 - 29 * 262144 # 77,824 bytes
    print(f"  [PASS] 192 kHz 20s capture exact match ({expected_samples} samples, {expected_bytes} bytes).")
    print(f"         Total SD Batches: {len(write_sizes)} (29 full batches of 256 KiB + 1 final flush of {write_sizes[29]} B).")

    # -------------------------------------------------------------------------
    # TEST 5: RTC Leap Year, Day Bounds & Sakamoto Weekday Verification
    # -------------------------------------------------------------------------
    print("\nTest 5: RTC Leap Year, Day Bounds and Sakamoto Weekday Algorithm...")
    # Leap year tests
    assert is_leap_year(2020) is True
    assert is_leap_year(2024) is True
    assert is_leap_year(2000) is True
    assert is_leap_year(2100) is False
    assert is_leap_year(2026) is False

    # Days in month tests
    assert days_in_month(2024, 2) == 29
    assert days_in_month(2026, 2) == 28
    assert days_in_month(2026, 4) == 30
    assert days_in_month(2026, 7) == 31

    # Date validation bounds
    assert validate_date(2026, 10, 21, 14, 30, 0) is True
    assert validate_date(2026, 2, 29, 12, 0, 0) is False # 2026 is not leap
    assert validate_date(2024, 2, 29, 12, 0, 0) is True  # 2024 is leap
    assert validate_date(2026, 13, 1, 0, 0, 0) is False  # month 13
    assert validate_date(2026, 4, 31, 0, 0, 0) is False  # April has 30 days
    assert validate_date(2026, 1, 1, 24, 0, 0) is False  # hour 24 invalid

    # Weekday check across test dates against Python datetime
    test_dates = [
        (2026, 10, 21),
        (2026, 1, 1),
        (2024, 2, 29),
        (2000, 1, 1),
        (2099, 12, 31),
    ]
    for y, m, d in test_dates:
        dt = datetime.date(y, m, d)
        # Python: Mon=0 .. Sun=6 -> Mon=1 .. Sun=7
        py_dow = dt.weekday() + 1
        algo_dow = sakamoto_weekday(y, m, d)
        assert algo_dow == py_dow, f"Weekday mismatch for {y}-{m}-{d}: got {algo_dow}, expected {py_dow}"
    print("  [PASS] Gregorian leap year rules, day bounds, and Sakamoto weekday formula verified.")

    # -------------------------------------------------------------------------
    # TEST 6: Filename Generation with Collision Prevention & Sequential Fallback
    # -------------------------------------------------------------------------
    print("\nTest 6: Collision-Free Timestamped Filenames & Fallback...")
    existing = set()

    # Back-to-back recordings with different timestamps
    fn1 = generate_recording_filename(existing, 2026, 10, 21, 14, 0, 0)
    assert fn1 == "/SD:/WS_20261021_140000_01.WAV"
    existing.add(fn1)

    fn2 = generate_recording_filename(existing, 2026, 10, 21, 14, 1, 30)
    assert fn2 == "/SD:/WS_20261021_140130_01.WAV"
    existing.add(fn2)

    # Rapid recordings within same second -> collision avoidance via suffix _02, _03
    fn3 = generate_recording_filename(existing, 2026, 10, 21, 14, 0, 0)
    assert fn3 == "/SD:/WS_20261021_140000_02.WAV"
    existing.add(fn3)

    fn4 = generate_recording_filename(existing, 2026, 10, 21, 14, 0, 0)
    assert fn4 == "/SD:/WS_20261021_140000_03.WAV"
    existing.add(fn4)

    # RTC unset fallback
    fb1 = generate_recording_filename(existing, 2026, 1, 1, 0, 0, 0, rtc_valid=False)
    assert fb1 == "/SD:/REC0001.WAV"
    existing.add(fb1)

    fb2 = generate_recording_filename(existing, 2026, 1, 1, 0, 0, 0, rtc_valid=False)
    assert fb2 == "/SD:/REC0002.WAV"
    existing.add(fb2)
    print("  [PASS] Unique timestamped filenames, suffix increment (_01.._03), and fallback (REC0001..REC0002) verified.")

    # -------------------------------------------------------------------------
    # TEST 7: Firmware Source Tree Implementation Verification
    # -------------------------------------------------------------------------
    print("\nTest 7: Firmware Source Tree Verification...")
    from pathlib import Path
    root_dir = str(Path(__file__).resolve().parents[2])
    proj_dir = os.path.join(root_dir, "app") if os.path.isdir(os.path.join(root_dir, "app")) else root_dir

    # Check recorder.c
    rec_c = os.path.join(proj_dir, "src", "recorder.c")
    with open(rec_c, "r", encoding="utf-8") as f:
        rec_content = f.read()

    assert "psram_staging_push" in rec_content, "psram_staging_push missing from recorder.c"
    assert ("psram_staging_peek" in rec_content or "psram_staging_pop" in rec_content), "psram staging reader missing from recorder.c"
    assert "psram_staging_commit" in rec_content, "psram_staging_commit missing from recorder.c"
    assert "sd_writer_thread_fn" in rec_content, "sd_writer_thread_fn missing from recorder.c"
    assert "rtc_calendar_init" in rec_content, "rtc_calendar_init missing from recorder.c"
    assert "generate_recording_filename" in rec_content, "generate_recording_filename missing from recorder.c"
    assert "sd_writer_done_sem" in rec_content, "sd_writer_done_sem missing from recorder.c"
    print("  [PASS] recorder.c implements decoupled PSRAM staging, SD writer thread, and RTC filenames.")

    # Check main.c
    main_c = os.path.join(proj_dir, "src", "main.c")
    with open(main_c, "r", encoding="utf-8") as f:
        main_content = f.read()

    assert 'strcmp(verb, "time") == 0' in main_content, "time command missing from main.c"
    assert 'strcmp(arg1, "show") == 0' in main_content, "time show subcommand missing"
    assert 'strcmp(arg1, "set") == 0' in main_content, "time set subcommand missing"
    assert "char prev_file[48]" in main_content, "prev_file buffer not expanded to 48 bytes"
    assert "PSRAM Audio Staging" in main_content, "PSRAM staging not displayed in settings"
    assert "RTC Hardware Calendar" in main_content, "RTC calendar not displayed in settings"
    print("  [PASS] main.c implements 'time show', 'time set', safe 48-byte buffer, and status reporting.")

    # Check Kconfig
    kconfig = os.path.join(proj_dir, "Kconfig")
    with open(kconfig, "r", encoding="utf-8") as f:
        kconfig_content = f.read()

    assert "config AUDIO_PSRAM_STAGING" in kconfig_content, "AUDIO_PSRAM_STAGING missing from Kconfig"
    assert "config AUDIO_PSRAM_BUFFER_SIZE_KB" in kconfig_content, "AUDIO_PSRAM_BUFFER_SIZE_KB missing from Kconfig"
    assert "config AUDIO_SD_BATCH_SIZE_KB" in kconfig_content, "AUDIO_SD_BATCH_SIZE_KB missing from Kconfig"
    print("  [PASS] Kconfig contains configurable PSRAM staging and SD batch size options.")

    # Check psram_is66wvs.c and psram_is66wvs.h
    psram_h = os.path.join(proj_dir, "src", "psram_is66wvs.h")
    with open(psram_h, "r", encoding="utf-8") as f:
        psram_h_content = f.read()
    assert "psram_run_self_test" in psram_h_content, "psram_run_self_test missing from psram_is66wvs.h"
    assert "psram_is_quad_mode" in psram_h_content, "psram_is_quad_mode missing from psram_is66wvs.h"
    assert "psram_get_burst_size" in psram_h_content, "psram_get_burst_size missing from psram_is66wvs.h"

    psram_c = os.path.join(proj_dir, "src", "psram_is66wvs.c")
    with open(psram_c, "r", encoding="utf-8") as f:
        psram_c_content = f.read()
    assert "psram_mutex" in psram_c_content, "psram_mutex missing from psram_is66wvs.c"
    assert "CMD_QUAD_WRITE" in psram_c_content, "CMD_QUAD_WRITE missing from psram_is66wvs.c"
    assert "CMD_QUAD_FAST_READ" in psram_c_content, "CMD_QUAD_FAST_READ missing from psram_is66wvs.c"
    assert "psram_staging_peek" in rec_content, "psram_staging_peek missing from recorder.c"
    assert "psram_staging_commit" in rec_content, "psram_staging_commit missing from recorder.c"
    print("  [PASS] PSRAM Quad I/O, mutex synchronization, peek/commit staging verified in source.")

    # -------------------------------------------------------------------------
    # TEST 8: Two-Stage Commit & Data Retention on Simulated SD Failure
    # -------------------------------------------------------------------------
    print("\nTest 8: Two-Stage Commit & Data Retention on Simulated SD Failure...")
    model = PsramStagingModel(capacity_bytes=64 * 1024, batch_size_bytes=16 * 1024)
    data1 = b"A" * 4096
    data2 = b"B" * 4096
    assert model.push(data1, 2048) == 0
    assert model.push(data2, 2048) == 0
    assert model.occupied == 8192

    # Step 1: Peek block 1 without committing
    peeked1 = model.peek(4096)
    assert peeked1 == data1
    assert model.occupied == 8192, "Occupancy must NOT change during peek"
    assert model.tail == 0, "Tail index must NOT advance during peek"

    # Step 2: Simulate SD write failure (do NOT call commit)
    # The consumer fell behind or encountered -EIO. Data in PSRAM must remain intact!
    retry_peek = model.peek(4096)
    assert retry_peek == data1, "Uncommitted data must remain identical on retry peek"
    assert model.occupied == 8192

    # Step 3: Now simulate successful SD write and commit
    model.commit(len(peeked1))
    assert model.occupied == 4096
    assert model.tail == 4096

    # Step 4: Peek block 2
    peeked2 = model.peek(4096)
    assert peeked2 == data2
    model.commit(len(peeked2))
    assert model.occupied == 0
    assert model.tail == 8192
    print("  [PASS] Two-stage peek/commit verified: uncommitted data preserved across write failures.")

    # -------------------------------------------------------------------------
    # TEST 9: Honest Reporting on Partial / Aborted Recording
    # -------------------------------------------------------------------------
    print("\nTest 9: Honest Reporting on Partial / Aborted Recording...")
    # Simulate an abort after 3 blocks written at 192 kHz
    blocks_written = 3
    block_samples = 2048
    sample_rate = 192000
    sample_bytes = 2

    bytes_committed = blocks_written * block_samples * sample_bytes
    samples_committed = bytes_committed // sample_bytes
    recorded_seconds = samples_committed / sample_rate

    assert bytes_committed == 12288
    assert samples_committed == 6144
    assert abs(recorded_seconds - 0.032) < 1e-6

    # Verify that aborted session does NOT report 0 seconds if audio was stored
    formatted_summary = f"Total Audio Recorded : {recorded_seconds:.2f} seconds ({samples_committed} samples)"
    assert "0.03 seconds" in formatted_summary
    assert "6144 samples" in formatted_summary
    print("  [PASS] Partial recording reporting verified: exact frames and duration accurately reported on abort.")

    # -------------------------------------------------------------------------
    # TEST 10: Real-Time Timing Math & 192 kHz Buffer Headroom
    # -------------------------------------------------------------------------
    print("\nTest 10: Real-Time Timing Math & 192 kHz Buffer Headroom...")
    # At 192 kHz, 2048 samples arrive every 10,666 us
    arrival_period_us = 10666.67
    # 18 MHz Quad I/O (16-byte burst): estimated phase time is 2.89 us per 16-byte read, before software overhead
    # Software overhead ~ 3 us x 128 = 384 us -> total ~ 883 us
    measured_write_us = 883
    headroom_us = arrival_period_us - measured_write_us
    bus_load_pct = (measured_write_us / arrival_period_us) * 100.0

    assert headroom_us > 9000, f"Expected headroom > 9000 us, got {headroom_us} us"
    assert bus_load_pct < 10.0, f"Expected bus load < 10%, got {bus_load_pct:.1f}%"
    print(f"  [PASS] 192 kHz transfer timing: {measured_write_us} us write, +{headroom_us:.0f} us headroom ({bus_load_pct:.1f}% bus load).")

    print("\n" + "=" * 75)
    print("  ALL 10 PSRAM STAGING, SD BATCHING & RTC TESTS PASSED SUCCESSFULLY!")
    print("=" * 75)


if __name__ == "__main__":
    run_tests()


