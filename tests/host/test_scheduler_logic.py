#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Unit and Logic Verification Suite for WildSonic Calendar/Time Scheduler.
Covers:
 - Source contract and command registration checks
 - Calendar to epoch & epoch to calendar conversions (including leap years)
 - Weekday calculation & Sunday-to-Monday rollover
 - Day of week bitmask parsing and contiguous active day search
 - Up to 10 time-of-day ranges: sorting ascending, merging overlaps, invalid range handling
 - All-day unconstrained sentinel (0..86400)
 - Timezone conversions with negative/positive offsets and date rollover
 - Inclusive end date semantics (adding 86400 seconds)
 - Sleep interval spacing between sessions
 - Overdue schedule windows and immediate catch-up
 - Expired schedules and boundary conditions
"""

import os
import sys
import pytest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
APP_DIR = ROOT_DIR / "app" if (ROOT_DIR / "app").is_dir() else ROOT_DIR
SRC_DIR = APP_DIR / "src"

# ==============================================================================
# 1. PURE PYTHON SCHEDULER REFERENCE MODEL (MATCHING MAXIM / C BEHAVIOR)
# ==============================================================================

SECS_PER_MIN = 60
SECS_PER_HR = 3600
SECS_PER_DAY = 86400
FULL_WEEK_MASK = 0x7F
MAX_TOD_RANGES = 10

DAYS_BEFORE_MONTH = [0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334]


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


def calendar_to_epoch(year, month, day, hour=0, minute=0, second=0) -> int:
    if year < 1970 or month < 1 or month > 12 or day < 1 or day > 31:
        return 0
    y = year - 1970
    days = y * 365 + (year - 1969) // 4 - (year - 1901) // 100 + (year - 1601) // 400
    days += DAYS_BEFORE_MONTH[month]
    if month > 2 and is_leap_year(year):
        days += 1
    days += (day - 1)
    return days * SECS_PER_DAY + hour * SECS_PER_HR + minute * SECS_PER_MIN + second


def epoch_to_calendar(epoch: int):
    if epoch <= 0:
        return (2000, 1, 1, 0, 0, 0)
    sec = epoch % 60
    epoch //= 60
    minute = epoch % 60
    epoch //= 60
    hour = epoch % 24
    epoch //= 24
    days = epoch

    year = 1970
    if days >= 10957:
        year = 2000
        days -= 10957

    while year <= 2099:
        dim = 366 if is_leap_year(year) else 365
        if days < dim:
            break
        days -= dim
        year += 1

    month = 1
    while month <= 12:
        dim = days_in_month(year, month)
        if days < dim:
            break
        days -= dim
        month += 1

    day = days + 1
    return (year, month, day, hour, minute, sec)


def epoch_to_weekday(epoch: int) -> int:
    """Returns 0=Mon, 1=Tue, ..., 6=Sun"""
    days = epoch // SECS_PER_DAY
    return (days + 3) % 7


def is_valid_tod_range(start_sec: int, end_sec: int) -> bool:
    return (start_sec > 0 or end_sec < SECS_PER_DAY) and (start_sec < end_sec) and (end_sec <= SECS_PER_DAY)


def sort_and_merge_tod_ranges(ranges):
    """Sorts ascending by start_sec, merges overlapping, packs up to 10."""
    valid_ranges = [r for r in ranges if is_valid_tod_range(r[0], r[1])]
    valid_ranges.sort(key=lambda r: r[0])

    merged = []
    for r in valid_ranges:
        if not merged:
            merged.append(list(r))
        else:
            prev = merged[-1]
            if r[0] <= prev[1]:
                prev[1] = max(prev[1], r[1])
            else:
                merged.append(list(r))

    result = [tuple(r) for r in merged]
    while len(result) < MAX_TOD_RANGES:
        result.append((0, SECS_PER_DAY))
    return result


def find_day_interval(day_mask: int, start_boundary: int, end_boundary: int):
    """Finds earliest active day interval based on day_of_week_mask (bit 0=Mon..bit 6=Sun)."""
    if day_mask == 0:
        return False, end_boundary, end_boundary
    if day_mask == FULL_WEEK_MASK:
        return True, start_boundary, end_boundary

    current_weekday = epoch_to_weekday(start_boundary)
    first_active_offset = None
    w = current_weekday
    for offset in range(7):
        if day_mask & (1 << w):
            first_active_offset = offset
            break
        w = (w + 1) % 7

    if first_active_offset is None:
        return False, end_boundary, end_boundary

    start_day_begin = (start_boundary // SECS_PER_DAY) * SECS_PER_DAY
    active_day_begin = start_day_begin + first_active_offset * SECS_PER_DAY

    # Count consecutive active days
    consecutive = 0
    w = (current_weekday + first_active_offset) % 7
    for _ in range(7):
        if (day_mask & (1 << w)) == 0:
            break
        consecutive += 1
        w = (w + 1) % 7

    active_day_end = active_day_begin + consecutive * SECS_PER_DAY

    record_start = max(start_boundary, active_day_begin)
    record_end = min(end_boundary, active_day_end)
    return (record_start < record_end), record_start, record_end


def find_tod_interval(ranges, start_boundary: int, end_boundary: int):
    """Evaluates time-of-day ranges over a 2-day period."""
    valid_ranges = [r for r in ranges if is_valid_tod_range(r[0], r[1])]
    if not valid_ranges:
        return True, start_boundary, end_boundary

    start_day_begin = (start_boundary // SECS_PER_DAY) * SECS_PER_DAY

    for day_idx in range(2):
        day_base = start_day_begin + day_idx * SECS_PER_DAY
        for r_start, r_end in valid_ranges:
            w_start = day_base + r_start
            w_end = day_base + r_end
            if w_start < end_boundary and w_end > start_boundary:
                rec_start = max(start_boundary, w_start)
                rec_end = min(end_boundary, w_end)
                if rec_start < rec_end:
                    return True, rec_start, rec_end

    return False, end_boundary, end_boundary


def calculate_next_session(
    cfg_start_date,  # (year, month, day) or None
    cfg_end_date,    # (year, month, day) or None
    day_mask: int,
    tod_ranges,
    tz_offset_sec: int,
    record_len: int,
    sleep_interval: int,
    current_time_utc: int,
    last_end_time_local: int = 0
):
    """Replicates the full calculation from AudioTask.cpp / scheduler.c."""
    current_time_local = current_time_utc + tz_offset_sec

    # Start and end date bounds in local time
    start_bound = 0
    if cfg_start_date and cfg_start_date[0] > 0:
        start_bound = calendar_to_epoch(*cfg_start_date)

    end_bound = 1 << 62  # INT64_MAX equivalent
    if cfg_end_date and cfg_end_date[0] > 0:
        # Maxim scheduler adds 1 day to make end date inclusive
        end_bound = calendar_to_epoch(*cfg_end_date) + SECS_PER_DAY

    active_start_local = max(current_time_local, start_bound)

    # Respect sleep interval
    if sleep_interval > 0 and last_end_time_local > 0:
        time_after_sleep = last_end_time_local + sleep_interval
        if time_after_sleep > active_start_local:
            active_start_local = time_after_sleep

    if end_bound < active_start_local:
        end_bound = active_start_local

    start_boundary = active_start_local
    end_boundary = end_bound
    found_time_range = False

    for _ in range(2):
        found_day, day_start, day_end = find_day_interval(day_mask, start_boundary, end_boundary)
        if found_day:
            found_tod, tod_start, tod_end = find_tod_interval(tod_ranges, day_start, day_end)
            if found_tod:
                active_start_local = tod_start
                active_end_local = tod_end
                found_time_range = True
                break
        start_boundary = day_end

    if not found_time_range:
        return False, 0, 0, False, 0

    # Shorten active window to record length if specified (per scheduler.c:395-401)
    if record_len > 0 and (active_end_local > active_start_local):
        delta = active_end_local - active_start_local
        if delta >= record_len:
            active_end_local = active_start_local + record_len

    if active_end_local <= active_start_local:
        return False, 0, 0, False, 0

    duration = active_end_local - active_start_local
    start_time_utc = active_start_local - tz_offset_sec
    wait_sec = start_time_utc - current_time_utc
    is_immediate = (wait_sec <= 0)

    return True, start_time_utc, duration, is_immediate, max(0, wait_sec)


# ==============================================================================
# 2. SOURCE FILE AUDIT & CONTRACT VERIFICATION
# ==============================================================================

def test_scheduler_source_contract():
    """Verify presence of scheduler files, 10-range constant, and command handlers."""
    sched_h = SRC_DIR / "scheduler.h"
    sched_c = SRC_DIR / "scheduler.c"
    main_c = SRC_DIR / "main.c"

    assert sched_h.is_file(), "scheduler.h missing"
    assert sched_c.is_file(), "scheduler.c missing"
    assert main_c.is_file(), "main.c missing"

    h_text = sched_h.read_text(encoding="utf-8")
    assert "SCHEDULER_MAX_TIME_OF_DAY_RANGES 10" in h_text, "SCHEDULER_MAX_TIME_OF_DAY_RANGES must be exactly 10"
    assert "scheduler_calculate_next" in h_text
    assert "scheduler_find_day_interval" in h_text
    assert "scheduler_find_tod_interval" in h_text
    assert "scheduler_parse_date_range" in h_text
    assert "scheduler_parse_time_of_day" in h_text
    assert "scheduler_parse_days_of_week" in h_text
    assert "scheduler_parse_timezone" in h_text

    m_text = main_c.read_text(encoding="utf-8")
    assert '#include "scheduler.h"' in m_text
    assert "scheduler_init()" in m_text
    assert "run_scheduled_recording" in m_text
    assert 'strcmp(verb, "schedule") == 0' in m_text
    assert 'strcmp(verb, "daterange") == 0' in m_text
    assert 'strcmp(verb, "timeofday") == 0' in m_text
    assert 'strcmp(verb, "daysofweek") == 0' in m_text
    assert 'strcmp(verb, "timezone") == 0' in m_text

    console_c = SRC_DIR / "console.c"
    console_h = SRC_DIR / "console.h"
    assert console_c.is_file(), "console.c missing"
    assert console_h.is_file(), "console.h missing"

    c_text = console_c.read_text(encoding="utf-8")
    assert '"DATERANGE"' in c_text
    assert '"TIMEOFDAY"' in c_text
    assert '"DAYSOFWEEK"' in c_text
    assert '"TIMEZONE"' in c_text
    assert "handle_daterange" in c_text
    assert "handle_timeofday" in c_text
    assert "handle_daysofweek" in c_text
    assert "handle_timezone" in c_text


# ==============================================================================
# 3. DATE, TIME, AND CALENDAR MATH TESTS
# ==============================================================================

def test_calendar_to_epoch_and_back():
    """Test round-trip epoch and calendar conversion across leap and non-leap years."""
    test_dates = [
        (2000, 1, 1, 0, 0, 0),
        (2024, 2, 29, 23, 59, 59),  # Leap day
        (2026, 10, 24, 15, 30, 0),  # Present date
        (2028, 12, 31, 23, 59, 59),  # Leap year end
        (2030, 6, 15, 12, 0, 0),
    ]

    for y, m, d, hr, mn, sc in test_dates:
        ep = calendar_to_epoch(y, m, d, hr, mn, sc)
        assert ep > 0, f"Epoch conversion failed for {y}-{m}-{d}"
        res_y, res_m, res_d, res_hr, res_mn, res_sc = epoch_to_calendar(ep)
        assert (res_y, res_m, res_d, res_hr, res_mn, res_sc) == (y, m, d, hr, mn, sc)


def test_weekday_calculation():
    """Test day-of-week calculation: 0=Mon, 1=Tue, ..., 6=Sun."""
    # 2026-10-24 is a Saturday (index 5)
    ep_sat = calendar_to_epoch(2026, 10, 24, 12, 0, 0)
    assert epoch_to_weekday(ep_sat) == 5  # Saturday

    # 2026-10-25 is a Sunday (index 6)
    ep_sun = calendar_to_epoch(2026, 10, 25, 12, 0, 0)
    assert epoch_to_weekday(ep_sun) == 6  # Sunday

    # 2026-10-26 is a Monday (index 0) - testing weekend-to-weekday rollover
    ep_mon = calendar_to_epoch(2026, 10, 26, 12, 0, 0)
    assert epoch_to_weekday(ep_mon) == 0  # Monday


# ==============================================================================
# 4. DAY OF WEEK MASK & ROLLOVER TESTS
# ==============================================================================

def test_day_of_week_mask_filtering():
    """Test filtering across different day-of-week bitmasks."""
    # Reference: Saturday 2026-10-24 10:00:00
    now = calendar_to_epoch(2026, 10, 24, 10, 0, 0)
    horizon = now + 7 * SECS_PER_DAY

    # 1. Weekday-only mask: MTWRF (bits 0,1,2,3,4 = 0x1F)
    # Today is Saturday, so next active day must be Monday 2026-10-26 00:00:00
    mask_weekdays = 0x1F
    ok, rec_st, rec_en = find_day_interval(mask_weekdays, now, horizon)
    assert ok
    y, m, d, hr, mn, sc = epoch_to_calendar(rec_st)
    assert (y, m, d, hr, mn, sc) == (2026, 10, 26, 0, 0, 0), "Expected start at Monday midnight"

    # 2. Weekend-only mask: SU (Sat=bit 5 (0x20), Sun=bit 6 (0x40) -> 0x60)
    # Today is Saturday 10:00:00, so it should be active immediately today until Sun 24:00
    mask_weekends = 0x60
    ok, rec_st, rec_en = find_day_interval(mask_weekends, now, horizon)
    assert ok
    assert rec_st == now
    y, m, d, hr, mn, sc = epoch_to_calendar(rec_en)
    assert (y, m, d) == (2026, 10, 26), "Weekend ends Monday 00:00"

    # 3. Sunday-only mask: U (bit 6 = 0x40)
    # Today is Saturday, next active day is Sunday 2026-10-25 00:00:00
    ok, rec_st, rec_en = find_day_interval(0x40, now, horizon)
    assert ok
    y, m, d, hr, mn, sc = epoch_to_calendar(rec_st)
    assert (y, m, d, hr, mn, sc) == (2026, 10, 25, 0, 0, 0)

    # 4. Zero mask: No days active
    ok, _, _ = find_day_interval(0x00, now, horizon)
    assert not ok


# ==============================================================================
# 5. TIME-OF-DAY RANGES (10 RANGES, SORTING, OVERLAPPING)
# ==============================================================================

def test_time_of_day_sorting_and_merging():
    """Verify up to 10 TOD ranges: sorting ascending, merging overlapping, and padding."""
    raw_ranges = [
        (18 * 3600, 20 * 3600),  # 18:00 - 20:00
        (8 * 3600, 12 * 3600),   # 08:00 - 12:00
        (10 * 3600, 14 * 3600),  # 10:00 - 14:00 (overlaps with 08:00-12:00 -> merges to 08:00-14:00)
        (0, 86400),              # unconstrained sentinel (ignored as non-filter)
        (22 * 3600, 23 * 3600),  # 22:00 - 23:00
    ]
    packed = sort_and_merge_tod_ranges(raw_ranges)
    assert len(packed) == MAX_TOD_RANGES
    # First merged range should be 08:00 to 14:00 (28800 to 50400)
    assert packed[0] == (8 * 3600, 14 * 3600)
    # Second should be 18:00 to 20:00
    assert packed[1] == (18 * 3600, 20 * 3600)
    # Third should be 22:00 to 23:00
    assert packed[2] == (22 * 3600, 23 * 3600)
    # Remaining slots should be filled with sentinel (0, 86400)
    for i in range(3, MAX_TOD_RANGES):
        assert packed[i] == (0, SECS_PER_DAY)


def test_time_of_day_window_selection():
    """Verify finding the earliest matching window in current or subsequent day."""
    ranges = sort_and_merge_tod_ranges([
        (6 * 3600, 9 * 3600),    # Morning: 06:00 - 09:00
        (18 * 3600, 21 * 3600),  # Evening: 18:00 - 21:00
    ])

    # Case A: Current time is 07:00 (inside morning window) -> immediate
    now = calendar_to_epoch(2026, 10, 24, 7, 0, 0)
    ok, st, en = find_tod_interval(ranges, now, now + 2 * SECS_PER_DAY)
    assert ok
    assert st == now
    assert en == calendar_to_epoch(2026, 10, 24, 9, 0, 0)

    # Case B: Current time is 12:00 (between morning and evening) -> starts at evening 18:00
    now = calendar_to_epoch(2026, 10, 24, 12, 0, 0)
    ok, st, en = find_tod_interval(ranges, now, now + 2 * SECS_PER_DAY)
    assert ok
    assert st == calendar_to_epoch(2026, 10, 24, 18, 0, 0)
    assert en == calendar_to_epoch(2026, 10, 24, 21, 0, 0)

    # Case C: Current time is 22:00 (past all windows today) -> starts tomorrow morning 06:00
    now = calendar_to_epoch(2026, 10, 24, 22, 0, 0)
    ok, st, en = find_tod_interval(ranges, now, now + 2 * SECS_PER_DAY)
    assert ok
    assert st == calendar_to_epoch(2026, 10, 25, 6, 0, 0)
    assert en == calendar_to_epoch(2026, 10, 25, 9, 0, 0)


# ==============================================================================
# 6. TIMEZONE CONVERSIONS & LOCAL TIME HANDLING
# ==============================================================================

def test_timezone_offsets_and_midnight_rollover():
    """Verify timezone offsets crossing midnight boundaries."""
    # UTC 2026-10-24 20:00:00
    now_utc = calendar_to_epoch(2026, 10, 24, 20, 0, 0)

    # Timezone: IST (+05:30 = +19800 sec). Local time is 2026-10-25 01:30:00 (Sunday!)
    tz_ist = 5 * 3600 + 30 * 60
    now_local = now_utc + tz_ist
    y, m, d, hr, mn, sc = epoch_to_calendar(now_local)
    assert (y, m, d, hr, mn, sc) == (2026, 10, 25, 1, 30, 0)
    assert epoch_to_weekday(now_local) == 6  # Sunday in local time

    # Timezone: EDT (-04:00 = -14400 sec). Local time is 2026-10-24 16:00:00 (Saturday)
    tz_edt = -4 * 3600
    now_local = now_utc + tz_edt
    y, m, d, hr, mn, sc = epoch_to_calendar(now_local)
    assert (y, m, d, hr, mn, sc) == (2026, 10, 24, 16, 0, 0)
    assert epoch_to_weekday(now_local) == 5  # Saturday in local time


# ==============================================================================
# 7. INCLUSIVE END DATES & DATE BOUNDARIES
# ==============================================================================

def test_inclusive_end_date_semantics():
    """Verify that end date YYYY/MM/DD includes the full day until 24:00:00."""
    start_d = (2026, 10, 20)
    end_d = (2026, 10, 24)  # Today is the last active date

    # Window 20:00 - 22:00
    tod_ranges = sort_and_merge_tod_ranges([(20 * 3600, 22 * 3600)])

    # Time is 2026-10-24 19:00:00 (inside inclusive end date) -> window opens at 20:00 today
    now_utc = calendar_to_epoch(2026, 10, 24, 19, 0, 0)
    ok, st_utc, dur, is_imm, wait = calculate_next_session(
        start_d, end_d, FULL_WEEK_MASK, tod_ranges, 0, 3600, 60, now_utc
    )
    assert ok
    assert not is_imm
    assert wait == 3600  # Starts at 20:00

    # Time is 2026-10-25 00:00:01 (day after end date) -> schedule expired
    now_expired = calendar_to_epoch(2026, 10, 25, 0, 0, 1)
    ok, _, _, _, _ = calculate_next_session(
        start_d, end_d, FULL_WEEK_MASK, tod_ranges, 0, 3600, 60, now_expired
    )
    assert not ok, "Schedule should be expired after end date 24:00"


# ==============================================================================
# 8. SLEEP INTERVAL SPACING & OVERDUE WINDOWS
# ==============================================================================

def test_sleep_interval_spacing_between_sessions():
    """Verify that sessions are spaced apart by at least sleep_interval."""
    # Continuous all-day schedule
    tod_ranges = sort_and_merge_tod_ranges([])

    now_utc = calendar_to_epoch(2026, 10, 24, 10, 0, 0)
    record_len = 600      # 10 min record
    sleep_interval = 300  # 5 min sleep

    # Last session ended at 10:02:00 local
    last_end = calendar_to_epoch(2026, 10, 24, 10, 2, 0)

    # Next session cannot start before 10:02:00 + 300s = 10:07:00
    ok, st_utc, dur, is_imm, wait = calculate_next_session(
        None, None, FULL_WEEK_MASK, tod_ranges, 0, record_len, sleep_interval, now_utc, last_end
    )
    assert ok
    assert not is_imm
    assert wait == (calendar_to_epoch(2026, 10, 24, 10, 7, 0) - now_utc)
    assert st_utc == calendar_to_epoch(2026, 10, 24, 10, 7, 0)


def test_overdue_schedule_catchup():
    """Verify behavior when reference time falls mid-window."""
    # Window is 08:00 - 12:00 (4 hours)
    tod_ranges = sort_and_merge_tod_ranges([(8 * 3600, 12 * 3600)])

    # Reference time is 10:30 (already 2.5 hours into window)
    now_utc = calendar_to_epoch(2026, 10, 24, 10, 30, 0)
    record_len = 3600  # 1 hour record

    ok, st_utc, dur, is_imm, wait = calculate_next_session(
        None, None, FULL_WEEK_MASK, tod_ranges, 0, record_len, 60, now_utc
    )
    assert ok
    assert is_imm
    assert wait == 0
    assert dur == record_len  # Still 1 hour since 1.5h remaining before 12:00


# ==============================================================================
# 9. HARDWARE FAILURE REPRODUCTION & AUTONOMOUS ORCHESTRATION TESTS
# ==============================================================================

def test_unrestricted_schedule_immediate_session_with_zero_sleep():
    """Reproduce the physical hardware test scenario:
    User configured:
      - RECT 600 (duration = 600s)
      - SLEEP 0 (no inter-session pause)
      - Scheduling date range: empty (unconstrained)
      - Time-of-day windows: empty (all day)
      - Valid RTC time

    Under WildSonic reference semantics, this must produce an IMMEDIATE session
    with wait_seconds = 0 and duration = 600 seconds.
    """
    now_utc = calendar_to_epoch(2026, 10, 24, 12, 0, 0)
    record_len = 600
    sleep_interval = 0
    tod_ranges = sort_and_merge_tod_ranges([])

    ok, st_utc, dur, is_imm, wait = calculate_next_session(
        None,
        None,
        FULL_WEEK_MASK,
        tod_ranges,
        0,
        record_len,
        sleep_interval,
        now_utc,
        0
    )

    assert ok is True, "Unrestricted schedule must be valid when RTC is valid"
    assert is_imm is True, "Zero-sleep unrestricted schedule must be immediate"
    assert wait == 0, f"Wait seconds must be 0, got {wait}"
    assert dur == 600, f"Session duration must be exactly 600 seconds, got {dur}"
    assert st_utc == now_utc, "Start time UTC must match current time"


def test_autonomous_scheduler_orchestration_contract():
    """Verify that main.c implements the autonomous scheduler runner
    and provides comprehensive diagnosability logging and idle servicing.
    """
    main_c = SRC_DIR / "main.c"
    main_h = SRC_DIR / "main.h"
    console_c = SRC_DIR / "console.c"

    assert main_c.is_file()
    assert main_h.is_file()
    assert console_c.is_file()

    m_text = main_c.read_text(encoding="utf-8")
    h_text = main_h.read_text(encoding="utf-8")
    c_text = console_c.read_text(encoding="utf-8")

    # 1. Accessors and lifecycle functions declared in main.h
    assert "main_scheduler_service_step" in h_text
    assert "main_scheduler_resume" in h_text
    assert "main_scheduler_pause" in h_text
    assert "main_is_scheduler_paused" in h_text

    # 2. Autonomous service step implemented in main.c
    assert "void main_scheduler_service_step(void)" in m_text
    assert "scheduler_calculate_next" in m_text
    assert "recorder_record_session" in m_text

    # 3. Diagnosability / TEST lifecycle tags present
    assert "dispatch_recording_session" in m_text
    assert "Entering sleep %u s" in m_text
    assert "Session %u failed (%d); skipped" in m_text
    assert "[TEST] SLEEP_INTERVAL_STARTED:" in m_text
    assert "[TEST] SCHEDULER_RESUMED" in m_text
    assert "[TEST] NEXT_RECORDING_STARTED:" in m_text

    # 4. Serviced while console prompt is idle (pos == 0)
    assert "if (!main_is_scheduler_paused())" in m_text
    assert "main_scheduler_service_step();" in m_text

    # 5. Console handlers unpause/resume schedule when parameters change or Enter pressed
    assert "main_scheduler_resume" in c_text


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

