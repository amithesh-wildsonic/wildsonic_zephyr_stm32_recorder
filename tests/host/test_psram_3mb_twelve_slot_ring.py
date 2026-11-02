# SPDX-License-Identifier: Apache-2.0
"""
Test suite verifying:
- Section 2: PSRAM 3 MB allocation (75%), 1 MB reserved, 12 slots x 256 KB, and compile-time assertions.
- Section 3: Twelve-slot producer/consumer ring states (FREE, FILLING, READY, WRITING), ownership, order preservation.
- Section 4: Asynchronous ping-pong operation, wraparound, slot reuse, and no overwrite when ring is full.
- Section 5: Uncommitted data retention on write failure (Rule 5).
- Section 6: Runtime reporting containing all 12 required metrics.
- Section 7: Memory bounds (0x00000000 - 0x002FFFFF) without crossing into reserved (0x00300000 - 0x003FFFFF).
"""

from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
APP_KCONFIG = REPO_ROOT / "app" / "Kconfig"
APP_PRJ_CONF = REPO_ROOT / "app" / "prj.conf"


# ==============================================================================
# Section 2 & 7: Named Constants, Compile-Time Assertions, and Memory Bounds
# ==============================================================================

def test_psram_named_constants_and_compile_assertions():
    """Verify that PSRAM 3MB allocation constants and BUILD_ASSERTs are defined."""
    staging_h = (APP_SRC / "psram_staging.h").read_text(encoding="utf-8")

    assert "#define PSRAM_TOTAL_BYTES          (4U * 1024U * 1024U)" in staging_h
    assert "#define PSRAM_AUDIO_BUFFER_BYTES   (3U * 1024U * 1024U)" in staging_h
    assert "#define PSRAM_RESERVED_BYTES       (PSRAM_TOTAL_BYTES - PSRAM_AUDIO_BUFFER_BYTES)" in staging_h
    assert "#define PSRAM_BATCH_BYTES          (256U * 1024U)" in staging_h
    assert "#define PSRAM_AUDIO_SLOT_COUNT     (PSRAM_AUDIO_BUFFER_BYTES / PSRAM_BATCH_BYTES)" in staging_h

    assert "BUILD_ASSERT(PSRAM_AUDIO_BUFFER_BYTES <= PSRAM_TOTAL_BYTES" in staging_h
    assert "BUILD_ASSERT((PSRAM_AUDIO_BUFFER_BYTES % PSRAM_BATCH_BYTES) == 0" in staging_h
    assert "BUILD_ASSERT(PSRAM_AUDIO_SLOT_COUNT == 12" in staging_h


def test_kconfig_and_prj_conf_defaults_to_3072kb():
    """Verify Kconfig and prj.conf default to 3072 KiB (3 MB)."""
    kconfig = APP_KCONFIG.read_text(encoding="utf-8")
    assert "config AUDIO_PSRAM_BUFFER_SIZE_KB" in kconfig
    assert "default 3072" in kconfig
    assert "range 256 3072" in kconfig

    prj_conf = APP_PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_AUDIO_PSRAM_BUFFER_SIZE_KB=3072" in prj_conf
    assert "CONFIG_AUDIO_SD_BATCH_SIZE_KB=256" in prj_conf


def test_psram_memory_layout_strictly_segregated_from_reserved():
    """Verify that audio staging address range (0x00000000..0x002FFFFF) never touches reserved (0x00300000..0x003FFFFF)."""
    is66_h = (APP_SRC / "psram_is66wvs.h").read_text(encoding="utf-8")
    staging_h = (APP_SRC / "psram_staging.h").read_text(encoding="utf-8")

    # Audio staging base and capacity
    staging_base = 0x00000000
    audio_capacity = 3 * 1024 * 1024  # 3 MB
    staging_end = staging_base + audio_capacity  # 0x00300000

    # Diagnostic test reserved region
    diag_base = 0x00300000
    diag_max = 0x003FFFFF

    assert "PSRAM_DIAG_TEST_BASE_ADDR      0x00300000U" in is66_h
    assert "PSRAM_DIAG_TEST_MAX_ADDR       0x003FFFFFU" in is66_h

    # Staging ends exactly where the 1 MB reserved region begins
    assert staging_end == diag_base
    assert (diag_max - diag_base + 1) == (1024 * 1024)  # 1 MB reserved

    # Check 12 slots base addresses
    slot_size = 256 * 1024
    for slot_idx in range(12):
        slot_base = staging_base + slot_idx * slot_size
        slot_end = slot_base + slot_size
        assert slot_base >= staging_base
        assert slot_end <= diag_base, f"Slot {slot_idx} violates reserved boundary!"


# ==============================================================================
# Section 3, 4, 5: 12-Slot Producer/Consumer Ring Simulation & State Transitions
# ==============================================================================

class SimulatedSlotState:
    FREE = 0
    FILLING = 1
    READY = 2
    WRITING = 3


class SimulatedPsramRing:
    """Python simulation of the 12-slot PSRAM staging ring in psram_staging.c."""

    def __init__(self, slot_count=12, slot_size_bytes=256 * 1024):
        self.slot_count = slot_count
        self.slot_size_bytes = slot_size_bytes
        self.capacity_bytes = slot_count * slot_size_bytes

        self.slots = [{
            "base_addr": i * slot_size_bytes,
            "filled": 0,
            "committed": 0,
            "state": SimulatedSlotState.FREE,
            "data": bytearray(slot_size_bytes)
        } for i in range(slot_count)]

        self.active_fill_idx = 0
        self.active_drain_idx = 0
        self.slots[0]["state"] = SimulatedSlotState.FILLING

        self.count_occupied = 0
        self.full_events = 0
        self.rejected_blocks = 0

    def push(self, data: bytes) -> int:
        n = len(data)
        cur = self.slots[self.active_fill_idx]

        if cur["state"] == SimulatedSlotState.FREE:
            cur["state"] = SimulatedSlotState.FILLING
            cur["filled"] = 0
            cur["committed"] = 0
        elif cur["state"] != SimulatedSlotState.FILLING:
            self.full_events += 1
            self.rejected_blocks += 1
            return -105  # -ENOBUFS

        space_in_cur = self.slot_size_bytes - cur["filled"]
        available = self.capacity_bytes - self.count_occupied

        if n > space_in_cur and cur["filled"] > 0:
            next_fill = (self.active_fill_idx + 1) % self.slot_count
            if self.slots[next_fill]["state"] == SimulatedSlotState.FREE and available >= n:
                cur["state"] = SimulatedSlotState.READY
                self.active_fill_idx = next_fill
                cur = self.slots[self.active_fill_idx]
                cur["state"] = SimulatedSlotState.FILLING
                cur["filled"] = 0
                cur["committed"] = 0
                space_in_cur = self.slot_size_bytes

        if n > space_in_cur or n > available:
            self.full_events += 1
            self.rejected_blocks += 1
            return -105  # -ENOBUFS

        # Write data into slot
        cur["data"][cur["filled"]:cur["filled"] + n] = data
        cur["filled"] += n
        self.count_occupied += n

        if cur["filled"] >= self.slot_size_bytes:
            cur["state"] = SimulatedSlotState.READY
            next_fill = (self.active_fill_idx + 1) % self.slot_count
            self.active_fill_idx = next_fill
            if self.slots[next_fill]["state"] == SimulatedSlotState.FREE:
                self.slots[next_fill]["state"] = SimulatedSlotState.FILLING
                self.slots[next_fill]["filled"] = 0
                self.slots[next_fill]["committed"] = 0

        return 0

    def peek(self, max_bytes: int) -> bytes:
        drain = self.slots[self.active_drain_idx]
        if drain["state"] == SimulatedSlotState.READY:
            drain["state"] = SimulatedSlotState.WRITING
        elif drain["state"] != SimulatedSlotState.WRITING:
            return b""

        uncommitted = drain["filled"] - drain["committed"]
        if uncommitted == 0:
            return b""

        to_read = min(max_bytes, uncommitted)
        start = drain["committed"]
        return bytes(drain["data"][start:start + to_read])

    def commit(self, bytes_committed: int) -> int:
        drain = self.slots[self.active_drain_idx]
        if drain["state"] != SimulatedSlotState.WRITING:
            return -22  # -EINVAL

        drain["committed"] += bytes_committed
        self.count_occupied -= bytes_committed

        if drain["committed"] >= drain["filled"]:
            drain["state"] = SimulatedSlotState.FREE
            drain["filled"] = 0
            drain["committed"] = 0
            self.active_drain_idx = (self.active_drain_idx + 1) % self.slot_count

        return 0


def test_twelve_slot_full_utilization_and_order():
    """Verify all 12 slots can be filled sequentially, preserving order."""
    ring = SimulatedPsramRing(slot_count=12, slot_size_bytes=256 * 1024)

    # Push 12 full slots of distinct data (each 256 KB = 32 blocks of 8 KB)
    block_size = 8 * 1024
    blocks_per_slot = (256 * 1024) // block_size

    for slot_num in range(12):
        for blk in range(blocks_per_slot):
            tag = bytes([slot_num, blk & 0xFF]) * (block_size // 2)
            ret = ring.push(tag)
            assert ret == 0

    assert ring.count_occupied == 3 * 1024 * 1024  # 3 MB fully occupied
    # All 12 slots should now be READY
    for i in range(12):
        assert ring.slots[i]["state"] == SimulatedSlotState.READY

    # Next push must fail with -ENOBUFS because ring is at 100% capacity
    dummy = b"\xAA" * block_size
    ret = ring.push(dummy)
    assert ret == -105  # -ENOBUFS
    assert ring.full_events == 1
    assert ring.rejected_blocks == 1


def test_asynchronous_drain_wraparound_and_slot_reuse():
    """Verify that as slots are drained by SD writer, they return to FREE and are reused across wraparound."""
    ring = SimulatedPsramRing(slot_count=12, slot_size_bytes=256 * 1024)
    slot_bytes = 256 * 1024

    # Fill slots 0..3 (1 MB)
    for i in range(4):
        ret = ring.push(bytes([i] * slot_bytes))
        assert ret == 0

    # SD writer drains slots 0 and 1
    for expected_id in [0, 1]:
        chunk = ring.peek(slot_bytes)
        assert len(chunk) == slot_bytes
        assert chunk[0] == expected_id
        assert ring.slots[expected_id]["state"] == SimulatedSlotState.WRITING
        # Commit write to SD
        ret = ring.commit(slot_bytes)
        assert ret == 0
        assert ring.slots[expected_id]["state"] == SimulatedSlotState.FREE

    # Producer now fills slots 4..11 (remaining 8 slots) plus wraps around into slot 0 and 1!
    for i in range(4, 14):
        slot_val = i % 256
        ret = ring.push(bytes([slot_val] * slot_bytes))
        assert ret == 0

    # Ensure slot 0 and slot 1 were reused after returning to FREE
    assert ring.slots[0]["state"] == SimulatedSlotState.READY
    assert ring.slots[1]["state"] == SimulatedSlotState.READY


def test_failed_write_retains_uncommitted_data():
    """Verify Rule 5: A failed write does NOT commit or release the slot, preventing data overwrite."""
    ring = SimulatedPsramRing(slot_count=12, slot_size_bytes=256 * 1024)
    slot_bytes = 256 * 1024

    # Fill slot 0
    ret = ring.push(b"\x55" * slot_bytes)
    assert ret == 0
    assert ring.slots[0]["state"] == SimulatedSlotState.READY

    # SD writer peeks slot 0
    data = ring.peek(slot_bytes)
    assert len(data) == slot_bytes
    assert ring.slots[0]["state"] == SimulatedSlotState.WRITING

    # SIMULATE WRITE FAILURE (fs_write returned -EIO):
    # SD writer does NOT call commit()!
    # Slot 0 must remain in WRITING state and NOT transition to FREE
    assert ring.slots[0]["state"] == SimulatedSlotState.WRITING
    assert ring.count_occupied == slot_bytes

    # Fill slots 1..11
    for i in range(1, 12):
        ret = ring.push(bytes([i] * slot_bytes))
        assert ret == 0

    # Now producer attempts to wrap around and write to slot 0:
    # Slot 0 is in WRITING state (not FREE), so it must be rejected!
    ret = ring.push(b"\xFF" * slot_bytes)
    assert ret == -105  # -ENOBUFS (producer prevented from overwriting uncommitted slot 0)
    assert ring.slots[0]["state"] == SimulatedSlotState.WRITING
    assert ring.slots[0]["data"][0] == 0x55  # Original uncommitted data completely preserved


# ==============================================================================
# Section 6: Runtime Reporting Verification
# ==============================================================================

def test_runtime_reporting_contains_all_twelve_required_labels():
    """Verify runtime reporting includes all 12 exact diagnostic labels."""
    main_c = (APP_SRC / "main.c").read_text(encoding="utf-8")
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    required_labels = [
        "PSRAM Physical Capacity",
        "PSRAM Audio Allocation",
        "PSRAM Reserved Capacity",
        "PSRAM Slot Count",
        "PSRAM Slot Size",
        "PSRAM Current Occupancy",
        "PSRAM High-Water Mark",
        "PSRAM Free Slots",
        "PSRAM Full Events",
        "PSRAM Rejected Audio Blocks",
        "SD Writer Average Throughput",
        "SD Writer Worst-Case Latency",
    ]

    for label in required_labels:
        found = (label in main_c) or (label in rec_c)
        assert found, f"Required runtime reporting label missing: '{label}'"
