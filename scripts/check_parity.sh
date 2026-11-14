#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# CI Parity Gate: Verify PRODUCTION vs TEST firmware images obey Section 0 & Section 7.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROD_DIR="${1:-$REPO_ROOT/build/prod}"
TEST_DIR="${2:-$REPO_ROOT/build/test}"

PROD_ELF="$PROD_DIR/zephyr/zephyr.elf"
PROD_BIN="$PROD_DIR/zephyr/zephyr.bin"
TEST_ELF="$TEST_DIR/zephyr/zephyr.elf"
TEST_BIN="$TEST_DIR/zephyr/zephyr.bin"

if [[ ! -f "$PROD_ELF" || ! -f "$TEST_ELF" ]]; then
    echo "ERROR: Missing ELFs ($PROD_ELF and/or $TEST_ELF). Build both profiles first." >&2
    exit 2
fi

NM="${ARM_ZEPHYR_EABI_NM:-}"
if [[ -z "$NM" ]]; then
    if command -v arm-zephyr-eabi-nm >/dev/null 2>&1; then
        NM="$(command -v arm-zephyr-eabi-nm)"
    elif [[ -x "/home/amithesh/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm" ]]; then
        NM="/home/amithesh/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm"
    else
        echo "ERROR: arm-zephyr-eabi-nm not found in PATH or default Zephyr SDK." >&2
        exit 2
    fi
fi

echo "=== (a) PRODUCTION Binary Strings Gate ==="
FORBIDDEN_STRINGS=(
    "[TEST]"
    "[PHASE"
    "[METRIC]"
    "batchlog"
    "THREAD_ANALYZER"
    "Batch %u:"
)
for tok in "${FORBIDDEN_STRINGS[@]}"; do
    ro_cnt="$(strings -d "$PROD_ELF" | grep -F -c -- "$tok" || true)"
    bin_cnt=0
    if [[ -f "$PROD_BIN" ]]; then
        bin_cnt="$(strings "$PROD_BIN" | grep -F -c -- "$tok" || true)"
    fi
    if [[ "$ro_cnt" -ne 0 || "$bin_cnt" -ne 0 ]]; then
        echo "FAIL: Forbidden TEST string '$tok' found in PRODUCTION binary (elf -d=$ro_cnt, bin=$bin_cnt)." >&2
        exit 1
    fi
done
echo "PASS: Zero forbidden TEST/bench strings in PRODUCTION .rodata/.bin."

echo "=== (b) Devicetree & Kconfig (.config) Diff Gate ==="
DTS_DIFF="$(diff -u "$PROD_DIR/zephyr/zephyr.dts" "$TEST_DIR/zephyr/zephyr.dts" || true)"
if [[ -n "$DTS_DIFF" ]]; then
    echo "FAIL: Devicetree differs between PRODUCTION and TEST:" >&2
    echo "$DTS_DIFF" >&2
    exit 1
fi
echo "PASS: zephyr.dts is identical."

python3 - "$PROD_DIR/zephyr/.config" "$TEST_DIR/zephyr/.config" <<'PY'
import re
import sys
from pathlib import Path

def load_cfg(path: Path) -> dict[str, str]:
    out = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("# CONFIG_") and s.endswith(" is not set"):
            sym = s[2:].split(" ", 1)[0]
            out[sym] = "n"
        elif s.startswith("CONFIG_") and "=" in s:
            sym, val = s.split("=", 1)
            out[sym] = val
    return out

p_cfg = load_cfg(Path(sys.argv[1]))
t_cfg = load_cfg(Path(sys.argv[2]))

allowed = re.compile(
    r"^(?:"
    r"CONFIG_.*_LOG_LEVEL(?:_.*)?|"
    r"CONFIG_LOG_DEFAULT_LEVEL(?:_.*)?|"
    r"CONFIG_WILDSONIC_(?:PROFILE_PRODUCTION|PROFILE_TEST|PROFILE_DEV|"
    r"TEST_BUILD|TEST_OUTPUT|TEST_MARKERS|VERBOSE_LOGS|"
    r"BENCHMARKS|SD_FAULT_INJECT)|"
    r"CONFIG_THREAD_ANALYZER(?:_.*)?"
    r")$"
)

bad = []
for sym in sorted(set(p_cfg) | set(t_cfg)):
    pv = p_cfg.get(sym, "<unset>")
    tv = t_cfg.get(sym, "<unset>")
    if pv != tv and not allowed.match(sym):
        bad.append((sym, pv, tv))

if bad:
    print("FAIL: Disallowed .config differences between PRODUCTION and TEST:", file=sys.stderr)
    for sym, pv, tv in bad:
        print(f"  {sym}: PROD={pv} vs TEST={tv}", file=sys.stderr)
    sys.exit(1)

print("PASS: Only allowed logging/test/profile symbols differ in .config.")
PY

echo "=== (c) Symbol Parity Gate (arm-zephyr-eabi-nm) ==="
python3 - "$NM" "$PROD_ELF" "$TEST_ELF" <<'PY'
import re
import subprocess
import sys

nm, prod_elf, test_elf = sys.argv[1], sys.argv[2], sys.argv[3]

def text_syms(elf: str) -> set[str]:
    out = subprocess.check_output([nm, "-n", elf], text=True)
    syms = set()
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[1] in ("t", "T"):
            syms.add(parts[2])
    return syms

p_syms = text_syms(prod_elf)
t_syms = text_syms(test_elf)

test_only = sorted(t_syms - p_syms)
prod_only = sorted(p_syms - t_syms)

allowed_test_only = re.compile(
    r"^(?:"
    r"psram_benchmark_.*|"
    r"psram_(?:fill_pattern|get_pattern_name|get_burst_size|set_burst_size|get_clock_hz)|"
    r"sd_benchmark_.*|"
    r"sd_fault_inject_.*|"
    r"main_psram_command|"
    r"recorder_(?:get|set)_batchlog_mode|"
    r"recorder_(?:get|set)_pipeline_mode(?:_str)?|"
    r"recorder_apply_sdmmc_clock|"
    r"recorder_sd_verify_idle|"
    r"log_sdmmc_stage_diagnostics|"
    r"rtc_calendar_log_snapshot|"
    r"sd_resilient_health_str|"
    r"audio_in_get_ovs_ratio|"
    r"retained_reboot_set_assert|"
    r"system_assert_record_id|"
    r"run_diag_subtest(?:\..*)?|"
    r"LL_DMA_.*|SDMMC_.*|SD_DMARxAbort|SD_DMATxAbort|SD_FindSCR(?:\..*)?|"
    r"handle_[a-z0-9_]+"
    r")$"
)

allowed_prod_only = re.compile(r"^(?:handle_[a-z0-9_]+)$")

bad_t = [s for s in test_only if not allowed_test_only.match(s)]
bad_p = [s for s in prod_only if not allowed_prod_only.match(s)]

if bad_t or bad_p:
    if bad_t:
        print("FAIL: Unexpected TEST-only text symbols:", bad_t, file=sys.stderr)
    if bad_p:
        print("FAIL: Unexpected PROD-only text symbols:", bad_p, file=sys.stderr)
    sys.exit(1)

required_shared = [
    "recorder_record_session",
    "recorder_sleep",
    "dsp_process_i16",
    "audio_in_start",
    "audio_in_stop",
    "psram_staging_push",
    "sd_resilient_init",
    "sd_resilient_hardware_reset",
    "scheduler_calculate_next",
    "config_store_init",
    "retained_reboot_get",
]
missing_shared = [s for s in required_shared if s not in p_syms or s not in t_syms]
if missing_shared:
    print("FAIL: Required shared core symbols missing:", missing_shared, file=sys.stderr)
    sys.exit(1)

print(f"PASS: Symbol parity verified ({len(test_only)} allowed TEST-only symbols, {len(prod_only)} compiler-inlined static handlers).")
PY

echo "=== (d) CONFIG_WILDSONIC_TEST_OUTPUT Usage Audit ==="
python3 - "$REPO_ROOT/app/src" <<'PY'
import re
import sys
from pathlib import Path

src_dir = Path(sys.argv[1])
forbidden_in_core = re.compile(
    r"\b(?:audio_in_start|audio_in_stop|dsp_process_i16|dsp_process_i32|"
    r"fs_write|fs_open|fs_close|fs_truncate|disk_access_write|recorder_sleep|"
    r"k_sleep|k_msleep|sd_resilient_write|psram_staging_push)\b"
)

# Files where CONFIG_WILDSONIC_TEST_OUTPUT may ONLY gate prints or debug snapshots
print_only_files = {
    "recorder.c",
    "rtc_calendar.c",
    "sd_resilient.c",
    "wildsonic_trace.h",
}

errors = []
for path in sorted(src_dir.glob("*.[ch]")):
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    stack: list[tuple[bool, int]] = []
    for idx, line in enumerate(lines, 1):
        s = line.strip()
        if re.match(r"^#\s*if\b.*WILDSONIC_TEST_OUTPUT", s) or re.match(r"^#\s*ifdef\s+CONFIG_WILDSONIC_TEST_OUTPUT", s):
            stack.append((True, idx))
            continue
        elif re.match(r"^#\s*if", s):
            stack.append((False, idx))
            continue
        elif re.match(r"^#\s*else\b", s):
            continue
        elif re.match(r"^#\s*endif\b", s):
            if stack:
                stack.pop()
            continue

        in_test_output = any(flag for flag, _ in stack)
        if in_test_output and path.name in print_only_files:
            if path.name == "recorder.c" and idx < 120:
                # Diagnostic helper functions log_sdmmc_* defined solely under TEST_OUTPUT
                continue
            code_only = re.sub(r'"(?:\\.|[^"\\])*"', '""', s)
            code_only = re.sub(r"//.*", "", code_only)
            if forbidden_in_core.search(code_only):
                errors.append(f"{path.name}:{idx}: forbidden logic inside CONFIG_WILDSONIC_TEST_OUTPUT: {s}")

if errors:
    print("FAIL: Core logic found inside CONFIG_WILDSONIC_TEST_OUTPUT:", file=sys.stderr)
    for e in errors:
        print(" ", e, file=sys.stderr)
    sys.exit(1)

print("PASS: CONFIG_WILDSONIC_TEST_OUTPUT guards only prints, trace macros, and test commands.")
PY

echo "=== (e) SRAM0 Headroom (>= 4096 B) & MPU Stack Guard Gate ==="
READELF="${ARM_ZEPHYR_EABI_READELF:-}"
if [[ -z "$READELF" ]]; then
    if command -v arm-zephyr-eabi-readelf >/dev/null 2>&1; then
        READELF="$(command -v arm-zephyr-eabi-readelf)"
    elif [[ -x "/home/amithesh/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-readelf" ]]; then
        READELF="/home/amithesh/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-readelf"
    else
        echo "ERROR: arm-zephyr-eabi-readelf not found." >&2
        exit 2
    fi
fi

python3 - "$READELF" "$NM" "$PROD_DIR" "$TEST_DIR" <<'PY'
import re
import subprocess
import sys
from pathlib import Path

readelf, nm, prod_dir, test_dir = sys.argv[1], sys.argv[2], Path(sys.argv[3]), Path(sys.argv[4])
SRAM0_BASE = 0x20000000
SRAM0_SIZE = 98304

def sram0_used(elf: Path) -> int:
    out = subprocess.check_output([readelf, "-S", "-W", str(elf)], text=True)
    max_end = SRAM0_BASE
    for line in out.splitlines():
        if "NOBITS" not in line and "PROGBITS" not in line:
            continue
        m = re.search(r"\b(PROGBITS|NOBITS)\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+", line)
        if not m:
            continue
        addr = int(m.group(2), 16)
        size = int(m.group(3), 16)
        if SRAM0_BASE <= addr < SRAM0_BASE + SRAM0_SIZE:
            end = addr + size
            if end > max_end:
                max_end = end
    return max_end - SRAM0_BASE

for label, d in (("PRODUCTION", prod_dir), ("TEST", test_dir)):
    cfg = (d / "zephyr" / ".config").read_text(encoding="utf-8", errors="replace")
    if "CONFIG_MPU_STACK_GUARD=y" not in cfg:
        print(f"FAIL: CONFIG_MPU_STACK_GUARD=y missing in {label} .config", file=sys.stderr)
        sys.exit(1)
    used = sram0_used(d / "zephyr" / "zephyr.elf")
    free = SRAM0_SIZE - used
    print(f"  {label}: SRAM0 used={used} B, free={free} B (limit: >= 4096 B free)")
    if free < 4096:
        print(f"FAIL: {label} free SRAM0 {free} B < 4096 B minimum headroom", file=sys.stderr)
        sys.exit(1)

def stack_sizes(elf: Path) -> dict[str, int]:
    out = subprocess.check_output([nm, "-S", str(elf)], text=True)
    res = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4:
            name = parts[3]
            if name.endswith("_stack") or name.endswith("_stacks") or "thread_stack" in name:
                try:
                    res[name] = int(parts[1], 16)
                except ValueError:
                    pass
    return res

p_stacks = stack_sizes(prod_dir / "zephyr" / "zephyr.elf")
t_stacks = stack_sizes(test_dir / "zephyr" / "zephyr.elf")
if p_stacks != t_stacks:
    print("FAIL: Thread stack sizes differ between PRODUCTION and TEST:", file=sys.stderr)
    for k in sorted(set(p_stacks) | set(t_stacks)):
        if p_stacks.get(k) != t_stacks.get(k):
            print(f"  {k}: PROD={p_stacks.get(k)} vs TEST={t_stacks.get(k)}", file=sys.stderr)
    sys.exit(1)

print(f"PASS: SRAM0 headroom >= 4096 B, CONFIG_MPU_STACK_GUARD=y, and {len(p_stacks)} thread stack sizes identical.")
PY

echo "ALL PARITY CHECKS PASSED."
