#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Compare TEST vs PRODUCTION console logs after stripping TEST-only traces.

Enforces Section 7 parity on captured logs: once TRACE_TEST, [TEST], [METRIC],
per-batch lines, SD/accounting summaries, and volatile timestamps/line numbers
are removed/normalized, the PRODUCTION and TEST execution transcripts must
match line-for-line.
"""

from __future__ import annotations

import argparse
import difflib
import re
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

TRACE_PREFIX_RE = re.compile(r"^C:\d+\s+\[F:\(([^)]+)\),\s*L:(\d+)\]\s*(.*)$")

TEST_ONLY_PREFIXES = (
    "[TEST]",
    "[METRIC]",
    "[FAULT_INJECT]",
    "[STORAGE GAP]",
    "[BACK-PRESSURE]",
    "[STAGE_DIAG]",
    "STAGE_DIAG:",
    "SDMMC_HW:",
    "SDMMC_BUS:",
    "SDMMC CMD",
    "SDMMC ",
    "Batch ",
    "Batch table ",
    "SD summary:",
    "SD worst fs_write:",
    "Accounting:",
    "ADC/DMA started:",
    "Scheduled start vs first sample:",
    "Storage recovery",
    "Storage pipeline mode set",
    "Wrote ",
    "SD hardware reset",
    "SD reset step",
    "lat=",
    "Storage health:",
    "Stop 2 aborted",
    "[SCHEDULE COMPLETE]",
    "Schedule started:",
    "Schedule target cycles",
    "Scheduled recording finished:",
    "Scheduler resuming",
    "Scheduler paused",
    "RTC calendar uninitialized; set DATETIME",
    "Slow SD write:",
    "Assert test requested",
)


def strip_trace_prefix(line: str) -> Tuple[Optional[str], Optional[str], str]:
    """Return (basename, line_num, message) if line matches TRACE format."""
    m = TRACE_PREFIX_RE.match(line)
    if not m:
        return None, None, line
    return m.group(1), m.group(2), m.group(3)


def is_test_only_line(line: str) -> bool:
    """Return True if line is emitted solely by TEST/DEV instrumentation."""
    s = line.strip()
    if not s:
        return False
    _, _, msg = strip_trace_prefix(s)
    msg_stripped = msg.lstrip()
    for prefix in TEST_ONLY_PREFIXES:
        if s.startswith(prefix) or msg_stripped.startswith(prefix):
            return True
    return False


def normalize_line(line: str, exact_values: bool = False) -> str:
    """Normalize line numbers and run-specific timestamps/filenames/mV."""
    s = line.rstrip("\r\n")
    s = re.sub(r"(\[F:\([^)]+\),\s*L:)\d+(\])", r"\1<L>\2", s)
    if exact_values:
        return s

    s = re.sub(r"^(Bootloader Build Date:\s*).*$", r"\1<DATE>", s)
    s = re.sub(r"^(Build Date:\s*).*$", r"\1<DATE>", s)
    s = re.sub(
        r"\b\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:\s*[+-]\d{2}:\d{2})?(?:\s*UTC)?",
        "<TIMESTAMP>",
        s,
    )
    s = re.sub(r"(Battery_voltage:\s*)\d+(\s*mV)", r"\1<MV>\2", s)
    s = re.sub(
        r"(SD Card Init Done \()\d+(\s*MB,\s*)\d+(\s*MB free\))",
        r"\1<TOTAL_MB>\2<FREE_MB>\3",
        s,
    )
    s = re.sub(
        r"(Last file detected in root folder:\s*)(?!NONE\b)\S+",
        r"\1<WAV>",
        s,
    )
    s = re.sub(
        r"(Recording start:[^\n]*,\s*)[^\s,]+\.wav\b",
        r"\1<WAV>",
        s,
    )
    return s


def filter_and_normalize(
    lines: Iterable[str],
    exact_values: bool = False,
) -> List[str]:
    """Filter blank/TEST-only lines and normalize remaining lines."""
    out: List[str] = []
    for raw in lines:
        s = raw.rstrip("\r\n")
        if not s.strip():
            continue
        if is_test_only_line(s):
            continue
        out.append(normalize_line(s, exact_values=exact_values))
    return out


def diff_logs(
    prod_lines: Iterable[str],
    test_lines: Iterable[str],
    exact_values: bool = False,
) -> List[str]:
    """Return unified diff lines between PRODUCTION and filtered TEST logs."""
    norm_prod = filter_and_normalize(prod_lines, exact_values=exact_values)
    norm_test = filter_and_normalize(test_lines, exact_values=exact_values)
    return list(
        difflib.unified_diff(
            norm_prod,
            norm_test,
            fromfile="prod.log (normalized)",
            tofile="test.log (stripped+normalized)",
            lineterm="",
        )
    )


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prod_log", type=Path, help="Path to captured PRODUCTION console log")
    parser.add_argument("test_log", type=Path, help="Path to captured TEST console log")
    parser.add_argument(
        "--exact-values",
        action="store_true",
        help="Do not normalize mV, free MB, timestamps, or filenames (only strip L:<n> and TEST lines)",
    )
    args = parser.parse_args(argv)

    prod_lines = args.prod_log.read_text(encoding="utf-8", errors="replace").splitlines()
    test_lines = args.test_log.read_text(encoding="utf-8", errors="replace").splitlines()

    diff = diff_logs(prod_lines, test_lines, exact_values=args.exact_values)
    if diff:
        print("\n".join(diff), file=sys.stderr)
        return 1

    norm_prod = filter_and_normalize(prod_lines, exact_values=args.exact_values)
    print(f"OK: PRODUCTION and TEST logs match across {len(norm_prod)} normalized lines.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
