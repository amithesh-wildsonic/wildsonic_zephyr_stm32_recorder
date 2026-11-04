from __future__ import annotations

import re
from typing import Any, Dict


def _maybe_float(raw: str):
    value = raw.strip().rstrip("%")
    try:
        return float(value)
    except ValueError:
        return None


def parse_sdmmc_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}

    # Check for storage recovery markers
    if re.search(r"Storage recovery:\s*SDMMC reinitialized and filesystem remounted", log_text, re.I):
        metrics["storage_recovery"] = "PASS"
    elif re.search(r"Storage recovery:\s*(?:result\s*)?[:=]?\s*(pass|ok|success)", log_text, re.I):
        metrics["storage_recovery"] = "PASS"
    elif re.search(r"Storage recovery:\s*(?:failed|fail|error)", log_text, re.I):
        metrics["storage_recovery"] = "FAIL"

    if re.search(r"\[TEST\]\s+SDMMC_IDLE", log_text, re.I):
        metrics["sdmmc_idle"] = "PASS"

    patterns = {
        # Active throughput
        "active_write_throughput": re.compile(
            r"(?:active\s+write\s+throughput|SD\s+Writer\s+Average\s+Throughput)\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb/s|mib/s|kb/s|kib/s|b/s|bytes/s)",
            re.I,
        ),
        # End-to-end throughput
        "end_to_end_throughput": re.compile(
            r"end-to-end\s+throughput\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb/s|mib/s|kb/s|kib/s|b/s|bytes/s)",
            re.I,
        ),
        # Aggregate writer throughput
        "aggregate_writer_throughput": re.compile(
            r"(?:aggregate\s+writer\s+throughput|SD\s+Writer\s+Aggregate\s+Throughput)\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb/s|mib/s|kb/s|kib/s|b/s|bytes/s)",
            re.I,
        ),
        # Batch latencies (ms or us)
        "avg_batch_latency": re.compile(
            r"(?:average\s+batch\s+latency|avg latency)\s*[:=]?\s*([0-9]*\.?[0-9]+)\s*(?:ms|us|s)",
            re.I,
        ),
        "worst_batch_latency": re.compile(
            r"(?:worst\s+batch\s+latency|SD\s+Writer\s+Worst-Case\s+Latency|worst batch)\s*[:=]?\s*([0-9]*\.?[0-9]+)\s*(?:ms|us|s)",
            re.I,
        ),
        # Errors
        "sdmmc_errors": re.compile(r"(?:sdmmc\s+errors|SD\s+error)\s*[:=]\s*(-?\d+)", re.I),
        "filesystem_errors": re.compile(r"filesystem\s+errors\s*[:=]\s*(\d+)", re.I),
        "buffer_swaps": re.compile(r"(\d+)\s+buffer\s+swaps", re.I),
        "pingpong_overlap_ms": re.compile(r"SD\s+Writer\s+Ping-Pong\s+Overlap\s*[:=]\s*(\d+)\s*ms", re.I),
        "card_init_time_ms": re.compile(r"card\s+init(?:ialization)?\s+time\s*[:=]\s*([0-9]*\.?[0-9]+)\s*ms", re.I),
        "mount_time_ms": re.compile(r"(?:filesystem\s+)?mount\s+time\s*[:=]\s*([0-9]*\.?[0-9]+)\s*ms", re.I),
    }

    for key, pattern in patterns.items():
        match = pattern.search(log_text)
        if not match:
            continue
        value = match.group(1)
        parsed = _maybe_float(value)
        if parsed is not None:
            metrics[key] = parsed

    # Parse SD writer time breakdown if present
    # fs_write=180 ms, psram_peek=12 ms, commit=8 ms, preemption/starve=20 ms
    breakdown_match = re.search(
        r"SD\s+Writer\s+Time\s+Breakdown\s*:\s*fs_write=(\d+)\s*ms,\s*psram_peek=(\d+)\s*ms,\s*commit=(\d+)\s*ms,\s*preemption/starve=(\d+)\s*ms",
        log_text,
        re.I,
    )
    if breakdown_match:
        metrics["sd_active_fs_ms"] = float(breakdown_match.group(1))
        metrics["sd_psram_read_ms"] = float(breakdown_match.group(2))
        metrics["sd_commit_ms"] = float(breakdown_match.group(3))
        metrics["sd_starve_ms"] = float(breakdown_match.group(4))

    # Parse SDBENCH matrix rows if present
    # e.g.: FatFs Block Write  |   16 KB    |   16 KB    |  128   |   1400 us     |   3100 us     |  2150 KB/s    | PASS
    sdbench_rows = re.findall(
        r"([A-Za-z0-9 _-]+)\s*\|\s*(\d+)\s*KB\s*\|\s*(\d+)\s*KB\s*\|\s*(\d+)\s*\|\s*(\d+)\s*us\s*\|\s*(\d+)\s*us\s*\|\s*(\d+)\s*KB/s\s*\|\s*(PASS|FAIL)",
        log_text,
    )
    if sdbench_rows:
        bench_data = []
        for row in sdbench_rows:
            bench_data.append({
                "test": row[0].strip(),
                "chunk_kb": int(row[1]),
                "total_kb": int(row[2]),
                "iterations": int(row[3]),
                "avg_us": int(row[4]),
                "worst_us": int(row[5]),
                "throughput_kb_s": int(row[6]),
                "status": row[7].upper(),
            })
        metrics["sdbench_results"] = bench_data
        # Extract best write throughput from benchmark
        throughputs = [r["throughput_kb_s"] for r in bench_data if "Write" in r["test"]]
        if throughputs:
            metrics["sdbench_max_write_kb_s"] = max(throughputs)
            metrics["sdbench_avg_write_kb_s"] = sum(throughputs) / len(throughputs)

    return metrics
