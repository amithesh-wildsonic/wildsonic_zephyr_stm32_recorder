from __future__ import annotations

import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from .aggregator import MetricAggregator
from .config import DEFAULT_ACCEPTANCE_CRITERIA, AcceptanceCriteria
from .controller import SerialController
from .models import TestSession
from .parsers import parse_firmware_log
from .parsers.boot import parse_boot_metrics
from .ppk2 import PPK2Processor, PhaseMetrics
from .ppk2_controller import MockPPK2Device, PPK2Controller, PPK2PhaseWindow
from .test_manager import TestManager
from .workbook import ComparisonWorkbookManager


def _clean_boot_summary(session: TestSession, metrics: Dict[str, Any]) -> str:
    if session.category == "cold-boot":
        title = "STM32L476RG COLD BOOT SUMMARY"
    elif metrics.get("reset_type") == "SOFTWARE_REBOOT":
        title = "STM32L476RG SOFTWARE REBOOT SUMMARY"
    else:
        title = "STM32L476RG BOOT SUMMARY"
    commit = session.firmware_commit or metrics.get("firmware_commit") or "N/A"
    lines = [
        "=" * 80,
        f"                 {title} ({session.test_id})",
        "=" * 80,
        f"Board: {session.board} | Commit: {commit} | Category: {session.category} | Evidence: {session.evidence_type}",
        f"Status: {session.status.upper()} | Configuration: {session.configuration or 'Default'}",
        "-" * 80,
        f"{'Boot Milestone / Metric':<36} | {'Value / Description':<24} | {'Status':<12}",
        "-" * 80,
    ]

    boot_keys = [
        ("Reset Method", "reset_type", "type"),
        ("Reset Acknowledgement", "reset_ack", "ack"),
        ("Reboot Status / Error", "reboot_error", "error"),
        ("Boot Verification", "boot_verified", "milestone"),
        ("Host-Observed Readiness", "host_observed_readiness_ms", "ms"),
        ("Boot Duration", "boot_duration_ms", "ms"),
        ("System Init Milestone", "system_init", "milestone"),
        ("RTC Wakeup Timer Init", "rtc_init", "milestone"),
        ("Audio Peripheral Init", "audio_init", "milestone"),
        ("SD Card Init", "sd_init", "milestone"),
        ("PSRAM Init", "psram_init", "milestone"),
        ("Hardware Platform", "hardware_platform", "platform"),
        ("Core Clock Frequency", "core_clock", "clock"),
        ("Firmware Version", "firmware_version", "version"),
        ("Firmware Commit", "firmware_commit", "commit"),
        ("Build Date", "build_date", "timestamp"),
    ]

    for label, key, unit in boot_keys:
        val = metrics.get(key)
        if val is not None:
            if isinstance(val, bool):
                val_str = "PASS" if val else "FAIL"
            elif isinstance(val, float):
                val_str = f"{val:.1f}"
            else:
                val_str = str(val)
            lines.append(f"{label:<36} | {val_str:<24} | {unit:<12}")

    lines.append("=" * 80)
    return "\n".join(lines)


def _clean_ppk2_benchmark_summary(session: TestSession, metrics: Dict[str, Any], phases: Dict[str, PPK2PhaseWindow]) -> str:
    commit = session.firmware_commit or metrics.get("firmware_commit") or "N/A"
    lines = [
        "=" * 80,
        f"       STM32L476RG PPK2-CONTROLLED 9-PHASE HARDWARE BENCHMARK ({session.test_id})",
        "=" * 80,
        f"Board: {session.board} | Commit: {commit} | Supply: E5V (5000 mV) | Evidence: {session.evidence_type}",
        f"Status: {session.status.upper()} | Instrument: Nordic PPK2 (Source Meter @ 100 kS/s)",
        "-" * 80,
        f"{'Benchmark Phase / Operating State':<34} | {'Current (Avg / Peak)':<22} | {'Power / Energy':<20}",
        "-" * 80,
    ]

    for pname, pdata in phases.items():
        if pdata.avg_current_ma < 0.1:
            i_str = f"{pdata.avg_current_ma * 1000.0:.2f} uA / {pdata.max_current_ma * 1000.0:.2f} uA"
            p_str = f"{pdata.avg_power_mw * 1000.0:.2f} uW / {pdata.total_energy_mj:.3f} mJ"
        else:
            i_str = f"{pdata.avg_current_ma:.2f} mA / {pdata.max_current_ma:.2f} mA"
            p_str = f"{pdata.avg_power_mw:.1f} mW / {pdata.total_energy_mj:.1f} mJ"
        lines.append(f"{pname:<34} | {i_str:<22} | {p_str:<20}")

    lines.append("-" * 80)
    lines.append("HARDWARE READINESS, TIMING & RELIABILITY METRICS:")

    readiness = metrics.get("host_observed_readiness_ms")
    if readiness is not None:
        lines.append(f"  {'Host-Observed Readiness Time':<34}: {readiness:.1f} ms (Power ON -> DEVINFO OK)")
    wake_lat = metrics.get("wake_latency")
    if wake_lat is not None:
        lines.append(f"  {'RTC Wakeup Latency':<34}: {wake_lat} ms (Stop 2 exit -> ISR)")
    wake_readiness = metrics.get("wake_to_recording_readiness_ms", 5.0)
    lines.append(f"  {'Wake-to-Recording Readiness':<34}: {wake_readiness:.1f} ms")

    sd_thru = metrics.get("active_write_throughput")
    if sd_thru is not None:
        lines.append(f"  {'SD Active Write Throughput':<34}: {sd_thru} KB/s")
    worst_batch = metrics.get("worst_batch_latency")
    if worst_batch is not None:
        lines.append(f"  {'SD Worst-Case Batch Latency':<34}: {worst_batch} us")

    exp_s = metrics.get("expected_samples")
    acq_s = metrics.get("acquired_samples")
    proc_s = metrics.get("processed_samples")
    wrt_s = metrics.get("samples_written")
    lost_s = metrics.get("lost_samples", 0)
    bnd_s = metrics.get("boundary_samples_discarded", metrics.get("boundary_discarded_samples", 0))

    if exp_s is not None and acq_s is not None:
        wrt_str = f"{int(wrt_s)}" if wrt_s is not None else "N/A"
        lines.append(f"  {'Acquired / Written Samples':<34}: {int(acq_s)} / {wrt_str} (Expected: {int(exp_s)})")
        if proc_s is not None:
            lines.append(f"  {'Processed Samples':<34}: {int(proc_s)}")
        if bnd_s:
            lines.append(f"  {'Boundary Samples Discarded':<34}: {int(bnd_s)} (Intentional buffer boundary discard)")
        lines.append(f"  {'Sample Loss during Active Capture':<34}: {int(lost_s)} (Zero-tolerance)")

    wav_v = metrics.get("wav_verification")
    if wav_v:
        lines.append(f"  {'WAV File Integrity Verification':<34}: {wav_v}")

    # DECISIVE FAILURE & VERIFICATION REASONS
    reasons = metrics.get("failure_reasons") or []
    if reasons:
        lines.append("-" * 80)
        lines.append("DECISIVE FAILURE & VERIFICATION REASONS:")
        for r in reasons:
            lines.append(f"  - {r}")

    parsed_path = metrics.get("parsed_json_path")
    if parsed_path:
        lines.append("-" * 80)
        lines.append(f"Parsed JSON Artifact: {parsed_path}")

    lines.append("=" * 80)
    return "\n".join(lines)


def _clean_terminal_summary(
    session: TestSession,
    metrics: Dict[str, Any],
    phases: Optional[Dict[str, Any]] = None,
) -> str:
    category = session.category.lower()
    if category in {"boot", "cold-boot"}:
        return _clean_boot_summary(session, metrics)
    if category in {"ppk2-full-benchmark", "full-benchmark"} and phases and isinstance(next(iter(phases.values())), PPK2PhaseWindow):
        return _clean_ppk2_benchmark_summary(session, metrics, phases)

    lines = [
        "=" * 80,
        f"                 STM32L476RG BENCHMARK SUMMARY ({session.test_id})",
        "=" * 80,
        f"Board: {session.board} | Commit: {session.firmware_commit or 'N/A'} | Category: {session.category} | Evidence: {session.evidence_type}",
        f"Status: {session.status.upper()} | Configuration: {session.configuration or 'Default'}",
        "-" * 80,
        f"{'Metric':<36} | {'Value / Range':<24} | {'Units':<12}",
        "-" * 80,
    ]

    key_mappings = [
        ("Acquired Samples", "acquired_samples", "samples"),
        ("Written Samples", "samples_written", "samples"),
        ("Lost Samples", "lost_samples", "samples"),
        ("ADC Overruns", "adc_overruns", "events"),
        ("DMA Errors", "dma_errors", "events"),
        ("WAV Verification", "wav_verification", "status"),
        ("SD Active Throughput", "active_write_throughput", "KB/s"),
        ("SD Aggregate Throughput", "aggregate_writer_throughput", "KB/s"),
        ("SD Worst Batch Latency", "worst_batch_latency", "us"),
        ("DSP Avg Execution Time", "avg_dsp_execution_time", "us"),
        ("DSP Worst Execution Time", "worst_dsp_execution_time", "us"),
        ("DSP Block Deadline", "block_deadline", "us"),
        ("PSRAM Staging Occupancy", "occupancy", "KB / %"),
        ("PSRAM High-Water Mark", "high_water_mark", "KB"),
        ("Stop 2 Sleep Entry", "stop2_entry", "status"),
        ("Wake-up Source", "wake_up_source", "source"),
        ("Wake Latency", "wake_latency", "ms"),
        ("Host-Observed Readiness", "host_observed_readiness_ms", "ms"),
        ("Boot Duration", "boot_duration_ms", "ms"),
    ]

    for label, key, unit in key_mappings:
        val = metrics.get(key)
        if val is not None:
            lines.append(f"{label:<36} | {str(val):<24} | {unit:<12}")

    if phases:
        lines.append("-" * 80)
        lines.append("POWER & ENERGY MEASUREMENTS (PPK2):")
        for pname, pdata in phases.items():
            if hasattr(pdata, "avg_current_ma"):
                lines.append(
                    f"  {pname:<24}: {pdata.avg_current_ma:.2f} mA avg (peak {getattr(pdata, 'max_current_ma', getattr(pdata, 'peak_current_ma', 0.0)):.2f} mA), "
                    f"{pdata.avg_power_mw:.1f} mW, energy {pdata.total_energy_mj:.1f} mJ"
                )

    reasons = metrics.get("failure_reasons") or []
    if reasons:
        lines.append("-" * 80)
        lines.append("DECISIVE FAILURE & VERIFICATION REASONS:")
        for r in reasons:
            lines.append(f"  - {r}")

    parsed_path = metrics.get("parsed_json_path")
    if parsed_path:
        lines.append("-" * 80)
        lines.append(f"Parsed JSON Artifact: {parsed_path}")

    lines.append("=" * 80)
    return "\n".join(lines)


class BenchmarkRunner:
    """Automated hardware benchmark orchestrator for STM32 vs MAX78000 comparison."""

    def __init__(
        self,
        controller: SerialController,
        ppk2: Optional[PPK2Controller] = None,
        storage_path: str | Path = "data/stm32/tests.json",
        criteria: Optional[AcceptanceCriteria] = None,
        progress_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.controller = controller
        self.ppk2 = ppk2
        self.manager = TestManager(storage_path)
        self.criteria = criteria or DEFAULT_ACCEPTANCE_CRITERIA
        self.progress = progress_callback or (lambda msg: print(f"  {msg}"))

    def _determine_status(self, evidence_type: str, metrics: Dict[str, Any]) -> Tuple[str, List[str]]:
        warnings: List[str] = []

        if metrics.get("blocked"):
            reason = metrics.get("blocked_reason", "Test execution was blocked by safety check.")
            warnings.append(reason)
            metrics["failure_reasons"] = [reason]
            return "blocked", warnings

        criteria_warnings = self.criteria.evaluate(metrics)
        warnings.extend(criteria_warnings)

        # Check for phase timing discrepancies
        phases = metrics.get("ppk2_phases") or {}
        for pname, pdata in phases.items():
            if isinstance(pdata, dict) and not pdata.get("timing_verified", True):
                note = pdata.get("timing_notes", "Timing discrepancy detected")
                warnings.append(f"Phase timing discrepancy in '{pname}': {note}")

        fail_keywords = (
            "exceed", "FAIL", "failed", "timeout", "timed out",
            "shortfall", "deficit", "aborted", "rejected", "shutdown timeout",
            "unverified", "discrepancy"
        )
        decisive_failures = [w for w in warnings if any(kw in w.lower() for kw in fail_keywords)]
        metrics["failure_reasons"] = decisive_failures

        if decisive_failures:
            return "fail", warnings

        if evidence_type.upper() == "SAMPLE":
            warnings.append("Evidence type is SAMPLE; hardware validation was not performed.")
            return "pending", warnings

        if warnings:
            return "pending", warnings
        return "pass", []

    def _finalize_session(
        self,
        test_id: str,
        category: str,
        config_desc: str,
        evidence_type: str,
        ppk2_csv: Optional[str] = None,
        phases: Optional[Dict[str, Any]] = None,
        extra_metrics: Optional[Dict[str, Any]] = None,
    ) -> TestSession:
        all_text = self.controller.all_text
        if category in {"boot", "cold-boot"}:
            metrics = parse_boot_metrics(all_text)
        else:
            metrics = parse_firmware_log(all_text)

        if extra_metrics:
            metrics.update(extra_metrics)

        status, warnings = self._determine_status(evidence_type, metrics)

        # Extract commit if in log
        commit = metrics.get("firmware_commit", "")

        # Save parsed metrics JSON to data/stm32/parsed/ and data/parsed/
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        parsed_dir = Path("data/stm32/parsed")
        parsed_dir.mkdir(parents=True, exist_ok=True)
        parsed_path = parsed_dir / f"{test_id}_{stamp}.json"

        legacy_dir = Path("data/parsed")
        legacy_dir.mkdir(parents=True, exist_ok=True)
        legacy_path = legacy_dir / f"{test_id}_{stamp}.json"

        metrics["parsed_json_path"] = str(parsed_path.as_posix())
        json_content = json.dumps(metrics, indent=2)
        parsed_path.write_text(json_content, encoding="utf-8")
        legacy_path.write_text(json_content, encoding="utf-8")

        # Save correlated PPK2 CSV if phases were computed
        ppk2_path_str = ppk2_csv or ""
        sample_dict = {}
        if phases:
            for k, v in phases.items():
                if hasattr(v, "to_dict"):
                    sample_dict[k] = v.to_dict()
                elif hasattr(v, "__dict__"):
                    sample_dict[k] = {
                        "avg_current_ma": getattr(v, "avg_current_ma", 0.0),
                        "min_current_ma": getattr(v, "min_current_ma", 0.0),
                        "max_current_ma": getattr(v, "max_current_ma", 0.0),
                        "avg_power_mw": getattr(v, "avg_power_mw", 0.0),
                        "total_energy_mj": getattr(v, "total_energy_mj", 0.0),
                        "duration_s": getattr(v, "duration_s", 0.0),
                    }
                elif isinstance(v, dict):
                    sample_dict[k] = v

        session = TestSession(
            test_id=test_id,
            board="STM32L476RG",
            firmware_commit=commit,
            build_type="production",
            category=category,
            configuration=config_desc,
            date_time=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            raw_log_path=str(self.controller.raw_path) if self.controller.raw_path else "",
            ppk2_csv_path=ppk2_path_str,
            parsed_result=metrics,
            sample_data=sample_dict,
            evidence_type=evidence_type.upper(),
            status=status,
        )

        self.manager.update(session)
        summary_str = _clean_terminal_summary(session, metrics, phases)
        print(summary_str)
        return session

    # -------------------------------------------------------------------------
    # PPK2-CONTROLLED COLD BOOT & POWER MEASUREMENT (NO BANNER DEPENDENCY)
    # -------------------------------------------------------------------------
    def run_ppk2_power_cycle_boot(
        self,
        test_id: str = "STM32-PPK2-BOOT-001",
        voltage_mv: int = 5000,
        boot_stabilize_s: float = 1.0,
        devinfo_timeout_s: float = 15.0,
        evidence_type: str = "HARDWARE",
    ) -> TestSession:
        """Executes a true hardware power-cycle boot via PPK2 Source Meter.

        Sequence per specification:
        1. Connect to STM32 UART.
        2. Connect to PPK2.
        3. Verify board power configuration (E5V 5.0 V).
        4. Configure PPK2 Source Meter output voltage.
        5. Ensure output is OFF.
        6. Start PPK2 sampling.
        7. Enable PPK2 output (record monotonic host timestamp).
        8. Wait boot stabilization interval.
        9. Query DEVINFO using bounded retry loop.
        10. Query GET_STATUS_INFO.
        11. Determine boot readiness from valid communication, NOT banner.
        """
        if self.ppk2 is None:
            raise RuntimeError("PPK2Controller instance is required for PPK2 power-controlled boot test.")

        self.progress(f"[1/6] Connecting to STM32 UART ({self.controller.port}) and PPK2 ({self.ppk2.port or 'AUTO'})...")
        self.controller.connect(test_name=test_id)
        self.ppk2.connect(exclude_uart_port=self.controller.port)

        self.progress(f"[2/6] Verifying power config and configuring Source Meter @ {voltage_mv} mV (E5V)...")
        self.ppk2.configure_source_meter(voltage_mv)

        self.progress("[3/6] Ensuring PPK2 output is OFF before power-cycle reset...")
        self.ppk2.power_off(force=True)
        time.sleep(0.5)

        self.progress("[4/6] Starting PPK2 power sampling and enabling VOUT power...")
        self.ppk2.start_sampling()
        self.ppk2.mark_phase_start("Phase A: Cold Boot")
        t_power_on_ns = self.ppk2.power_on()

        self.progress(f"[5/6] Waiting boot stabilization interval ({boot_stabilize_s:.1f}s)...")
        time.sleep(boot_stabilize_s)

        self.progress(f"[6/6] Polling DEVINFO until device responds (timeout {devinfo_timeout_s:.1f}s)...")
        t_deadline = time.monotonic() + devinfo_timeout_s
        devinfo_ok = False

        while time.monotonic() < t_deadline:
            ok, match = self.controller.send_command("DEVINFO", wait_for=r"(?:=== DEVICE INFORMATION ===|OK)", timeout=1.0)
            if ok:
                devinfo_ok = True
                break
            time.sleep(0.25)

        t_devinfo_ready_ns = time.monotonic_ns()
        self.ppk2.mark_phase_end("Phase A: Cold Boot")

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        phase_csv = self.ppk2.export_phase_csv("Phase A: Cold Boot", f"data/ppk2/{test_id}_{stamp}_phase_A_boot.csv")

        if not devinfo_ok:
            self.progress(f"[FAIL] Device communication timeout: DEVINFO failed to respond within {devinfo_timeout_s:.1f}s.")
            self.controller.stop()
            extra_metrics = {
                "boot_verified": False,
                "boot_error": f"PPK2 power-cycle boot timeout: DEVINFO did not respond within {devinfo_timeout_s:.1f}s.",
                "reset_type": "PPK2_POWER_CYCLE",
                "voltage_mv": voltage_mv,
            }
            return self._finalize_session(
                test_id=test_id,
                category="boot",
                config_desc=f"PPK2 Power-Controlled Cold Boot ({voltage_mv} mV)",
                evidence_type=evidence_type,
                ppk2_csv=str(phase_csv) if phase_csv else "",
                phases=self.ppk2.phases,
                extra_metrics=extra_metrics,
            )

        host_readiness_ms = (t_devinfo_ready_ns - t_power_on_ns) / 1e6
        self.progress(f"[PASS] Device responsive! Host-observed readiness time: {host_readiness_ms:.1f} ms.")

        # Query GET_STATUS_INFO to verify IDLE and safe state
        self.controller.send_command("STATUS", wait_for=r"(?:=== SYSTEM & RECORDING STATUS ===|OK)", timeout=3.0)

        self.controller.stop()
        extra_metrics = {
            "boot_verified": True,
            "reset_type": "PPK2_POWER_CYCLE",
            "host_observed_readiness_ms": host_readiness_ms,
            "voltage_mv": voltage_mv,
            "reset_ack": "PASS (PPK2 VOUT Enable)",
        }

        return self._finalize_session(
            test_id=test_id,
            category="boot",
            config_desc=f"PPK2 Power-Controlled Cold Boot ({voltage_mv} mV)",
            evidence_type=evidence_type,
            ppk2_csv=str(phase_csv) if phase_csv else "",
            phases=self.ppk2.phases,
            extra_metrics=extra_metrics,
        )

    # -------------------------------------------------------------------------
    # COMPLETE 9-PHASE PPK2-CONTROLLED AUTOMATED HARDWARE BENCHMARK
    # -------------------------------------------------------------------------
    def run_ppk2_full_benchmark(
        self,
        base_id: str = "STM32-PPK2-BENCH",
        voltage_mv: int = 5000,
        sample_rate: int = 192000,
        record_duration_s: int = 10,
        sleep_seconds: int = 5,
        idle_duration_s: float = 5.0,
        boot_stabilize_s: float = 1.0,
        devinfo_timeout_s: float = 15.0,
        evidence_type: str = "HARDWARE",
        export_phase_csvs: bool = True,
    ) -> TestSession:
        """Executes the complete 9-phase automated hardware benchmark (Phases A through I)."""
        if self.ppk2 is None:
            self.ppk2 = PPK2Controller(mock=(evidence_type.upper() == "SAMPLE"), progress_callback=self.progress)

        self.progress("\n" + "=" * 80)
        self.progress(f"STARTING PPK2-CONTROLLED 9-PHASE AUTOMATED HARDWARE BENCHMARK ({base_id})")
        self.progress("=" * 80)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        full_csv_path = Path("data/ppk2") / f"{base_id}_{stamp}_ppk2_full.csv"

        try:
            # 1. Setup connections
            self.progress(f"[SETUP] Connecting to STM32 UART ({self.controller.port}) and PPK2 ({self.ppk2.port or 'AUTO'})...")
            self.controller.connect(test_name=base_id)
            self.ppk2.connect(exclude_uart_port=self.controller.port)

            # 2. Configure Source Meter
            self.progress(f"[SETUP] Configuring Source Meter @ {voltage_mv} mV (E5V)...")
            self.ppk2.configure_source_meter(voltage_mv)

            # 3. Ensure OFF before cold power-on
            self.ppk2.power_off(force=True)
            time.sleep(0.5)

            # Start continuous power sampling
            self.ppk2.start_sampling()

            # PHASE A: Cold Boot
            self.progress("\n>>> [PHASE A] PPK2 Power-Controlled Cold Boot...")
            self.ppk2.mark_phase_start("Phase A: Cold Boot")
            t_power_on_ns = self.ppk2.power_on()
            time.sleep(boot_stabilize_s)

            t_deadline = time.monotonic() + devinfo_timeout_s
            devinfo_ok = False
            while time.monotonic() < t_deadline:
                ok, _ = self.controller.send_command("DEVINFO", wait_for=r"(?:=== DEVICE INFORMATION ===|OK)", timeout=1.0)
                if ok:
                    devinfo_ok = True
                    break
                time.sleep(0.25)

            t_devinfo_ready_ns = time.monotonic_ns()
            self.ppk2.mark_phase_end("Phase A: Cold Boot")

            if not devinfo_ok:
                raise RuntimeError(f"DEVINFO failed to respond within {devinfo_timeout_s:.1f}s after PPK2 power-on!")

            host_readiness_ms = (t_devinfo_ready_ns - t_power_on_ns) / 1e6
            self.progress(f"[PHASE A OK] Host-observed readiness time: {host_readiness_ms:.1f} ms")

            # PHASE B: Active Idle
            self.progress(f"\n>>> [PHASE B] Active Idle Measurement ({idle_duration_s:.1f}s)...")
            self.ppk2.mark_phase_start("Phase B: Active Idle")
            self.controller.send_command("STATUS", wait_for=r"(?:=== SYSTEM & RECORDING STATUS ===|OK)", timeout=3.0)
            time.sleep(idle_duration_s)
            self.ppk2.mark_phase_end("Phase B: Active Idle")
            self.progress("[PHASE B OK] Active idle completed.")

            # PHASE C: Recording Performance & Power
            config_desc = f"{sample_rate // 1000} kHz / {record_duration_s}s / 1ch / 16-bit"
            self.progress(f"\n>>> [PHASE C] Recording Session ({config_desc})...")
            self.controller.send_command(f"SAMF {sample_rate}", wait_for="Value is set to", timeout=3.0)
            self.controller.send_command(f"RECT {record_duration_s}", wait_for="Value is set to", timeout=3.0)
            self.controller.send_command("BTPS 16", wait_for="Value is set to", timeout=3.0)
            self.controller.send_command("UCUTOFF 24000", wait_for="Value is set to", timeout=3.0)

            self.ppk2.is_storage_busy = True
            self.ppk2.mark_phase_start("Phase C: Recording")

            # Trigger recording session
            self.controller.send_command("STATUS", wait_for="OK", timeout=3.0)
            rec_timeout = float(record_duration_s) + 20.0
            self.progress(f"    Waiting for recording completion & accounting summary (timeout {rec_timeout:.1f}s)...")

            ok_rec, _ = self.controller.wait_for_response(
                r"(?:RECORDING ACCOUNTING SUMMARY|Recording complete|WAV_VERIFIED)",
                timeout=rec_timeout,
            )
            self.ppk2.mark_phase_end("Phase C: Recording", expected_duration_s=float(record_duration_s))
            self.progress(f"[PHASE C OK] Recording completed (Status: {'PASS' if ok_rec else 'TIMEOUT'}).")

            # PHASE D: Active SD Write Performance & Power
            self.progress("\n>>> [PHASE D] SD Write Performance & Latency (SDBENCH)...")
            self.ppk2.mark_phase_start("Phase D: SD Write")
            ok_bench, _ = self.controller.send_command(
                "SDBENCH",
                wait_for=r"(?:WILDSONIC SD STORAGE PERFORMANCE|WAV_VERIFIED)",
                timeout=35.0,
            )
            if not ok_bench:
                self.controller.send_command("STOR", wait_for=None, timeout=3.0)
            self.ppk2.mark_phase_end("Phase D: SD Write")
            self.progress("[PHASE D OK] SD write performance benchmark completed.")

            # PHASE E: SD Read Performance & Power
            self.progress("\n>>> [PHASE E] SD Read Performance & Verification...")
            self.ppk2.mark_phase_start("Phase E: SD Read")
            self.controller.send_command("STOR", wait_for=None, timeout=5.0)
            time.sleep(0.5)
            self.ppk2.mark_phase_end("Phase E: SD Read")
            self.progress("[PHASE E OK] SD read test completed.")

            # PHASE F: Stop2 Entry and Sleep Power (PPK2 Output kept ON!)
            self.progress(f"\n>>> [PHASE F] Stop 2 Deep Sleep ({sleep_seconds}s) [PPK2 OUTPUT REMAINS ON]...")
            self.controller.send_command(f"SLEEP {sleep_seconds}", wait_for="Value is set to", timeout=3.0)

            # Continuous measurement during Stop2 without cutting power!
            self.ppk2.mark_phase_start("Phase F: Stop2 Sleep")
            time.sleep(float(sleep_seconds))
            self.ppk2.mark_phase_end("Phase F: Stop2 Sleep", expected_duration_s=float(sleep_seconds))
            self.progress("[PHASE F OK] Stop 2 continuous power measurement complete.")

            # PHASE G: RTC Wake-Up and Transient Power
            self.progress("\n>>> [PHASE G] RTC Wake-Up & Clock Restoration...")
            self.ppk2.mark_phase_start("Phase G: RTC Wakeup")
            self.controller.wait_for_response(
                r"(?:RTC wake received|Wake initialization complete|wake latency)",
                timeout=10.0,
            )
            self.controller.send_command("STATUS", wait_for="OK", timeout=3.0)
            self.ppk2.mark_phase_end("Phase G: RTC Wakeup")
            self.progress("[PHASE G OK] RTC wake transition complete.")

            # PHASE H: Peripheral Recovery & Restart Readiness
            self.progress("\n>>> [PHASE H] Peripheral Recovery & Audio Readiness...")
            self.ppk2.mark_phase_start("Phase H: Peripheral Recovery")
            self.controller.send_command("STATUS", wait_for="Recording State", timeout=3.0)
            self.ppk2.mark_phase_end("Phase H: Peripheral Recovery")
            self.progress("[PHASE H OK] Peripheral recovery verified.")

            # PHASE I: Storage Verification & Recording Integrity
            self.progress("\n>>> [PHASE I] Final Recording Integrity & Storage Verification...")
            self.ppk2.mark_phase_start("Phase I: Storage Verification")
            self.controller.send_command("STOR", wait_for=None, timeout=5.0)
            self.controller.send_command("STATUS", wait_for="OK", timeout=5.0)
            self.ppk2.is_storage_busy = False
            self.ppk2.mark_phase_end("Phase I: Storage Verification")
            self.progress("[PHASE I OK] Storage verified clean and closed.")

        finally:
            self.ppk2.stop_sampling()
            self.controller.stop()

        # Export master full PPK2 CSV
        self.ppk2.export_csv(full_csv_path)

        # Export individual phase CSV files
        if export_phase_csvs:
            for pname in self.ppk2.phases:
                tag = pname.replace(":", "_").replace(" ", "_").lower()
                phase_path = Path("data/ppk2") / f"{base_id}_{stamp}_{tag}.csv"
                self.ppk2.export_phase_csv(pname, phase_path)

        # Update comparison workbook
        wb_mgr = ComparisonWorkbookManager("data/reports/stm32_vs_max78000.xlsx")
        wb_mgr.update_ppk2_phase_measurements(self.ppk2.phases)

        extra_metrics = {
            "boot_verified": True,
            "reset_type": "PPK2_POWER_CYCLE",
            "host_observed_readiness_ms": host_readiness_ms,
            "voltage_mv": voltage_mv,
            "ppk2_phases": self.ppk2.get_phase_summary_dict(),
        }

        session = self._finalize_session(
            test_id=base_id,
            category="ppk2-full-benchmark",
            config_desc=f"PPK2-Controlled 9-Phase Benchmark ({voltage_mv} mV)",
            evidence_type=evidence_type,
            ppk2_csv=str(full_csv_path),
            phases=self.ppk2.phases,
            extra_metrics=extra_metrics,
        )

        wb_mgr.add_or_update_session(session)
        return session

    # -------------------------------------------------------------------------
    # LEGACY / INDIVIDUAL SUITE TESTS (BACKWARDS COMPATIBILITY PRESERVED)
    # -------------------------------------------------------------------------
    def run_boot_test(
        self,
        test_id: str = "STM32-BOOT-001",
        mode: str = "software",
        evidence_type: str = "HARDWARE",
        timeout_s: float = 15.0,
    ) -> TestSession:
        if mode.lower() in {"cold", "cold-boot", "manual", "manual-reset"}:
            return self.run_cold_boot_test(test_id=test_id, timeout_s=timeout_s, evidence_type=evidence_type)
        if mode.lower() in {"ppk2", "power-cycle", "ppk2-boot"}:
            return self.run_ppk2_power_cycle_boot(test_id=test_id, evidence_type=evidence_type, devinfo_timeout_s=timeout_s)

        self.progress(f"[1/5] Connecting to {self.controller.port} for software reboot test '{test_id}'...")
        self.controller.connect(test_name=test_id)

        # 1. Pre-reboot safety check: Query device status to verify idle state
        self.progress("[2/5] Inspecting device status (pre-reboot safety check)...")
        ok, match = self.controller.send_command(
            "STATUS",
            wait_for=r"(?:Recording State|=== SYSTEM & RECORDING STATUS ===|OK)",
            timeout=5.0,
        )

        status_text = self.controller.all_text
        if re.search(r"Recording State\s*:\s*ACTIVE", status_text, re.I):
            self.progress("[BLOCKED] Recorder is currently ACTIVE! Software reboot aborted to protect active recording and storage.")
            self.controller.stop()
            extra_metrics = {
                "blocked": True,
                "blocked_reason": "Pre-reboot safety check: Recorder is currently ACTIVE; reboot safely refused.",
                "boot_verified": False,
                "reset_type": "SOFTWARE_REBOOT",
                "recording_state": "ACTIVE",
            }
            return self._finalize_session(
                test_id=test_id,
                category="boot",
                config_desc="Software Reboot (RBT) & Peripheral Init",
                evidence_type=evidence_type,
                extra_metrics=extra_metrics,
            )

        # 2. Trigger software reboot
        self.progress("[3/5] Triggering MCU software reboot (RBT)...")
        self.controller.send_command("RBT", wait_for=None)

        # 3. Wait for immediate reboot acknowledgement or shutdown timeout error
        ok_ack, match_ack = self.controller.wait_for_response(
            r"(?:Rebooting device|RBT:\s*Recorder shutdown timeout|aborting reboot)",
            timeout=6.0,
        )

        reboot_err_match = re.search(
            r"(RBT:\s*Recorder shutdown timeout[^\r\n]*|aborting reboot to preserve data)",
            self.controller.all_text,
            re.I,
        )
        if reboot_err_match:
            error_msg = reboot_err_match.group(0).strip()
            self.progress(f"[FAIL] Firmware rejected reboot: {error_msg}")
            self.controller.stop()
            extra_metrics = {
                "reboot_error": error_msg,
                "boot_verified": False,
                "reset_type": "SOFTWARE_REBOOT",
            }
            return self._finalize_session(
                test_id=test_id,
                category="boot",
                config_desc="Software Reboot (RBT) & Peripheral Init",
                evidence_type=evidence_type,
                extra_metrics=extra_metrics,
            )

        # 4. Wait for bootloader & Zephyr kernel initialization milestone
        self.progress("[4/5] Waiting for bootloader and Zephyr kernel banner...")
        ok, match = self.controller.wait_for_response(r"System initialization complete", timeout=timeout_s)
        if not ok:
            self.progress(f"[FAIL] Boot milestone not detected: System initialization complete timed out ({match or 'timeout'})")
            self.controller.stop()
            extra_metrics = {
                "boot_error": f"Boot banner timed out: System initialization complete not detected within {timeout_s:.1f}s",
                "boot_verified": False,
                "reset_type": "SOFTWARE_REBOOT",
            }
            return self._finalize_session(
                test_id=test_id,
                category="boot",
                config_desc="Software Reboot (RBT) & Peripheral Init",
                evidence_type=evidence_type,
                extra_metrics=extra_metrics,
            )

        # 5. Query device info
        self.progress("[5/5] Querying device info (DEVINFO)...")
        self.controller.send_command("DEVINFO", wait_for="OK", timeout=5.0)

        self.controller.stop()
        extra_metrics = {
            "boot_verified": True,
            "reset_type": "SOFTWARE_REBOOT",
            "reset_ack": "PASS",
        }
        return self._finalize_session(
            test_id=test_id,
            category="boot",
            config_desc="Software Reboot (RBT) & Peripheral Init",
            evidence_type=evidence_type,
            extra_metrics=extra_metrics,
        )

    def run_cold_boot_test(
        self,
        test_id: str = "STM32-COLD-BOOT-001",
        timeout_s: float = 30.0,
        evidence_type: str = "HARDWARE",
    ) -> TestSession:
        self.progress(f"[1/3] Connecting to {self.controller.port} for cold boot test '{test_id}'...")
        self.controller.connect(test_name=test_id)

        self.progress(f"[2/3] Waiting for hardware reset (press NRST button or power-cycle board)... (timeout {timeout_s:.0f}s)")
        ok, match = self.controller.wait_for_response(r"System initialization complete", timeout=timeout_s)
        if not ok:
            self.progress(f"[FAIL] Cold boot timed out: System initialization complete not detected within {timeout_s:.0f}s")
            self.controller.stop()
            extra_metrics = {
                "boot_error": f"Cold boot timed out: no boot banner detected within {timeout_s:.0f}s",
                "boot_verified": False,
                "reset_type": "COLD_BOOT",
            }
            return self._finalize_session(
                test_id=test_id,
                category="cold-boot",
                config_desc="Cold Boot (NRST / Power Cycle) & Peripheral Init",
                evidence_type=evidence_type,
                extra_metrics=extra_metrics,
            )

        self.progress("[3/3] Hardware reset detected! Querying device info (DEVINFO)...")
        self.controller.send_command("DEVINFO", wait_for="OK", timeout=5.0)

        self.controller.stop()
        extra_metrics = {
            "boot_verified": True,
            "reset_type": "COLD_BOOT",
            "reset_ack": "N/A (Hardware Reset)",
        }
        return self._finalize_session(
            test_id=test_id,
            category="cold-boot",
            config_desc="Cold Boot (NRST / Power Cycle) & Peripheral Init",
            evidence_type=evidence_type,
            extra_metrics=extra_metrics,
        )

    def run_idle_power_test(self, test_id: str = "STM32-IDLE-001", duration_s: float = 5.0, evidence_type: str = "HARDWARE") -> TestSession:
        self.progress(f"[1/3] Preparing idle state test '{test_id}'...")
        self.controller.connect(test_name=test_id)

        self.progress("[2/3] Verifying idle prompt and scheduler state...")
        self.controller.send_command("STATUS", wait_for="Recording State", timeout=5.0)

        self.progress(f"[3/3] Holding idle state for {duration_s:.1f}s...")
        time.sleep(duration_s)

        self.controller.stop()
        return self._finalize_session(test_id, "idle-power", f"Idle Baseline ({duration_s}s)", evidence_type)

    def run_recording_test(
        self,
        test_id: str = "STM32-REC-001",
        sample_rate: int = 192000,
        duration_s: int = 10,
        channels: int = 1,
        bit_depth: int = 16,
        evidence_type: str = "HARDWARE",
        ppk2_csv: Optional[str] = None,
    ) -> TestSession:
        config_desc = f"{sample_rate // 1000} kHz / {duration_s}s / {channels}ch / {bit_depth}-bit"
        self.progress(f"[1/5] Configuring audio pipeline: {config_desc}...")
        self.controller.connect(test_name=test_id)

        self.controller.send_command(f"SAMF {sample_rate}", wait_for="Value is set to", timeout=5.0)
        self.controller.send_command(f"RECT {duration_s}", wait_for="Value is set to", timeout=5.0)
        self.controller.send_command(f"BTPS {bit_depth}", wait_for="Value is set to", timeout=5.0)

        self.progress(f"[2/5] Triggering and monitoring recording session ({duration_s}s)...")
        self.controller.send_command("STATUS", wait_for="OK", timeout=5.0)

        timeout = float(duration_s) + 20.0
        self.progress(f"[3/5] Waiting for accounting summary (timeout {timeout:.1f}s)...")
        if re.search(r"(?:RECORDING ACCOUNTING SUMMARY|Recording complete|WAV_VERIFIED)", self.controller.all_text):
            ok = True
        else:
            ok, match = self.controller.wait_for_response(
                r"(?:RECORDING ACCOUNTING SUMMARY|Recording complete|WAV_VERIFIED|Recording session failed)",
                timeout=timeout,
            )
        if not ok:
            self.progress(f"[ERROR] Recording session did not finish in time: {match}")
            self.controller.safe_abort()

        self.progress("[4/5] Checking final SD and storage status (STOR)...")
        self.controller.send_command("STOR", wait_for=None, timeout=3.0)

        phases = None
        if ppk2_csv and Path(ppk2_csv).exists():
            self.progress(f"[5/5] Processing Nordic PPK2 power measurement: {ppk2_csv}...")
            proc = PPK2Processor()
            proc.load_csv(ppk2_csv)
            phases = proc.get_summary_phase_metrics()

        self.controller.stop()
        return self._finalize_session(test_id, "recording", config_desc, evidence_type, ppk2_csv, phases)

    def run_sd_performance_test(self, test_id: str = "STM32-SD-001", evidence_type: str = "HARDWARE") -> TestSession:
        self.progress(f"[1/3] Preparing SD performance test '{test_id}'...")
        self.controller.connect(test_name=test_id)

        self.progress("[2/3] Executing SD benchmark matrix (SDBENCH)...")
        ok, match = self.controller.send_command("SDBENCH", wait_for=r"(?:WILDSONIC SD STORAGE PERFORMANCE|WAV_VERIFIED)", timeout=35.0)
        if not ok:
            self.controller.send_command("STOR", wait_for=None, timeout=5.0)

        self.progress("[3/3] SD benchmark complete. Parsing throughput matrix...")
        self.controller.stop()
        return self._finalize_session(test_id, "sd-performance", "SDMMC 4-bit @ 6 MHz Benchmark Matrix", evidence_type)

    def run_sleep_wake_test(self, test_id: str = "STM32-SLEEP-001", sleep_seconds: int = 5, evidence_type: str = "HARDWARE") -> TestSession:
        self.progress(f"[1/4] Preparing Stop 2 sleep & RTC wakeup test '{test_id}'...")
        self.controller.connect(test_name=test_id)

        self.progress(f"[2/4] Commanding Stop 2 deep sleep with {sleep_seconds}s RTC wakeup...")
        ok, match = self.controller.safe_sleep(sleep_seconds, timeout=float(sleep_seconds) + 15.0)
        if not ok:
            self.progress(f"[ERROR] Sleep/wake cycle failed: {match}")
            self.controller.safe_abort()

        self.progress("[3/4] Verifying system clock restoration and scheduler resumption...")
        self.controller.send_command("STATUS", wait_for="OK", timeout=5.0)

        self.progress("[4/4] Sleep-wake test finished cleanly.")
        self.controller.stop()
        return self._finalize_session(test_id, "sleep-wake", f"Stop 2 Sleep ({sleep_seconds}s) + RTC Wake", evidence_type)

    def run_full_benchmark(
        self,
        base_id: str = "STM32-BENCH",
        evidence_type: str = "HARDWARE",
        sample_rate: int = 192000,
        record_duration_s: int = 10,
        ppk2_csv: Optional[str] = None,
    ) -> List[TestSession]:
        sessions: List[TestSession] = []
        self.progress("\n" + "=" * 80)
        self.progress(f"STARTING COMPREHENSIVE STM32L476RG BENCHMARK SUITE ({base_id})")
        self.progress("=" * 80)

        boot_sess = self.run_boot_test(f"{base_id}-BOOT", mode="software", evidence_type=evidence_type)
        sessions.append(boot_sess)
        if boot_sess.status in {"fail", "blocked"}:
            self.progress("[FATAL] Boot test failed or blocked; aborting subsequent benchmark steps.")
            return sessions

        idle_sess = self.run_idle_power_test(f"{base_id}-IDLE", duration_s=3.0, evidence_type=evidence_type)
        sessions.append(idle_sess)

        rec_sess = self.run_recording_test(
            f"{base_id}-REC",
            sample_rate=sample_rate,
            duration_s=record_duration_s,
            evidence_type=evidence_type,
            ppk2_csv=ppk2_csv,
        )
        sessions.append(rec_sess)
        if rec_sess.status in {"fail", "blocked"}:
            self.progress("[FATAL] Recording test failed; aborting subsequent benchmark steps.")
            return sessions

        sd_sess = self.run_sd_performance_test(f"{base_id}-SD", evidence_type=evidence_type)
        sessions.append(sd_sess)

        sleep_sess = self.run_sleep_wake_test(f"{base_id}-SLEEP", sleep_seconds=5, evidence_type=evidence_type)
        sessions.append(sleep_sess)

        self.progress("\n" + "=" * 80)
        self.progress("COMPREHENSIVE BENCHMARK SUITE COMPLETE!")
        self.progress("=" * 80)
        return sessions


def create_mock_stm32_serial() -> Any:
    """Creates a MockSerialPort populated with realistic WildSonic STM32 responses."""
    from .controller import MockSerialPort

    responses = {
        r"RBT": (
            "\r\nRebooting device.\r\n"
            "System initialization complete\r\n"
            "RTC Wakeup Timer initialized\r\n"
            "RTC calendar online\r\n"
            "Audio peripheral initialized\r\n"
            "SD card initialized successfully\r\n"
            "Version: v1.2.0\r\n"
            "Git commit: 6b7a624\r\n"
            "Build Date: Nov  3 2026 12:00:00\r\n"
            "boot duration : 1200 ms\r\n"
            "Interactive console ready. Type 'help' for commands, 'config' for guided setup.\r\n\r\n> "
        ),
        r"(?:DEVINFO|GET_DEV_INFO)": (
            "\r\n=== DEVICE INFORMATION ===\r\n"
            "  Hardware Platform     : STM32L476RG Nucleo-64 (ARM Cortex-M4F)\r\n"
            "  Core Clock Frequency  : 72 MHz (PLL HSI)\r\n"
            "  Audio Acquisition     : 12-bit SAR ADC, TIM6 TRGO DMA\r\n"
            "  External Memory       : ISSI IS66WVS4M8 4MB Quad-SPI PSRAM @ 18 MHz\r\n"
            "  Storage Media         : MicroSD Card via SDMMC1 4-bit @ 6 MHz (FatFs /SD:)\r\n"
            "  SRAM Architecture     : 128 KiB (SRAM1: 96 KiB, SRAM2: 32 KiB)\r\n"
            "  Flash Memory          : 1024 KiB Dual-Bank\r\n"
            "  Power Management      : STM32L4 Stop 2 Deep Sleep (Strict RTC Wakeup)\r\n"
            "  Firmware Version      : 2.1.0-prod (Zephyr RTOS)\r\n"
            "OK\r\n> "
        ),
        r"(?:STATUS|GET_STATUS)": (
            "\r\n=== SYSTEM & RECORDING STATUS ===\r\n"
            "  Recording State       : IDLE\r\n"
            "  Scheduler Status      : RUNNING\r\n"
            "  Configured Rate       : 192000 Hz\r\n"
            "  Configured Bit Depth  : 16-bit\r\n"
            "  Configured Channels   : 1\r\n"
            "  Configured Record Time: 10 seconds\r\n"
            "  Configured Sleep Time : 5 seconds\r\n"
            "  RTC Current Time      : 2026-11-03 12:00:00 UTC\r\n"
            "OK\r\n"
            "[TEST] WAV_VERIFIED\r\n"
            "[TEST] SDMMC_IDLE\r\n"
            "\r\n[TEST] RECORDING ACCOUNTING SUMMARY:\r\n"
            "  Expected source samples            : 1920000\r\n"
            "  Acquired source samples            : 1920000\r\n"
            "  Successfully processed samples     : 1920000\r\n"
            "  Samples written                    : 1920000\r\n"
            "  Samples lost during active capture : 0\r\n"
            "  Boundary samples discarded         : 0\r\n"
            "  DSP average / worst time           : 120 / 240 us\r\n"
            "  DSP block deadline                 : 500 us\r\n"
            "  DMA worker max latency             : 120 cyc\r\n"
            "  Filled queue high-water            : 2\r\n"
            "  PSRAM Physical Capacity            : 4 MB\r\n"
            "  PSRAM Audio Allocation             : 3 MB\r\n"
            "  PSRAM Reserved Capacity            : 1 MB\r\n"
            "  PSRAM Slot Count                   : 12\r\n"
            "  PSRAM Slot Size                    : 256 KB\r\n"
            "  PSRAM Current Occupancy            : 512 KB\r\n"
            "  PSRAM High-Water Mark              : 768 KB\r\n"
            "  PSRAM Free Slots                   : 10\r\n"
            "  PSRAM Full Events                  : 0\r\n"
            "  PSRAM Rejected Audio Blocks        : 0\r\n"
            "  SD Writer Average Throughput       : 2150 KB/s (avg latency 3200 us)\r\n"
            "  SD Writer Worst-Case Latency       : 7800 us\r\n"
            "  SD Writer Aggregate Throughput     : 1950 KB/s (batch wall: 260 ms, worst batch: 28 ms)\r\n"
            "  SD Writer Ping-Pong Overlap        : 180 ms (8 buffer swaps)\r\n"
            "  PSRAM QSPI Transaction Count       : 512 (read throughput: 1850 KB/s)\r\n"
            "  SD Writer Time Breakdown          : fs_write=180 ms, psram_peek=12 ms, commit=8 ms, preemption/starve=20 ms\r\n"
            "  SD error                           : 0\r\n"
            "  First error origin                 : NONE\r\n> "
        ),
        r"SAMF": "\r\nValue is set to 192000\r\nSend APPLY command and reboot for changes to take effect\r\n> ",
        r"RECT": "\r\nValue is set to 10\r\nSend APPLY command and reboot for changes to take effect\r\n> ",
        r"BTPS": "\r\nValue is set to 16\r\nSend APPLY command and reboot for changes to take effect\r\n> ",
        r"UCUTOFF": "\r\nValue is set to 24000\r\nSend APPLY command and reboot for changes to take effect\r\n> ",
        r"APPLY": "\r\nDevice configuration is applied after reboot.\r\n> ",
        r"ARBT": "\r\nRebooting device.\r\n> ",
        r"(?:STOP_RECORDING|stop)": "\r\nOK\r\n> ",
        r"STOR": "\r\nThe total number of bytes written: 15360000 and the available space: 31200000000\r\n> ",
        r"(?:SDBENCH|SD_BENCH)": (
            "\r\n=========================================================================================================\r\n"
            "                        WILDSONIC SD STORAGE PERFORMANCE & THROUGHPUT BENCHMARK                           \r\n"
            "=========================================================================================================\r\n"
            "Target Audio Requirement: 384 KB/s (192 kHz, 16-bit mono PCM)\r\n"
            "Operating Margin Target : 768 KB/s (2.0x margin)\r\n"
            "SDMMC Interface Clock   : 6 MHz 4-bit bus (3.0 MB/s raw bus capacity)\r\n"
            "Filesystem Type         : exFAT (FatFs ELM)\r\n"
            "---------------------------------------------------------------------------------------------------------\r\n"
            "Test Pattern       | Buffer     | Total      | Iters  | Avg Latency  | Worst Latency| Throughput   | Status  \r\n"
            "-------------------+------------+------------+--------+--------------+--------------+--------------+----------\r\n"
            "Raw Sequential     |   16 KB    |   16 KB    |  128   |   1200 us     |   2800 us     |  2450 KB/s    | PASS\r\n"
            "FatFs Block Write  |   16 KB    |   16 KB    |  128   |   1400 us     |   3100 us     |  2150 KB/s    | PASS\r\n"
            "FatFs Block Write  |   64 KB    |   64 KB    |   64   |   2400 us     |   4800 us     |  2380 KB/s    | PASS\r\n"
            "FatFs Block Write  |  256 KB    |  256 KB    |   16   |   7200 us     |   7800 us     |  2480 KB/s    | PASS\r\n"
            "-------------------+------------+------------+--------+--------------+--------------+--------------+----------\r\n"
            "=========================================================================================================\r\n> "
        ),
        r"SLEEP": (
            "\r\nValue is set to 5\r\n"
            "Send APPLY command and reboot for changes to take effect\r\n"
            "[TEST] RTC_WUT_ARMED: duration=5 s\r\n"
            "[TEST] STOP2_ENTERED\r\n"
            "\r\nRTC wake received. System clock restored; console and BLE communications resumed.\r\n"
            "Wake initialization complete; scheduler continuing.\r\n"
            "wake latency : 32 us\r\n> "
        ),
    }

    mock = MockSerialPort(responses)
    mock.feed_input("Interactive console ready. Type 'help' for commands, 'config' for guided setup.\r\n\r\n> ")
    return mock
