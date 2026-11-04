from __future__ import annotations

import csv
import enum
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


class ErrorSeverity(str, enum.Enum):
    WARNING = "WARNING"
    RECOVERABLE = "RECOVERABLE"
    CRITICAL = "CRITICAL"
    FATAL = "FATAL"


class ErrorSubsystem(str, enum.Enum):
    SDMMC = "SDMMC"
    FILESYSTEM = "FILESYSTEM"
    AUDIO = "AUDIO"
    MEMORY = "MEMORY"
    SYSTEM = "SYSTEM"


@dataclass
class StructuredError:
    timestamp_iso: str
    uptime_s: float
    session_id: int
    sample_rate: int
    subsystem: ErrorSubsystem
    severity: ErrorSeverity
    error_code: str
    normalized_signature: str
    raw_message: str
    context_before: str = ""
    context_after: str = ""
    recovered: bool = False
    recovery_attempted: bool = False
    recovery_successful: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_iso": self.timestamp_iso,
            "uptime_s": round(self.uptime_s, 3),
            "session_id": self.session_id,
            "sample_rate": self.sample_rate,
            "subsystem": self.subsystem.value,
            "severity": self.severity.value,
            "error_code": self.error_code,
            "normalized_signature": self.normalized_signature,
            "raw_message": self.raw_message,
            "recovered": self.recovered,
            "recovery_attempted": self.recovery_attempted,
            "recovery_successful": self.recovery_successful,
        }


@dataclass
class ErrorSignatureTracker:
    signature: str
    subsystem: ErrorSubsystem
    severity: ErrorSeverity
    total_occurrences: int = 0
    occurrences_before_recovery: int = 0
    occurrences_after_recovery: int = 0
    first_seen_uptime_s: float = 0.0
    last_seen_uptime_s: float = 0.0
    successful_power_cycles: int = 0
    failed_recovery_attempts: int = 0
    is_persistent: bool = False

    def record_occurrence(self, uptime_s: float, was_recovered: bool) -> None:
        self.total_occurrences += 1
        if self.first_seen_uptime_s == 0.0:
            self.first_seen_uptime_s = uptime_s
        self.last_seen_uptime_s = uptime_s

        if was_recovered:
            self.occurrences_after_recovery += 1
        else:
            self.occurrences_before_recovery += 1


@dataclass
class RecoveryEvent:
    event_id: str
    timestamp_iso: str
    uptime_s: float
    trigger_error_signature: str
    discharge_duration_s: float
    boot_stabilize_s: float
    devinfo_latency_ms: float
    status_verified: bool
    reconfigured: bool
    result: str  # SUCCESS, FAILED, ABORTED
    total_recovery_duration_s: float
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "event_id": self.event_id,
            "timestamp_iso": self.timestamp_iso,
            "uptime_s": round(self.uptime_s, 3),
            "trigger_error_signature": self.trigger_error_signature,
            "discharge_duration_s": round(self.discharge_duration_s, 2),
            "boot_stabilize_s": round(self.boot_stabilize_s, 2),
            "devinfo_latency_ms": round(self.devinfo_latency_ms, 1),
            "status_verified": self.status_verified,
            "reconfigured": self.reconfigured,
            "result": self.result,
            "total_recovery_duration_s": round(self.total_recovery_duration_s, 2),
            "notes": self.notes,
        }


@dataclass
class PowerTrendInterval:
    timestamp_s: float
    sample_count: int
    min_current_ma: float
    avg_current_ma: float
    max_current_ma: float
    peak_current_ma: float
    avg_voltage_v: float
    avg_power_mw: float
    interval_energy_mj: float
    accumulated_energy_mj: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_s": round(self.timestamp_s, 3),
            "sample_count": self.sample_count,
            "min_current_ma": round(self.min_current_ma, 4),
            "avg_current_ma": round(self.avg_current_ma, 4),
            "max_current_ma": round(self.max_current_ma, 4),
            "peak_current_ma": round(self.peak_current_ma, 4),
            "avg_voltage_v": round(self.avg_voltage_v, 3),
            "avg_power_mw": round(self.avg_power_mw, 3),
            "interval_energy_mj": round(self.interval_energy_mj, 2),
            "accumulated_energy_mj": round(self.accumulated_energy_mj, 2),
        }


@dataclass
class SessionTelemetry:
    session_id: int
    start_uptime_s: float
    end_uptime_s: float
    duration_s: float
    sample_rate: int
    bit_depth: int
    expected_samples: int
    acquired_samples: int
    processed_samples: int
    samples_written: int
    boundary_discarded: int
    lost_samples: int
    wav_verified: str
    active_write_throughput_kb_s: float
    worst_batch_latency_us: float
    avg_dsp_us: float
    worst_dsp_us: float
    adc_overruns: int
    dma_errors: int
    psram_full_events: int
    is_complete: bool

    def to_dict(self) -> Dict[str, Any]:
        return {
            "session_id": self.session_id,
            "start_uptime_s": round(self.start_uptime_s, 2),
            "end_uptime_s": round(self.end_uptime_s, 2),
            "duration_s": round(self.duration_s, 2),
            "sample_rate": self.sample_rate,
            "bit_depth": self.bit_depth,
            "expected_samples": self.expected_samples,
            "acquired_samples": self.acquired_samples,
            "processed_samples": self.processed_samples,
            "samples_written": self.samples_written,
            "boundary_discarded": self.boundary_discarded,
            "lost_samples": self.lost_samples,
            "wav_verified": self.wav_verified,
            "active_write_throughput_kb_s": round(self.active_write_throughput_kb_s, 1),
            "worst_batch_latency_us": round(self.worst_batch_latency_us, 1),
            "avg_dsp_us": round(self.avg_dsp_us, 1),
            "worst_dsp_us": round(self.worst_dsp_us, 1),
            "adc_overruns": self.adc_overruns,
            "dma_errors": self.dma_errors,
            "psram_full_events": self.psram_full_events,
            "is_complete": self.is_complete,
        }


class ErrorDetector:
    """Detects, classifies, and normalizes firmware and hardware errors from UART stream."""

    # Ordered detection rules: (pattern, subsystem, severity, code, signature_template)
    RULES: List[Tuple[re.Pattern, ErrorSubsystem, ErrorSeverity, str, str]] = [
        # RTOS and System Faults (FATAL)
        (re.compile(r"HardFault|Fault\s+handler|r0/a1:|EXC_RETURN", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_HARDFAULT", "SYSTEM_HARDFAULT"),
        (re.compile(r"UsageFault|UNDEFINSTR|NOCP|INVPC", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_USAGEFAULT", "SYSTEM_USAGEFAULT"),
        (re.compile(r"BusFault|PRECISERR|IBUSERR", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_BUSFAULT", "SYSTEM_BUSFAULT"),
        (re.compile(r"MemManage|IACCVIOL|DACCVIOL", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_MEMMANAGE", "SYSTEM_MEMMANAGE"),
        (re.compile(r"ASSERTION\s+FAIL|ASSERT\s+failed|k_panic|Kernel\s+panic", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_ASSERT", "SYSTEM_ASSERTION_FAIL"),
        (re.compile(r"Watchdog\s+reset|IWDG\s+reset|WWDG\s+reset", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_WATCHDOG", "SYSTEM_WATCHDOG_RESET"),
        (re.compile(r"Fatal\s+fault\s+in\s+thread|Thread\s+aborted", re.I), ErrorSubsystem.SYSTEM, ErrorSeverity.FATAL, "SYS_THREAD_FAIL", "SYSTEM_THREAD_FATAL"),
        (re.compile(r"Stack\s+overflow|stack\s+smashing", re.I), ErrorSubsystem.MEMORY, ErrorSeverity.FATAL, "MEM_STACK_OVERFLOW", "MEMORY_STACK_OVERFLOW"),

        # SDMMC Errors
        (re.compile(r"DATA_CRC_FAIL|SDMMC_STA_DCRCFAIL|CRC\s+failed|Data\s+CRC\s+error", re.I), ErrorSubsystem.SDMMC, ErrorSeverity.RECOVERABLE, "SDMMC_CRC", "SDMMC_DATA_CRC_FAIL"),
        (re.compile(r"DATA_TIMEOUT|SDMMC_STA_DTIMEOUT|Data\s+timeout", re.I), ErrorSubsystem.SDMMC, ErrorSeverity.RECOVERABLE, "SDMMC_DTIMEOUT", "SDMMC_DATA_TIMEOUT"),
        (re.compile(r"CMD_TIMEOUT|SDMMC_STA_CTIMEOUT|Command\s+timeout|cmd\s+timeout", re.I), ErrorSubsystem.SDMMC, ErrorSeverity.RECOVERABLE, "SDMMC_CTIMEOUT", "SDMMC_CMD_TIMEOUT"),
        (re.compile(r"SDMMC_STA_TXUNDERR|SDMMC_STA_RXOVERR|SDMMC\s+transfer\s+failed|transfer\s+error", re.I), ErrorSubsystem.SDMMC, ErrorSeverity.RECOVERABLE, "SDMMC_XFER_ERR", "SDMMC_TRANSFER_ERROR"),
        (re.compile(r"SDMMC\s+controller\s+not\s+idle|SDMMC\s+idle\s+check\s+timed\s+out|SDMMC\s+busy\s+timeout", re.I), ErrorSubsystem.SDMMC, ErrorSeverity.RECOVERABLE, "SDMMC_IDLE_TIMEOUT", "SDMMC_IDLE_TIMEOUT"),
        (re.compile(r"SDMMC\s+recovery\s+failed|Failed\s+to\s+reinitialize\s+SDMMC", re.I), ErrorSubsystem.SDMMC, ErrorSeverity.CRITICAL, "SDMMC_RECOVERY_FAIL", "SDMMC_RECOVERY_FAILED"),

        # Filesystem Errors
        (re.compile(r"f_mount\s+failed|Failed\s+to\s+mount\s+SD|disk_mount\s+error|fs_mount\s+error", re.I), ErrorSubsystem.FILESYSTEM, ErrorSeverity.CRITICAL, "FS_MOUNT_ERR", "FS_MOUNT_ERROR"),
        (re.compile(r"f_open\s+failed|Failed\s+to\s+open\s+WAV|file\s+creation\s+failed", re.I), ErrorSubsystem.FILESYSTEM, ErrorSeverity.RECOVERABLE, "FS_OPEN_ERR", "FS_FILE_CREATION_FAILED"),
        (re.compile(r"f_write\s+failed|fs_write\s+error|Disk\s+write\s+failure", re.I), ErrorSubsystem.FILESYSTEM, ErrorSeverity.RECOVERABLE, "FS_WRITE_ERR", "FS_WRITE_FAILED"),
        (re.compile(r"WAV\s+finalization\s+failed|Failed\s+to\s+close\s+WAV|WAV\s+header\s+update\s+failed", re.I), ErrorSubsystem.FILESYSTEM, ErrorSeverity.RECOVERABLE, "FS_WAV_FIN_ERR", "FS_WAV_FINALIZATION_FAILED"),
        (re.compile(r"Storage\s+recovery\s+failed|filesystem\s+remount\s+failed", re.I), ErrorSubsystem.FILESYSTEM, ErrorSeverity.CRITICAL, "FS_RECOVERY_FAIL", "FS_STORAGE_RECOVERY_FAILED"),

        # Memory (PSRAM)
        (re.compile(r"PSRAM\s+initialization\s+failed|PSRAM\s+self-test\s+failed", re.I), ErrorSubsystem.MEMORY, ErrorSeverity.CRITICAL, "MEM_PSRAM_INIT", "MEMORY_PSRAM_INIT_FAILED"),
        (re.compile(r"PSRAM\s+transaction\s+error|QSPI\s+error|PSRAM\s+read/write\s+mismatch", re.I), ErrorSubsystem.MEMORY, ErrorSeverity.CRITICAL, "MEM_PSRAM_TXN", "MEMORY_PSRAM_TRANSACTION_ERROR"),
        (re.compile(r"audio\s+block\s+rejected|PSRAM\s+Rejected\s+Audio\s+Blocks\s*:\s*([1-9]\d*)", re.I), ErrorSubsystem.MEMORY, ErrorSeverity.CRITICAL, "MEM_PSRAM_REJECTED", "MEMORY_PSRAM_REJECTED_BLOCKS"),
        (re.compile(r"Audio\s+queue\s+full|No\s+free\s+PSRAM\s+slots|Queue\s+exhausted", re.I), ErrorSubsystem.MEMORY, ErrorSeverity.CRITICAL, "MEM_QUEUE_FULL", "MEMORY_QUEUE_EXHAUSTED"),
        (re.compile(r"PSRAM\s+Full\s+Events\s*:\s*([1-9]\d*)|psram_full=([1-9]\d*)", re.I), ErrorSubsystem.MEMORY, ErrorSeverity.WARNING, "MEM_PSRAM_FULL", "MEMORY_PSRAM_FULL_EVENT"),

        # Audio and Acquisition
        (re.compile(r"Samples\s+lost\s+during\s+active\s+capture\s*:\s*([1-9]\d*)|lost\s+samples\s*:\s*([1-9]\d*)", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.CRITICAL, "AUDIO_SAMPLE_LOSS", "AUDIO_CAPTURE_SAMPLE_LOSS"),
        (re.compile(r"Sample\s+shortfall:\s*expected\s+(\d+)", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.CRITICAL, "AUDIO_SHORTFALL", "AUDIO_SAMPLE_SHORTFALL"),
        (re.compile(r"ADC\s+overrun|adc_ovr=([1-9]\d*)", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.WARNING, "AUDIO_ADC_OVR", "AUDIO_ADC_OVERRUN"),
        (re.compile(r"DMA\s+error|dma_err=([1-9]\d*)", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.WARNING, "AUDIO_DMA_ERR", "AUDIO_DMA_ERROR"),
        (re.compile(r"DSP\s+overrun|dsp_ovr=([1-9]\d*)", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.WARNING, "AUDIO_DSP_OVR", "AUDIO_DSP_OVERRUN"),
        (re.compile(r"DSP\s+deadline\s+missed", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.WARNING, "AUDIO_DSP_DEADLINE", "AUDIO_DSP_DEADLINE_MISSED"),
        (re.compile(r"Missed\s+DMA\s+callback", re.I), ErrorSubsystem.AUDIO, ErrorSeverity.WARNING, "AUDIO_DMA_CALLBACK", "AUDIO_DMA_CALLBACK_MISSED"),
    ]

    @classmethod
    def scan_line(
        cls,
        line: str,
        uptime_s: float,
        session_id: int,
        sample_rate: int,
        context_before: str = "",
    ) -> Optional[StructuredError]:
        """Scans a single line or segment of UART text for known error patterns."""
        for pattern, subsystem, severity, code, sig_tpl in cls.RULES:
            m = pattern.search(line)
            if m:
                # Normalize line to create deterministic signature
                sig = sig_tpl
                now_iso = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
                return StructuredError(
                    timestamp_iso=now_iso,
                    uptime_s=uptime_s,
                    session_id=session_id,
                    sample_rate=sample_rate,
                    subsystem=subsystem,
                    severity=severity,
                    error_code=code,
                    normalized_signature=sig,
                    raw_message=line.strip(),
                    context_before=context_before[-200:] if context_before else "",
                )
        return None


class AssessmentEngine:
    """Evaluates evidence to classify errors as likely hardware-related, likely firmware-related, or undetermined."""

    @staticmethod
    def assess(
        trackers: Dict[str, ErrorSignatureTracker],
        recoveries: List[RecoveryEvent],
        power_cycles_count: int,
    ) -> Tuple[str, str, List[str]]:
        """Returns (classification, rationale, evidence_points)."""
        evidence_points: List[str] = []

        # Analyze persistent errors
        persistent_sigs = [t for t in trackers.values() if t.is_persistent or t.occurrences_after_recovery > 2]
        fatal_sigs = [t for t in trackers.values() if t.severity == ErrorSeverity.FATAL]
        crc_sigs = [t for t in trackers.values() if "CRC" in t.signature]
        sw_fault_sigs = [t for t in trackers.values() if any(k in t.signature for k in ("ASSERTION", "USAGEFAULT", "MEMMANAGE"))]
        dma_starve_sigs = [t for t in trackers.values() if any(k in t.signature for k in ("FULL_EVENT", "DEADLINE", "OVERRUN"))]

        # 1. Check for clear firmware fault indicators
        if sw_fault_sigs or (fatal_sigs and not crc_sigs):
            evidence_points.append("Fatal software faults or assertion failures detected in RTOS kernel.")
            if sw_fault_sigs:
                evidence_points.append(f"Firmware exception signatures: {', '.join(t.signature for t in sw_fault_sigs)}")
            return (
                "Likely firmware-related",
                "Observed errors stem from software exception handlers, assertion traps, or kernel panic conditions.",
                evidence_points,
            )

        # 2. Check for persistent hardware / physical medium indicators
        if crc_sigs and any(t.successful_power_cycles > 0 and t.occurrences_after_recovery > 0 for t in crc_sigs):
            evidence_points.append("SDMMC DATA_CRC_FAIL persisted across cold power cycles and reinitialization.")
            evidence_points.append(f"Total power cycles performed: {power_cycles_count}.")
            return (
                "Likely hardware-related",
                "Repeated SDMMC hardware CRC errors persist across power-cycled controller resets, indicating electrical bus degradation, contact impedance, or card medium defect.",
                evidence_points,
            )

        # 3. Check for firmware scheduling / timing starvation
        if dma_starve_sigs and not crc_sigs:
            evidence_points.append("Buffer starvation or DSP deadline misses observed without physical bus errors.")
            return (
                "Likely firmware-related",
                "Pipeline latency spikes or priority inversions caused queue congestion without physical hardware bus errors.",
                evidence_points,
            )

        # 4. Default / inconclusive
        if trackers:
            evidence_points.append(f"Total error signatures observed: {len(trackers)}.")
            evidence_points.append(f"Total recovery attempts: {len(recoveries)}.")
            return (
                "Undetermined",
                "Observed errors could not be decisively attributed exclusively to hardware or firmware within available diagnostic markers.",
                evidence_points,
            )

        return (
            "Undetermined",
            "No errors observed during stability run; system operated within nominal baseline parameters.",
            ["Zero error occurrences detected across test duration."],
        )

