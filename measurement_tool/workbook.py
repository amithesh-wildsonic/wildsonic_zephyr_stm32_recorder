from __future__ import annotations

import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import openpyxl
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

from .models import TestSession


# Style definitions
HEADER_FILL = PatternFill(start_color="1F4E79", end_color="1F4E79", fill_type="solid")
HEADER_FONT = Font(name="Calibri", size=11, bold=True, color="FFFFFF")
SUBHEADER_FILL = PatternFill(start_color="D9E1F2", end_color="D9E1F2", fill_type="solid")
SUBHEADER_FONT = Font(name="Calibri", size=10, bold=True, color="000000")
REGULAR_FONT = Font(name="Calibri", size=10, color="000000")
BOLD_FONT = Font(name="Calibri", size=10, bold=True, color="000000")
PASS_FILL = PatternFill(start_color="E2EFDA", end_color="E2EFDA", fill_type="solid")
PENDING_FILL = PatternFill(start_color="FFF2CC", end_color="FFF2CC", fill_type="solid")
FAIL_FILL = PatternFill(start_color="FCE4D6", end_color="FCE4D6", fill_type="solid")

THIN_BORDER = Border(
    left=Side(style="thin", color="D9D9D9"),
    right=Side(style="thin", color="D9D9D9"),
    top=Side(style="thin", color="D9D9D9"),
    bottom=Side(style="thin", color="D9D9D9"),
)


def backup_workbook(path: str | Path) -> Optional[Path]:
    source = Path(path)
    if not source.exists():
        return None
    backup_name = f"{source.stem}_{datetime.now():%Y%m%d_%H%M%S}.bak{source.suffix}"
    destination = source.with_name(backup_name)
    shutil.copy2(source, destination)
    return destination


def _apply_row_style(ws, row_idx: int, font=REGULAR_FONT, fill=None, alignment=None, border=THIN_BORDER):
    for cell in ws[row_idx]:
        if font:
            cell.font = font
        if fill:
            cell.fill = fill
        if alignment:
            cell.alignment = alignment
        if border:
            cell.border = border


def _auto_fit_columns(ws, max_cols: int = 15):
    for col_idx in range(1, min(ws.max_column + 1, max_cols + 1)):
        col_letter = get_column_letter(col_idx)
        max_len = 0
        for row in range(1, ws.max_row + 1):
            val = ws.cell(row, col_idx).value
            if val is not None:
                max_len = max(max_len, len(str(val)))
        ws.column_dimensions[col_letter].width = max(12, min(max_len + 4, 50))


class ComparisonWorkbookManager:
    """Manages the comparison workbook. Result cells start blank and are populated only from measured evidence."""

    REQUIRED_SHEETS = [
        "Start Here",
        "Test Setup",
        "Executive Summary",
        "Hardware Config",
        "Comparison Summary",
        "Power Consumption",
        "Power & Sleep",
        "Sleep Current & RTC",
        "Wake & RTC",
        "Recording Perf & Integrity",
        "Recording Integrity",
        "SD Card Performance",
        "SD Card I-O",
        "Audio ADC Performance",
        "Audio & ADC",
        "PSRAM Performance",
        "RTOS & Memory",
        "RTOS & Resources",
        "Interfaces & Reliability",
        "Config Verification",
        "Stability Test",
        "Execution Log",
        "Test Log",
        "Evidence & References",
        "STM32 Measurements",  # Legacy / individual capture sheet
    ]

    def __init__(self, workbook_path: str | Path = "data/reports/stm32_vs_max78000.xlsx") -> None:
        self.path = Path(workbook_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.workbook = self._load_or_create()

    def _load_or_create(self) -> openpyxl.Workbook:
        if self.path.exists():
            import io
            with open(self.path, "rb") as f:
                in_mem = io.BytesIO(f.read())
            wb = openpyxl.load_workbook(in_mem, data_only=False)
            needs_save = False
            for name in self.REQUIRED_SHEETS:
                if name not in wb.sheetnames:
                    wb.create_sheet(name)
                    self._init_sheet_by_name(wb, name)
                    needs_save = True
            if needs_save:
                self._safe_save(wb)
            return wb
        wb = openpyxl.Workbook()
        default_sheet = wb.active
        default_sheet.title = "Start Here"
        for name in self.REQUIRED_SHEETS:
            if name not in wb.sheetnames:
                wb.create_sheet(name)
        self._init_all_sheets(wb)
        self._safe_save(wb)
        return wb

    def _safe_save(self, wb: openpyxl.Workbook) -> Path:
        try:
            wb.save(self.path)
            return self.path
        except PermissionError:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            fallback = self.path.with_name(f"{self.path.stem}_updated_{stamp}{self.path.suffix}")
            wb.save(fallback)
            print(f"[WARNING] Primary workbook locked by spreadsheet viewer. Saved copy to: {fallback.name}")
            return fallback

    def _init_sheet_by_name(self, wb: openpyxl.Workbook, name: str) -> None:
        init_dispatch = {
            "Start Here": self._populate_start_here,
            "Test Setup": self._populate_test_setup,
            "Executive Summary": self._populate_executive_summary,
            "Hardware Config": self._populate_hardware_configuration,
            "Comparison Summary": self._populate_comparison_summary,
            "Power Consumption": self._populate_power_consumption,
            "Power & Sleep": self._populate_power_sleep,
            "Sleep Current & RTC": self._populate_sleep_current_rtc,
            "Wake & RTC": self._populate_wake_rtc,
            "Recording Perf & Integrity": self._populate_recording_performance_integrity,
            "Recording Integrity": self._populate_recording_integrity,
            "SD Card Performance": self._populate_sd_card_performance_comparison,
            "SD Card I-O": self._populate_sd_card_io,
            "Audio ADC Performance": self._populate_audio_adc_performance,
            "Audio & ADC": self._populate_audio_adc,
            "PSRAM Performance": self._populate_psram_performance,
            "RTOS & Memory": self._populate_rtos_memory_resources,
            "RTOS & Resources": self._populate_rtos_resources,
            "Interfaces & Reliability": self._populate_interfaces_reliability,
            "Config Verification": self._populate_configuration_verification,
            "Stability Test": self._populate_stability_test_headers,
            "Execution Log": self._populate_hardware_test_execution_log,
            "Test Log": self._populate_test_log_headers,
            "Evidence & References": self._populate_evidence_references,
            "STM32 Measurements": self._populate_stm32_measurements_headers,
        }
        if name in init_dispatch:
            init_dispatch[name](wb[name])

    def _init_all_sheets(self, wb: openpyxl.Workbook) -> None:
        self._populate_start_here(wb["Start Here"])
        self._populate_test_setup(wb["Test Setup"])
        self._populate_executive_summary(wb["Executive Summary"])
        self._populate_hardware_configuration(wb["Hardware Config"])
        self._populate_comparison_summary(wb["Comparison Summary"])
        self._populate_power_consumption(wb["Power Consumption"])
        self._populate_power_sleep(wb["Power & Sleep"])
        self._populate_sleep_current_rtc(wb["Sleep Current & RTC"])
        self._populate_wake_rtc(wb["Wake & RTC"])
        self._populate_recording_performance_integrity(wb["Recording Perf & Integrity"])
        self._populate_recording_integrity(wb["Recording Integrity"])
        self._populate_sd_card_performance_comparison(wb["SD Card Performance"])
        self._populate_sd_card_io(wb["SD Card I-O"])
        self._populate_audio_adc_performance(wb["Audio ADC Performance"])
        self._populate_audio_adc(wb["Audio & ADC"])
        self._populate_psram_performance(wb["PSRAM Performance"])
        self._populate_rtos_memory_resources(wb["RTOS & Memory"])
        self._populate_rtos_resources(wb["RTOS & Resources"])
        self._populate_interfaces_reliability(wb["Interfaces & Reliability"])
        self._populate_configuration_verification(wb["Config Verification"])
        self._populate_stability_test_headers(wb["Stability Test"])
        self._populate_hardware_test_execution_log(wb["Execution Log"])
        self._populate_test_log_headers(wb["Test Log"])
        self._populate_evidence_references(wb["Evidence & References"])
        self._populate_stm32_measurements_headers(wb["STM32 Measurements"])

    def _populate_blank_reference_sheet(self, ws, headers: List[str], row_templates: Optional[List[List[Any]]] = None) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(headers)
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)
        rows = row_templates or []
        for row in rows:
            ws.append(row)
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_executive_summary(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Section", "Metric", "MAX78000 Reference", "STM32 Measured", "Status", "Evidence / Note"],
            [
                ["Executive Summary", "Overall platform readiness", None, None, "REFERENCE", "Populate from verified evidence only"],
                ["Power", "Recording current and energy", None, None, "REFERENCE", "Use verified PPK2 phase measurements only"],
                ["Sleep", "STOP2 verification status", None, None, "UNVERIFIED", "Do not promote unverified sleep current to PASS"],
            ],
        )

    def _populate_hardware_configuration(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Parameter", "MAX78000 Value", "STM32 Value", "Measurement Method", "Status", "Evidence"],
            [
                ["Board / MCU", None, None, "Hardware banner / datasheet", "REFERENCE", "Documented platform setup"],
                ["Firmware commit", None, None, "UART banner / build metadata", "REFERENCE", "SHA recorded from hardware evidence"],
                ["Supply / instrumentation", "3.3 V rail / PMIC", "3.3 V rail / PPK2", "Bench instrumentation", "REFERENCE", "Measured at test fixture"],
            ],
        )

    def _populate_power_consumption(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Measurement", "MAX78000 Reference", "STM32 Measured", "Units", "Status", "Evidence"],
            [
                ["Active recording current", None, None, "mA", "NOT RUN", "Awaiting verified phase capture"],
                ["Active recording power", None, None, "mW", "NOT RUN", "Awaiting verified phase capture"],
                ["Average energy", None, None, "mJ", "NOT RUN", "Awaiting verified energy calculation"],
            ],
        )

    def _populate_sleep_current_rtc(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Sleep Metric", "Expected Value", "MAX78000", "STM32 Measured", "Status", "Verification / Note"],
            [
                ["Expected STOP2 current", "~6 mA", None, None, "EXPECTED", "Expected value, not measured"],
                ["HW-002 sleep current", "Invalid", None, "52.74 mA", "INVALID", "Recorder remained active; not a valid sleep measurement"],
                ["Verified STOP2 current", None, None, None, "UNVERIFIED", "Blank until actual STOP2 evidence exists"],
                ["RTC wake verification", None, None, None, "UNVERIFIED", "Must include wake event and restoration checks"],
            ],
        )

    def _populate_recording_performance_integrity(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Test Condition", "Expected Samples", "Acquired", "Written", "Lost", "WAV Verified", "Status", "Evidence"],
            [
                ["192 kHz / 10 s / 16-bit", None, None, None, None, None, "NOT RUN", "Awaiting raw log + JSON evidence"],
                ["96 kHz / 10 s / 16-bit", None, None, None, None, None, "NOT RUN", "Awaiting raw log + JSON evidence"],
            ],
        )

    def _populate_sd_card_performance_comparison(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Operation", "Chunk Size", "MAX78000 Throughput", "STM32 Throughput", "Min", "Avg", "Max", "P95", "Worst Latency", "Errors", "Data Integrity", "Status", "Evidence"],
            [
                ["Raw sequential read", "16 KB", None, None, None, None, None, None, None, None, None, "NOT RUN", "No matched evidence"],
                ["Raw sequential write", "128 KB", None, None, None, None, None, None, None, None, None, "NOT RUN", "No matched evidence"],
                ["FatFs write", "256 KB", None, None, None, None, None, None, None, None, None, "NOT RUN", "No matched evidence"],
            ],
        )

    def _populate_audio_adc_performance(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Audio / ADC Metric", "MAX78000 Reference", "STM32 Measured", "Delta / Comparison", "Status", "Measurement Method"],
            [
                ["Sample rate error", None, None, None, "NOT RUN", "Measured from raw capture and log"],
                ["ADC ENOB / jitter", None, None, None, "NOT RUN", "Measured from sample timing and waveform"],
            ],
        )

    def _populate_psram_performance(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["PSRAM Metric", "MAX78000 Reference", "STM32 Measured", "Units", "Status", "Evidence"],
            [
                ["Write bandwidth", None, None, "MB/s", "NOT RUN", "No verified PSRAM evidence"],
                ["Read bandwidth", None, None, "MB/s", "NOT RUN", "No verified PSRAM evidence"],
            ],
        )

    def _populate_rtos_memory_resources(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Resource", "MAX78000", "STM32", "Utilization / Capacity", "Status", "Evidence"],
            [
                ["CPU avg / max utilization", None, None, None, "NOT RUN", "Awaiting system traces"],
                ["Thread runtime and stack headroom", None, None, None, "NOT RUN", "Awaiting runtime log"],
            ],
        )

    def _populate_configuration_verification(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Setting", "Request", "Command Accepted", "Persisted", "Staged", "Applied", "Active", "Status", "Evidence"],
            [
                ["SAMF", None, None, None, None, None, None, "UNVERIFIED", "Configuration verification required"],
                ["BTPS", None, None, None, None, None, None, "UNVERIFIED", "Configuration verification required"],
                ["RECT", None, None, None, None, None, None, "UNVERIFIED", "Configuration verification required"],
            ],
        )

    def _populate_hardware_test_execution_log(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Timestamp", "Test ID", "Command / Phase", "Result", "Status", "Evidence / Note"],
            [
                [None, None, "Benchmark sequence start", "NOT RUN", "NOT RUN", "No hardware test executed in this workbook snapshot"],
            ],
        )

    def _populate_evidence_references(self, ws) -> None:
        self._populate_blank_reference_sheet(
            ws,
            ["Test ID", "Firmware SHA", "Build Config", "Raw UART Log", "PPK2 CSV", "Parsed JSON", "Timestamp", "Hardware Setup", "Status"],
            [
                [None, None, None, None, None, None, None, None, "REFERENCE"],
            ],
        )

    def _populate_start_here(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["WildSonic Acoustic Recorder – Platform Comparison & Benchmarking"])
        ws["A1"].font = Font(name="Calibri", size=14, bold=True, color="1F4E79")
        ws.append(["Hardware Platforms: STMicroelectronics STM32L476RG vs Maxim Integrated MAX78000"])
        ws["A2"].font = Font(name="Calibri", size=11, italic=True)
        ws.append([])

        headers = ["Section", "Sheet Name", "Description"]
        ws.append(headers)
        _apply_row_style(ws, 4, font=HEADER_FONT, fill=HEADER_FILL)

        sections = [
            ("Overview & Setup", "Test Setup", "Hardware architectures, clock trees, power rails, firmware revisions"),
            ("Executive Comparison", "Comparison Summary", "Side-by-side key metrics comparison with deltas and status"),
            ("Power & Energy", "Power & Sleep", "Current, voltage, power, and energy consumption across 7 operating states"),
            ("Sleep & Wakeup", "Wake & RTC", "Stop 2 entry, wakeup latency, RTC drift, clock recovery time"),
            ("Acoustic Reliability", "Recording Integrity", "Sample drops, ADC overruns, DMA errors, WAV header integrity"),
            ("Storage Throughput", "SD Card I-O", "4-bit SDMMC throughput, latency, batch write stability, recovery"),
            ("Audio Acquisition", "Audio & ADC", "ADC ENOB, sampling rate jitter, SNR, THD, analog front-end response"),
            ("System Resources", "RTOS & Resources", "Zephyr thread usage, SRAM utilization, Flash footprint, PSRAM bandwidth"),
            ("Hardware Interfaces", "Interfaces & Reliability", "Interface error rates, long-duration stability, recovery rates"),
            ("Stress & Stability", "Stability Test", "Overnight soak tests, automatic recovery history, power cycles, fault classifications"),
            ("Execution History", "Test Log", "Chronological audit trail of all automated hardware and sample test runs"),
        ]

        for sec, sheet, desc in sections:
            ws.append([sec, sheet, desc])
            _apply_row_style(ws, ws.max_row)

        _auto_fit_columns(ws)

    def _populate_test_setup(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Parameter / Dimension", "Maxim MAX78000 (Reference)", "STM32L476RG (Zephyr Target)", "Notes / Method"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        data = [
            ("Core Architecture", "Dual-core: Cortex-M4F @ 100MHz + RISC-V @ 60MHz", "Single Cortex-M4F @ 72MHz (PLL HSI)", "ARMv7E-M with FPU"),
            ("RTOS / Framework", "Custom Bare-metal / FreeRTOS-like task runner", "Zephyr RTOS v4.5.0-rc2", "Cooperative & preemptive threads"),
            ("Audio Acquisition", "I2S digital microphone / MAX9867 codec", "On-chip 12-bit SAR ADC + 8x/16x OVS (TIM6 TRGO DMA)", "15-bit ENOB via oversampling"),
            ("Intermediate Staging", "On-chip 256 KB OCM circular buffer", "4 MB ISSI IS66WVS4M8 QUADSPI PSRAM (3 MB audio staging)", "12 x 256 KB ring, 8s @ 192 kHz"),
            ("Persistent Storage", "SDHC via SPI/SDHC (FatFs)", "4-bit native SDMMC1 @ 6 MHz (FatFs ELM /SD:)", "3.0 MB/s raw bus capacity"),
            ("Real-Time Clock", "Dual: internal RTC + external MCP7940N I2C RTC", "On-chip hardware RTC calendar + Wakeup Timer (LSE 32.768 kHz)", "BCD shadow registers, VBAT"),
            ("Low-Power Sleep", "Power latch (GPIO 1.7) cuts SoC VDD (~1.8-2.5 uA)", "STM32 Stop 2 deep sleep with SRAM1 retention (~2.6-3.2 uA)", "Full RAM & RTC preserved"),
            ("Flash Memory", "512 KB Single-Bank Flash", "1024 KB Dual-Bank Flash", "Flash wear-leveling config"),
            ("Internal SRAM", "128 KB System SRAM", "128 KB (96 KB SRAM1 + 32 KB SRAM2)", "Zero-wait state execution"),
            ("Supply Voltage", "3.3V (TI BQ25700A PMIC / BQ34110 Fuel Gauge)", "3.3V Regulated (NUCLEO-L476RG / Bench Supply)", "Monitored via Nordic PPK2"),
            ("Firmware Baseline", "WildSonic_Firmware-feature-dev (v1.2.0)", "wildsonic_zephyr_stm32_recorder (v1.2.0)", "Production SHA verified"),
        ]

        for row in data:
            ws.append([row[0], None, row[2], row[3]])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_comparison_summary(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Metric Category", "Specific Metric", "MAX78000 Baseline", "STM32 Min", "STM32 Avg", "STM32 Max", "Delta / Comparison", "Status", "Notes"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for category, metric in [("CPU", "Average CPU utilization"), ("CPU", "Maximum CPU utilization"), ("RTOS", "Thread runtime / stack headroom"), ("RTOS", "Queue and memory-slab high-water"), ("Power", "Recording average / peak current"), ("Power", "Verified Stop2 sleep current"), ("RTC", "RTC timing error / drift"), ("Wake", "Wake-to-firmware-ready"), ("Recording", "Dropped audio blocks"), ("Storage", "SD throughput / worst batch latency"), ("Reliability", "Unexpected resets / faults"), ("Reliability", "Overall test result")]:
            ws.append([category, metric, None, None, None, None, None, None, "Populate from verified test evidence"])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_power_sleep(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append([
            "Operating State / Phase", "MAX78000 Current (mA)", "MAX78000 Power (mW)",
            "STM32 Min (mA)", "STM32 Avg (mA)", "STM32 Max (mA)", "STM32 Peak (mA)",
            "STM32 Power (mW)", "STM32 Energy (mJ)", "Duration (s)", "Notes / Conditions"
        ])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for phase in ["Idle", "Recording", "SD write", "Stop2 sleep", "RTC wake", "Peripheral recovery"]:
            ws.append([phase] + [None] * 9 + ["PPK2 phase capture required"])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_wake_rtc(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Parameter", "MAX78000 Baseline", "STM32 Min", "STM32 Avg", "STM32 Max", "Delta", "Notes / Verification"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for metric in ["Requested sleep duration", "Actual sleep duration", "RTC timing error", "RTC drift", "Wake source", "Wake-to-firmware-ready", "Wake-to-recording-ready", "Successful wake cycles", "Missed wake cycles", "Verified Stop2 entries"]:
            ws.append([metric, None, None, None, None, None, "Awaiting verified RTC/UART evidence"])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_recording_integrity(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Sample Rate", "Bit Depth", "Duration", "Expected Samples", "Acquired Samples", "Written Samples", "Lost Samples", "ADC Overruns", "DMA Errors", "WAV Verified", "Status"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for rate in (192000, 96000, 48000):
            ws.append([f"{rate} Hz", "16-bit", None, None, None, None, None, None, None, None, None])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_sd_card_io(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Test Condition / Chunk Size", "MAX78000 Throughput", "STM32 Min (KB/s)", "STM32 Avg (KB/s)", "STM32 Max (KB/s)", "Worst Latency (us)", "Error Count", "Recovery Status"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for item in ["8 KB write", "16 KB write", "32 KB write", "64 KB write", "128 KB write", "Continuous recording"]:
            ws.append([item] + [None] * 7)
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_audio_adc(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Acoustic Parameter", "MAX78000 (MAX9867 Codec)", "STM32L476RG (ADC1 8x/16x OVS)", "Delta / Comparison", "Notes / Measurement Method"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for metric in ["Sample rate", "Sample-rate error", "ADC overruns", "DMA errors"]:
            ws.append([metric, None, None, None, "Measured values only; no analog/noise tests added"])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_rtos_resources(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Resource / Metric", "MAX78000 Baseline", "STM32L476RG (Zephyr)", "Utilization / Capacity", "Status"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for metric in ["CPU average / maximum", "Thread runtime", "Scheduler response latency", "Minimum stack headroom", "Queue high-water", "Memory-slab high-water", "Thread faults"]:
            ws.append([metric, None, None, None, None])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_interfaces_reliability(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        ws.append(["Interface / Subsystem", "Clock Rate / Bus Width", "Error Rate", "Continuous Duration", "Recovery Rate", "Status"])
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)

        # Intentionally blank: populate only from verified hardware evidence.
        for metric in ["SDMMC errors", "PSRAM errors", "ADC/DMA errors", "Unexpected resets", "Watchdog resets", "Recording success rate", "Wake success rate", "Overall reliability result"]:
            ws.append([metric, None, None, None, None, None])
            _apply_row_style(ws, ws.max_row)
        _auto_fit_columns(ws)

    def _populate_stability_test_headers(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        headers = [
            "Test ID", "Sample Rate (Hz)", "Bit Depth", "Target Duration (h)",
            "Actual Uptime (s)", "Sessions", "Total Rec Time (s)", "Total Errors",
            "Error Breakdown", "Power Cycles", "Successful Recoveries", "Failed Recoveries",
            "Incomplete WAVs", "DEVINFO Latency Avg (ms)", "SD Write Throughput Avg (KB/s)",
            "SD Worst Latency (us)", "DSP Avg (us)", "Power Avg (mW)", "Energy Consumed (mJ)",
            "Hardware vs Firmware Assessment", "Status", "Notes / Summary"
        ]
        ws.append(headers)
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)
        _auto_fit_columns(ws)

    def _populate_test_log_headers(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        headers = [
            "Test ID", "Timestamp", "Category", "Configuration", "Status",
            "Evidence Type", "Firmware Commit", "Raw Log Path", "PPK2 CSV Path",
            "Parsed JSON Path", "Notes / Summary"
        ]
        ws.append(headers)
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)
        _auto_fit_columns(ws)

    def _populate_stm32_measurements_headers(self, ws) -> None:
        ws.views.sheetView[0].showGridLines = True
        headers = [
            "Test ID", "Board", "Firmware Commit", "Build Type", "Test Category",
            "Test Configuration", "Date/Time", "Raw Log Path", "PPK2 CSV Path",
            "Parsed Result", "Status", "Notes", "Evidence References",
            "Missing Measurement Warning"
        ]
        ws.append(headers)
        _apply_row_style(ws, 1, font=HEADER_FONT, fill=HEADER_FILL)
        _auto_fit_columns(ws)

    def add_or_update_session(self, session: TestSession) -> Dict[str, Any]:
        """Updates STM32 Measurements, Test Log, and summary comparison metrics."""
        backup_workbook(self.path)

        # 1. Update STM32 Measurements (Legacy format for tests)
        ws_m = self.workbook["STM32 Measurements"]
        row_idx = None
        for row in ws_m.iter_rows(min_row=2, values_only=False):
            if row[0].value == session.test_id:
                row_idx = row[0].row
                break
        if row_idx is None:
            row_idx = ws_m.max_row + 1

        note_text = f"Evidence Type: {session.evidence_type}"
        if session.evidence_type == "SAMPLE":
            note_text += " | SAMPLE DATA"
        if session.sample_data:
            note_text += " | sample PPK2 values retained separately from hardware metrics"

        warnings = session.parsed_result.get("warnings", []) if isinstance(session.parsed_result, dict) else []
        warning_str = "; ".join(warnings) if warnings else ""

        m_vals = [
            session.test_id,
            session.board,
            session.firmware_commit,
            session.build_type,
            session.category,
            session.configuration,
            session.date_time,
            session.raw_log_path,
            session.ppk2_csv_path,
            str(session.parsed_result),
            session.status.lower(),
            note_text,
            f"{session.raw_log_path}; {session.ppk2_csv_path}".strip("; "),
            warning_str,
        ]

        for col_idx, val in enumerate(m_vals, start=1):
            ws_m.cell(row=row_idx, column=col_idx, value=val)
            _apply_row_style(ws_m, row_idx)

        # 2. Update Test Log sheet
        ws_log = self.workbook["Test Log"]
        log_row_idx = None
        for row in ws_log.iter_rows(min_row=2, values_only=False):
            if row[0].value == session.test_id:
                log_row_idx = row[0].row
                break
        if log_row_idx is None:
            log_row_idx = ws_log.max_row + 1

        parsed_json_ref = (
            session.parsed_result.get("parsed_json_path")
            if isinstance(session.parsed_result, dict) and session.parsed_result.get("parsed_json_path")
            else f"data/stm32/parsed/{session.test_id}.json"
        )
        failure_reasons = (
            session.parsed_result.get("failure_reasons", [])
            if isinstance(session.parsed_result, dict)
            else []
        )
        summary_note = note_text
        if failure_reasons:
            summary_note += " | Failure Reasons: " + "; ".join(failure_reasons)

        log_vals = [
            session.test_id,
            session.date_time,
            session.category,
            session.configuration,
            session.status.upper(),
            session.evidence_type.upper(),
            session.firmware_commit,
            session.raw_log_path,
            session.ppk2_csv_path,
            parsed_json_ref,
            summary_note,
        ]

        for col_idx, val in enumerate(log_vals, start=1):
            cell = ws_log.cell(row=log_row_idx, column=col_idx, value=val)
            _apply_row_style(ws_log, log_row_idx)
            if col_idx == 5:
                cell.fill = PASS_FILL if session.status.lower() == "pass" else (FAIL_FILL if session.status.lower() == "fail" else PENDING_FILL)

        # Save workbook
        saved_path = self._safe_save(self.workbook)
        return {
            "workbook": str(saved_path),
            "sheet": "STM32 Measurements",
            "row": row_idx,
            "status": session.status,
            "evidence_type": session.evidence_type,
            "missing_measurement_warning": warning_str,
        }

    def update_ppk2_phase_measurements(
        self,
        phases: Dict[str, Any],
        stop2_verified: Optional[bool] = None,
        notes_overrides: Optional[Dict[str, str]] = None,
    ) -> None:
        """Updates Power & Sleep comparison sheet with measured STM32 PPK2 phase telemetry."""
        if stop2_verified is None:
            stop2_verified = False
        ws_pwr = self.workbook["Power & Sleep"]
        phase_map = {
            "Idle / Baseline": ["Phase B: Active Idle", "idle", "idle-power"],
            "Active Recording (192 kHz)": ["Phase C: Recording", "recording"],
            "Active SD Write (Batch)": ["Phase D: SD Write", "sd-write", "sd-performance"],
            "Active SD Read (Verify)": ["Phase E: SD Read", "sd-read"],
            "Stop 2 Deep Sleep": ["Phase F: Stop2 Sleep", "sleep-wake", "stop2"],
            "RTC Wake-up Transition": ["Phase G: RTC Wakeup", "rtc-wake", "wake"],
        }
        notes_overrides = notes_overrides or {}

        for row in ws_pwr.iter_rows(min_row=2, values_only=False):
            state_name = str(row[0].value) if row[0].value else ""
            for target_state, alias_list in phase_map.items():
                if target_state.lower() in state_name.lower():
                    matched_p = None
                    for alias in alias_list:
                        for p_key, p_val in phases.items():
                            if alias.lower() in p_key.lower():
                                matched_p = p_val
                                break
                        if matched_p:
                            break

                    if matched_p:
                        if isinstance(matched_p, dict):
                            min_ma = matched_p.get("min_current_ma", 0.0)
                            avg_ma = matched_p.get("avg_current_ma", 0.0)
                            max_ma = matched_p.get("max_current_ma", 0.0)
                            power_mw = matched_p.get("avg_power_mw", 0.0)
                            energy_mj = matched_p.get("total_energy_mj", 0.0)
                            dur_s = matched_p.get("duration_s", 0.0)
                            timing_ver = matched_p.get("timing_verified", True)
                        else:
                            min_ma = getattr(matched_p, "min_current_ma", 0.0)
                            avg_ma = getattr(matched_p, "avg_current_ma", 0.0)
                            max_ma = getattr(matched_p, "max_current_ma", 0.0)
                            power_mw = getattr(matched_p, "avg_power_mw", 0.0)
                            energy_mj = getattr(matched_p, "total_energy_mj", 0.0)
                            dur_s = getattr(matched_p, "duration_s", 0.0)
                            timing_ver = getattr(matched_p, "timing_verified", True)

                        row[3].value = round(min_ma, 4)
                        row[4].value = round(avg_ma, 4)
                        row[5].value = round(max_ma, 4)
                        row[6].value = round(max_ma, 4)
                        row[7].value = round(power_mw, 3)
                        row[8].value = round(energy_mj, 2)
                        row[9].value = round(dur_s, 2)

                        if target_state in notes_overrides:
                            row[10].value = notes_overrides[target_state]
                        elif target_state == "Stop 2 Deep Sleep" and not stop2_verified:
                            row[10].value = "UNVERIFIED: Stop 2 entry not detected; MCU remained active"
                        elif target_state == "RTC Wake-up Transition" and not stop2_verified:
                            row[10].value = "UNVERIFIED: No RTC wake event observed"
                        elif target_state == "Active Recording (192 kHz)" and not timing_ver:
                            row[10].value = "ADC1 + TIM6 DMA + PSRAM write [Timing misaligned: mid-session start]"

        self._safe_save(self.workbook)

    def update_recording_integrity_result(
        self,
        rate_hz: int,
        bit_depth: int,
        duration_s: int,
        expected: int,
        acquired: int,
        written: int,
        lost: int = 0,
        adc_ovr: int = 0,
        dma_err: int = 0,
        wav_verified: str = "PASS",
        status: str = "PASS",
    ) -> None:
        """Updates matching row in Recording Integrity worksheet."""
        ws = self.workbook["Recording Integrity"]
        target_rate_str = f"{rate_hz} Hz"
        target_dur_str = f"{duration_s} s"

        for row in ws.iter_rows(min_row=2, values_only=False):
            r_rate = str(row[0].value) if row[0].value else ""
            r_dur = str(row[2].value) if row[2].value else ""
            if target_rate_str.lower() in r_rate.lower() and target_dur_str.lower() in r_dur.lower():
                row[3].value = expected
                row[4].value = acquired
                row[5].value = written
                row[6].value = lost
                row[7].value = adc_ovr
                row[8].value = dma_err
                row[9].value = wav_verified.upper()
                row[10].value = status.upper()
                row[10].fill = PASS_FILL if status.upper() == "PASS" else FAIL_FILL
                break
        self._safe_save(self.workbook)

    def update_sd_io_result(
        self,
        avg_throughput_kb_s: float,
        worst_latency_us: float,
        error_count: int = 0,
    ) -> None:
        """Updates SD Card I-O worksheet with measured throughput and latency."""
        ws = self.workbook["SD Card I-O"]
        for row in ws.iter_rows(min_row=2, values_only=False):
            cond = str(row[0].value) if row[0].value else ""
            if "256 KB (Batch)" in cond or "Batch" in cond:
                row[3].value = round(avg_throughput_kb_s, 1)
                row[5].value = round(worst_latency_us, 1)
                row[6].value = error_count
                break
        self._safe_save(self.workbook)

    def update_wake_rtc_result(self, stop2_verified: bool, wake_latency_us: Optional[float] = None) -> None:
        """Updates Wake & RTC worksheet reflecting verified or unverified sleep events."""
        ws = self.workbook["Wake & RTC"]
        for row in ws.iter_rows(min_row=2, values_only=False):
            param = str(row[0].value) if row[0].value else ""
            if "Wakeup Latency" in param:
                if not stop2_verified:
                    row[3].value = "UNVERIFIED"
                    row[6].value = "UNVERIFIED: No Stop 2 wake event in test run"
                elif wake_latency_us is not None:
                    row[3].value = f"{wake_latency_us:.1f} us"
            elif "Stop 2 Sleep Entry Rejection" in param and not stop2_verified:
                row[6].value = "UNVERIFIED: Scheduler continued running next session"
        self._safe_save(self.workbook)

    def update_stability_test_results(self, d: Dict[str, Any]) -> None:
        """Updates or appends a row in the dedicated Stability Test worksheet."""
        if "Stability Test" not in self.workbook.sheetnames:
            ws = self.workbook.create_sheet("Stability Test")
            self._populate_stability_test_headers(ws)
        ws = self.workbook["Stability Test"]

        row_idx = None
        for row in ws.iter_rows(min_row=2, values_only=False):
            if row[0].value == d["test_id"]:
                row_idx = row[0].row
                break
        if row_idx is None:
            row_idx = ws.max_row + 1

        assessment = d.get("hardware_vs_firmware_assessment", {})
        assess_str = f"{assessment.get('classification', 'Undetermined')}: {assessment.get('rationale', '')}"

        row_vals = [
            d["test_id"],
            d.get("sample_rate", 96000),
            d.get("bit_depth", 16),
            round(d.get("target_duration_s", 0) / 3600.0, 2),
            round(d.get("actual_uptime_s", 0), 1),
            d.get("recording_sessions_count", 0),
            round(d.get("total_recording_time_s", 0), 1),
            d.get("total_errors_count", 0),
            f"{d.get('unique_error_signatures_count', 0)} signatures",
            d.get("power_cycles_count", 0),
            d.get("successful_recoveries_count", 0),
            d.get("failed_recoveries_count", 0),
            d.get("incomplete_wav_count", 0),
            round(d.get("recovery_latency_ms", {}).get("avg", 0.0), 1),
            round(d.get("sd_write_throughput_kb_s", {}).get("avg", 0.0), 1),
            round(d.get("sd_batch_latency_us", {}).get("max", 0.0), 0),
            round(d.get("dsp_execution_us", {}).get("avg", 0.0), 1),
            round(d.get("power_metrics", {}).get("avg_power_mw", 0.0), 2),
            round(d.get("power_metrics", {}).get("accumulated_energy_mj", 0.0), 1),
            assess_str,
            d.get("status", "PENDING").upper(),
            d.get("terminal_reason", ""),
        ]

        for col_idx, val in enumerate(row_vals, start=1):
            cell = ws.cell(row=row_idx, column=col_idx, value=val)
            _apply_row_style(ws, row_idx)
            if col_idx == 21:
                status_u = str(val).upper()
                cell.fill = PASS_FILL if status_u == "PASS" else (FAIL_FILL if status_u == "FAIL" else PENDING_FILL)

        _auto_fit_columns(ws)
        self._safe_save(self.workbook)

