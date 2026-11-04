from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from .controller import SerialController
from .models import TestSession
from .parsers import parse_firmware_log
from .ppk2_controller import PPK2Controller
from .ppk2_import import parse_ppk2_csv
from .report_generator import update_test_result
from .serial_capture import SerialCaptureSession
from .stability_runner import StabilityRunner
from .test_manager import TestManager
from .test_runner import BenchmarkRunner, create_mock_stm32_serial


def _infer_evidence_type(raw_log_path: str, ppk2_csv_path: str, explicit: str | None = None) -> str:
    if explicit:
        return explicit.upper()
    samples = [raw_log_path, ppk2_csv_path]
    text = " ".join(samples).lower()
    if "sample" in text or "mock" in text:
        return "SAMPLE"
    return "HARDWARE"


def _normalize_commit(commit: str) -> str:
    return (commit or "").strip()


def create_test(args):
    session = TestSession(
        test_id=args.test_id,
        board=args.board,
        firmware_commit=args.firmware_commit,
        build_type=args.build_type,
        category=args.category,
        configuration=args.configuration,
        raw_log_path=args.raw_log,
        ppk2_csv_path=args.ppk2,
        evidence_type=_infer_evidence_type(args.raw_log, args.ppk2, getattr(args, "evidence_type", None)),
        status=args.status,
    )
    manager = TestManager(args.storage)
    created = manager.create(session)
    print(json.dumps(created.to_dict(), indent=2))


def parse_log(args):
    log_path = Path(args.log)
    text = log_path.read_text(encoding="utf-8", errors="replace")
    parsed = parse_firmware_log(text)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(parsed, indent=2), encoding="utf-8")
    print(json.dumps(parsed, indent=2))


def import_ppk2(args):
    parsed = parse_ppk2_csv(args.csv)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(parsed, indent=2), encoding="utf-8")
    print(json.dumps(parsed, indent=2))


def update_report(args):
    manager = TestManager(args.storage)
    session = manager.get(args.test_id)
    if session is None:
        raise SystemExit(f"Unknown test ID: {args.test_id}")
    if args.log:
        session.raw_log_path = args.log
        text = Path(args.log).read_text(encoding="utf-8", errors="replace")
        session.parsed_result = parse_firmware_log(text)
    if args.ppk2:
        session.ppk2_csv_path = args.ppk2
        session.sample_data = parse_ppk2_csv(args.ppk2)
        if session.evidence_type == "HARDWARE" and "sample" in args.ppk2.lower():
            session.evidence_type = "SAMPLE"
    if args.evidence_type:
        session.evidence_type = args.evidence_type.upper()
    else:
        session.evidence_type = _infer_evidence_type(session.raw_log_path, session.ppk2_csv_path, session.evidence_type)
    if args.commit:
        session.firmware_commit = args.commit
    if args.status:
        session.status = args.status.lower()
    manager.update(session)
    result = update_test_result(args.workbook, session)
    print(json.dumps(result, indent=2))


def capture_serial(args):
    capture = SerialCaptureSession(args.port, args.baudrate, args.output_dir)
    capture.start(args.test_name, args.board)
    print(json.dumps({"raw_log": str(capture.raw_path), "timestamped_log": str(capture.timestamped_path)}))


def run_test_cmd(args):
    is_mock = getattr(args, "mock", False) or args.port.upper() in {"MOCK", "SIM", "NONE"}
    mock_port = create_mock_stm32_serial() if is_mock else None
    evidence_type = "SAMPLE" if is_mock else getattr(args, "evidence_type", "HARDWARE")

    controller = SerialController(
        port=args.port,
        baudrate=args.baudrate,
        output_dir="data/stm32/raw_logs",
        progress_callback=lambda msg: print(f"  {msg}"),
        mock_port=mock_port,
    )

    ppk2_port = getattr(args, "ppk2_port", None)
    voltage_mv = getattr(args, "voltage", 5000)
    power_rail = getattr(args, "power_rail", "E5V").upper()
    if power_rail == "VDD" and voltage_mv == 5000:
        voltage_mv = 3300

    ppk2_ctrl = None
    category = args.category.lower()
    if ppk2_port or category in {"ppk2-boot", "ppk2-benchmark", "full-benchmark", "stability-test", "stability"}:
        ppk2_mock = is_mock or (ppk2_port and ppk2_port.upper() in {"MOCK", "SIM", "MOCK_PPK2"})
        ppk2_ctrl = PPK2Controller(
            port=ppk2_port if not ppk2_mock else "MOCK_PPK2",
            voltage_mv=voltage_mv,
            mock=ppk2_mock,
            progress_callback=lambda msg: print(f"  {msg}"),
        )

    runner = BenchmarkRunner(
        controller=controller,
        ppk2=ppk2_ctrl,
        storage_path=args.storage,
        progress_callback=lambda msg: print(f"  {msg}"),
    )

    mode = getattr(args, "mode", "software")
    sessions = []

    if category in {"stability-test", "stability"}:
        dur_h = getattr(args, "duration_hours", 8.0)
        dur_s = getattr(args, "duration_seconds", None)
        stab_runner = StabilityRunner(
            controller=controller,
            ppk2=ppk2_ctrl,
            sample_rate=args.rate or 96000,
            bit_depth=getattr(args, "bit_depth", 16),
            voltage_mv=voltage_mv,
            duration_hours=dur_h,
            duration_seconds=dur_s,
            recovery_timeout_s=getattr(args, "recovery_timeout", 15.0),
            discharge_duration_s=getattr(args, "power_off_interval", 2.0),
            max_signature_recoveries=getattr(args, "max_signature_recoveries", 5),
            max_total_power_cycles=getattr(args, "max_total_power_cycles", 10),
            trend_interval_s=getattr(args, "ppk2_trend_interval", 1.0),
            progress_callback=lambda msg: print(f"  {msg}"),
        )
        sess = stab_runner.run(
            test_id=args.test_id or "STM32-STABILITY-001",
            evidence_type=evidence_type,
        )
        sessions.append(sess)
    elif category in {"ppk2-benchmark", "full-benchmark"}:
        sess = runner.run_ppk2_full_benchmark(
            base_id=args.test_id or "STM32-PPK2-BENCH",
            voltage_mv=voltage_mv,
            sample_rate=args.rate,
            record_duration_s=args.duration,
            sleep_seconds=args.sleep_seconds,
            boot_stabilize_s=getattr(args, "boot_stabilize", 1.0),
            devinfo_timeout_s=getattr(args, "devinfo_timeout", 15.0),
            evidence_type=evidence_type,
        )
        sessions.append(sess)
    elif category in {"ppk2-boot"}:
        sess = runner.run_ppk2_power_cycle_boot(
            test_id=args.test_id or "STM32-PPK2-BOOT-001",
            voltage_mv=voltage_mv,
            boot_stabilize_s=getattr(args, "boot_stabilize", 1.0),
            devinfo_timeout_s=getattr(args, "devinfo_timeout", 15.0),
            evidence_type=evidence_type,
        )
        sessions.append(sess)
    elif category == "boot":
        sessions.append(runner.run_boot_test(test_id=args.test_id or "STM32-BOOT-001", mode=mode, evidence_type=evidence_type))
    elif category in {"cold-boot", "cold"}:
        sessions.append(runner.run_cold_boot_test(test_id=args.test_id or "STM32-COLD-BOOT-001", evidence_type=evidence_type))
    elif category == "idle-power":
        sessions.append(runner.run_idle_power_test(test_id=args.test_id or "STM32-IDLE-001", duration_s=args.duration, evidence_type=evidence_type))
    elif category == "recording":
        sessions.append(runner.run_recording_test(
            test_id=args.test_id or "STM32-REC-001",
            sample_rate=args.rate,
            duration_s=args.duration,
            evidence_type=evidence_type,
            ppk2_csv=args.ppk2,
        ))
    elif category == "sd-performance":
        sessions.append(runner.run_sd_performance_test(test_id=args.test_id or "STM32-SD-001", evidence_type=evidence_type))
    elif category == "sleep-wake":
        sessions.append(runner.run_sleep_wake_test(test_id=args.test_id or "STM32-SLEEP-001", sleep_seconds=args.sleep_seconds, evidence_type=evidence_type))
    else:
        raise SystemExit(f"Unsupported category: {args.category}")

    # Update comparison workbook
    for s in sessions:
        update_test_result(args.workbook, s)

    print(f"\n[OK] Executed {len(sessions)} test sequence(s). Workbook updated: {args.workbook}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 measurement capture, automated benchmarking, and reporting")
    subparsers = parser.add_subparsers(dest="command", required=True)

    create_parser = subparsers.add_parser("create-test", help="Create a test session record")
    create_parser.add_argument("--test-id", required=True)
    create_parser.add_argument("--board", default="STM32L476RG")
    create_parser.add_argument("--firmware-commit", default="")
    create_parser.add_argument("--build-type", default="")
    create_parser.add_argument("--category", default="")
    create_parser.add_argument("--configuration", default="")
    create_parser.add_argument("--raw-log", default="")
    create_parser.add_argument("--ppk2", default="")
    create_parser.add_argument("--evidence-type", choices=["HARDWARE", "SAMPLE"], default=None)
    create_parser.add_argument("--status", default="pending")
    create_parser.add_argument("--storage", default="data/stm32/tests.json")
    create_parser.set_defaults(func=create_test)

    parse_parser = subparsers.add_parser("parse-log", help="Parse a raw STM32 log file")
    parse_parser.add_argument("--log", required=True)
    parse_parser.add_argument("--output", default="")
    parse_parser.set_defaults(func=parse_log)

    ppk_parser = subparsers.add_parser("import-ppk2", help="Parse a Nordic PPK2 CSV export")
    ppk_parser.add_argument("--csv", required=True)
    ppk_parser.add_argument("--output", default="")
    ppk_parser.set_defaults(func=import_ppk2)

    report_parser = subparsers.add_parser("report", help="Update comparison workbook")
    report_parser.add_argument("--test-id", required=True)
    report_parser.add_argument("--workbook", default="data/reports/stm32_vs_max78000.xlsx")
    report_parser.add_argument("--log", default="")
    report_parser.add_argument("--ppk2", default="")
    report_parser.add_argument("--commit", default="")
    report_parser.add_argument("--evidence-type", choices=["HARDWARE", "SAMPLE"], default=None)
    report_parser.add_argument("--status", default="")
    report_parser.add_argument("--storage", default="data/stm32/tests.json")
    report_parser.set_defaults(func=update_report)

    capture_parser = subparsers.add_parser("capture", help="Capture a serial session")
    capture_parser.add_argument("--port", required=True)
    capture_parser.add_argument("--baudrate", type=int, default=115200)
    capture_parser.add_argument("--test-name", default="stm32_capture")
    capture_parser.add_argument("--board", default="STM32L476RG")
    capture_parser.add_argument("--output-dir", default="data/stm32/raw_logs")
    capture_parser.set_defaults(func=capture_serial)

    run_parser = subparsers.add_parser("run-test", help="Run automated test sequence over UART & PPK2")
    run_parser.add_argument(
        "--category",
        choices=[
            "ppk2-benchmark",
            "ppk2-boot",
            "stability-test",
            "stability",
            "boot",
            "cold-boot",
            "idle-power",
            "recording",
            "sd-performance",
            "sleep-wake",
            "full-benchmark",
        ],
        default="ppk2-benchmark",
    )
    run_parser.add_argument("--mode", choices=["software", "cold", "ppk2"], default="software", help="Boot mode: software (RBT), cold (NRST), or ppk2 (power cycle)")
    run_parser.add_argument("--port", default="MOCK", help="STM32 ST-Link UART serial port (e.g. COM30 or MOCK)")
    run_parser.add_argument("--ppk2-port", default="AUTO", help="Nordic PPK2 USB serial port (e.g. COM31, AUTO, or MOCK)")
    run_parser.add_argument("--voltage", type=int, default=5000, help="PPK2 Source Meter voltage in mV (default 5000 mV for Nucleo E5V)")
    run_parser.add_argument("--power-rail", choices=["E5V", "VDD"], default="E5V", help="Target board power rail (E5V=5.0V, VDD=3.3V)")
    run_parser.add_argument("--boot-stabilize", type=float, default=1.0, help="Boot stabilization interval in seconds after power-on (default 1.0s)")
    run_parser.add_argument("--devinfo-timeout", type=float, default=15.0, help="Maximum bounded timeout polling DEVINFO after power-on (default 15.0s)")
    run_parser.add_argument("--baudrate", type=int, default=115200)
    run_parser.add_argument("--test-id", default=None)
    run_parser.add_argument("--rate", type=int, default=192000)
    run_parser.add_argument("--duration", type=int, default=10)
    run_parser.add_argument("--duration-hours", type=float, default=8.0, help="Stability test duration in hours (e.g. 8.0, 12.0)")
    run_parser.add_argument("--duration-seconds", type=float, default=None, help="Stability test custom duration in seconds")
    run_parser.add_argument("--bit-depth", type=int, choices=[16], default=16, help="Audio bit depth (16-bit)")
    run_parser.add_argument("--recovery-timeout", type=float, default=15.0, help="Automatic recovery DEVINFO timeout in seconds")
    run_parser.add_argument("--power-off-interval", type=float, default=2.0, help="Power-off discharge duration in seconds")
    run_parser.add_argument("--max-signature-recoveries", type=int, default=5, help="Maximum recoveries allowed for the same error signature")
    run_parser.add_argument("--max-total-power-cycles", type=int, default=10, help="Maximum total automatic power cycles allowed per test")
    run_parser.add_argument("--ppk2-trend-interval", type=float, default=1.0, help="PPK2 trend aggregation interval in seconds (default 1.0s)")
    run_parser.add_argument("--sleep-seconds", type=int, default=5)
    run_parser.add_argument("--ppk2", default="")
    run_parser.add_argument("--evidence-type", choices=["HARDWARE", "SAMPLE"], default="HARDWARE")
    run_parser.add_argument("--mock", action="store_true", help="Run with mock simulated STM32 serial interface and mock PPK2")
    run_parser.add_argument("--storage", default="data/stm32/tests.json")
    run_parser.add_argument("--workbook", default="data/reports/stm32_vs_max78000.xlsx")
    run_parser.set_defaults(func=run_test_cmd)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
