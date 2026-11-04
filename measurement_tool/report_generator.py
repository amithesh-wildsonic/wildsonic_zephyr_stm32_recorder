from __future__ import annotations

import re
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from openpyxl import Workbook, load_workbook

from .models import TestSession
from .workbook import ComparisonWorkbookManager, backup_workbook


def _extract_duration_seconds(value: str) -> float | None:
    if not value:
        return None
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:s|sec|second|seconds)", value, re.IGNORECASE)
    if not match:
        return None
    return float(match.group(1))


def _valid_commit(value: str) -> bool:
    commit = (value or "").strip()
    if not commit:
        return False
    if commit.lower() in {"a1b2c3d", "placeholder", "sample", "unknown", "n/a"}:
        return False
    return bool(re.fullmatch(r"[0-9a-fA-F]{7,40}", commit))


def _session_warnings(session: TestSession) -> list[str]:
    warnings: list[str] = []

    if session.evidence_type == "SAMPLE":
        warnings.append("Evidence type is SAMPLE; hardware validation was not performed.")
    if not _valid_commit(session.firmware_commit):
        warnings.append("Firmware commit identifier is missing or invalid.")

    metrics = session.parsed_result or {}

    # Boot & safety check warnings
    if metrics.get("blocked"):
        warnings.append(metrics.get("blocked_reason", "Test execution was blocked by safety check."))
    if metrics.get("reboot_error"):
        warnings.append(f"MCU reboot aborted or failed: {metrics['reboot_error']}")
    if metrics.get("boot_error"):
        warnings.append(f"Boot verification failed: {metrics['boot_error']}")
    if metrics.get("boot_verified") is False:
        warnings.append("Boot verification failed: boot banner or system initialization milestone was not detected.")

    expected = metrics.get("expected_samples")
    acquired = metrics.get("acquired_samples")
    processed = metrics.get("processed_samples")
    written = metrics.get("samples_written")
    lost = metrics.get("lost_samples")
    boundary = float(metrics.get("boundary_samples_discarded") or metrics.get("boundary_discarded_samples") or 0.0)

    if expected is not None and acquired is not None:
        if float(acquired) < float(expected):
            deficit = int(float(expected) - float(acquired))
            warnings.append(
                f"Sample shortfall: expected {int(float(expected))} samples but acquired {int(float(acquired))} "
                f"({deficit} expected samples not acquired; session incomplete)."
            )
        elif float(acquired) > (float(expected) + boundary):
            warnings.append("Sample counter mismatch: acquired samples exceed expected samples.")

    if acquired is not None and processed is not None:
        if float(processed) < float(acquired):
            unproc = int(float(acquired) - float(processed))
            warnings.append(f"Sample processing deficit: {unproc} acquired samples were not processed.")
        elif float(processed) > float(acquired):
            warnings.append("Sample counter mismatch: processed samples exceed acquired samples.")

    if processed is not None and written is not None:
        if (float(written) + boundary) < float(processed):
            unwritten = int(float(processed) - (float(written) + boundary))
            warnings.append(f"Sample storage deficit: {unwritten} processed samples were not written to SD.")
        elif float(written) > float(processed):
            warnings.append("Sample counter mismatch: written samples exceed processed samples.")

    if lost is not None and float(lost) > 0:
        warnings.append(f"Active capture samples lost: {int(float(lost))} samples.")

    if metrics.get("stop2_verified") is False:
        missing = metrics.get("stop2_missing_evidence", [])
        missing_str = f" ({'; '.join(missing)})" if missing else ""
        warnings.append(f"Stop2 deep sleep unverified: missing required multi-stage lifecycle evidence{missing_str}.")

    config_duration = _extract_duration_seconds(session.configuration)
    parsed_duration = metrics.get("recording_duration")
    if config_duration is not None and parsed_duration is not None:
        if abs(float(parsed_duration) - float(config_duration)) > 0.5:
            warnings.append(
                f"Recording duration mismatch: configured {config_duration}s does not match parsed {parsed_duration}s."
            )

    if session.ppk2_csv_path and session.evidence_type == "SAMPLE":
        warnings.append("PPK2 sample data is present alongside sample evidence; hardware data is not validated.")

    return warnings


def _effective_status(session: TestSession) -> str:
    if session.status.lower() in {"fail", "blocked"}:
        return session.status.lower()
    warnings = _session_warnings(session)
    if warnings:
        return "pending"
    if session.status.lower() == "pass":
        return "pass"
    return "pending"


def ensure_workbook(path: str | Path) -> Workbook:
    manager = ComparisonWorkbookManager(path)
    return manager.workbook


def update_test_result(workbook_path: str | Path, session: TestSession) -> Dict[str, Any]:
    warnings = _session_warnings(session)
    status = _effective_status(session)
    session.status = status

    if not isinstance(session.parsed_result, dict):
        session.parsed_result = {}
    session.parsed_result["warnings"] = warnings

    manager = ComparisonWorkbookManager(workbook_path)
    result = manager.add_or_update_session(session)
    result["status"] = status
    return result


def update_summary(workbook_path: str | Path, summary_rows: Dict[str, Any]) -> Dict[str, Any]:
    manager = ComparisonWorkbookManager(workbook_path)
    ws = manager.workbook["Comparison Summary"]
    ws.append([])
    ws.append(["Summary", "Value"])
    for key, value in summary_rows.items():
        ws.append([key, str(value)])
    manager.workbook.save(manager.path)
    return {"workbook": str(manager.path), "summary": summary_rows}
