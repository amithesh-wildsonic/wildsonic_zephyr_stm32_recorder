from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict


@dataclass
class ParsedMetrics:
    source: str
    metrics: Dict[str, Any] = field(default_factory=dict)
    raw_text: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"source": self.source, "metrics": self.metrics, "raw_text": self.raw_text}


@dataclass
class SessionRecord:
    test_id: str
    board: str = "STM32L476RG"
    firmware_commit: str = ""
    build_type: str = ""
    category: str = ""
    configuration: str = ""
    date_time: str = field(default_factory=lambda: datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"))
    raw_log_path: str = ""
    ppk2_csv_path: str = ""
    parsed_result: Dict[str, Any] = field(default_factory=dict)
    sample_data: Dict[str, Any] = field(default_factory=dict)
    evidence_type: str = "HARDWARE"
    status: str = "pending"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "test_id": self.test_id,
            "board": self.board,
            "firmware_commit": self.firmware_commit,
            "build_type": self.build_type,
            "category": self.category,
            "configuration": self.configuration,
            "date_time": self.date_time,
            "raw_log_path": self.raw_log_path,
            "ppk2_csv_path": self.ppk2_csv_path,
            "parsed_result": self.parsed_result,
            "sample_data": self.sample_data,
            "evidence_type": self.evidence_type,
            "status": self.status,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SessionRecord":
        return cls(
            test_id=data.get("test_id", ""),
            board=data.get("board", "STM32L476RG"),
            firmware_commit=data.get("firmware_commit", ""),
            build_type=data.get("build_type", ""),
            category=data.get("category", ""),
            configuration=data.get("configuration", ""),
            date_time=data.get("date_time", datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")),
            raw_log_path=data.get("raw_log_path", ""),
            ppk2_csv_path=data.get("ppk2_csv_path", ""),
            parsed_result=data.get("parsed_result", {}),
            sample_data=data.get("sample_data", {}),
            evidence_type=data.get("evidence_type", "HARDWARE"),
            status=data.get("status", "pending"),
        )


TestSession = SessionRecord
SessionRecord.__test__ = False
TestSession.__test__ = False
