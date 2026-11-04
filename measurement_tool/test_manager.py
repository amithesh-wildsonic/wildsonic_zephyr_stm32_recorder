from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from .models import TestSession


class TestManager:
    def __init__(self, storage_path: str | Path = "data/stm32/tests.json") -> None:
        self.storage_path = Path(storage_path)
        self.storage_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.storage_path.exists():
            self.storage_path.write_text("[]", encoding="utf-8")

    def _read_all(self) -> List[Dict[str, Any]]:
        try:
            data = json.loads(self.storage_path.read_text(encoding="utf-8"))
            if isinstance(data, list):
                return data
        except json.JSONDecodeError:
            pass
        return []

    def _write_all(self, rows: List[Dict[str, Any]]) -> None:
        self.storage_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")

    def create(self, session: TestSession) -> TestSession:
        rows = self._read_all()
        existing = {row.get("test_id") for row in rows}
        if session.test_id in existing:
            raise ValueError(f"Test ID '{session.test_id}' already exists")
        rows.append(session.to_dict())
        self._write_all(rows)
        return session

    def list(self) -> List[TestSession]:
        return [TestSession.from_dict(row) for row in self._read_all()]

    def get(self, test_id: str) -> Optional[TestSession]:
        for row in self._read_all():
            if row.get("test_id") == test_id:
                return TestSession.from_dict(row)
        return None

    def update(self, session: TestSession) -> TestSession:
        rows = self._read_all()
        updated = False
        for index, row in enumerate(rows):
            if row.get("test_id") == session.test_id:
                rows[index] = session.to_dict()
                updated = True
                break
        if not updated:
            rows.append(session.to_dict())
        self._write_all(rows)
        return session

    def delete(self, test_id: str) -> None:
        rows = [row for row in self._read_all() if row.get("test_id") != test_id]
        self._write_all(rows)
