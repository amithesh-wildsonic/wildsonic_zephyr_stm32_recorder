from __future__ import annotations

import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import serial


class SerialCaptureSession:
    def __init__(self, port: str, baudrate: int = 115200, output_dir: str | Path = "data/stm32/raw_logs") -> None:
        self.port = port
        self.baudrate = baudrate
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._serial: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._raw_path: Optional[Path] = None
        self._timestamped_path: Optional[Path] = None
        self._last_error: Optional[str] = None
        self._received_lines: list[str] = []

    @property
    def raw_path(self) -> Optional[Path]:
        return self._raw_path

    @property
    def timestamped_path(self) -> Optional[Path]:
        return self._timestamped_path

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def start(self, test_name: str, board: str = "STM32L476RG") -> tuple[Path, Path]:
        if self._serial is not None and self._serial.is_open:
            return self._raw_path, self._timestamped_path
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in test_name).strip("_") or "capture"
        self._raw_path = self.output_dir / f"{safe_name}_{board}_{stamp}.raw.log"
        self._timestamped_path = self.output_dir / f"{safe_name}_{board}_{stamp}.timestamped.log"
        self._raw_path.parent.mkdir(parents=True, exist_ok=True)
        self._timestamped_path.parent.mkdir(parents=True, exist_ok=True)
        self._stop_event.clear()
        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return self._raw_path, self._timestamped_path

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                if self._serial is None or not self._serial.is_open:
                    break
                data = self._serial.read(4096)
                if not data:
                    time.sleep(0.1)
                    continue
                raw_line = data.decode("utf-8", errors="replace")
                self._received_lines.append(raw_line)
                with self._raw_path.open("a", encoding="utf-8", errors="replace") as raw_file:
                    raw_file.write(raw_line)
                timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
                with self._timestamped_path.open("a", encoding="utf-8", errors="replace") as ts_file:
                    for line in raw_line.splitlines(True):
                        ts_file.write(f"[{timestamp}] {line}")
            except serial.SerialException as exc:
                self._last_error = str(exc)
                break
            except Exception as exc:  # pragma: no cover - defensive
                self._last_error = str(exc)
                break
        self.stop()

    def stop(self) -> None:
        self._stop_event.set()
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        self._serial = None

    def close(self) -> None:
        self.stop()
