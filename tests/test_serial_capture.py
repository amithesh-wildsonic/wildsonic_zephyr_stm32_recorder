import time
from pathlib import Path

from measurement_tool.serial_capture import SerialCaptureSession
import measurement_tool.serial_capture as serial_capture_module


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self._chunks = [b"first line\r\n", b"second line\r\n", b""]

    def read(self, size):
        if not self._chunks:
            return b""
        chunk = self._chunks.pop(0)
        time.sleep(0.05)
        return chunk

    def close(self):
        self.is_open = False


def test_serial_capture_creates_raw_and_timestamped_logs(monkeypatch, tmp_path):
    monkeypatch.setattr(serial_capture_module.serial, "Serial", lambda *args, **kwargs: FakeSerial())

    output_dir = tmp_path / "serial"
    session = SerialCaptureSession("COM4", 115200, output_dir)
    raw_path, ts_path = session.start("demo_test", board="STM32L476RG")

    time.sleep(0.3)
    session.stop()

    assert raw_path.exists()
    assert ts_path.exists()
    assert "first line" in raw_path.read_text(encoding="utf-8", errors="replace")
    assert "second line" in ts_path.read_text(encoding="utf-8", errors="replace")
    assert "[" in ts_path.read_text(encoding="utf-8", errors="replace")
