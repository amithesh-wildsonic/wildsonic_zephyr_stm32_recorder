from __future__ import annotations

import collections
import logging
import re
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Pattern, Tuple, Union

try:
    import serial  # type: ignore
except ImportError:
    serial = None

logger = logging.getLogger("measurement_tool.controller")


# Authoritative set of valid command verbs supported by the WildSonic STM32 firmware
SUPPORTED_COMMAND_VERBS = {
    "HELP", "?",
    "STATUS", "STAT", "GET_STATUS",
    "DEVINFO", "INFO", "GET_DEV_INFO",
    "CONFIG", "GET_CONFIG",
    "SAMF", "BTPS", "BITS", "NUMCH", "CHANNELS", "AUDIO",
    "RECT", "SLEEP", "DATERANGE", "TIMEOFDAY", "DAYSOFWEEK",
    "DATETIME", "TIMEZONE", "GAIN", "LCUTOFF", "UCUTOFF",
    "FLDR", "STOR", "VERSION", "SN", "DNAME", "PW",
    "APPLY", "RBT", "ARBT", "DEFAULT", "SDFMT",
    "STOP", "STOP_RECORDING",
    "PSRAM", "SDBENCH", "SD_BENCH", "SOURCE",
    "SUN", "SUNMODE", "SUNRISEBEF", "SUNRISEAFT", "SUNSETBEF", "SUNSETAFT",
    "TEST_ALARM", "ASSERT", "BATT", "SIG", "LBAT", "NBAT", "FULLCAP", "SENDSTS",
    "I2C", "RTCRST", "SMSKW", "SMSKEY", "A2PPHONE", "P2APHONE", "SURL", "SPORT",
    "POSTROUTE", "POSTCTNT", "STEST", "SWS", "SMSFREQ", "BLEPAIR", "WILDLY", "ENVSTS", "LOC",
}


class MockSerialPort:
    """Mock serial port for deterministic host and unit testing."""

    def __init__(self, responses: Optional[Dict[str, str]] = None) -> None:
        self.is_open = True
        self.responses = responses or {}
        self.write_history: List[bytes] = []
        self._read_buffer = bytearray()
        self._lock = threading.Lock()

    @property
    def rx_buffer(self) -> bytearray:
        return self._read_buffer

    @property
    def sent_commands(self) -> List[str]:
        with self._lock:
            return [b.decode("utf-8", errors="replace").strip() for b in self.write_history]

    def write(self, data: bytes) -> int:
        with self._lock:
            self.write_history.append(data)
            cmd = data.decode("utf-8", errors="replace").strip()
            # Look up simulated response
            for pattern, resp in self.responses.items():
                if pattern in cmd or re.search(pattern, cmd, re.I):
                    self._read_buffer.extend(resp.encode("utf-8"))
                    break
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            if not self._read_buffer:
                return b""
            chunk = self._read_buffer[:size]
            del self._read_buffer[:size]
            return bytes(chunk)

    def feed_input(self, text: str) -> None:
        with self._lock:
            self._read_buffer.extend(text.encode("utf-8"))

    def queue_response(self, cmd_pattern: str, response: Union[str, bytes]) -> None:
        with self._lock:
            resp_str = response.decode("utf-8", errors="replace") if isinstance(response, bytes) else response
            self.responses[cmd_pattern] = resp_str

    def close(self) -> None:
        self.is_open = False


class SerialController:
    """Robust UART controller for automated hardware benchmarking.

    Captures all raw bytes in the background without dropping,
    generates timestamped logs, validates commands before transmission,
    handles timeouts, and prints only concise status updates to the console.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        output_dir: str | Path = "data/stm32/raw_logs",
        progress_callback: Optional[Callable[[str], None]] = None,
        mock_port: Optional[Any] = None,
        live_console: bool = True,
        line_callback: Optional[Callable[[str, str, int], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.progress_callback = progress_callback or (lambda msg: None)
        self.live_console = live_console
        self.line_callback = line_callback

        self._serial: Optional[Any] = mock_port
        self._is_mock = mock_port is not None
        self._raw_path: Optional[Path] = None
        self._timestamped_path: Optional[Path] = None

        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._line_history: collections.deque[str] = collections.deque(maxlen=2000)
        self._all_captured_text: List[str] = []
        self._lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._start_time: Optional[float] = None
        self._start_monotonic_ns: Optional[int] = None

    @property
    def raw_path(self) -> Optional[Path]:
        return self._raw_path

    @property
    def timestamped_path(self) -> Optional[Path]:
        return self._timestamped_path

    @property
    def all_text(self) -> str:
        with self._lock:
            return "".join(self._all_captured_text)

    def read_lines(self) -> List[str]:
        """Returns and drains newly captured complete lines."""
        with self._lock:
            lines = list(self._line_history)
            self._line_history.clear()
            return lines

    def validate_command(self, cmd_line: str) -> Tuple[bool, str]:
        stripped = cmd_line.strip()
        if not stripped:
            return True, ""
        verb = stripped.split()[0].upper()
        if verb not in SUPPORTED_COMMAND_VERBS:
            return False, f"Command verb '{verb}' is not supported by STM32 firmware"
        return True, ""

    def connect(
        self,
        test_name: str = "benchmark",
        board: str = "STM32L476RG",
        raw_filename: Optional[str] = None,
        timestamped_filename: Optional[str] = None,
    ) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in test_name).strip("_") or "capture"
        self._raw_path = self.output_dir / (raw_filename or f"{safe_name}_{board}_{stamp}.raw.log")
        self._timestamped_path = self.output_dir / (timestamped_filename or f"{safe_name}_{board}_{stamp}.timestamped.log")

        self._stop_event.clear()
        self._start_time = time.time()
        self._start_monotonic_ns = time.monotonic_ns()

        if self._is_mock and self._serial is not None:
            self._serial.is_open = True
        elif not self._is_mock:
            if serial is None:
                raise RuntimeError("pyserial is required for hardware serial communication")
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self._reader_thread = threading.Thread(target=self._background_read_loop, daemon=True)
        self._reader_thread.start()
        self.progress_callback(f"Connected to {self.port} at {self.baudrate} baud (logging to {self._raw_path.name})")

    def _background_read_loop(self) -> None:
        pending_line = bytearray()
        while True:
            try:
                if self._serial is None or not getattr(self._serial, "is_open", True):
                    break
                data = self._serial.read(8192)
                if not data:
                    if self._stop_event.is_set():
                        break
                    time.sleep(0.01)
                    continue

                # 1. Always record raw bytes directly to raw log file
                with self._raw_path.open("ab") as raw_f:
                    raw_f.write(data)

                # 2. Process text lines and write timestamped entries
                text_chunk = data.decode("utf-8", errors="replace")
                with self._lock:
                    self._all_captured_text.append(text_chunk)

                pending_line.extend(data)
                while b"\n" in pending_line:
                    idx = pending_line.index(b"\n")
                    line_bytes = pending_line[: idx + 1]
                    del pending_line[: idx + 1]
                    line_str = line_bytes.decode("utf-8", errors="replace")

                    now_str = datetime.now().astimezone().isoformat(timespec="milliseconds")
                    elapsed_ns = max(0, time.monotonic_ns() - (self._start_monotonic_ns or time.monotonic_ns()))
                    ts_line = f"[{now_str}] [mono_ns={elapsed_ns}] {line_str}"
                    with self._timestamped_path.open("a", encoding="utf-8", errors="replace") as ts_f:
                        ts_f.write(ts_line)

                    with self._lock:
                        self._line_history.append(line_str)
                    if self.line_callback:
                        try:
                            self.line_callback(line_str, now_str, elapsed_ns)
                        except Exception:
                            logger.exception("UART line callback failed")
                    if self.live_console:
                        print(line_str, end="", flush=True)

            except Exception as exc:
                self._last_error = str(exc)
                break

        if pending_line:
            line_str = pending_line.decode("utf-8", errors="replace")
            now_str = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
            with self._timestamped_path.open("a", encoding="utf-8", errors="replace") as ts_f:
                ts_f.write(f"[{now_str}] {line_str}\n")

    def send_command(
        self,
        cmd: str,
        wait_for: Optional[Union[str, Pattern]] = ">",
        timeout: float = 5.0,
    ) -> Tuple[bool, str]:
        valid, err_msg = self.validate_command(cmd)
        if not valid:
            self.progress_callback(f"[ERROR] Rejected command: {err_msg}")
            return False, err_msg

        if self._serial is None or not getattr(self._serial, "is_open", True):
            return False, "Serial port is not connected"

        # Record position before sending
        start_idx = len(self._all_captured_text)
        cmd_bytes = f"{cmd.strip()}\r\n".encode("utf-8")
        self._serial.write(cmd_bytes)

        if not wait_for:
            return True, ""

        return self.wait_for_response(wait_for, timeout=timeout, start_idx=start_idx)

    def wait_for_response(
        self,
        expected: Union[str, Pattern],
        timeout: float = 10.0,
        start_idx: int = 0,
    ) -> Tuple[bool, str]:
        deadline = time.time() + timeout
        pattern = re.compile(expected, re.I) if isinstance(expected, str) else expected

        while time.time() < deadline:
            with self._lock:
                recent_text = "".join(self._all_captured_text[start_idx:])
                match = pattern.search(recent_text)
                if match:
                    return True, match.group(0)
            time.sleep(0.05)

        return False, f"Timed out waiting for '{pattern.pattern}' after {timeout:.1f}s"

    def safe_sleep(self, seconds: int, timeout: float = 15.0) -> Tuple[bool, str]:
        """Safely commands Stop 2 deep sleep, verifying that seconds > 0 and wakeup is configured."""
        if seconds <= 0 or seconds > 86400:
            return False, f"Invalid sleep duration: {seconds}s (must be 1 to 86400)"

        self.progress_callback(f"Configuring RTC wakeup and entering Stop 2 sleep for {seconds}s...")
        success, resp = self.send_command(f"SLEEP {seconds}", wait_for="Value is set to", timeout=timeout)
        if not success:
            self.progress_callback(f"[ERROR] Failed to set SLEEP interval: {resp}")
            return False, resp

        # Wait for Stop 2 confirmation or wake
        ok, match = self.wait_for_response(r"(?:STOP2_ENTERED|RTC wake received|stop2\s+entry)", timeout=timeout + seconds + 5.0)
        return ok, match

    def safe_abort(self) -> None:
        """Immediately and safely aborts ongoing recording and restores system to idle."""
        self.progress_callback("Initiating safe abort...")
        try:
            if self._serial and getattr(self._serial, "is_open", True):
                self._serial.write(b"\r\nstop\r\n")
                time.sleep(0.2)
                self._serial.write(b"\r\n")
        except Exception:
            pass

    def stop(self) -> None:
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if not self._is_mock:
            if self._serial is not None and getattr(self._serial, "is_open", False):
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = None

    def close(self) -> None:
        self.stop()
