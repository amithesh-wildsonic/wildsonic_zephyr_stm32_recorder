from __future__ import annotations

import time
from pathlib import Path
import pytest

from measurement_tool.controller import (
    MockSerialPort,
    SerialController,
    SUPPORTED_COMMAND_VERBS,
)


def test_command_validation_accepts_valid_verbs():
    controller = SerialController(port="MOCK", mock_port=MockSerialPort())
    valid_commands = [
        "HELP",
        "STATUS",
        "DEVINFO",
        "SAMF 192000",
        "RECT 20",
        "BTPS 16",
        "SLEEP 5",
        "RBT",
        "STOP",
        "stop",
        "SDBENCH",
        "PSRAM status",
        "STOR",
    ]
    for cmd in valid_commands:
        valid, msg = controller.validate_command(cmd)
        assert valid is True, f"Failed for {cmd}: {msg}"


def test_command_validation_rejects_unsupported_verbs():
    controller = SerialController(port="MOCK", mock_port=MockSerialPort())
    invalid_commands = [
        "HACK_SYSTEM",
        "RUN_ARBITRARY_CODE",
        "DO_NOTHING_NOW",
        "INVALID_CMD_XYZ",
    ]
    for cmd in invalid_commands:
        valid, msg = controller.validate_command(cmd)
        assert valid is False
        assert "not supported" in msg


def test_serial_controller_mock_roundtrip(tmp_path):
    mock = MockSerialPort({
        r"HELP": "\r\nList of commands\r\nHELP\r\nSAMF\r\n> ",
        r"STATUS": "\r\n=== SYSTEM & RECORDING STATUS ===\r\n  Recording State : IDLE\r\nOK\r\n> ",
    })
    mock.feed_input("Interactive console ready. Type 'help' for commands.\r\n> ")

    controller = SerialController(
        port="MOCK",
        output_dir=tmp_path / "logs",
        mock_port=mock,
        progress_callback=lambda msg: None,
    )
    controller.connect(test_name="unit_test")

    # Send valid command
    ok, match = controller.send_command("STATUS", wait_for="Recording State", timeout=2.0)
    assert ok is True
    assert "Recording State" in match

    # Verify safe abort
    controller.safe_abort()
    assert b"stop" in mock.write_history[-2] or b"stop" in mock.write_history[-1]

    # Verify timeout on unmatched pattern
    ok_to, err = controller.wait_for_response(r"THIS_WILL_NEVER_APPEAR", timeout=0.2)
    assert ok_to is False
    assert "Timed out" in err

    controller.stop()

    # Verify log files created
    assert controller.raw_path.exists()
    assert controller.timestamped_path.exists()
    raw_content = controller.raw_path.read_text(encoding="utf-8")
    assert "Recording State : IDLE" in raw_content
    ts_content = controller.timestamped_path.read_text(encoding="utf-8")
    assert "[" in ts_content


def test_safe_sleep_validation():
    mock = MockSerialPort({
        r"SLEEP 5": "\r\nValue is set to 5\r\n[TEST] STOP2_ENTERED\r\n> ",
    })
    controller = SerialController(port="MOCK", mock_port=mock)
    controller.connect("sleep_test")

    # Rejects invalid duration
    ok_neg, msg_neg = controller.safe_sleep(0)
    assert ok_neg is False
    assert "Invalid sleep duration" in msg_neg

    ok_neg2, msg_neg2 = controller.safe_sleep(-10)
    assert ok_neg2 is False

    # Valid sleep succeeds
    ok_sleep, match = controller.safe_sleep(5, timeout=2.0)
    assert ok_sleep is True
    assert "STOP2_ENTERED" in match

    controller.stop()

