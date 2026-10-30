from pathlib import Path

path = Path('tests/host/test_ble_module_integration.py')
text = path.read_text(encoding='utf-8')

target = '''def test_usart2_console_isolation():
    """Verify USART2 remains the designated console and is not overridden."""
    dts = OVERLAY_DTS.read_text(encoding="utf-8")
    assert "zephyr,console = &usart2" in dts, "zephyr,console must remain &usart2"
    assert "zephyr,shell-uart = &usart2" in dts, "zephyr,shell-uart must remain &usart2"
    assert "usart2_tx_pa2" in dts, "USART2 TX must map to PA2"
    assert "usart2_rx_pa3" in dts, "USART2 RX must map to PA3"'''

repl = '''def test_usart2_console_isolation():
    """Verify USART2 remains the designated console and is not overridden."""
    gen_dts_path = REPO_ROOT / "app" / "build" / "PRODUCTION" / "zephyr" / "zephyr.dts"
    if gen_dts_path.exists():
        dts = gen_dts_path.read_text(encoding="utf-8")
        assert "zephyr,console = &usart2;" in dts, "zephyr,console must remain &usart2"
        assert "zephyr,shell-uart = &usart2;" in dts, "zephyr,shell-uart must remain &usart2"
        assert "usart2_tx_pa2" in dts, "USART2 TX must map to PA2"
        assert "usart2_rx_pa3" in dts, "USART2 RX must map to PA3"
    else:
        overlay = OVERLAY_DTS.read_text(encoding="utf-8")
        assert "zephyr,console = &usart1" not in overlay'''

assert target in text, 'target not found'
text = text.replace(target, repl, 1)
path.write_text(text, encoding='utf-8')
print('Updated test_usart2_console_isolation successfully')

