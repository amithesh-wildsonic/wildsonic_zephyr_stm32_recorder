from pathlib import Path
import re
import struct
import zlib


ROOT = Path(__file__).resolve().parents[2] / "app"


def pcm_sample(index: int) -> int:
    bits = ((index * 40503 + 17389) ^ (index >> 3)) & 0xFFFF
    return bits - 0x10000 if bits & 0x8000 else bits


def test_replay_crc_golden_vectors_are_deterministic_little_endian():
    payload = b"".join(struct.pack("<h", pcm_sample(i)) for i in range(1024))
    assert len(payload) == 2048
    assert zlib.crc32(payload) == 0x12E85B34


def test_replay_is_opt_in_and_preserves_existing_adc_sink_flow():
    kconfig = (ROOT / "Kconfig").read_text()
    assert "config AUDIO_INTEGRITY_REPLAY" not in kconfig, "CONFIG_AUDIO_INTEGRITY_REPLAY should be removed from Kconfig"

    adc = (ROOT / "src" / "audio_in_stm32l4.c").read_text()
    assert "audio_in_integrity_sample" not in adc, "audio_in_integrity_sample should be removed from audio_in_stm32l4.c"
    assert "audio_in_set_integrity_replay" not in adc, "audio_in_set_integrity_replay should be removed from audio_in_stm32l4.c"

    adc_h = (ROOT / "src" / "audio_in.h").read_text()
    assert "audio_in_integrity_sample" not in adc_h, "audio_in_integrity_sample should be removed from audio_in.h"


def test_readback_parser_walks_riff_chunks_and_checks_pipeline_crc():
    recorder = (ROOT / "src" / "recorder.c").read_text()
    recorder_h = (ROOT / "src" / "recorder.h").read_text()

    assert "integrity_readback_crc" not in recorder, "integrity_readback_crc should be removed from recorder.c"
    assert "integrity_expected_crc32" not in recorder_h, "integrity_expected_crc32 should be removed from recorder.h"
    assert "integrity_pipeline_crc32" not in recorder_h, "integrity_pipeline_crc32 should be removed from recorder.h"
    assert "integrity_readback_crc32" not in recorder_h, "integrity_readback_crc32 should be removed from recorder.h"


def test_test_build_fragment_enables_replay_explicitly():
    assert not (ROOT / "configs" / "integrity_replay.conf").exists(), "integrity_replay.conf should be deleted"
    assert not (ROOT / "configs" / "test_integrity_replay.conf").exists(), "test_integrity_replay.conf should be deleted"


if __name__ == "__main__":
    test_replay_crc_golden_vectors_are_deterministic_little_endian()
    test_replay_is_opt_in_and_preserves_existing_adc_sink_flow()
    test_readback_parser_walks_riff_chunks_and_checks_pipeline_crc()
    test_test_build_fragment_enables_replay_explicitly()
    print("ALL test_integrity_replay TESTS PASSED!")
