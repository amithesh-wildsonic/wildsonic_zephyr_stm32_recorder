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
    assert "config AUDIO_INTEGRITY_REPLAY" in kconfig
    assert "default n" in kconfig.split("config AUDIO_INTEGRITY_REPLAY", 1)[1].split("config AUDIO_SAMPLE_RATE", 1)[0]

    adc = (ROOT / "src" / "audio_in_stm32l4.c").read_text()
    process = adc.split("static bool process_half", 1)[1].split("static volatile uint32_t dma_ht_callbacks", 1)[0]
    assert process.index("diag_raw_sum += local_sum") < process.index("audio_in_integrity_sample")
    assert process.index("audio_in_integrity_sample") < process.index("sink->submit")
    callback = adc.split("static void dma_cb", 1)[1].split("void audio_in_get_callback_counts", 1)[0]
    assert "fs_read" not in callback and "crc32" not in callback.lower()


def test_readback_parser_walks_riff_chunks_and_checks_pipeline_crc():
    recorder = (ROOT / "src" / "recorder.c").read_text()
    parser = recorder.split("static int integrity_readback_crc", 1)[1].split("K_MEM_SLAB_DEFINE_STATIC", 1)[0]
    assert 'memcmp(chunk_hdr, "data", 4)' in parser
    assert "chunk_end > riff_end" in parser
    assert "fs_read(&file, buf, amount)" in parser
    assert "fs_read_exact(&file, chunk_hdr" in parser
    assert "pipeline_crc_bytes == exact_bytes" in recorder
    assert "integrity_expected_crc32 == st.integrity_readback_crc32" in recorder


def test_test_build_fragment_enables_replay_explicitly():
    conf = (ROOT / "configs" / "integrity_replay.conf").read_text()
    assert re.search(r"^CONFIG_AUDIO_INTEGRITY_REPLAY=y$", conf, re.MULTILINE)


if __name__ == "__main__":
    test_replay_crc_golden_vectors_are_deterministic_little_endian()
    test_replay_is_opt_in_and_preserves_existing_adc_sink_flow()
    test_readback_parser_walks_riff_chunks_and_checks_pipeline_crc()
    test_test_build_fragment_enables_replay_explicitly()
    print("ALL test_integrity_replay TESTS PASSED!")
