#!/usr/bin/env python3
"""
Test script for WAV header generation, 16-bit / 32-bit PCM parsing,
and structural integrity validation routines (self-contained, native parser).
"""

import os
import struct
import sys
import tempfile
import numpy as np


def generate_test_wav(filepath, sample_rate=192000, bits=32, duration_sec=1.0, freq=1000.0,
                      add_dc=True, add_clipping=False, add_flatline=False, add_discontinuity=False):
    """
    Generate synthetic RIFF/WAVE matching the firmware's 512-byte padded header format.
    """
    channels = 1
    block_align = channels * (bits // 8)
    byte_rate = sample_rate * block_align
    total_samples = int(sample_rate * duration_sec)
    data_bytes = total_samples * block_align
    header_bytes = 32768
    junk_bytes = header_bytes - 12 - 24 - 8 - 8

    # Generate signal
    t = np.linspace(0, duration_sec, total_samples, endpoint=False)
    sig = np.sin(2.0 * np.pi * freq * t)

    if bits == 16:
        pcm = (sig * 16000.0).astype(np.int16)
        if add_dc:
            pcm = (pcm.astype(np.int32) + 2000).clip(-32768, 32767).astype(np.int16)
        if add_clipping:
            pcm[:100] = 32767
            pcm[100:200] = -32768
        if add_flatline:
            pcm[500:600] = 1234
        if add_discontinuity:
            pcm[1000] = -32000
            pcm[1001] = +32000
        raw_pcm = pcm.tobytes()
    elif bits == 32:
        pcm = (sig * 1000000000.0).astype(np.int32)
        if add_dc:
            pcm = (pcm.astype(np.int64) + 50000000).clip(-2147483648, 2147483647).astype(np.int32)
        if add_clipping:
            pcm[:100] = 2147483647
            pcm[100:200] = -2147483648
        if add_flatline:
            pcm[500:600] = 12345678
        if add_discontinuity:
            pcm[1000] = -2000000000
            pcm[1001] = +2000000000
        raw_pcm = pcm.tobytes()
    else:
        raise ValueError("Unsupported bit depth")

    # Construct cluster-aligned padded header
    hdr = bytearray(header_bytes)
    hdr[0:4] = b"RIFF"
    struct.pack_into("<I", hdr, 4, (header_bytes - 8) + data_bytes)
    hdr[8:12] = b"WAVE"

    hdr[12:16] = b"fmt "
    struct.pack_into("<I", hdr, 16, 16)
    struct.pack_into("<H", hdr, 20, 1)  # PCM
    struct.pack_into("<H", hdr, 22, channels)
    struct.pack_into("<I", hdr, 24, sample_rate)
    struct.pack_into("<I", hdr, 28, byte_rate)
    struct.pack_into("<H", hdr, 32, block_align)
    struct.pack_into("<H", hdr, 34, bits)

    hdr[36:40] = b"JUNK"
    struct.pack_into("<I", hdr, 40, junk_bytes)

    data_hdr_start = 36 + 8 + junk_bytes
    hdr[data_hdr_start:data_hdr_start + 4] = b"data"
    struct.pack_into("<I", hdr, data_hdr_start + 4, data_bytes)

    with open(filepath, "wb") as f:
        f.write(hdr)
        f.write(raw_pcm)


def parse_wav_native(filepath):
    """
    Native RIFF/WAVE parser for self-contained test execution.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    filesize = os.path.getsize(filepath)
    with open(filepath, "rb") as f:
        header = f.read(min(32768, filesize))
        assert header[0:4] == b"RIFF", "Invalid RIFF identifier"
        riff_size = struct.unpack_from("<I", header, 4)[0]
        assert header[8:12] == b"WAVE", "Invalid WAVE identifier"

        # Search chunks in header
        offset = 12
        fmt_found = False
        data_found = False
        sample_rate = 0
        bits_per_sample = 0
        block_align = 0
        byte_rate = 0
        data_size = 0
        data_offset = 0

        while offset + 8 <= len(header):
            tag = header[offset:offset+4]
            sz = struct.unpack_from("<I", header, offset+4)[0]
            if tag == b"fmt ":
                fmt_found = True
                audio_format = struct.unpack_from("<H", header, offset+8)[0]
                channels = struct.unpack_from("<H", header, offset+10)[0]
                sample_rate = struct.unpack_from("<I", header, offset+12)[0]
                byte_rate = struct.unpack_from("<I", header, offset+16)[0]
                block_align = struct.unpack_from("<H", header, offset+20)[0]
                bits_per_sample = struct.unpack_from("<H", header, offset+22)[0]
            elif tag == b"data":
                data_found = True
                data_size = sz
                data_offset = offset + 8
                break
            offset += 8 + sz
            # Word align
            if offset % 2 != 0:
                offset += 1

        assert fmt_found, "fmt chunk not found"
        assert data_found, "data chunk not found"

        f.seek(data_offset)
        pcm_bytes = f.read(data_size)

    dtype = np.int16 if bits_per_sample == 16 else np.int32
    samples = np.frombuffer(pcm_bytes, dtype=dtype)
    total_frames = len(samples)

    # Clipping count
    max_val = 32767 if bits_per_sample == 16 else 2147483647
    min_val = -32768 if bits_per_sample == 16 else -2147483648
    clipping_count = int(np.sum((samples >= max_val) | (samples <= min_val)))

    # Flatline detection
    diffs = np.diff(samples)
    flat_runs = 0
    max_flat_run = 0
    cur_run = 0
    for d in diffs:
        if d == 0:
            cur_run += 1
            if cur_run > max_flat_run:
                max_flat_run = cur_run
        else:
            cur_run = 0

    # Discontinuity detection
    disc_threshold = 30000 if bits_per_sample == 16 else 1500000000
    disc_count = int(np.sum(np.abs(diffs) > disc_threshold))

    return {
        "metadata": {
            "fmt": {
                "sample_rate": sample_rate,
                "bits_per_sample": bits_per_sample,
                "block_align": block_align,
                "byte_rate": byte_rate
            },
            "total_frames": total_frames,
            "data_size_bytes": data_size,
            "filesize_bytes": filesize
        },
        "analysis": {
            "total_clipping_count": clipping_count,
            "max_flatline_run_samples": max_flat_run,
            "discontinuities_count": disc_count
        }
    }


def run_tests():
    with tempfile.TemporaryDirectory() as tmpdir:
        # Test 1: Clean 192 kHz 32-bit WAV
        w32_clean = os.path.join(tmpdir, "test_32bit_clean.wav")
        generate_test_wav(w32_clean, sample_rate=192000, bits=32, duration_sec=0.1, freq=1000.0)

        data = parse_wav_native(w32_clean)
        meta = data["metadata"]
        analysis = data["analysis"]
        assert meta["fmt"]["sample_rate"] == 192000
        assert meta["fmt"]["bits_per_sample"] == 32
        assert meta["fmt"]["block_align"] == 4
        assert meta["fmt"]["byte_rate"] == 768000
        assert meta["total_frames"] == 19200
        assert analysis["total_clipping_count"] == 0
        assert analysis["discontinuities_count"] == 0
        print("Test 1: 32-bit clean WAV analysis passed.")

        # Test 2: Clean 192 kHz 16-bit WAV
        w16_clean = os.path.join(tmpdir, "test_16bit_clean.wav")
        generate_test_wav(w16_clean, sample_rate=192000, bits=16, duration_sec=0.1, freq=1000.0)

        data = parse_wav_native(w16_clean)
        meta = data["metadata"]
        assert meta["fmt"]["sample_rate"] == 192000
        assert meta["fmt"]["bits_per_sample"] == 16
        assert meta["fmt"]["block_align"] == 2
        assert meta["fmt"]["byte_rate"] == 384000
        print("Test 2: 16-bit clean WAV analysis passed.")

        # Test 3: Defect detection (clipping, flatline, discontinuity)
        w32_defect = os.path.join(tmpdir, "test_32bit_defect.wav")
        generate_test_wav(w32_defect, sample_rate=192000, bits=32, duration_sec=0.1, freq=1000.0,
                          add_clipping=True, add_flatline=True, add_discontinuity=True)

        data = parse_wav_native(w32_defect)
        analysis = data["analysis"]
        assert analysis["total_clipping_count"] > 0, "Failed to detect clipping"
        assert analysis["max_flatline_run_samples"] >= 32, "Failed to detect flatline"
        assert analysis["discontinuities_count"] > 0, "Failed to detect discontinuity"
        print("Test 3: Defect detection (clipping, flatline, discontinuity) passed.")

        # Test 4: Exact 5-second 192 kHz 32-bit recording with partial last block
        w32_5sec = os.path.join(tmpdir, "test_32bit_5sec_exact.wav")
        generate_test_wav(w32_5sec, sample_rate=192000, bits=32, duration_sec=5.0, freq=1000.0)

        data = parse_wav_native(w32_5sec)
        meta = data["metadata"]
        analysis = data["analysis"]
        assert meta["fmt"]["sample_rate"] == 192000
        assert meta["fmt"]["bits_per_sample"] == 32
        assert meta["total_frames"] == 960000, f"Expected 960,000 frames, got {meta['total_frames']}"
        assert meta["data_size_bytes"] == 3840000, f"Expected 3,840,000 bytes, got {meta['data_size_bytes']}"
        assert meta["filesize_bytes"] == 32768 + 3840000
        full_blocks = meta["total_frames"] // 2048
        partial_samples = meta["total_frames"] % 2048
        assert full_blocks == 468, f"Expected 468 full blocks, got {full_blocks}"
        assert partial_samples == 1536, f"Expected 1536 partial samples, got {partial_samples}"
        assert analysis["discontinuities_count"] == 0
        print("Test 4: Exact 5-second 192 kHz (960,000 samples, 468 full + 1536 partial block) passed.")

        # Test 5: Arbitrary non-block-aligned duration (0.05 s = 9,600 samples at 192 kHz)
        w32_partial = os.path.join(tmpdir, "test_32bit_partial_block.wav")
        generate_test_wav(w32_partial, sample_rate=192000, bits=32, duration_sec=0.05, freq=1000.0)

        data = parse_wav_native(w32_partial)
        meta = data["metadata"]
        assert meta["total_frames"] == 9600, f"Expected 9,600 frames, got {meta['total_frames']}"
        assert meta["data_size_bytes"] == 9600 * 4
        full_blocks = meta["total_frames"] // 2048
        partial_samples = meta["total_frames"] % 2048
        assert full_blocks == 4, f"Expected 4 full blocks, got {full_blocks}"
        assert partial_samples == 1408, f"Expected 1408 partial samples, got {partial_samples}"
        print("Test 5: Arbitrary duration (0.05 s / 9,600 samples, 4 full + 1408 partial block) passed.")

        # Test 6: Detect 512 padded samples defect (regressed behavior simulation)
        w32_padded = os.path.join(tmpdir, "test_32bit_padded_defect.wav")
        dur_padded = 960512.0 / 192000.0
        generate_test_wav(w32_padded, sample_rate=192000, bits=32, duration_sec=dur_padded, freq=1000.0)
        data = parse_wav_native(w32_padded)
        meta = data["metadata"]
        excess_frames = meta["total_frames"] - 960000
        assert excess_frames == 512, f"Expected 512 excess frames in defect simulation, got {excess_frames}"
        assert meta["data_size_bytes"] == 3842048
        print("Test 6: 512-sample padding defect successfully detected and quantified.")

        # Test 7: Early capture abort simulation (verify aborted capture does not pass)
        w_missing = os.path.join(tmpdir, "nonexistent_aborted.wav")
        try:
            parse_wav_native(w_missing)
            passed = False
        except FileNotFoundError:
            passed = True
        assert passed, "Native parser should raise FileNotFoundError on missing file"
        print("Test 7: Aborted capture file-missing check properly returned error code.")

    print("\nAll host-side WAV validation tests PASSED!")


if __name__ == "__main__":
    run_tests()
