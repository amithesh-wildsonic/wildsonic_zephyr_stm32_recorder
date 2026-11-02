#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
STM32L476RG Firmware Release Package Generator & Header Validator
=================================================================
Packages raw Zephyr binary images into self-verifying, bootloader-compatible
release packages (.bin) for the WildSonic STM32 dual-bank bootloader.

Dual-Bank Layout:
  Bank 1 (0x08000000 - 0x0807FFFF, 512 KB): Bootloader
  Bank 2 (0x08080000 - 0x080F7FFF, 480 KB): Application Image
  Bank 2 (0x080F8000 - 0x080FFFFF,  32 KB): Config Partition

Header Format (64 bytes):
  uint32_t magic;           // 0x57533332 ("WS32")
  uint16_t format_version;  // 1
  uint16_t flags;           // 0
  uint32_t target_mcu_id;   // 0x00004760 (STM32L476)
  uint8_t  version_major;   // e.g. 2
  uint8_t  version_minor;   // e.g. 1
  uint8_t  version_patch;   // e.g. 0
  uint8_t  reserved1;       // 0
  uint32_t payload_offset;  // 64 (start of payload)
  uint32_t payload_size;    // size in bytes
  uint32_t target_load_addr;// 0x08080000
  uint32_t entry_point;     // Vector table reset handler
  uint32_t payload_crc32;   // IEEE 802.3 CRC32 of payload
  uint32_t config_version;  // Compatible config version (1)
  uint8_t  reserved2[20];   // Padding / future reserved
  uint32_t header_crc32;    // CRC32 of header bytes 0..59
"""

import argparse
import struct
import sys
import zlib
from pathlib import Path

RELEASE_HEADER_MAGIC = 0x57533332  # "WS32"
HEADER_FORMAT_VERSION = 1
STM32L476_MCU_ID = 0x00004760
DEFAULT_TARGET_LOAD_ADDR = 0x08080000
DEFAULT_CONFIG_VERSION = 1
HEADER_SIZE = 64


def compute_crc32(data: bytes) -> int:
    """Standard IEEE 802.3 CRC32."""
    return zlib.crc32(data) & 0xFFFFFFFF


def parse_version(ver_str: str):
    """Parse 'X.Y.Z' into (major, minor, patch)."""
    parts = ver_str.strip().lstrip("v").split(".")
    major = int(parts[0]) if len(parts) > 0 else 1
    minor = int(parts[1]) if len(parts) > 1 else 0
    patch = int(parts[2]) if len(parts) > 2 else 0
    return major, minor, patch


def create_release_package(
    input_bin_path: Path,
    output_pkg_path: Path,
    version_str: str = "2.1.0",
    target_load_addr: int = DEFAULT_TARGET_LOAD_ADDR,
    target_mcu_id: int = STM32L476_MCU_ID,
    config_version: int = DEFAULT_CONFIG_VERSION,
) -> bool:
    if not input_bin_path.exists():
        print(f"Error: Input binary '{input_bin_path}' not found.", file=sys.stderr)
        return False

    payload = input_bin_path.read_bytes()
    payload_size = len(payload)
    if payload_size < 8:
        print(f"Error: Input binary too small ({payload_size} bytes).", file=sys.stderr)
        return False

    # Extract initial SP and Reset Vector from Cortex-M vector table
    sp, reset_handler = struct.unpack_from("<II", payload, 0)

    # Sanity checks on Cortex-M vector table
    # SP must be in SRAM (0x20000000 - 0x20020000)
    if not (0x20000000 <= sp <= 0x20020000):
        print(f"Warning: Vector SP (0x{sp:08X}) outside expected STM32 SRAM range.")
    # Reset vector must have Thumb bit set (odd)
    if (reset_handler & 1) == 0:
        print(f"Warning: Reset vector (0x{reset_handler:08X}) does not have Thumb bit set.")

    major, minor, patch = parse_version(version_str)
    payload_crc = compute_crc32(payload)

    # Pack first 60 bytes of header (everything before header_crc32)
    # Format:
    #   I (magic)
    #   H (format_version)
    #   H (flags)
    #   I (target_mcu_id)
    #   B (major)
    #   B (minor)
    #   B (patch)
    #   B (reserved1)
    #   I (payload_offset)
    #   I (payload_size)
    #   I (target_load_addr)
    #   I (entry_point)
    #   I (payload_crc32)
    #   I (config_version)
    #   24s (reserved2)
    header_pre = struct.pack(
        "<IHHIBBBBIIIIII20s",
        RELEASE_HEADER_MAGIC,
        HEADER_FORMAT_VERSION,
        0,  # flags
        target_mcu_id,
        major,
        minor,
        patch,
        0,  # reserved1
        HEADER_SIZE,
        payload_size,
        target_load_addr,
        reset_handler,
        payload_crc,
        config_version,
        b"\x00" * 20,
    )
    assert len(header_pre) == 60, f"Header prefix length must be 60, got {len(header_pre)}"

    header_crc = compute_crc32(header_pre)
    full_header = header_pre + struct.pack("<I", header_crc)
    assert len(full_header) == HEADER_SIZE, f"Full header size must be 64, got {len(full_header)}"

    output_pkg_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_pkg_path, "wb") as f:
        f.write(full_header)
        f.write(payload)

    print(f"Successfully packaged release image:")
    print(f"  Input Binary : {input_bin_path} ({payload_size} bytes)")
    print(f"  Output Image : {output_pkg_path} ({len(full_header) + payload_size} bytes)")
    print(f"  Version      : {major}.{minor}.{patch}")
    print(f"  Target MCU   : 0x{target_mcu_id:08X} (STM32L476)")
    print(f"  Load Address : 0x{target_load_addr:08X}")
    print(f"  Entry Point  : 0x{reset_handler:08X}")
    print(f"  Payload CRC32: 0x{payload_crc:08X}")
    print(f"  Header CRC32 : 0x{header_crc:08X}")
    return True


def verify_release_package(pkg_path: Path) -> bool:
    if not pkg_path.exists():
        print(f"Error: Package '{pkg_path}' not found.", file=sys.stderr)
        return False

    data = pkg_path.read_bytes()
    if len(data) < HEADER_SIZE:
        print(f"Error: File '{pkg_path}' smaller than 64-byte header.", file=sys.stderr)
        return False

    header_bytes = data[:HEADER_SIZE]
    (
        magic,
        fmt_ver,
        flags,
        mcu_id,
        major,
        minor,
        patch,
        r1,
        offset,
        size,
        load_addr,
        entry,
        payload_crc,
        cfg_ver,
        r2,
        header_crc,
    ) = struct.unpack("<IHHIBBBBIIIIII20sI", header_bytes)

    if magic != RELEASE_HEADER_MAGIC:
        print(f"FAIL: Bad magic 0x{magic:08X} (expected 0x{RELEASE_HEADER_MAGIC:08X})", file=sys.stderr)
        return False

    computed_hdr_crc = compute_crc32(header_bytes[:60])
    if computed_hdr_crc != header_crc:
        print(f"FAIL: Header CRC mismatch (computed 0x{computed_hdr_crc:08X}, expected 0x{header_crc:08X})", file=sys.stderr)
        return False

    if len(data) < offset + size:
        print(f"FAIL: File truncated (got {len(data)}, expected {offset + size})", file=sys.stderr)
        return False

    payload = data[offset : offset + size]
    computed_payload_crc = compute_crc32(payload)
    if computed_payload_crc != payload_crc:
        print(f"FAIL: Payload CRC mismatch (computed 0x{computed_payload_crc:08X}, expected 0x{payload_crc:08X})", file=sys.stderr)
        return False

    sp, reset_handler = struct.unpack_from("<II", payload, 0)
    print(f"VALID RELEASE PACKAGE: {pkg_path.name}")
    print(f"  Version      : {major}.{minor}.{patch}")
    print(f"  MCU ID       : 0x{mcu_id:08X}")
    print(f"  Load Address : 0x{load_addr:08X}")
    print(f"  Payload Size : {size} bytes")
    print(f"  Reset Vector : 0x{reset_handler:08X}")
    print(f"  Initial SP   : 0x{sp:08X}")
    print(f"  Payload CRC  : 0x{payload_crc:08X} (OK)")
    print(f"  Header CRC   : 0x{header_crc:08X} (OK)")
    return True


def main():
    parser = argparse.ArgumentParser(description="WildSonic STM32 Release Package Generator")
    parser.add_argument("-i", "--input", type=Path, help="Path to raw application binary (.bin)")
    parser.add_argument("-o", "--output", type=Path, help="Path for generated release package (.bin)")
    parser.add_argument("-v", "--version", default="2.1.0", help="Release version string (e.g. 2.1.0)")
    parser.add_argument("--load-addr", type=lambda x: int(x, 0), default=DEFAULT_TARGET_LOAD_ADDR, help="Target flash load address (default 0x08080000)")
    parser.add_argument("--verify", type=Path, help="Verify an existing package file")

    args = parser.parse_args()

    if args.verify:
        success = verify_release_package(args.verify)
        sys.exit(0 if success else 1)

    if not args.input or not args.output:
        parser.print_help()
        sys.exit(1)

    success = create_release_package(
        args.input,
        args.output,
        version_str=args.version,
        target_load_addr=args.load_addr,
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
