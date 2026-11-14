#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate or verify release_manifest.json for PRODUCTION & TEST images (Section 7g)."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Optional


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def git_info(repo_root: Path) -> Dict[str, Any]:
    commit = subprocess.check_output(
        ["git", "-C", str(repo_root), "rev-parse", "HEAD"], text=True
    ).strip()
    branch = subprocess.check_output(
        ["git", "-C", str(repo_root), "rev-parse", "--abbrev-ref", "HEAD"], text=True
    ).strip()
    status = subprocess.check_output(
        ["git", "-C", str(repo_root), "status", "--porcelain"], text=True
    ).strip()
    return {
        "commit": commit,
        "short_commit": commit[:12],
        "branch": branch,
        "dirty": bool(status),
    }


def artifact_entry(build_dir: Path) -> Dict[str, Any]:
    zephyr_dir = build_dir / "zephyr"
    entry: Dict[str, Any] = {"build_dir": str(build_dir)}
    for name in ("zephyr.elf", "zephyr.bin", "zephyr.hex"):
        p = zephyr_dir / name
        if p.is_file():
            entry[name] = {
                "path": str(p),
                "size_bytes": p.stat().st_size,
                "sha256": sha256_file(p),
            }
    return entry


def build_manifest(
    repo_root: Path,
    prod_dir: Path,
    test_dir: Path,
    bootloader_dir: Optional[Path] = None,
) -> Dict[str, Any]:
    info = git_info(repo_root)
    manifest: Dict[str, Any] = {
        "schema_version": 1,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"
        ),
        "git": info,
        "images": {
            "production": artifact_entry(prod_dir),
            "test": artifact_entry(test_dir),
        },
    }
    if bootloader_dir is not None and (bootloader_dir / "zephyr").is_dir():
        manifest["images"]["bootloader"] = artifact_entry(bootloader_dir)
    return manifest


def verify_manifest(repo_root: Path, manifest_path: Path) -> bool:
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    info = git_info(repo_root)
    if data.get("git", {}).get("commit") != info["commit"]:
        print(
            f"FAIL: manifest commit {data.get('git', {}).get('commit')} != HEAD {info['commit']}; "
            "TEST must be re-run before release.",
            file=sys.stderr,
        )
        return False
    for profile, entry in data.get("images", {}).items():
        for name, meta in entry.items():
            if not isinstance(meta, dict) or "sha256" not in meta:
                continue
            p = Path(meta["path"])
            if not p.is_file():
                print(f"FAIL: missing {profile}/{name} at {p}", file=sys.stderr)
                return False
            actual = sha256_file(p)
            if actual != meta["sha256"]:
                print(
                    f"FAIL: SHA-256 mismatch for {profile}/{name}: expected {meta['sha256']}, got {actual}",
                    file=sys.stderr,
                )
                return False
    print(f"OK: release_manifest verified for commit {info['short_commit']}.")
    return True


def main(argv: Optional[list[str]] = None) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prod-dir", type=Path, default=repo_root / "build" / "prod")
    parser.add_argument("--test-dir", type=Path, default=repo_root / "build" / "test")
    parser.add_argument("--bootloader-dir", type=Path, default=repo_root / "build" / "bootloader")
    parser.add_argument("--out", type=Path, default=repo_root / "release_manifest.json")
    parser.add_argument("--verify", action="store_true", help="Verify existing manifest matches HEAD and binaries")
    args = parser.parse_args(argv)

    if args.verify:
        return 0 if verify_manifest(repo_root, args.out) else 1

    for req_dir, label in ((args.prod_dir, "production"), (args.test_dir, "test")):
        if not (req_dir / "zephyr" / "zephyr.elf").is_file():
            print(f"Error: {label} ELF not found at {req_dir / 'zephyr' / 'zephyr.elf'}", file=sys.stderr)
            return 2

    manifest = build_manifest(repo_root, args.prod_dir, args.test_dir, args.bootloader_dir)
    args.out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.out} (commit {manifest['git']['short_commit']}, dirty={manifest['git']['dirty']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
