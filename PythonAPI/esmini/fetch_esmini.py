#!/usr/bin/env python3

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Utility to locate or download prebuilt esminiLib shared libraries."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import urllib.request
import zipfile

DEFAULT_ESMINI_VERSION = "v3.10.2"

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_VENDOR_DIR = PACKAGE_DIR / "_vendor"


def get_platform_library_names() -> tuple[str, ...]:
    """Return candidate shared library filenames for the current OS."""
    system = platform.system().lower()
    if system == "darwin":
        return ("libesminiLib.dylib", "esminiLib.dylib")
    if system == "windows":
        return ("esminiLib.dll", "libesminiLib.dll")
    return ("libesminiLib.so", "esminiLib.so")


def get_release_asset_name() -> str:
    """Return the official esmini GitHub release zip asset name for the host OS."""
    system = platform.system().lower()
    if system == "darwin":
        return "esmini-bin_macOS.zip"
    if system == "windows":
        return "esmini-bin_Windows.zip"
    if system == "linux":
        return "esmini-bin_Linux.zip"
    raise RuntimeError(f"Unsupported operating system for prebuilt esmini binaries: {platform.system()}")


def _search_root_for_library(root: Path) -> Path | None:
    """Search an esmini installation/extraction directory for esminiLib."""
    if not root.exists():
        return None
    if root.is_file():
        return root

    lib_names = get_platform_library_names()
    candidate_subdirs = (
        root / "bin",
        root / "lib",
        root / "esmini" / "bin",
        root / "esmini" / "lib",
        root,
    )
    for subdir in candidate_subdirs:
        for name in lib_names:
            candidate = subdir / name
            if candidate.is_file():
                return candidate.resolve()

    for name in lib_names:
        matches = list(root.rglob(name))
        if matches:
            return matches[0].resolve()
    return None


def find_esmini_root(explicit_path: str | Path | None = None) -> Path | None:
    """Find the root directory of an esmini installation (containing bin/ and resources/)."""
    lib_path = find_esmini_library(explicit_path=explicit_path, auto_download=False)
    if lib_path is None:
        return None
    # Typically <esmini_root>/bin/libesminiLib.*
    if lib_path.parent.name in ("bin", "lib", "Release", "Debug"):
        return lib_path.parent.parent
    return lib_path.parent


def find_esmini_library(
    explicit_path: str | Path | None = None,
    auto_download: bool = False,
    version: str = DEFAULT_ESMINI_VERSION,
) -> Path | None:
    """Locate the esminiLib shared library on disk.

    Search order:
    1. ``explicit_path`` argument (file or directory)
    2. ``ESMINI_LIB_PATH`` environment variable
    3. ``ESMINI_ROOT`` environment variable
    4. Local vendor cache ``PythonAPI/esmini/_vendor/``
    5. System library directories (``/usr/local/lib``, ``/opt/homebrew/lib``, ``/usr/lib``)
    6. If ``auto_download=True``, download the official release into ``_vendor/``.
    """
    search_targets: list[Path] = []
    if explicit_path:
        search_targets.append(Path(explicit_path).expanduser())
    if os.environ.get("ESMINI_LIB_PATH"):
        search_targets.append(Path(os.environ["ESMINI_LIB_PATH"]).expanduser())
    if os.environ.get("ESMINI_ROOT"):
        search_targets.append(Path(os.environ["ESMINI_ROOT"]).expanduser())
    search_targets.append(DEFAULT_VENDOR_DIR)
    for sys_dir in ("/usr/local/lib", "/opt/homebrew/lib", "/usr/lib"):
        p = Path(sys_dir)
        if p.is_dir():
            search_targets.append(p)

    for target in search_targets:
        found = _search_root_for_library(target)
        if found is not None:
            return found

    if auto_download:
        return download_esmini(version=version, dest_dir=DEFAULT_VENDOR_DIR)

    return None


def _clear_macos_quarantine(target_dir: Path) -> None:
    """Remove macOS Gatekeeper quarantine extended attributes on extracted binaries."""
    if platform.system().lower() != "darwin":
        return
    xattr_bin = shutil.which("xattr")
    if xattr_bin:
        subprocess.run(
            [xattr_bin, "-cr", str(target_dir)],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def download_esmini(
    version: str = DEFAULT_ESMINI_VERSION,
    dest_dir: str | Path = DEFAULT_VENDOR_DIR,
    force: bool = False,
) -> Path:
    """Download and extract the official esmini binary release from GitHub."""
    dest_path = Path(dest_dir).expanduser().resolve()
    version_tag = version if version.startswith("v") else f"v{version}"
    target_install_dir = dest_path / f"esmini-{version_tag}"

    if not force:
        existing = _search_root_for_library(target_install_dir)
        if existing is not None:
            _clear_macos_quarantine(target_install_dir)
            return existing

    asset_name = get_release_asset_name()
    url = f"https://github.com/esmini/esmini/releases/download/{version_tag}/{asset_name}"
    dest_path.mkdir(parents=True, exist_ok=True)
    zip_path = dest_path / f"esmini-{version_tag}-{asset_name}"

    print(f"[esmini] Downloading {url} ...")
    urllib.request.urlretrieve(url, zip_path)

    if target_install_dir.exists():
        shutil.rmtree(target_install_dir)
    target_install_dir.mkdir(parents=True, exist_ok=True)

    print(f"[esmini] Extracting {zip_path.name} to {target_install_dir} ...")
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(target_install_dir)

    try:
        zip_path.unlink()
    except OSError:
        pass

    _clear_macos_quarantine(target_install_dir)

    # Ensure shared libraries and binaries have executable permissions on POSIX
    if os.name == "posix":
        for root_dir, _, files in os.walk(target_install_dir):
            for fname in files:
                fpath = Path(root_dir) / fname
                if fpath.suffix in (".dylib", ".so", "") or "bin" in fpath.parts:
                    try:
                        fpath.chmod(fpath.stat().st_mode | 0o755)
                    except OSError:
                        pass

    lib_path = _search_root_for_library(target_install_dir)
    if lib_path is None:
        raise RuntimeError(
            f"Downloaded {url} into {target_install_dir}, but could not find any of "
            f"{get_platform_library_names()}"
        )
    print(f"[esmini] Ready: {lib_path}")
    return lib_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Locate or download prebuilt esminiLib shared library for CARLA."
    )
    parser.add_argument(
        "--version",
        default=DEFAULT_ESMINI_VERSION,
        help=f"esmini release tag to download (default: {DEFAULT_ESMINI_VERSION})",
    )
    parser.add_argument(
        "--dest",
        default=str(DEFAULT_VENDOR_DIR),
        help=f"Destination directory (default: {DEFAULT_VENDOR_DIR})",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-download even if already cached.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Only check if esminiLib is already available without downloading.",
    )
    args = parser.parse_args(argv)

    if args.check_only:
        lib = find_esmini_library(auto_download=False)
        if lib is None:
            print("esminiLib not found.", file=sys.stderr)
            return 1
        print(lib)
        return 0

    lib = download_esmini(version=args.version, dest_dir=args.dest, force=args.force)
    print(lib)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

