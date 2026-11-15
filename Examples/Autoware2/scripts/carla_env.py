#!/usr/bin/env python3
"""Environment helpers locating CARLA PythonAPI, esmini bindings, and example paths."""

from __future__ import annotations

import os
import sys
from pathlib import Path

EXAMPLE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SENSOR_CONFIG = EXAMPLE_ROOT / "configs" / "sensors.yaml"
DEFAULT_XOSC = EXAMPLE_ROOT / "scenarios" / "town10hd_autoware_cut_in.xosc"
DEFAULT_DDS_HOST_XML = EXAMPLE_ROOT / "configs" / "cyclonedds" / "cyclonedds_host.xml"


def find_carla_root() -> Path:
    """Locate the active CARLA checkout or packaged Shipping build directory."""
    candidates: list[Path] = []
    if env_root := os.environ.get("CARLA_ROOT"):
        candidates.append(Path(env_root).expanduser().resolve())

    repo_root = EXAMPLE_ROOT.parent.parent
    candidates.extend([
        repo_root / "Build" / "Package" / "Carla-1.0-Mac-Shipping",
        repo_root,
    ])

    for root in candidates:
        if (root / "PythonAPI").is_dir():
            return root
    raise RuntimeError(
        "Unable to locate CARLA PythonAPI. Set CARLA_ROOT or run from the CARLA repository."
    )


def setup_pythonapi() -> Path:
    """Add CARLA PythonAPI and esmini directories to sys.path and import carla."""
    carla_root = find_carla_root()
    pythonapi_dir = carla_root / "PythonAPI"
    carla_pkg_dir = pythonapi_dir / "carla"
    for path in (pythonapi_dir, carla_pkg_dir):
        path_str = str(path)
        if path.is_dir() and path_str not in sys.path:
            sys.path.insert(0, path_str)
    return carla_root
