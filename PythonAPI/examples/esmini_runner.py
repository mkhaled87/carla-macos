#!/usr/bin/env python3

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Run ASAM OpenSCENARIO 1.x (.xosc) files in CARLA using esminiLib."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Ensure PythonAPI/ is on sys.path when invoked directly as a script
_PYTHON_API_DIR = Path(__file__).resolve().parent.parent
if str(_PYTHON_API_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_API_DIR))

from esmini import CarlaEsminiRunner, RunnerConfig  # noqa: E402


def parse_param_overrides(raw_params: list[str] | None) -> dict[str, str]:
    """Parse ``KEY=VALUE`` CLI strings into a dictionary."""
    result: dict[str, str] = {}
    if not raw_params:
        return result
    for item in raw_params:
        if "=" not in item:
            raise ValueError(f"Invalid --param '{item}': expected KEY=VALUE format.")
        k, v = item.split("=", 1)
        result[k.strip()] = v.strip()
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Execute ASAM OpenSCENARIO (.xosc) scenarios in CARLA via esminiLib."
    )
    parser.add_argument(
        "--xosc",
        "--openscenario",
        dest="xosc",
        required=True,
        help="Path to the OpenSCENARIO (.xosc) file to execute.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="CARLA server host address (default: 127.0.0.1).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=2000,
        help="CARLA server RPC port (default: 2000).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="CARLA client connection timeout in seconds (default: 15.0).",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=20.0,
        help="Simulation tick frequency in Hz (default: 20.0 -> dt=0.05s).",
    )
    parser.add_argument(
        "--time-scale",
        type=float,
        default=1.0,
        help="Real-time pacing multiplier (1.0 = real-time, 0.0 = unthrottled).",
    )
    parser.add_argument(
        "--max-time",
        type=float,
        default=None,
        help="Maximum scenario simulation time in seconds.",
    )
    parser.add_argument(
        "--reload-world",
        action="store_true",
        help="Load the CARLA map / OpenDRIVE referenced by the .xosc file before running.",
    )
    parser.add_argument(
        "--passive-tick",
        action="store_true",
        help="Use world.wait_for_tick() instead of world.tick() (when an external node ticks CARLA).",
    )

    # Ego provisioning group
    ego_group = parser.add_mutually_exclusive_group()
    ego_group.add_argument(
        "--spawn-ego",
        action="store_true",
         default=True,
        help="Spawn the Ego vehicle in CARLA at the initial .xosc pose (default).",
    )
    ego_group.add_argument(
        "--wait-for-ego",
        action="store_true",
        help="Wait for an external AD stack to spawn the Ego vehicle in CARLA, then attach to it.",
    )

    parser.add_argument(
        "--ego-wait-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for an external Ego vehicle when --wait-for-ego is used (default: 30.0).",
    )
    parser.add_argument(
        "--no-teleport-ego",
        action="store_true",
        help="Do not teleport an already-spawned external Ego vehicle to the .xosc initial pose at t=0.",
    )
    parser.add_argument(
        "--keep-ego",
        action="store_true",
        help="Do not destroy the runner-spawned Ego vehicle when the scenario ends.",
    )
    parser.add_argument(
        "--ego-name",
        default=None,
        help="OpenSCENARIO entity name of the Ego vehicle (auto-detected if omitted).",
    )
    parser.add_argument(
        "--ego-role-name",
        default="hero",
        help="CARLA role_name (and ros_name) for the Ego vehicle (default: 'hero', or 'ego' with --autoware).",
    )
    parser.add_argument(
        "--ego-blueprint",
        default=None,
        help="Explicit CARLA blueprint ID for the Ego vehicle (e.g., vehicle.lincoln.mkz_2020).",
    )
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="Enable CARLA native ROS 2 interface on the Ego vehicle.",
    )
    parser.add_argument(
        "--autoware",
        action="store_true",
        help="Configure the Ego vehicle for CARLA native ROS 2 Autoware control (ros2_autoware_control=true).",
    )

    # Visualization & esmini options
    parser.add_argument(
        "--follow-ego",
        action="store_true",
        help="Follow the Ego vehicle with the CARLA spectator camera.",
    )
    parser.add_argument(
        "--draw-ghost",
        action="store_true",
        help="Draw the OpenSCENARIO ghost vehicle (if present) in CARLA via world.debug.",
    )
    parser.add_argument(
        "--no-center-offset",
        action="store_true",
        help="Disable rear-axle to bounding-box-center offset translation for vehicles.",
    )
    parser.add_argument(
        "--esmini-viewer",
        type=int,
        default=0,
        help="esmini viewer mode bitmask (0=off, 1=window, default: 0).",
    )
    parser.add_argument(
        "--param",
        action="append",
         default=[],
        metavar="KEY=VALUE",
        help="Override an OpenSCENARIO ParameterDeclaration (repeatable, e.g. --param HostSpeed=20).",
    )
    parser.add_argument(
        "--add-path",
        action="append",
        default=[],
        metavar="DIR",
        help="Additional search directory for OpenDRIVE (.xodr) or Catalog files (repeatable).",
    )
    parser.add_argument(
        "--esmini-lib",
        default=None,
        help="Explicit path to libesminiLib (.dylib / .so / .dll).",
    )
    parser.add_argument(
        "--record-dat",
        default=None,
        help="Path to save an esmini .dat recording file.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for esmini.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Disable esmini console and logfile output.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run the scenario in esminiLib without connecting to a CARLA server.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    dt = 1.0 / max(1.0, float(args.hz))
    params = parse_param_overrides(args.param)

    config = RunnerConfig(
        xosc_path=args.xosc,
        host=args.host,
        port=args.port,
        client_timeout=args.timeout,
        delta_seconds=dt,
        sync_mode=not args.passive_tick,
        passive_tick=args.passive_tick,
        reload_world=args.reload_world,
        time_scale=args.time_scale,
        max_sim_time=args.max_time,
        ego_name=args.ego_name,
        ego_role_name=args.ego_role_name,
        ego_blueprint=args.ego_blueprint,
        spawn_ego=not args.wait_for_ego,
        wait_for_ego=args.wait_for_ego,
        ego_wait_timeout=args.ego_wait_timeout,
        teleport_external_ego=not args.no_teleport_ego,
        keep_ego_on_exit=args.keep_ego,
        enable_ros2=args.ros2,
        autoware_mode=args.autoware,
        follow_ego=args.follow_ego,
        draw_ghost=args.draw_ghost,
        apply_center_offset=not args.no_center_offset,
        esmini_viewer=args.esmini_viewer,
        disable_esmini_log=args.quiet,
        record_dat_path=args.record_dat,
        random_seed=args.seed,
        parameters=params,
        additional_paths=args.add_path,
        esmini_lib_path=args.esmini_lib,
        dry_run=args.dry_run,
    )

    runner = CarlaEsminiRunner(config)
    try:
        sim_duration = runner.run()
        print(
            f"[esmini_runner] Scenario '{Path(args.xosc).name}' finished at t={sim_duration:.2f}s "
            f"({len(runner.triggered_conditions)} conditions triggered)."
        )
        return 0
    except KeyboardInterrupt:
        print("\n[esmini_runner] Interrupted by user, cleaning up...")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

