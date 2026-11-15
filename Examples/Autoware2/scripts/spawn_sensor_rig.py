#!/usr/bin/env python3
"""Spawn the Ego vehicle and configurable native ROS 2 sensor rig in CARLA.

Reads `configs/sensors.yaml` so cameras, LiDARs, IMU, GNSS, and radar can be adapted
at runtime without rebuilding CARLA or Autoware.
"""

from __future__ import annotations

import argparse
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from carla_env import (
    DEFAULT_SENSOR_CONFIG,
    DEFAULT_XOSC,
    EXAMPLE_ROOT,
    setup_pythonapi,
)

setup_pythonapi()
import carla  # noqa: E402


def parse_ego_init_from_xosc(xosc_path: Path) -> tuple[int, int, float, float] | None:
    """Extract Ego initial LanePosition (roadId, laneId, s, offset) from OpenSCENARIO."""
    if not xosc_path.is_file():
        return None
    tree = ET.parse(xosc_path)
    root = tree.getroot()
    for priv in root.findall(".//Init/Actions/Private"):
        if priv.attrib.get("entityRef") != "ego":
            continue
        lane_pos = priv.find(".//LanePosition")
        if lane_pos is not None:
            return (
                int(lane_pos.attrib["roadId"]),
                int(lane_pos.attrib["laneId"]),
                float(lane_pos.attrib.get("s", "0.0")),
                float(lane_pos.attrib.get("offset", "0.0")),
            )
    return None


def resolve_ego_spawn_transform(
    world: carla.World, cfg: dict, xosc_path: Path | None
) -> carla.Transform:
    """Compute Ego spawn transform from OpenSCENARIO init pose or YAML fallback."""
    carla_map = world.get_map()
    if xosc_path is not None:
        lane_pos = parse_ego_init_from_xosc(xosc_path)
        if lane_pos is not None:
            road_id, lane_id, s_val, _offset = lane_pos
            wp = carla_map.get_waypoint_xodr(road_id, lane_id, s_val)
            if wp is not None:
                tf = wp.transform
                tf.location.z += 0.35
                return tf

    fallback = cfg["vehicle"]["default_spawn"]
    return carla.Transform(
        carla.Location(x=float(fallback["x"]), y=float(fallback["y"]), z=float(fallback["z"])),
        carla.Rotation(yaw=float(fallback["yaw"])),
    )


def destroy_existing_ego(world: carla.World, role_name: str) -> None:
    """Cleanly remove any existing Ego actor and attached child sensors."""
    actors = world.get_actors()
    ego_ids = {
        actor.id
        for actor in actors.filter("vehicle.*")
        if actor.attributes.get("role_name") == role_name
    }
    if not ego_ids:
        return

    for actor in actors:
        parent = actor.parent
        if parent is not None and parent.id in ego_ids:
            actor.destroy()
    for actor_id in ego_ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            actor.destroy()
    time.sleep(0.2)


def make_relative_transform(entry: dict, wheelbase: float) -> carla.Transform:
    """Convert sensor placement YAML (`car_center` or `rear_axle`) into CARLA Transform."""
    tf = entry.get("transform", {})
    x = float(tf.get("x", 0.0))
    y = float(tf.get("y", 0.0))
    z = float(tf.get("z", 0.0))
    if entry.get("reference", "car_center") == "rear_axle":
        x -= wheelbase * 0.5
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(
            roll=float(tf.get("roll", 0.0)),
            pitch=float(tf.get("pitch", 0.0)),
            yaw=float(tf.get("yaw", 0.0)),
        ),
    )


def spawn_single_sensor(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    ego: carla.Actor,
    entry: dict,
    wheelbase: float,
) -> tuple[carla.Actor, list[str]]:
    """Spawn one sensor from YAML definition and enable its native C++ ROS 2 publisher."""
    bp_id = entry["blueprint"]
    bp = bp_lib.find(bp_id)
    ros_name = entry.get("ros_name", entry.get("name", "sensor"))
    ros_topic = entry.get("ros_topic_name", "")

    if bp.has_attribute("ros_name"):
        bp.set_attribute("ros_name", ros_name)
    if ros_topic and bp.has_attribute("ros_topic_name"):
        bp.set_attribute("ros_topic_name", ros_topic)

    for key, value in entry.get("attributes", {}).items():
        if bp.has_attribute(key):
            bp.set_attribute(key, str(value))

    rel_tf = make_relative_transform(entry, wheelbase)
    sensor = world.spawn_actor(
        bp,
        rel_tf,
        attach_to=ego,
        attachment_type=carla.AttachmentType.Rigid,
    )
    if hasattr(sensor, "enable_for_ros"):
        sensor.enable_for_ros()

    topics = expected_topics(bp_id, ros_topic)
    return sensor, topics


def expected_topics(bp_id: str, base_topic: str) -> list[str]:
    """Return ROS 2 topics published natively by CARLA C++ for a sensor blueprint."""
    if not base_topic:
        return []
    if bp_id.startswith("sensor.camera."):
        return [f"{base_topic}/image", f"{base_topic}/camera_info"]
    if bp_id == "sensor.other.autoware_gnss":
        return [f"{base_topic}/pose", f"{base_topic}/pose_with_covariance"]
    if bp_id == "sensor.other.vehicle_status":
        return [
            f"{base_topic}/velocity_status",
            f"{base_topic}/steering_status",
            f"{base_topic}/control_mode",
            f"{base_topic}/gear_status",
        ]
    return [base_topic]


def position_spectator_behind(world: carla.World, ego: carla.Actor) -> None:
    """Place the CARLA spectator camera in a clear chase view behind Ego."""
    tf = ego.get_transform()
    fwd = tf.get_forward_vector()
    spectator_tf = carla.Transform(
        carla.Location(
            x=tf.location.x - fwd.x * 10.0,
            y=tf.location.y - fwd.y * 10.0,
            z=tf.location.z + 5.0,
        ),
        carla.Rotation(pitch=-18.0, yaw=tf.rotation.yaw),
    )
    world.get_spectator().set_transform(spectator_tf)


def export_xodr_if_missing(world: carla.World) -> None:
    """Save CARLA's runtime OpenDRIVE XML alongside the example maps if absent."""
    xodr_path = EXAMPLE_ROOT / "maps" / "Town10HD" / "Town10HD_Opt.xodr"
    if not xodr_path.is_file():
        xodr_path.parent.mkdir(parents=True, exist_ok=True)
        xodr_path.write_text(world.get_map().to_opendrive(), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default="Town10HD_Opt", help="Ensure this CARLA map is active")
    parser.add_argument("--config", default=str(DEFAULT_SENSOR_CONFIG), help="Path to sensors.yaml")
    parser.add_argument("--xosc", default=str(DEFAULT_XOSC), help="Scenario file for Ego init pose")
    parser.add_argument("--wait", action="store_true", help="Keep script running until Ctrl-C")
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    client = carla.Client(args.host, args.port)
    client.set_timeout(15.0)
    world = client.get_world()
    if args.map and not world.get_map().name.endswith(args.map):
        print(f"Loading CARLA map '{args.map}'...")
        world = client.load_world(args.map)

    # Keep CARLA's C++ TF off so Autoware robot_state_publisher & EKF own /tf cleanly.
    if hasattr(world, "set_publish_tf"):
        world.set_publish_tf(False)

    # Ensure deterministic 20 Hz async ticking until Step 5 switches to sync lockstep.
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    export_xodr_if_missing(world)

    veh_cfg = cfg["vehicle"]
    role_name = veh_cfg.get("role_name", "ego")
    wheelbase = float(veh_cfg.get("wheelbase", 2.850))
    destroy_existing_ego(world, role_name)

    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.find(veh_cfg["blueprint"])
    ego_bp.set_attribute("role_name", role_name)
    if veh_cfg.get("ros2_autoware_control", True) and ego_bp.has_attribute("ros2_autoware_control"):
        ego_bp.set_attribute("ros2_autoware_control", "true")
    if "color" in veh_cfg and ego_bp.has_attribute("color"):
        ego_bp.set_attribute("color", str(veh_cfg["color"]))

    xosc_path = Path(args.xosc) if args.xosc else None
    spawn_tf = resolve_ego_spawn_transform(world, cfg, xosc_path)
    ego = world.spawn_actor(ego_bp, spawn_tf)
    if hasattr(ego, "enable_for_ros"):
        ego.enable_for_ros()

    spawned_rows: list[tuple[str, str, str, list[str]]] = []
    vs_cfg = cfg.get("vehicle_status", {})
    if vs_cfg.get("enabled", True):
        _, topics = spawn_single_sensor(world, bp_lib, ego, vs_cfg, wheelbase)
        spawned_rows.append(("vehicle_status", vs_cfg["blueprint"], vs_cfg["ros_name"], topics))

    for sensor_cfg in cfg.get("sensors", []):
        if not sensor_cfg.get("enabled", True):
            continue
        _, topics = spawn_single_sensor(world, bp_lib, ego, sensor_cfg, wheelbase)
        spawned_rows.append((
            sensor_cfg["name"],
            sensor_cfg["blueprint"],
            sensor_cfg.get("ros_name", sensor_cfg["name"]),
            topics,
        ))

    time.sleep(0.25)
    position_spectator_behind(world, ego)

    loc = ego.get_location()
    yaw = ego.get_transform().rotation.yaw
    print(f"Spawned Ego id={ego.id} ({veh_cfg['blueprint']}, role_name='{role_name}')")
    print(f"  CARLA pose : x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}, yaw={yaw:.1f} deg")
    print(f"  Active sensors ({len(spawned_rows)} from {Path(args.config).name}):")
    for name, bp_id, frame, topics in spawned_rows:
        print(f"    - {name:<16} [{bp_id}] frame='{frame}' -> {', '.join(topics)}")

    if args.wait:
        print("\nStreaming native ROS 2 topics. Press Ctrl-C to stop waiting...")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
