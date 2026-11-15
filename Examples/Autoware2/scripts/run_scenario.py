#!/usr/bin/env python3
"""Step 5: Run OpenSCENARIO cut-in co-simulation with Autoware 2.1.0 in closed loop.

Connects to the CARLA server, computes a downstream lane goal from Ego's localized
position, instructs Autoware 2.1.0 inside the Docker container to plan the route and
engage Autonomous mode, and steps `CarlaEsminiRunner` + `world.tick()` in 20 Hz lockstep
while esmini orchestrates NPCs (`lead_npc`, `cut_in_npc`, `sidewalk_walker`).
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

from carla_env import DEFAULT_XOSC, setup_pythonapi

setup_pythonapi()
import carla  # noqa: E402
from esmini.runner import CarlaEsminiRunner, RunnerConfig  # noqa: E402


def find_ego(world: carla.World, role_name: str) -> carla.Actor:
    """Locate the already-spawned Ego vehicle by `role_name`."""
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    raise RuntimeError(
        f"Ego vehicle with role_name='{role_name}' not found. Run Step 2 (spawn_sensor_rig.py) first."
    )


def carla_to_ros_pose(tf: carla.Transform) -> tuple[float, float, float, float, float, float, float]:
    """Convert a CARLA LHS Transform into right-handed ROS 2 map pose + quaternion."""
    yaw_ros = -math.radians(tf.rotation.yaw)
    half_yaw = 0.5 * yaw_ros
    return (
        tf.location.x,
        -tf.location.y,
        tf.location.z,
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def compute_downstream_goal(
    world: carla.World, ego: carla.Actor, distance_m: float
) -> tuple[carla.Transform, tuple[float, float, float, float, float, float, float]]:
    """Walk forward along Ego's lane in CARLA OpenDRIVE to pick a valid route goal."""
    carla_map = world.get_map()
    wp = carla_map.get_waypoint(ego.get_location(), project_to_road=True)
    if wp is None:
        raise RuntimeError("Ego is not on a mapped lane.")

    start_road = wp.road_id
    traveled = 0.0
    step_m = 2.0
    while traveled < distance_m:
        nxt = wp.next(step_m)
        if not nxt:
            break
        candidate = nxt[0]
        # Stop safely before entering a downstream junction when staying on Road 1.
        if candidate.is_junction and candidate.road_id != start_road:
            break
        wp = candidate
        traveled += step_m

    print(
        f"Computed downstream goal (+{traveled:.1f} m along road_id={wp.road_id}, lane_id={wp.lane_id}): "
        f"CARLA=({wp.transform.location.x:.2f}, {wp.transform.location.y:.2f})"
    )
    return wp.transform, carla_to_ros_pose(wp.transform)


def engage_autoware_in_container(
    container_name: str,
    ros_goal: tuple[float, float, float, float, float, float, float],
) -> None:
    """Invoke autoware_helper.py inside the running Autoware Docker container."""
    goal_args = [f"{v:.6f}" for v in ros_goal]
    cmd = [
        "docker",
        "exec",
        container_name,
        "bash",
        "-c",
        (
            "source /opt/ros/jazzy/setup.bash && "
            "source /opt/autoware/setup.bash && "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            "export CYCLONEDDS_URI=file:///carla_example/configs/cyclonedds/cyclonedds_container.xml && "
            "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1} && "
            f"python3 /carla_example/scripts/autoware_helper.py drive --goal {' '.join(goal_args)}"
        ),
    ]
    print(f"Sending route & engaging Autonomous mode in container '{container_name}'...")
    subprocess.run(cmd, check=True)


def update_spectator_chase(world: carla.World, ego: carla.Actor) -> None:
    """Smoothly update the CARLA spectator camera behind Ego."""
    tf = ego.get_transform()
    fwd = tf.get_forward_vector()
    spectator_tf = carla.Transform(
        carla.Location(
            x=tf.location.x - fwd.x * 10.5,
            y=tf.location.y - fwd.y * 10.5,
            z=tf.location.z + 5.0,
        ),
        carla.Rotation(pitch=-18.0, yaw=tf.rotation.yaw),
    )
    world.get_spectator().set_transform(spectator_tf)


def nearest_npc_distance(world: carla.World, ego: carla.Actor) -> tuple[float | None, str]:
    """Return horizontal center-to-center distance to closest non-ego actor."""
    ego_loc = ego.get_location()
    best_dist: float | None = None
    best_name = "-"
    for actor in world.get_actors():
        if actor.id == ego.id:
            continue
        if not (actor.type_id.startswith("vehicle.") or actor.type_id.startswith("walker.")):
            continue
        d = ego_loc.distance(actor.get_location())
        if best_dist is None or d < best_dist:
            best_dist = d
            best_name = actor.attributes.get("role_name") or actor.type_id
    return best_dist, best_name


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--xosc", default=str(DEFAULT_XOSC), help="Path to OpenSCENARIO .xosc file")
    parser.add_argument("--container", default="autoware_2_1", help="Running Autoware Docker container")
    parser.add_argument("--role-name", default="ego", help="CARLA role_name of Ego vehicle")
    parser.add_argument("--goal-distance", type=float, default=90.0, help="Downstream goal distance [m]")
    parser.add_argument("--max-time", type=float, default=45.0, help="Maximum simulation time [s]")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(15.0)
    world = client.get_world()
    ego = find_ego(world, args.role_name)

    goal_tf, ros_goal = compute_downstream_goal(world, ego, args.goal_distance)
    engage_autoware_in_container(args.container, ros_goal)

    # Monitor collisions cleanly on Ego.
    collisions: list[str] = []
    col_bp = world.get_blueprint_library().find("sensor.other.collision")
    col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)

    def on_collision(event) -> None:
        other = event.other_actor.type_id if event.other_actor else "unknown"
        collisions.append(other)

    col_sensor.listen(on_collision)

    cfg = RunnerConfig(
        xosc_path=str(Path(args.xosc).resolve()),
        host=args.host,
        port=args.port,
        delta_seconds=0.05,
        sync_mode=True,
        max_sim_time=args.max_time,
        ego_name="ego",
        ego_role_name=args.role_name,
        spawn_ego=False,
        wait_for_ego=True,
        teleport_external_ego=False,
        keep_ego_on_exit=True,
        enable_ros2=True,
        autoware_mode=True,
        autoware_sensors=False,
        follow_ego=False,
    )

    start_loc = ego.get_location()
    min_npc_gap: float | None = None
    max_speed_mps = 0.0
    sim_t = 0.0

    print("\nStarting OpenSCENARIO + Autoware 2.1.0 closed-loop execution (20 Hz lockstep)...")
    try:
        with CarlaEsminiRunner(cfg) as runner:
            step_idx = 0
            while runner.step():
                sim_t += 0.05
                step_idx += 1
                update_spectator_chase(world, ego)

                vel = ego.get_velocity()
                speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
                max_speed_mps = max(max_speed_mps, speed)

                ego_loc = ego.get_location()
                dist_goal = ego_loc.distance(goal_tf.location)
                dist_driven = ego_loc.distance(start_loc)
                npc_dist, npc_name = nearest_npc_distance(world, ego)
                if npc_dist is not None:
                    min_npc_gap = npc_dist if min_npc_gap is None else min(min_npc_gap, npc_dist)

                if step_idx % 20 == 0:
                    gap_str = f"{npc_dist:.1f}m ({npc_name})" if npc_dist is not None else "n/a"
                    print(
                        f"  t={sim_t:5.1f}s | speed={speed*3.6:5.1f} km/h | "
                        f"driven={dist_driven:5.1f}m | to_goal={dist_goal:5.1f}m | nearest_npc={gap_str}"
                    )

                if dist_goal < 4.5:
                    print(f"\n[OK] Reached goal within {dist_goal:.2f} m at t={sim_t:.1f}s!")
                    break
                if collisions:
                    print(f"\n[WARN] Collision detected with: {collisions[0]}", file=sys.stderr)
                    break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            col_sensor.stop()
            col_sensor.destroy()
        except Exception:
            pass
        # Return CARLA to responsive async ticking once scenario exits.
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
        except Exception:
            pass

    driven_final = ego.get_location().distance(start_loc)
    gap_summary = f"{min_npc_gap:.2f} m" if min_npc_gap is not None else "n/a"
    print("\n================ Scenario Summary ================")
    print(f"  Sim time        : {sim_t:.1f} s")
    print(f"  Distance driven : {driven_final:.2f} m")
    print(f"  Peak speed      : {max_speed_mps * 3.6:.1f} km/h ({max_speed_mps:.2f} m/s)")
    print(f"  Closest NPC gap : {gap_summary}")
    print(f"  Collisions      : {len(collisions)}")
    print("==================================================")
    return 1 if collisions else 0


if __name__ == "__main__":
    sys.exit(main())
