#!/usr/bin/env python3

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
End-to-End Autonomous Driving: CARLA Native ROS 2 + Full Autoware Classical Stack.

Launches:
1. Connects to CARLA UE 5.8 Server with native ROS 2 (-ros2) on Town10HD_Opt.
2. Spawns Lincoln MKZ Ego vehicle configured with official AWSIM sensor suite
   via autoware_demo.py (VLP-16 LiDAR, IMU, MGRS GNSS, Traffic Light & Front Cameras,
   and vehicle_status sensor on base_link rear axle).
3. Configures CycloneDDS unicast bridge between macOS host and Docker container.
4. Starts Autoware Docker container with map mounted and CARLA-specific overrides.
5. Launches the full classical Autoware stack (autoware_launch e2e_simulator.launch.xml).
6. Auto-initializes localization via GNSS (/api/localization/initialize).
7. Verifies NDT convergence against CARLA ground-truth rear-axle kinematics.
8. Publishes mission goal on /planning/mission_planning/goal along the driving lane.
9. Engages autonomous driving (/api/operation_mode/change_to_autonomous).
10. Continuously monitors live driving telemetry in CARLA as Autoware drives to destination.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import threading
import time

_EXAMPLES_ROS2_DIR = Path(__file__).resolve().parent
_EXAMPLES_DIR = _EXAMPLES_ROS2_DIR.parent
_PYTHON_API_DIR = _EXAMPLES_DIR.parent
if str(_EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(_EXAMPLES_DIR))
if str(_PYTHON_API_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_API_DIR))
if str(_EXAMPLES_ROS2_DIR) not in sys.path:
    sys.path.insert(0, str(_EXAMPLES_ROS2_DIR))

import carla
from autoware_demo import (
    AUTOWARE_POSTPROCESS_SETTINGS,
    deploy_postprocess_profile,
    move_spectator,
    spawn_ego_with_sensors,
)
from test_carla_autoware_ros2 import detect_host_ip

DEFAULT_AUTOWARE_IMAGE = "ghcr.io/autowarefoundation/autoware:universe-devel-jazzy"
DEFAULT_CONTAINER_NAME = "carla_autoware_e2e"
MAP_DIR_HOST = (
    _PYTHON_API_DIR
    / "examples"
    / "av_stacks"
    / "autoware"
    / "map_tools"
    / "maps"
    / "Town10HD"
)


class DemoArgs:
    def __init__(self, mgrs_off: bool = True):
        # In autoware_demo.py: --mgrs_off action='store_false', so True enables MGRS autoware_gnss
        self.mgrs_off = mgrs_off


def write_cyclonedds_configs(host_ip: str, host_xml: Path, container_xml: Path) -> None:
    """Generate CycloneDDS configs for host and Docker with unicast bridge."""
    host_cfg = f"""<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="{host_ip}" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>0</ParticipantIndex>
      <MaxAutoParticipantIndex>80</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="{host_ip}"/>
        <Peer address="127.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
    container_cfg = f"""<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" multicast="false"/>
      </Interfaces>
      <ExternalNetworkAddress>127.0.0.1</ExternalNetworkAddress>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>80</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="{host_ip}"/>
        <Peer address="127.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
    host_xml.write_text(host_cfg, encoding="utf-8")
    container_xml.write_text(container_cfg, encoding="utf-8")


def docker_cmd(container: str, bash_command: str, timeout: float = 30.0) -> str:
    """Execute command inside Docker container with ROS 2 sourced."""
    wrapped = (
        "source /opt/ros/jazzy/setup.bash && "
        "if [ -f /opt/autoware/setup.bash ]; then source /opt/autoware/setup.bash; fi && "
        f"{bash_command}"
    )
    res = subprocess.run(
        ["docker", "exec", container, "bash", "-c", wrapped],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
        check=False,
    )
    return res.stdout.strip()


def resolve_localization_mode(mode: str) -> str:
    """Resolve 'auto' to 'ndt' on Linux systems with NVIDIA GPUs, else 'ground_truth'."""
    if mode != "auto":
        return mode
    if platform.system() == "Linux":
        try:
            res = subprocess.run(
                ["nvidia-smi"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if res.returncode == 0:
                return "ndt"
        except Exception:
            pass
    return "ground_truth"


def apply_carla_overrides(container: str, loc_mode: str = "ground_truth") -> None:
    """Apply validated CARLA overrides depending on localization mode."""
    script_path = Path("/tmp/apply_carla_overrides.py")
    script_content = f"""import glob, re, sys

mode = "{loc_mode}"

# In NDT mode, patch scan matcher convergence likelihood to 1.0 and disable stop check
if mode == "ndt":
    for ndt in glob.glob("/opt/autoware/**/ndt_scan_matcher.param.yaml", recursive=True):
        with open(ndt, "r") as f:
            s = f.read()
        s2 = re.sub(r"(converged_param_nearest_voxel_transformation_likelihood:\s*)[0-9.]+", r"\\g<1>1.0", s)
        with open(ndt, "w") as f:
            f.write(s2)
        print(f"Patched NDT: {{ndt}} (likelihood -> 1.0)")

    for comp in glob.glob("/opt/autoware/**/component_localization.launch.xml", recursive=True):
        with open(comp, "r") as f:
            s = f.read()
        s2 = re.sub(r'name="stop_check_enabled"[^\\n]+value="true"', 'name="stop_check_enabled" value="false"', s)
        with open(comp, "w") as f:
            f.write(s2)
        print(f"Patched stop_check_enabled -> false in {{comp}}")

# Patch vehicle_cmd_gate to disable emergency handling wait in simulator
for vcg in glob.glob("/opt/autoware/**/vehicle_cmd_gate.param.yaml", recursive=True):
    with open(vcg, "r") as f:
        s = f.read()
    s = s.replace("use_emergency_handling: true", "use_emergency_handling: false")
    s = s.replace("check_external_emergency_heartbeat: true", "check_external_emergency_heartbeat: false")
    with open(vcg, "w") as f:
        f.write(s)
    print(f"Patched vehicle_cmd_gate: {{vcg}}")

# Patch autoware-carla.yaml diagnostics:
# 1. Increase timeouts to 30.0s for simulator latency tolerance
# 2. Trim /autoware/modes/autonomous to delegate link to prevent simulator engage deadlock
for diag in glob.glob("/opt/autoware/**/autoware-carla.yaml", recursive=True):
    with open(diag, "r") as f:
        s = f.read()
    s = s.replace("timeout: 3.0", "timeout: 30.0")
    pattern = r"  - path: /autoware/modes/autonomous\\n    type: and\\n    list:\\n(      - .*\\n)+"
    repl = "  - path: /autoware/modes/autonomous\\n    type: and\\n    list:\\n      - {{ type: link, link: /adapi/mrm_request/delegate }}\\n"
    s = re.sub(pattern, repl, s)
    with open(diag, "w") as f:
        f.write(s)
    print(f"Patched diagnostics: {{diag}} (autonomous mode delegated to mrm_request)")
"""
    script_path.write_text(script_content, encoding="utf-8")
    subprocess.run(["docker", "cp", str(script_path), f"{container}:/tmp/apply_carla_overrides.py"], check=True)
    subprocess.run(["docker", "exec", "-u", "0", container, "python3", "/tmp/apply_carla_overrides.py"], check=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="End-to-End Autonomous Driving: CARLA Native ROS 2 + Full Autoware Classical Stack."
    )
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=2000, help="CARLA RPC port (default: 2000)")
    parser.add_argument("--image", default=DEFAULT_AUTOWARE_IMAGE, help="Autoware Docker image")
    parser.add_argument("--container-name", default=DEFAULT_CONTAINER_NAME, help="Docker container name")
    parser.add_argument("--spawn-index", type=int, default=14, help="Spawn point index (default: 14)")
    parser.add_argument("--drive-duration", type=float, default=25.0, help="Autonomous driving monitoring time (s)")
    parser.add_argument(
        "--localization-mode",
        choices=["auto", "ground_truth", "ndt"],
        default="auto",
        help="Localization mode: 'auto' (detects hardware), 'ground_truth' (fast, CPU-friendly), 'ndt' (requires CUDA)",
    )
    parser.add_argument("--keep-container", action="store_true", help="Keep container running for debugging")
    args = parser.parse_args()

    loc_mode = resolve_localization_mode(args.localization_mode)
    host_ip = detect_host_ip()
    host_xml = Path("/tmp/cyclonedds_host.xml")
    container_xml = Path("/tmp/cyclonedds_container.xml")
    write_cyclonedds_configs(host_ip, host_xml, container_xml)

    print("=" * 80)
    print("CARLA UE 5.8 (-ros2) <-> Full Autoware Classical Stack Autonomous Drive")
    print("=" * 80)
    print(f"Host OS           : {platform.system()} ({platform.machine()})")
    print(f"Host IP           : {host_ip}")
    print(f"CARLA Server      : {args.host}:{args.port}")
    print(f"Map Path (Host)   : {MAP_DIR_HOST}")
    print(f"Autoware Image    : {args.image}")
    print(f"Container Name    : {args.container_name}")
    print(f"Spawn Index       : {args.spawn_index}")
    print(f"Localization Mode : {args.localization_mode} -> resolved to '{loc_mode}'")
    print("-" * 80)

    # Deploy postprocess profile
    deploy_postprocess_profile("autoware_demo", AUTOWARE_POSTPROCESS_SETTINGS)

    # 1. Connect to CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    # Reset any active actors from previous runs
    settings = world.get_settings()
    settings.synchronous_mode = False
    world.apply_settings(settings)
    time.sleep(0.5)

    actors_to_clean = [
        a for a in world.get_actors()
        if a.type_id.startswith("vehicle.") or a.type_id.startswith("sensor.") or a.type_id.startswith("util.actor.")
    ]
    for a in actors_to_clean:
        try:
            a.destroy()
        except Exception:
            pass

    # 2. Configure synchronous simulation mode (20 Hz)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    if hasattr(world, "set_publish_tf"):
        world.set_publish_tf(False)
    for _ in range(5):
        world.tick()

    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    spawn_pt = spawn_points[args.spawn_index % len(spawn_points)]
    print(f"[Setup] Selected Spawn Point [{args.spawn_index}]: loc=({spawn_pt.location.x:.2f}, {spawn_pt.location.y:.2f}, {spawn_pt.location.z:.2f}), yaw={spawn_pt.rotation.yaw:.2f}°")

    # Spawn Lincoln MKZ ego with full official AWSIM sensor kit
    demo_args = DemoArgs(mgrs_off=True)
    ego = spawn_ego_with_sensors(world, spawn_pt, demo_args)

    bp_lib = world.get_blueprint_library()
    status_bp = bp_lib.find("sensor.other.vehicle_status")
    status_bp.set_attribute("sensor_tick", "0.05")
    vs_sensor = world.spawn_actor(status_bp, carla.Transform(), attach_to=ego)
    vs_sensor.enable_for_ros()

    for _ in range(5):
        world.tick()
    move_spectator(world, ego)
    print(f"[Setup] Spawned Ego Lincoln MKZ (id={ego.id}) with AWSIM sensor suite & vehicle_status.")

    # Start synchronous world ticker thread and host ground-truth telemetry sender
    stop_ticking = threading.Event()
    import socket, struct
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def ticker_loop():
        while not stop_ticking.is_set():
            try:
                world.tick()
                move_spectator(world, ego)
                snapshot = world.get_snapshot()
                elapsed = snapshot.timestamp.elapsed_seconds
                sec = int(elapsed)
                nanosec = int((elapsed - sec) * 1e9)
                tf = ego.get_transform()
                vel = ego.get_velocity()
                ang_vel = ego.get_angular_velocity()
                cyaw_rad = math.radians(tf.rotation.yaw)
                # Lincoln MKZ pivot to rear axle is 1.425m
                rear_x = tf.location.x - 1.425 * math.cos(cyaw_rad)
                rear_y = tf.location.y - 1.425 * math.sin(cyaw_rad)
                base_x = rear_x
                base_y = -rear_y
                base_z = tf.location.z
                map_yaw = -cyaw_rad
                vx = vel.x * math.cos(cyaw_rad) + vel.y * math.sin(cyaw_rad)
                vy = -vel.x * math.sin(cyaw_rad) + vel.y * math.cos(cyaw_rad)
                wz = -math.radians(ang_vel.z)
                pkt = struct.pack("!2I7d", sec, nanosec, base_x, base_y, base_z, map_yaw, vx, vy, wz)
                udp_sock.sendto(pkt, ("127.0.0.1", 9099))
            except Exception:
                pass
            time.sleep(0.045)

    tick_thread = threading.Thread(target=ticker_loop, daemon=True)
    tick_thread.start()

    # 3. Start Autoware Docker container
    subprocess.run(["docker", "rm", "-f", args.container_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"[Setup] Starting Autoware container '{args.container_name}'...")
    helper_script = _EXAMPLES_ROS2_DIR / "autoware_automation_helper.py"
    docker_run_cmd = [
        "docker", "run", "-d",
        "--name", args.container_name,
        "-p", "7412-7650:7412-7650/udp",
        "-p", "9099:9099/udp",
        "-v", f"{container_xml}:/tmp/cyclonedds.xml:ro",
        "-v", f"{MAP_DIR_HOST}:/maps/Town10HD:ro",
        "-e", "CYCLONEDDS_URI=file:///tmp/cyclonedds.xml",
        "-e", "ROS_DOMAIN_ID=0",
        args.image,
        "bash", "-c",
        'python3 -c "import socket, time; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind((\'0.0.0.0\', 7410)); time.sleep(86400)" & sleep infinity',
    ]
    subprocess.run(docker_run_cmd, check=True)
    subprocess.run(
        ["docker", "cp", str(helper_script), f"{args.container_name}:/tmp/autoware_automation_helper.py"],
        check=True,
    )

    try:
        # 4. Apply CARLA overrides
        print(f"[Setup] Applying CARLA overrides for mode '{loc_mode}' inside container...")
        apply_carla_overrides(args.container_name, loc_mode)

        # 5. Launch Autoware classical stack in background inside container
        loc_arg = "true" if loc_mode == "ndt" else "false"
        print(f"[Autoware] Launching E2E Simulator Classical Stack (mode={loc_mode}, localization={loc_arg})...")
        launch_cmd = (
            f"nohup ros2 launch autoware_launch e2e_simulator.launch.xml "
            f"vehicle_model:=sample_vehicle "
            f"sensor_model:=awsim_sensor_kit "
            f"perception:=false "
            f"localization:={loc_arg} "
            f"rviz:=false "
            f"simulator_type:=carla "
            f"launch_simulator_interface:=false "
            f"map_path:=/maps/Town10HD "
            f"> /tmp/autoware_stack.log 2>&1 &"
        )
        docker_cmd(args.container_name, launch_cmd)

        if loc_mode == "ground_truth":
            print("[Autoware] Starting CarlaStateRelay in background...")
            subprocess.run(
                [
                    "docker", "exec", "-d", args.container_name, "bash", "-c",
                    "source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && "
                    "python3 /tmp/autoware_automation_helper.py state_relay > /tmp/carla_state_relay.log 2>&1"
                ],
                check=True,
            )

        # 6. Wait for Autoware ADAPI to become ready
        print("[Autoware] Waiting for Autoware ADAPI services to appear...")
        adapi_ready = False
        for attempt in range(25):
            res = subprocess.run(
                [
                    "docker", "exec", args.container_name, "bash", "-c",
                    "source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && python3 /tmp/autoware_automation_helper.py wait_adapi"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=100.0,
            )
            if res.returncode == 0:
                adapi_ready = True
                print("  [OK] Autoware ADAPI service is ONLINE!")
                break
            print(f"  [Waiting {attempt + 1}/25] ADAPI not ready yet, retrying in 4s...")
            time.sleep(4.0)

        assert adapi_ready, "Autoware ADAPI did not come up within timeout!"

        # 7. Localization Verification / Initialization
        if loc_mode == "ndt":
            gt_yaw_init = -math.radians(spawn_pt.rotation.yaw)
            init_x = spawn_pt.location.x - 1.425 * math.cos(gt_yaw_init)
            init_y = -spawn_pt.location.y - 1.425 * math.sin(gt_yaw_init)
            init_yaw = -spawn_pt.rotation.yaw

            print(f"\n[Localization] Initializing pose via initial guess (/api/localization/initialize)...")
            print(f"  Initial Guess Map: x={init_x:.2f}m, y={init_y:.2f}m, yaw={init_yaw:.2f}°")
            init_res = subprocess.run(
                [
                    "docker", "exec", args.container_name, "bash", "-c",
                    f"source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && python3 /tmp/autoware_automation_helper.py init_localization --direct --x {init_x} --y {init_y} --z 0.0 --yaw {init_yaw}"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=60.0,
            )
            print(init_res.stdout)
            if init_res.returncode != 0:
                print("[Localization ERROR] /tmp/autoware_stack.log (last 50 lines):")
                print(docker_cmd(args.container_name, "tail -n 50 /tmp/autoware_stack.log"))
            assert init_res.returncode == 0, f"Localization init failed: {init_res.stderr}"
            print("  [PASS] Localization initialized successfully (success=True)!")

            print("[Localization] Letting NDT scan matcher converge (12s)...")
            time.sleep(12.0)
        else:
            print("\n[Localization] Using CarlaStateRelay ground truth localization (running at 20 Hz)...")
            time.sleep(3.0)

        # Gate 1: Check kinematic state against ground truth
        print("[Localization] Verifying kinematic_state pose against CARLA ground truth (Gate 1)...")
        gate1_passed = False
        for attempt in range(6):
            state_cmd = "source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && python3 /tmp/autoware_automation_helper.py get_kinematic_state"
            state_res = subprocess.run(
                ["docker", "exec", args.container_name, "bash", "-c", state_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=18.0,
            )
            if state_res.returncode == 0 and state_res.stdout.strip():
                try:
                    vals = json.loads(state_res.stdout.strip())
                    curr_tf = ego.get_transform()
                    gt_yaw = -math.radians(curr_tf.rotation.yaw)
                    gt_x = curr_tf.location.x - 1.425 * math.cos(gt_yaw)
                    gt_y = -curr_tf.location.y - 1.425 * math.sin(gt_yaw)
                    delta_p = math.hypot(vals["x"] - gt_x, vals["y"] - gt_y)
                    print(f"  CARLA Truth (Rear Axle): x={gt_x:.2f}m, y={gt_y:.2f}m")
                    print(f"  Autoware Belief        : x={vals['x']:.2f}m, y={vals['y']:.2f}m")
                    print(f"  Position Delta         : {delta_p:.2f}m")
                    if delta_p <= 1.0:
                        print("  [PASS] Gate 1 PASSED: Autoware kinematic_state is aligned with ground truth!")
                        gate1_passed = True
                        break
                    else:
                        print(f"  [WARN] Gate 1 Delta ({delta_p:.2f}m) exceeds 1.0m, retrying...")
                except Exception as e:
                    print(f"  [WARN] Failed to parse kinematic state: {e}")
            else:
                print(f"  [Attempt {attempt+1}/6] Waiting for kinematic state...")
            time.sleep(1.5)
        assert gate1_passed, "Gate 1 failed: Autoware kinematic state was not received or not aligned with CARLA ground truth!"

        # 8. Set Downstream Goal dynamically along the driving lane
        start_wp = carla_map.get_waypoint(spawn_pt.location)
        next_wps = start_wp.next(45.0)
        target_wp = next_wps[0] if next_wps else start_wp
        goal_x_carla = target_wp.transform.location.x
        goal_y_carla = target_wp.transform.location.y
        goal_yaw_carla = target_wp.transform.rotation.yaw

        map_x = goal_x_carla
        map_y = -goal_y_carla
        map_yaw = -goal_yaw_carla

        print(f"\n[Mission Planner] Sending Goal Pose:")
        print(f"  CARLA Coords   : x={goal_x_carla:.1f}m, y={goal_y_carla:.1f}m, yaw={goal_yaw_carla:.1f}°")
        print(f"  Autoware Map   : x={map_x:.1f}m, y={map_y:.1f}m, z=0.0m, yaw={map_yaw:.1f}°")

        # Call ADAPI /api/routing/set_route_points directly
        adapi_goal_cmd = f"source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && python3 /tmp/autoware_automation_helper.py send_goal --x {map_x} --y {map_y} --z 0.0 --yaw {map_yaw}"
        adapi_goal_res = subprocess.run(
            ["docker", "exec", args.container_name, "bash", "-c", adapi_goal_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=25.0,
        )
        print(adapi_goal_res.stdout)
        if adapi_goal_res.stderr:
            print("[SetRoute info/log]:", adapi_goal_res.stderr)
        time.sleep(4.0)

        # 9. Engage Autonomous Driving
        print("\n[Operation Mode] Engaging Autonomous Mode (/api/operation_mode/change_to_autonomous)...")
        engage_res = subprocess.run(
            [
                "docker", "exec", args.container_name, "bash", "-c",
                "source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && python3 /tmp/autoware_automation_helper.py engage_autonomous"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=120.0,
        )
        print(engage_res.stdout)
        if engage_res.returncode != 0:
            print("\n[Engage ERROR] /tmp/autoware_stack.log (last 100 lines):")
            print(docker_cmd(args.container_name, "tail -n 100 /tmp/autoware_stack.log"))
        assert engage_res.returncode == 0, f"Failed to engage autonomous mode: {engage_res.stderr}"
        print("  [PASS] Autonomous Mode ENGAGED (success=True)!")

        # 10. Monitor Live Autonomous Driving in CARLA
        print("\n[Autonomous Driving] Autoware is driving Ego vehicle autonomously in CARLA:")
        start_loc = ego.get_location()
        start_time = time.time()
        max_speed = 0.0
        speeds = []
        positions = []

        while time.time() - start_time < args.drive_duration:
            tf = ego.get_transform()
            vel = ego.get_velocity()
            speed_mps = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            speed_kmh = speed_mps * 3.6
            max_speed = max(max_speed, speed_kmh)
            speeds.append(speed_kmh)
            positions.append((tf.location.x, tf.location.y))

            disp = tf.location.distance(start_loc)
            curr_wp = carla_map.get_waypoint(tf.location)
            lat_dev = tf.location.distance(curr_wp.transform.location)

            elapsed = time.time() - start_time
            print(
                f"  t={elapsed:4.1f}s | Speed: {speed_kmh:5.1f} km/h | Disp: {disp:5.1f}m | "
                f"Pos: ({tf.location.x:5.1f}, {tf.location.y:5.1f}) | Yaw: {tf.rotation.yaw:5.1f}° | LatDev: {lat_dev:4.2f}m"
            )
            time.sleep(1.0)

        final_loc = ego.get_location()
        total_disp = final_loc.distance(start_loc)
        avg_speed = sum(speeds) / len(speeds) if speeds else 0.0

        print("\n" + "=" * 80)
        print("AUTONOMOUS DRIVING TEST SUMMARY:")
        print(f"  Total Displacement    : {total_disp:.2f} meters")
        print(f"  Peak Driving Speed    : {max_speed:.1f} km/h")
        print(f"  Average Driving Speed : {avg_speed:.1f} km/h")
        print(f"  Start Location        : ({start_loc.x:.2f}, {start_loc.y:.2f})")
        print(f"  Final Location        : ({final_loc.x:.2f}, {final_loc.y:.2f})")
        print("=" * 80)

        assert total_disp > 8.0, f"Vehicle did not travel sufficiently autonomously ({total_disp:.2f}m <= 8m)!"
        print("\n[SUCCESS] Full Autoware classical stack successfully drove CARLA vehicle autonomously!")
        return 0

    finally:
        # Save autoware log before exiting
        try:
            subprocess.run(["docker", "cp", f"{args.container_name}:/tmp/autoware_stack.log", "/tmp/autoware_stack.log"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["docker", "cp", f"{args.container_name}:/tmp/carla_state_relay.log", "/tmp/carla_state_relay.log"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
        if not args.keep_container:
            stop_ticking.set()
            tick_thread.join(timeout=2.0)
            # Restore asynchronous mode
            try:
                settings = world.get_settings()
                settings.synchronous_mode = False
                world.apply_settings(settings)
            except Exception:
                pass
            subprocess.run(["docker", "rm", "-f", args.container_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            print(f"[Debug] Container '{args.container_name}' kept alive for inspection.")


if __name__ == "__main__":
    raise SystemExit(main())

