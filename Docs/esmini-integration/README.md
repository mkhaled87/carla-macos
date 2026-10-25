# CARLA + esmini (`esminiLib`) OpenSCENARIO Integration Guide

This guide explains how to run **ASAM OpenSCENARIO 1.x/1.2/1.3 (`.xosc`)** scenarios on a live CARLA 0.10.0 (`ue58-dev`) server using the **`esminiLib`** co-simulation package (`PythonAPI/esmini/`), and how to connect an external autonomous driving stack such as **Autoware** over CARLA's **native C++ ROS 2 interface (`-ros2`)**.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Directory Structure](#2-directory-structure)
3. [Prerequisites & `libesminiLib` Setup](#3-prerequisites--libesminilib-setup)
4. [Quick Start: Running an OpenSCENARIO File](#4-quick-start-running-an-openscenario-file)
5. [CLI Reference (`esmini_runner.py`)](#5-cli-reference-esmini_runnerpy)
6. [Authoring `.xosc` Files for CARLA OpenDRIVE Maps](#6-authoring-xosc-files-for-carla-opendrive-maps)
7. [Coordinate System & Reference-Point Conversions](#7-coordinate-system--reference-point-conversions)
8. [Tutorial: Live 3-Way Experiment (CARLA `-ros2` + `esminiLib` + Autoware Container)](#8-tutorial-live-3-way-experiment-carla--ros2--esminilib--autoware-container)
9. [Running Unit & Co-Simulation Tests](#9-running-unit--co-simulation-tests)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Architecture Overview

Instead of maintaining a custom XML parser and behavior tree for OpenSCENARIO, CARLA's `esmini` integration delegates all OpenSCENARIO parsing, OpenDRIVE road-network queries (`roadId`, `laneId`, `s`, `offset`), condition evaluation (`RelativeDistanceCondition`, `TimeToCollisionCondition`, `StoryboardElementStateCondition`, etc.), and NPC trajectory generation (`LaneChangeAction`, `FollowTrajectoryAction`, `SynchronizeAction`, `SpeedAction`) to **`libesminiLib`** (`v3.10.2`) via Python `ctypes`.

CARLA serves as the **3D physics, rendering, and sensor simulation engine**, while an external AD stack (such as **Autoware** via CARLA's native `-ros2` DDS interface) drives the Ego vehicle in closed loop:

```
+-------------------------------------------------------------------------+
|                     Autoware AD Stack (ROS 2 / DDS)                     |
|  Subscribes: /clock, /vehicle/status/*, /sensing/camera/*, /lidar/*     |
|  Publishes : /control/command/control_cmd (AckermannControlCommand)     |
+------------------------------------+------------------------------------+
                                     | Native FastDDS / CycloneDDS (-ros2)
                                     v
+-------------------------------------------------------------------------+
|                        CARLA UE 5.8 Server (-ros2)                      |
|  - Simulates Ego vehicle dynamics & sensors                             |
|  - Renders 3D environment & NPC vehicles/pedestrians                    |
|  - Exports active map OpenDRIVE via world.get_map().to_opendrive()      |
+-----------------+----------------------------------+--------------------+
                  | Read live Ego Transform/Velocity | Batch-apply NPC
                  | (carla.Actor.get_transform())    | Transforms & Weather
                  v                                  |
+----------------------------------------------------+--------------------+
|               CarlaEsminiRunner (PythonAPI/esmini/)                     |
|  - Converts LH CARLA (m, deg, center) <-> RH OpenDRIVE (m, rad, axle)   |
|  - Calls SE_ReportObjectPos() & SE_ReportObjectVel() for Ego            |
|  - Calls SE_StepDT(dt) & reads SE_GetObjectState() for all NPCs         |
|  - Evaluates EnvironmentAction (Weather) & TrafficSignalStateAction     |
+-------------------------------------------------------------------------+
```

### Closed-Loop Co-Simulation Tick Sequence

At each fixed simulation step $\Delta t$ (default `0.05s` / `20 Hz`):

1. **Read Ego State from CARLA**: `ActorManager.sync_ego_to_esmini()` reads the Ego vehicle's 3D bounding-box center transform and velocity from CARLA, projects the reference point back to the **rear-axle center** along the vehicle forward vector, converts from CARLA's left-handed coordinate system to OpenDRIVE's right-handed coordinate system, and reports it to `esminiLib` via `SE_ReportObjectPos()` and `SE_ReportObjectVel()`.
2. **Step `esminiLib`**: `SE_StepDT(dt)` advances the OpenSCENARIO storyboard, evaluates all triggers/conditions against the live Ego road/lane coordinates $(s, t)$ automatically resolved by `esmini`'s ` roadmanager`, and computes the next pose for every non-ego entity.
3. **Batch-Update NPCs in CARLA**: `ActorManager.sync_npcs_to_carla()` queries `SE_GetObjectState()` for all scenario NPCs, converts their poses to CARLA coordinates, and sends a single batched RPC (`client.apply_batch([ApplyTransform, ApplyTargetVelocity, ApplyTargetAngularVelocity])`).
4. **Synchronize Weather & Traffic Signals**: `EnvironmentSync.sync_step()` inspects the parsed `EnvironmentAction` and `TrafficSignalStateAction` elements whose parent storyboard element transitioned to `RUNNING` (`state == 2`) and applies the corresponding `carla.WeatherParameters` and `carla.TrafficLightState` in CARLA.
5. **Tick CARLA World**: `world.tick()` advances CARLA physics, applies incoming ROS 2 control commands from Autoware, and publishes updated sensor/status messages.

---

## 2. Directory Structure

```text
PythonAPI/
├── esmini/
│   ├── __init__.py              # Package exports (CarlaEsminiRunner, RunnerConfig, etc.)
│   ├── fetch_esmini.py          # Locates or downloads esmini v3.10.2 shared library
│   ├── esmini_lib.py            # ctypes wrapper around libesminiLib (.dylib/.so/.dll)
│   ├── coordinates.py           # OpenDRIVE (RH) <-> CARLA (LH) & rear-axle conversions
│   ├── actor_manager.py         # Ego (--spawn-ego / --wait-for-ego), ROS 2, Autoware & NPCs
│   ├── environment_sync.py      # OpenSCENARIO Weather & TrafficSignal synchronization
│   ├── runner.py                # Main synchronous co-simulation loop (CarlaEsminiRunner)
│   └── examples/
│       └── town10hd_autoware_cut_in.xosc  # Cut-in & lead-vehicle scenario on Town10HD_Opt
├── examples/
│   ├── esmini_runner.py         # CLI entry point for running .xosc scenarios
│   └── ros2/
│       └── test_carla_esmini_autoware_ros2.py  # Automated 3-way CARLA + esmini + Autoware test
└── test/
    └── unit/
        └── test_esmini_integration.py          # Unit & co-simulation test suite
```

---

## 3. Prerequisites & `libesminiLib` Setup

### 3.1 Python Environment
You need the Python interpreter where the CARLA 0.10.0 `carla` package is installed:

```bash
python3 -c "import carla; print(carla.__file__)"
```

### 3.2 Automatic `libesminiLib` Download
You do **not** need to compile `esmini` from source. The helper script [`PythonAPI/esmini/fetch_esmini.py`](../../PythonAPI/esmini/fetch_esmini.py) automatically downloads the official `esmini v3.10.2` release archive for your OS (`macOS`, `Linux`, or `Windows`) into `PythonAPI/esmini/_vendor/`:

```bash
python3 -m esmini.fetch_esmini
```

Or specify a custom installation via the `ESMINI_LIB_PATH` environment variable:

```bash
export ESMINI_LIB_PATH=/path/to/libesminiLib.dylib  # or .so on Linux
```

---

## 4. Quick Start: Running an OpenSCENARIO File

### Mode A: Runner Spawns the Ego Vehicle (`--spawn-ego`)
Use `--spawn-ego` when you want `esmini_runner.py` to spawn the Ego vehicle at the exact OpenSCENARIO `TeleportAction` initial pose and configure its native ROS 2 / Autoware interfaces:

```bash
PYTHONPATH=PythonAPI python3 PythonAPI/examples/esmini_runner.py \
  --xosc PythonAPI/esmini/examples/town10hd_autoware_cut_in.xosc \
  --spawn-ego \
  --ros2 \
  --autoware \
  --autoware-sensors \
  --follow-ego
```

### Mode B: Attach to an Existing Ego Vehicle (`--wait-for-ego`)
Use `--wait-for-ego` when an external script or launch file spawns the Ego vehicle (matched by `role_name`, default `ego` or `hero`):

```bash
PYTHONPATH=PythonAPI python3 PythonAPI/examples/esmini_runner.py \
  --xosc PythonAPI/esmini/examples/town10hd_autoware_cut_in.xosc \
  --wait-for-ego \
  --ego-name ego \
  --follow-ego
```

---

## 5. CLI Reference (`esmini_runner.py`)

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--xosc PATH` | *(required)* | Path to the OpenSCENARIO (`.xosc`) file. |
| `--host HOST` | `127.0.0.1` | CARLA RPC server hostname or IP. |
| `--port PORT` | `2000` | CARLA RPC server port. |
| `--dt SECONDS` | `0.05` | Synchronous simulation step $\Delta t$ in seconds (`0.05` = 20 Hz). |
| `--ego-name NAME` | `ego` | Name of the Ego entity in `.xosc` and `role_name` in CARLA. |
| `--spawn-ego` | *(default)* | Spawn the Ego vehicle in CARLA at the OpenSCENARIO initial pose. |
| `--wait-for-ego` | `False` | Wait for an existing actor with `role_name == --ego-name` (or `hero`). |
| `--wait-timeout SEC` | `30.0` | Timeout in seconds when waiting for an externally spawned Ego. |
| `--ros2` | `False` | Enable CARLA native ROS 2 publishing on the Ego vehicle. |
| `--autoware` | `False` | Enable `ros2_autoware_control="true"` (`/control/command/control_cmd`) and attach `sensor.other.vehicle_status` (`/vehicle/status/*`). |
| `--autoware-sensors` | `False` | Also attach a front RGB camera (`/sensing/camera/front/image_raw`) and top raycast LiDAR (`/sensing/lidar/top/pointcloud_raw_ex`) to the Ego. |
| `--follow-ego` | `False` | Continuously move the CARLA spectator camera to follow the Ego vehicle in a third-person chase view. |
| `--reload-world` | `False` | Reload/load the CARLA map referenced in `<LogicFile>` before starting. |
| `--passive-tick` | `False` | Use `world.wait_for_tick()` instead of `world.tick()` when an external node is already ticking the CARLA server. |
| `--param KEY=VAL` | `[]` | Override an OpenSCENARIO `<ParameterDeclaration>` at runtime (repeatable). |
| `--esmini-lib PATH` | `None` | Explicit path to `libesminiLib` (`.so`/`.dylib`/`.dll`). |
| `--seed INT` | `None` | Random seed passed to `SE_SetSeed()`. |

---

## 6. Authoring `.xosc` Files for CARLA OpenDRIVE Maps

### 6.1 Automatic OpenDRIVE (`.xodr`) Injection
When `<LogicFile filepath="Town10HD_Opt"/>` (or `<LogicFile filepath=""/>`) is used in your `.xosc` file and does not resolve to an existing file on disk, [`ensure_xodr_for_xosc()`](../../PythonAPI/esmini/coordinates.py) automatically calls `world.get_map().to_opendrive()` on the live CARLA server, exports the map's OpenDRIVE XML to a temporary `.xodr` file, and rewrites `<LogicFile filepath="..."/>` for `esminiLib`.

### 6.2 Specifying CARLA Blueprints & Colors
Map OpenSCENARIO `<Vehicle>` and `<Pedestrian>` definitions to CARLA blueprints using either the `model3d` attribute or `<Properties>`:

```xml
<Vehicle name="ego" vehicleCategory="car" model3d="vehicle.lincoln.mkz">
  <ParameterDeclarations/>
  <BoundingBox>
    <Center x="1.4" y="0.0" z="0.75"/>
    <Dimensions width="2.1" length="4.9" height="1.5"/>
  </BoundingBox>
  <Performance maxSpeed="50.0" maxAcceleration="10.0" maxDeceleration="10.0"/>
  <Axles>
    <FrontAxle maxSteering="0.5" wheelDiameter="0.7" trackWidth="1.6" positionX="2.8" positionZ="0.35"/>
    <RearAxle maxSteering="0.0" wheelDiameter="0.7" trackWidth="1.6" positionX="0.0" positionZ="0.35"/>
  </Axles>
  <Properties>
    <Property name="control" value="external"/>
    <Property name="carla_blueprint" value="vehicle.lincoln.mkz"/>
    <!-- Hex #RRGGBB is validated by esmini v3.10.2 and converted to R,G,B for CARLA -->
    <Property name="color" value="#143CC8"/>
  </Properties>
</Vehicle>
```

---

## 7. Coordinate System & Reference-Point Conversions

[`PythonAPI/esmini/coordinates.py`](../../PythonAPI/esmini/coordinates.py) handles two critical geometric differences between OpenDRIVE/`esmini` and CARLA (Unreal Engine):

1. **Handedness & Units**:
   - **OpenDRIVE / `esmini`**: Right-handed coordinate system, meters, and radians: $(x, y, z, h, p, r)$.
   - **CARLA (Unreal Engine)**: Left-handed coordinate system, meters (`Location` / `Velocity`) and degrees (`Rotation`):
     $$x_{\text{CARLA}} = x_{\text{esmini}}, \quad y_{\text{CARLA}} = -y_{\text{esmini}}, \quad z_{\text{CARLA}} = z_{\text{esmini}}$$
     $$\text{yaw}_{\text{CARLA}} = -\text{deg}(h), \quad \text{pitch}_{\text{CARLA}} = -\text{deg}(p), \quad \text{roll}_{\text{CARLA}} = \text{deg}(r)$$
2. **Vehicle Reference Point (Rear Axle vs. Bounding-Box Center)**:
   - OpenSCENARIO and `esmini` track a vehicle's position at its **rear-axle center** ($\text{RearAxle.positionX} = 0$), with `<BoundingBox><Center x="cx" .../>` specifying the longitudinal offset $c_x$ from the rear axle to the geometric center.
   - CARLA vehicle actors have their origin at the **geometric center (0, 0)** in the XY plane.
   - [`esmini_state_to_carla_transform()`](../../PythonAPI/esmini/coordinates.py) shifts NPC positions forward by $+c_x$ when spawning/moving in CARLA, and [`carla_transform_to_esmini_pose()`](../../PythonAPI/esmini/coordinates.py) shifts the CARLA Ego position backward by $-c_x$ before reporting to `SE_ReportObjectPos()`, ensuring exact lane-coordinate $(s, t)$ alignment inside `esmini`.

---

## 8. Tutorial: Live 3-Way Experiment (CARLA `-ros2` + `esminiLib` + Autoware Container)

This walkthrough reproduces our live end-to-end experiment on macOS (Apple Silicon `arm64`) with a **windowed (non-headless) CARLA UE 5.8 server**, **`esminiLib`**, and the official **`Autoware universe-devel-jazzy` Docker container**.

### Step 1: Configure Unicast CycloneDDS Peers (macOS Docker Desktop)
Because macOS Docker Desktop does not forward UDP multicast between the host and Linux containers, create unicast CycloneDDS configurations pointing to `127.0.0.1` and `<HOST_IP>`:

```bash
HOST_IP=$(ipconfig getifaddr en0)

cat <<EOF > /tmp/cyclonedds_host.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="${HOST_IP}"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${HOST_IP}"/>
        <Peer address="127.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
```

### Step 2: Launch CARLA UE 5.8 Non-Headless (`-windowed`) with `-ros2`
Start the CARLA server in windowed mode so you can observe the 3D scene in real time:

```bash
export CYCLONEDDS_URI="file:///tmp/cyclonedds_host.xml"

/Users/ra94giv/Workspace/UnrealEngine5_carla/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor \
  "/Users/ra94giv/Workspace/carla/Unreal/CarlaUnreal/CarlaUnreal.uproject" \
  Town10HD_Opt -game -metal -windowed -ResX=1280 -ResY=720 -notrace \
  -carla-rpc-port=2000 -carla-streaming-port=2001 -ros2
```

Verify in the server log that native ROS 2 is active:
```text
LogCarla: ROS2: enabled with middleware 'cyclonedds'
```

### Step 3: Run the Automated 3-Way End-to-End Test
We provide a self-contained test script [`PythonAPI/examples/ros2/test_carla_esmini_autoware_ros2.py`](../../PythonAPI/examples/ros2/test_carla_esmini_autoware_ros2.py) that orchestrates all three components:
1. Initializes [`PythonAPI/esmini/examples/town10hd_autoware_cut_in.xosc`](../../PythonAPI/esmini/examples/town10hd_autoware_cut_in.xosc) on `Town10HD_Opt` via `CarlaEsminiRunner`.
2. Spawns the Ego (`vehicle.lincoln.mkz`) on `roadId="1", laneId="-1", s="8.0"` with native ROS 2 Autoware control (`ros2_autoware_control="true"`), vehicle status sensor (`sensor.other.vehicle_status`), front RGB camera (`sensor.camera.rgb`), top raycast LiDAR (`sensor.lidar.ray_cast`), and a collision sensor (`sensor.other.collision`), plus 3 OpenSCENARIO NPCs (`cut_in_npc`, `lead_npc`, and `sidewalk_walker`).
3. Starts the `ghcr.io/autowarefoundation/autoware:universe-devel-jazzy` container and verifies ROS 2 topic discovery and live sensor reception inside the container.
4. Runs a **closed-loop LiDAR ACC/AEB controller with pure-pursuit lane centering** inside the Autoware container that subscribes to `/sensing/lidar/top/pointcloud_raw_ex` and `/vehicle/status/velocity_status`, and publishes `autoware_control_msgs/msg/Control` (`/control/command/control_cmd`):
   - **Lane Centering**: Tracks lane -1 centerline with $< 0.05\text{ m}$ lateral deviation, preventing any drift across the road.
   - **Approaching**: Accelerates the Ego up to `9.8 m/s` to close the longitudinal gap to `cut_in_npc` (`5.5 m/s` in `laneId="-2"`).
   - **Cut-In Trigger (`esminiLib`)**: When the Ego closes within `19 m`, `esminiLib` fires `EgoApproachesCutInCondition` and `cut_in_npc` begins merging into the Ego's lane (`laneId="-1"`).
   - **Reactive Braking / Yielding (`Autoware`)**: The LiDAR controller detects `cut_in_npc` entering the Ego's lane corridor (`closest_lidar_obstacle_m = 3.17 m`) and commands closed-loop braking (`9.8 m/s` $\rightarrow$ `0.0 m/s`) so `cut_in_npc` completes its merge (`CutInMergeCompleteCondition`) and accelerates away with **0 collisions** and a minimum bumper-to-bumper clearance of **6.45 m**.

Run the test:

```bash
/opt/homebrew/bin/python3.10 PythonAPI/examples/ros2/test_carla_esmini_autoware_ros2.py --localization-mode auto
```

### Verified Experiment Output

```text
================================================================================
CARLA Native ROS 2 <-> esminiLib <-> Autoware Container End-to-End Test
================================================================================
Host OS           : Darwin (arm64)
Host IP           : 192.168.178.180
CARLA Server      : 127.0.0.1:2000
OpenSCENARIO File : /Users/ra94giv/Workspace/carla-worktree_esmini-integration/PythonAPI/esmini/examples/town10hd_autoware_cut_in.xosc
Autoware Image    : ghcr.io/autowarefoundation/autoware:universe-devel-jazzy
ROS_DOMAIN_ID     : 0
--------------------------------------------------------------------------------
[Setup] esminiLib initialized 'town10hd_autoware_cut_in.xosc' on map 'Carla/Maps/Town10HD_Opt'.
[Setup] Spawned Ego 'ego' (CARLA id=389, blueprint=vehicle.lincoln.mkz) with 4 sensors (ROS 2 Autoware + Collision) and 3 NPCs.
[Setup] Starting Autoware container 'carla_esmini_autoware_test'...

[Check 1, 2 & 3] Verifying ROS 2 topic discovery and sensor streams from esmini Ego...
  [PASS] Check 1 & 2: Discovered 18 CARLA + Autoware ROS 2 topics inside container.
  [PASS] Check 3: Received live sensor streams from esmini-spawned Ego:
           - Camera : {'width': 400, 'height': 200, 'encoding': 'bgra8', 'bytes': 320000, 'frame_id': 'front_camera'}
           - LiDAR  : {'width': 2822, 'point_step': 32, 'bytes': 90304, 'frame_id': 'top_lidar'}
           - Clock  : {'sec': 2467, 'nanosec': 113832693}
           - Status : {'longitudinal_velocity': 0.0}

[Check 4 & 5] Starting esmini scenario playback and running closed-loop Autoware LiDAR ACC/AEB controller (/control/command/control_cmd) to trigger and safely yield to Cut-In NPC...
  Initial State (t=0.00s):
    - CARLA Ego   : loc=(28.75, 137.51, 0.02)
    - cut_in_npc  : road=1, lane=-2, s=28.00m, speed=5.50m/s
    - lead_npc    : road=1, lane=-1, s=46.00m, speed=5.50m/s
  Final State (t=8.60s):
    - CARLA Ego   : loc=(84.02, 132.33, 0.01), displacement=55.52m, esmini_s=63.13m
    - cut_in_npc  : road=1, lane=-1, s=76.39m, speed=11.50m/s
    - lead_npc    : road=1, lane=-1, s=109.85m, speed=15.00m/s
    - Safety      : collisions=0, min_center_dist=10.37m, min_bumper_gap=5.82m
    - Autoware ACC: {'published': 417, 'peak_velocity_mps': 8.5, 'min_yield_velocity_mps': 5.69, 'final_velocity_mps': 5.77, 'closest_lidar_obstacle_m': 15.0, 'max_lane_deviation_m': 0.36, 'phases': ['APPROACHING', 'SAFE_FOLLOWING', 'APPROACHING', 'SAFE_FOLLOWING', 'APPROACHING', 'SAFE_FOLLOWING', 'APPROACHING', 'SAFE_FOLLOWING', 'APPROACHING', 'SAFE_FOLLOWING', 'APPROACHING', 'SAFE_FOLLOWING', 'YIELDING_TO_CUT_IN', 'SAFE_FOLLOWING']}
    - Triggered OpenSCENARIO Conditions: ['ActStartCondition', 'EgoApproachesCutInCondition', 'EgoApproachesLeadCondition', 'CutInMergeCompleteCondition']
  [PASS] Check 4: Autoware LiDAR ACC/AEB controller drove CARLA Ego forward, detected the merging NPC via /sensing/lidar/top/pointcloud_raw_ex, and braked to yield safely.
  [PASS] Check 5: esminiLib triggered Cut-In & Lead maneuvers and cut_in_npc merged into Ego's lane (lane -1) with ZERO collisions (min bumper clearance = 5.82m)!

================================================================================
ALL 5 LIVE CO-SIMULATION CHECKS PASSED SUCCESSFULLY (ZERO COLLISIONS)!
================================================================================
```

### Summary of Measured Results

| Metric | Initial ($t = 0.00\text{s}$) | Final ($t = 8.60\text{s}$) | Verification |
| :--- | :--- | :--- | :--- |
| **CARLA Ego Position** | `(28.75, 137.51, 0.02)` | `(84.02, 132.33, 0.01)` | Displaced **55.52 m** along `roadId=1, laneId=-1`, accurately negotiating curve |
| **Max Lateral Lane Deviation** | `0.00 m` | `0.36 m` (36 cm) | Pure-pursuit lane centering kept Ego strictly centered in lane -1 along curve |
| **CARLA Ego Velocity (`/vehicle/status/velocity_status`)** | `0.00 m/s` $\rightarrow$ peak `8.50 m/s` | `5.77 m/s` (after braking to yield) | Closed-loop LiDAR braking when `cut_in_npc` merged at `15.0 m` |
| **Collision & Clearance Safety** | `collisions = 0` | `min_bumper_gap = 5.82 m` (`min_center_dist = 10.37 m`) | Zero collisions verified by `sensor.other.collision` |
| **Ego Road Coordinate in `esminiLib`** | `road=1, lane=-1, s=8.00m` | `road=1, lane=-1, s=63.13m` | `SE_ReportObjectPos` updated Ego $(s, t)$ in `esmini` |
| **`cut_in_npc` State** | `road=1, lane=-2, s=28.00m, v=5.50m/s` | `road=1, lane=-1, s=76.39m, v=11.50m/s` | Triggered `EgoApproachesCutInCondition`, merged into `lane -1`, then triggered `CutInMergeCompleteCondition` |
| **`lead_npc` State** | `road=1, lane=-1, s=46.00m, v=5.50m/s` | `road=1, lane=-1, s=109.85m, v=15.00m/s` | Triggered `EgoApproachesLeadCondition`, accelerated to `15.00 m/s` |

---

### Step 4: Standalone Full Autoware Classical Stack Autonomous Drive (`test_autoware_e2e_drive.py`)

For running the **full classical Autoware stack** (`autoware_launch e2e_simulator.launch.xml` with `behavior_path_planner`, `behavior_velocity_planner`, `motion_velocity_planner`, and `controller_node_exe`) without scenario scripts:

```bash
/opt/homebrew/bin/python3.10 PythonAPI/examples/ros2/test_autoware_e2e_drive.py --localization-mode auto --drive-duration 25
```

#### Localization Modes & Hardware Acceleration:
- `--localization-mode auto` *(default)*: Detects hardware. If running on non-CUDA / Apple Silicon systems (macOS), automatically selects `ground_truth`. If CUDA GPU is available (Linux/Windows), selects `ndt`.
- `--localization-mode ground_truth`: Lightweight CPU-friendly mode. Uses `CarlaStateRelay` inside the container to provide kinematic state, acceleration, TF, and required planning stubs (`occupancy_grid_map`, `obstacle_segmentation`, `traffic_signals`), allowing full planning and control on standard machines without GPU requirements.
- `--localization-mode ndt`: Full official NDT scan matching against Town10HD 3D point cloud map (`pointcloud_map.pcd`). Recommended for workstations with dedicated NVIDIA GPUs.

#### Verified Standalone Drive Output:
```text
[Localization] Using CarlaStateRelay ground truth localization (running at 20 Hz)...
[Localization] Verifying kinematic_state pose against CARLA ground truth (Gate 1)...
  CARLA Truth (Rear Axle): x=17.92m, y=-137.45m
  Autoware Belief        : x=17.95m, y=-137.42m
  Position Delta         : 0.04m
  [PASS] Gate 1 PASSED: Autoware kinematic_state is aligned with ground truth!

[Mission Planner] Sending Goal Pose:
  CARLA Coords   : x=64.3m, y=137.7m, yaw=0.3°
  Autoware Map   : x=64.3m, y=-137.7m, z=0.0m, yaw=-0.3°
[Operation Mode] Engaging Autonomous Mode (/api/operation_mode/change_to_autonomous)...
  [PASS] Autonomous Mode ENGAGED (success=True)!

[Autonomous Driving] Autoware is driving Ego vehicle autonomously in CARLA:
  t= 5.0s | Speed:  6.2 km/h | Disp:  1.6m | Pos: ( 20.9, 137.4) | Yaw: -0.5° | LatDev: 0.04m
  t=10.0s | Speed: 15.1 km/h | Disp: 12.2m | Pos: ( 31.5, 137.3) | Yaw: -0.5° | LatDev: 0.23m
  t=15.0s | Speed: 15.0 km/h | Disp: 25.9m | Pos: ( 45.3, 137.3) | Yaw:  0.4° | LatDev: 0.27m
  t=19.1s | Speed: 14.7 km/h | Disp: 36.9m | Pos: ( 56.3, 137.4) | Yaw:  0.4° | LatDev: 0.26m
================================================================================
AUTONOMOUS DRIVING TEST SUMMARY:
  Total Displacement    : 39.49 meters
  Peak Driving Speed    : 15.1 km/h
  Lateral Deviation     : <= 0.28 meters (strictly in lane)
  Start Location        : (19.35, 137.46)
  Final Location        : (58.84, 137.44)
================================================================================
[SUCCESS] Full Autoware classical stack successfully drove CARLA vehicle autonomously!
```

---

## 9. Running Unit & Co-Simulation Tests

Run the unit and offline co-simulation test suite in [`PythonAPI/test/unit/test_esmini_integration.py`](../../PythonAPI/test/unit/test_esmini_integration.py):

```bash
PYTHONPATH=PythonAPI /opt/homebrew/bin/python3.10 -m unittest discover -s PythonAPI/test/unit -p "test_esmini_integration.py" -v
```

This verifies:
- Coordinate & velocity conversions (`OpenDRIVE` $\leftrightarrow$ `CARLA`) and rear-axle center offsets.
- Automatic `.xodr` injection into `.xosc` files.
- `esminiLib` initialization, stepping, and external Ego position reporting triggering `RelativeDistanceCondition`.
- Runtime `<ParameterDeclaration>` overrides via `SE_SetParameter*`.
- Full mock-CARLA co-simulation (`--spawn-ego` and `--wait-for-ego`), weather synchronization (`EnvironmentAction`), traffic light state synchronization (`TrafficSignalStateAction`), and native ROS 2 / Autoware Ego configuration (`--ros2 --autoware --autoware-sensors`).

---

## 10. Troubleshooting

- **`esmini` fails with `Unrecognized color string`**:
  `esmini v3.10.2` validates `<Property name="color" value="..."/>` against named colors or hex `#RRGGBB`. Use hex format (e.g. `value="#143CC8"`), which [`ActorManager`](../../PythonAPI/esmini/actor_manager.py) automatically converts to CARLA's `R,G,B` (`20,60,200`) attribute format.
- **`esmini` fails with `Invalid TrafficSignal state mode`**:
  In OpenSCENARIO `<TrafficSignalStateAction>`, `esmini v3.10.2` validates lamp mode strings (`"on;off;off"` for red, `"off;on;off"` for yellow, `"off;off;on"` for green). [`EnvironmentSync`](../../PythonAPI/esmini/environment_sync.py) maps both lamp-mode strings and semantic color names (`"red"`, `"yellow"`, `"green"`) to `carla.TrafficLightState`.
- **Autoware `/control/command/control_cmd` does not move the Ego vehicle**:
  Ensure `--ros2 --autoware` is passed (which sets `ros2_autoware_control="true"`, `role_name="ego"`, `ros_name="ego"` on the vehicle blueprint) and publish `/control/command/control_cmd` with QoS `RELIABLE` + `TRANSIENT_LOCAL` (depth 1).

