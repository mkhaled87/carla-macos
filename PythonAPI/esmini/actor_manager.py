# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""CARLA actor lifecycle, blueprint resolution, Ego state reporting, and NPC batch updates."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import time
from typing import Any

from .coordinates import (
    carla_angular_velocity_to_esmini,
    carla_transform_to_esmini_pose,
    carla_velocity_to_esmini,
    esmini_angular_velocity_to_carla,
    esmini_state_to_carla_transform,
    esmini_velocity_to_carla,
    make_location,
    make_vector3d,
)
from .esmini_lib import (
    EsminiLib,
    SEControllerType,
    SELightMode,
    SEObjectType,
    SEScenarioObjectState,
    SEVehicleCategory,
    SEVehicleLightType,
    SEVisibilityMask,
)

try:
    import carla as _carla_module
except ImportError:
    _carla_module = None


# ==============================================================================
# -- Color & Blueprint Mapping Tables ------------------------------------------
# ==============================================================================

NAMED_COLORS_RGB: dict[str, str] = {
    "white": "255,255,255",
    "black": "15,15,15",
    "dark": "35,35,40",
    "grey": "128,128,128",
    "gray": "128,128,128",
    "silver": "192,192,192",
    "red": "200,20,20",
    "blue": "20,60,200",
    "yellow": "235,200,15",
    "green": "20,160,40",
    "orange": "230,110,15",
    "brown": "110,65,30",
}

VEHICLE_CATEGORY_PATTERNS: dict[int, tuple[str, ...]] = {
    SEVehicleCategory.CAR: (
        "vehicle.tesla.model3",
        "vehicle.lincoln.mkz_2020",
        "vehicle.lincoln.mkz_2017",
        "vehicle.audi.tt",
        "vehicle.audi.a2",
        "vehicle.nissan.patrol",
        "vehicle.dodge.charger_2020",
        "vehicle.ford.crown",
    ),
    SEVehicleCategory.VAN: (
        "vehicle.mercedes.sprinter",
        "vehicle.volkswagen.t2",
        "vehicle.volkswagen.t2_2021",
        "vehicle.ford.ambulance",
    ),
    SEVehicleCategory.TRUCK: (
        "vehicle.carlamotors.carlacola",
        "vehicle.carlamotors.european_hgv",
        "vehicle.carlamotors.firetruck",
    ),
    SEVehicleCategory.SEMI_TRAILER: (
        "vehicle.carlamotors.european_hgv",
        "vehicle.carlamotors.carlacola",
    ),
    SEVehicleCategory.TRAILER: (
        "vehicle.carlamotors.carlacola",
    ),
    SEVehicleCategory.BUS: (
        "vehicle.mitsubishi.fusorosa",
        "vehicle.volkswagen.t2",
    ),
    SEVehicleCategory.MOTORBIKE: (
        "vehicle.kawasaki.ninja",
        "vehicle.yamaha.yzf",
        "vehicle.harley-davidson.low_rider",
        "vehicle.vespa.zx125",
    ),
    SEVehicleCategory.BICYCLE: (
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
    ),
}

# CARLA VehicleLightState bitmask constants (matches carla::rpc::VehicleLightState::LightState)
CARLA_LIGHT_NONE = 0
CARLA_LIGHT_POSITION = 1 << 0
CARLA_LIGHT_LOW_BEAM = 1 << 1
CARLA_LIGHT_HIGH_BEAM = 1 << 2
CARLA_LIGHT_BRAKE = 1 << 3
CARLA_LIGHT_RIGHT_BLINKER = 1 << 4
CARLA_LIGHT_LEFT_BLINKER = 1 << 5
CARLA_LIGHT_REVERSE = 1 << 6
CARLA_LIGHT_FOG = 1 << 7
CARLA_LIGHT_INTERIOR = 1 << 8
CARLA_LIGHT_SPECIAL1 = 1 << 9
CARLA_LIGHT_SPECIAL2 = 1 << 10

ESMINI_TO_CARLA_LIGHT_BITS: tuple[tuple[SEVehicleLightType, int], ...] = (
    (SEVehicleLightType.DAYTIME_RUNNING_LIGHTS, CARLA_LIGHT_POSITION),
    (SEVehicleLightType.TAIL_LIGHTS, CARLA_LIGHT_POSITION),
    (SEVehicleLightType.LOW_BEAM, CARLA_LIGHT_LOW_BEAM | CARLA_LIGHT_POSITION),
    (SEVehicleLightType.HIGH_BEAM, CARLA_LIGHT_HIGH_BEAM | CARLA_LIGHT_POSITION),
    (SEVehicleLightType.BRAKE_LIGHTS, CARLA_LIGHT_BRAKE),
    (SEVehicleLightType.INDICATOR_LEFT, CARLA_LIGHT_LEFT_BLINKER),
    (SEVehicleLightType.INDICATOR_RIGHT, CARLA_LIGHT_RIGHT_BLINKER),
    (SEVehicleLightType.WARNING_LIGHTS, CARLA_LIGHT_LEFT_BLINKER | CARLA_LIGHT_RIGHT_BLINKER),
    (SEVehicleLightType.REVERSING_LIGHTS, CARLA_LIGHT_REVERSE),
    (SEVehicleLightType.FOG_LIGHTS, CARLA_LIGHT_FOG),
    (SEVehicleLightType.FOG_LIGHTS_FRONT, CARLA_LIGHT_FOG),
    (SEVehicleLightType.FOG_LIGHTS_REAR, CARLA_LIGHT_FOG),
    (SEVehicleLightType.SPECIAL_PURPOSE_LIGHTS, CARLA_LIGHT_SPECIAL1),
)


@dataclass
class ManagedActorInfo:
    """Metadata tracked for each scenario entity bridged between esmini and CARLA."""

    esmini_id: int
    name: str
    object_type: int
    object_category: int
    is_ego: bool
    spawned_by_runner: bool
    carla_actor: Any
    carla_actor_id: int
    center_offset_x: float = 0.0
    center_offset_y: float = 0.0
    center_offset_z: float = 0.0
    carla_bbox_x: float = 0.0
    carla_bbox_y: float = 0.0
    walker_half_height: float | None = None
    last_light_mask: int = 0
    wheel_rotation: float = 0.0
    properties: dict[str, str] = field(default_factory=dict)
    attached_sensors: list[Any] = field(default_factory=list)


class ActorManager:
    """Manages Ego and NPC actors in CARLA synchronized with an ``EsminiLib`` session."""

    def __init__(
        self,
        esmini: EsminiLib,
        carla_client: Any,
        carla_world: Any,
        ego_name: str | None = None,
        ego_role_name: str = "hero",
        ego_blueprint: str | None = None,
        spawn_ego: bool = True,
        wait_for_ego: bool = False,
        ego_wait_timeout: float = 30.0,
        teleport_external_ego_to_init: bool = True,
        keep_ego_on_exit: bool = False,
        enable_ros2: bool = False,
        autoware_mode: bool = False,
        autoware_sensors: bool = False,
        apply_center_offset: bool = True,
        draw_ghost: bool = False,
        carla_mod: Any = None,
    ) -> None:
        self.esmini = esmini
        self.client = carla_client
        self.world = carla_world
        self.carla_mod = carla_mod if carla_mod is not None else _carla_module

        self.ego_name_hint = ego_name
        self.autoware_sensors = autoware_sensors
        self.autoware_mode = autoware_mode or autoware_sensors
        self.enable_ros2 = enable_ros2 or self.autoware_mode
        if self.autoware_mode and ego_role_name == "hero":
            self.ego_role_name = "ego"
        else:
            self.ego_role_name = ego_role_name

        self.ego_blueprint_override = ego_blueprint
        self.spawn_ego = spawn_ego and not wait_for_ego
        self.wait_for_ego = wait_for_ego
        self.ego_wait_timeout = ego_wait_timeout
        self.teleport_external_ego_to_init = teleport_external_ego_to_init
        self.keep_ego_on_exit = keep_ego_on_exit
        self.apply_center_offset = apply_center_offset
        self.draw_ghost = draw_ghost

        self.ego_esmini_id: int | None = None
        self.actors: dict[int, ManagedActorInfo] = {}
        self._blueprint_library: Any = (
            self.world.get_blueprint_library() if self.world is not None else None
        )

    # ------------------------------------------------------------------
    # Ego Identification & Setup
    # ------------------------------------------------------------------

    def resolve_ego_esmini_id(self) -> int | None:
        """Identify which esmini object represents the Ego vehicle."""
        obj_ids = self.esmini.get_object_ids()
        if not obj_ids:
            return None

        # 1. Explicit name provided by user
        if self.ego_name_hint:
            explicit_id = self.esmini.get_id_by_name(self.ego_name_hint)
            if explicit_id >= 0:
                return explicit_id
            for oid in obj_ids:
                if self.esmini.get_object_name(oid).lower() == self.ego_name_hint.lower():
                    return oid

        # 2. Entity with ExternalController (ctrl_type == 1) or property control="external"
        for idx, oid in enumerate(obj_ids):
            st = self.esmini.get_object_state(oid)
            if st is not None and st.ctrl_type == SEControllerType.EXTERNAL:
                return oid
            props = self.esmini.get_object_properties(idx)
            if props.get("control", "").lower() == "external":
                return oid

        # 3. Common OpenSCENARIO ego entity names
        ego_aliases = ("ego", "hero", "ego_vehicle", "egovehicle", "host", "sut")
        for oid in obj_ids:
            name = self.esmini.get_object_name(oid).strip().lower()
            if name in ego_aliases or name.startswith("ego") or name.startswith("hero"):
                return oid

        # 4. Default to first scenario object
        return obj_ids[0]

    def initialize_actors(self) -> None:
        """Spawn or attach to the Ego vehicle, register the Ego callback, and spawn initial NPCs."""
        self.ego_esmini_id = self.resolve_ego_esmini_id()
        obj_ids = self.esmini.get_object_ids()

        for idx, oid in enumerate(obj_ids):
            state = self.esmini.get_object_state(oid)
            if state is None:
                continue
            props = self.esmini.get_object_properties(idx)
            if oid == self.ego_esmini_id:
                self._setup_ego_actor(oid, state, props)
            else:
                if self._is_object_active_in_sim(state):
                    self._spawn_npc_actor(oid, state, props)

    def _setup_ego_actor(
        self, esmini_id: int, state: SEScenarioObjectState, props: dict[str, str]
    ) -> None:
        name = self.esmini.get_object_name(esmini_id) or "Ego"
        carla_actor = None
        spawned_by_runner = False

        if self.wait_for_ego:
            carla_actor = self._wait_for_external_ego(name)
            spawned_by_runner = False
        elif self.spawn_ego:
            # First check if an external actor with matching role_name already exists
            existing = self._find_existing_ego_actor(name)
            if existing is not None:
                carla_actor = existing
                spawned_by_runner = False
            else:
                carla_actor = self._spawn_ego_actor(esmini_id, state, props)
                spawned_by_runner = True

        carla_bbox_x, carla_bbox_y, walker_hh = self._extract_actor_geometry(
            carla_actor, state
        )

        # If we attached to an already-spawned external ego and teleport is enabled,
        # align it with the initial OpenSCENARIO pose at t=0.
        if (
            carla_actor is not None
            and not spawned_by_runner
            and self.teleport_external_ego_to_init
        ):
            init_tf = esmini_state_to_carla_transform(
                state,
                apply_center_offset=self.apply_center_offset,
                carla_bbox_location_x=carla_bbox_x,
                carla_bbox_location_y=carla_bbox_y,
                walker_half_height=walker_hh,
                z_clearance=0.05,
                carla_mod=self.carla_mod,
            )
            carla_actor.set_transform(init_tf)
            zero_vec = make_vector3d(0.0, 0.0, 0.0, carla_mod=self.carla_mod)
            if hasattr(carla_actor, "set_target_velocity"):
                carla_actor.set_target_velocity(zero_vec)
            if hasattr(carla_actor, "set_target_angular_velocity"):
                carla_actor.set_target_angular_velocity(zero_vec)

        attached_sensors: list[Any] = []
        if carla_actor is not None and spawned_by_runner and self.autoware_mode:
            attached_sensors = self._spawn_autoware_ego_sensors(carla_actor)

        info = ManagedActorInfo(
            esmini_id=esmini_id,
            name=name,
            object_type=state.objectType,
            object_category=state.objectCategory,
            is_ego=True,
            spawned_by_runner=spawned_by_runner,
            carla_actor=carla_actor,
            carla_actor_id=int(carla_actor.id) if carla_actor is not None else -1,
            center_offset_x=float(state.centerOffsetX),
            center_offset_y=float(state.centerOffsetY),
            center_offset_z=float(state.centerOffsetZ),
            carla_bbox_x=carla_bbox_x,
            carla_bbox_y=carla_bbox_y,
            walker_half_height=walker_hh,
            properties=props,
            attached_sensors=attached_sensors,
        )
        self.actors[esmini_id] = info

        # Register esmini object callback for Ego so that even if the .xosc file uses
        # DefaultController (ctrl_type == 0) on Ego, esmini's internal step never overwrites
        # the CARLA Ego's actual position or speed.
        self.esmini.register_object_callback(esmini_id, self._on_ego_esmini_callback)

    def _spawn_autoware_ego_sensors(self, ego_actor: Any) -> list[Any]:
        """Attach native ROS 2 Autoware status (and optional perception) sensors to Ego."""
        if self.world is None or self._blueprint_library is None:
            return []

        from .coordinates import make_transform

        sensors: list[Any] = []

        # 1. Vehicle status sensor (/vehicle/status/*) required by CARLA's native ROS 2 Autoware bridge
        status_bp = self._find_blueprint_by_id_or_filter("sensor.other.vehicle_status")
        if status_bp is not None:
            if status_bp.has_attribute("ros_name"):
                status_bp.set_attribute("ros_name", "vehicle_status")
            if status_bp.has_attribute("role_name"):
                status_bp.set_attribute("role_name", "vehicle_status")
            if status_bp.has_attribute("sensor_tick"):
                status_bp.set_attribute("sensor_tick", "0.05")
            status_tf = make_transform(carla_mod=self.carla_mod)
            try:
                status_sensor = self.world.spawn_actor(
                    status_bp, status_tf, attach_to=ego_actor
                )
            except TypeError:
                status_sensor = self.world.spawn_actor(status_bp, status_tf)
            if status_sensor is not None:
                if hasattr(status_sensor, "enable_for_ros"):
                    status_sensor.enable_for_ros()
                sensors.append(status_sensor)

        if not self.autoware_sensors:
            return sensors

        # 2. Optional Front RGB Camera (/sensing/camera/front/image & camera_info)
        cam_bp = self._find_blueprint_by_id_or_filter("sensor.camera.rgb")
        if cam_bp is not None:
            for attr, val in (
                ("image_size_x", "400"),
                ("image_size_y", "200"),
                ("fov", "90.0"),
                ("sensor_tick", "0.1"),
                ("ros_name", "front_camera"),
                ("role_name", "front_camera"),
                ("ros_topic_name", "/sensing/camera/front"),
            ):
                if cam_bp.has_attribute(attr):
                    cam_bp.set_attribute(attr, val)
            cam_tf = make_transform(
                make_location(x=1.5, y=0.0, z=1.6, carla_mod=self.carla_mod),
                carla_mod=self.carla_mod,
            )
            try:
                cam_sensor = self.world.spawn_actor(
                    cam_bp, cam_tf, attach_to=ego_actor
                )
            except TypeError:
                cam_sensor = self.world.spawn_actor(cam_bp, cam_tf)
            if cam_sensor is not None:
                if hasattr(cam_sensor, "enable_for_ros"):
                    cam_sensor.enable_for_ros()
                sensors.append(cam_sensor)

        # 3. Optional Top Ray-Cast LiDAR (/sensing/lidar/top/pointcloud_raw_ex)
        lidar_bp = self._find_blueprint_by_id_or_filter("sensor.lidar.ray_cast")
        if lidar_bp is not None:
            for attr, val in (
                ("channels", "32"),
                ("range", "50.0"),
                ("points_per_second", "100000"),
                ("rotation_frequency", "20.0"),
                ("sensor_tick", "0.1"),
                ("ros_name", "top_lidar"),
                ("role_name", "top_lidar"),
                ("ros_topic_name", "/sensing/lidar/top/pointcloud_raw_ex"),
            ):
                if lidar_bp.has_attribute(attr):
                    lidar_bp.set_attribute(attr, val)
            lidar_tf = make_transform(
                make_location(x=0.0, y=0.0, z=2.2, carla_mod=self.carla_mod),
                carla_mod=self.carla_mod,
            )
            try:
                lidar_sensor = self.world.spawn_actor(
                    lidar_bp, lidar_tf, attach_to=ego_actor
                )
            except TypeError:
                lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_tf)
            if lidar_sensor is not None:
                if hasattr(lidar_sensor, "enable_for_ros"):
                    lidar_sensor.enable_for_ros()
                sensors.append(lidar_sensor)

        return sensors

    def _find_existing_ego_actor(self, esmini_name: str) -> Any | None:
        if self.world is None:
            return None
        candidates = self.world.get_actors().filter("vehicle.*")
        target_names = {
            self.ego_role_name.lower(),
            esmini_name.lower(),
        }
        if self.ego_name_hint:
            target_names.add(self.ego_name_hint.lower())

        for actor in candidates:
            attrs = getattr(actor, "attributes", {}) or {}
            role = attrs.get("role_name", "").lower()
            ros_name = attrs.get("ros_name", "").lower()
            if role in target_names or (ros_name and ros_name in target_names):
                return actor
        return None

    def _wait_for_external_ego(self, esmini_name: str) -> Any:
        deadline = time.monotonic() + max(0.1, self.ego_wait_timeout)
        while time.monotonic() < deadline:
            actor = self._find_existing_ego_actor(esmini_name)
            if actor is not None:
                return actor
            if self.world is not None:
                settings = self.world.get_settings()
                if getattr(settings, "synchronous_mode", False):
                    self.world.tick()
                else:
                    time.sleep(0.05)
            else:
                time.sleep(0.05)

        raise TimeoutError(
            f"Timed out after {self.ego_wait_timeout:.1f}s waiting for external CARLA ego vehicle "
            f"with role_name='{self.ego_role_name}' (or '{esmini_name}')."
        )

    def _spawn_ego_actor(
        self, esmini_id: int, state: SEScenarioObjectState, props: dict[str, str]
    ) -> Any:
        if self.world is None:
            return None

        bp = self.resolve_blueprint(
            esmini_id,
            state,
            props,
            explicit_blueprint=self.ego_blueprint_override,
            is_ego=True,
        )
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", self.ego_role_name)
        if self.enable_ros2 and bp.has_attribute("ros_name"):
            bp.set_attribute("ros_name", self.ego_role_name)
        if self.autoware_mode and bp.has_attribute("ros2_autoware_control"):
            bp.set_attribute("ros2_autoware_control", "true")

        spawn_tf = esmini_state_to_carla_transform(
            state,
            apply_center_offset=self.apply_center_offset,
            z_clearance=0.05,
            carla_mod=self.carla_mod,
        )
        actor = self.world.try_spawn_actor(bp, spawn_tf)
        if actor is None:
            # Retry with higher Z clearance in case road mesh is slightly above OpenDRIVE Z
            spawn_tf_high = esmini_state_to_carla_transform(
                state,
                apply_center_offset=self.apply_center_offset,
                z_clearance=0.35,
                carla_mod=self.carla_mod,
            )
            actor = self.world.spawn_actor(bp, spawn_tf_high)

        if self.enable_ros2 and hasattr(actor, "enable_for_ros"):
            actor.enable_for_ros()

        # Refine XY position if the spawned CARLA blueprint has a non-zero bounding_box.location.x
        carla_bbox_x, carla_bbox_y, walker_hh = self._extract_actor_geometry(actor, state)
        if abs(carla_bbox_x) > 1e-3 or abs(carla_bbox_y) > 1e-3:
            refined_tf = esmini_state_to_carla_transform(
                state,
                apply_center_offset=self.apply_center_offset,
                carla_bbox_location_x=carla_bbox_x,
                carla_bbox_location_y=carla_bbox_y,
                walker_half_height=walker_hh,
                z_clearance=0.05,
                carla_mod=self.carla_mod,
            )
            actor.set_transform(refined_tf)

        # If initial speed in .xosc is > 0, apply initial forward target velocity
        if abs(state.speed) > 1e-3 and hasattr(actor, "set_target_velocity"):
            vx = state.speed * math.cos(state.h) * math.cos(state.p)
            vy = state.speed * math.sin(state.h) * math.cos(state.p)
            vz = -state.speed * math.sin(state.p)
            actor.set_target_velocity(
                esmini_velocity_to_carla(vx, vy, vz, carla_mod=self.carla_mod)
            )

        return actor

    # ------------------------------------------------------------------
    # NPC Spawning & Blueprint Resolution
    # ------------------------------------------------------------------

    def _is_object_active_in_sim(self, state: SEScenarioObjectState) -> bool:
        """Check whether an esmini object should be materialized in CARLA."""
        # visibilityMask bit 1 = GRAPHICS, bit 2 = TRAFFIC, bit 4 = SENSORS
        if state.visibilityMask == 0:
            return False
        if math.isnan(state.x) or math.isnan(state.y):
            return False
        return True

    def _spawn_npc_actor(
        self, esmini_id: int, state: SEScenarioObjectState, props: dict[str, str]
    ) -> ManagedActorInfo | None:
        name = self.esmini.get_object_name(esmini_id) or f"npc_{esmini_id}"
        if self.world is None:
            info = ManagedActorInfo(
                esmini_id=esmini_id,
                name=name,
                object_type=state.objectType,
                object_category=state.objectCategory,
                is_ego=False,
                spawned_by_runner=True,
                carla_actor=None,
                carla_actor_id=-1,
                center_offset_x=float(state.centerOffsetX),
                center_offset_y=float(state.centerOffsetY),
                center_offset_z=float(state.centerOffsetZ),
                properties=props,
            )
            self.actors[esmini_id] = info
            return info

        bp = self.resolve_blueprint(esmini_id, state, props, is_ego=False)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", name)

        spawn_tf = esmini_state_to_carla_transform(
            state,
            apply_center_offset=self.apply_center_offset,
            z_clearance=0.02 if state.objectType != SEObjectType.PEDESTRIAN else 0.05,
            carla_mod=self.carla_mod,
        )
        actor = self.world.try_spawn_actor(bp, spawn_tf)
        if actor is None:
            spawn_tf_high = esmini_state_to_carla_transform(
                state,
                apply_center_offset=self.apply_center_offset,
                z_clearance=0.35,
                carla_mod=self.carla_mod,
            )
            actor = self.world.try_spawn_actor(bp, spawn_tf_high)

        if actor is None:
            return None

        # Disable physics simulation on NPC vehicles/props so they follow esmini's
        # exact analytical trajectories while retaining collision bounds for sensors & Ego.
        if state.objectType != SEObjectType.PEDESTRIAN and hasattr(
            actor, "set_simulate_physics"
        ):
            actor.set_simulate_physics(False)

        carla_bbox_x, carla_bbox_y, walker_hh = self._extract_actor_geometry(actor, state)
        exact_tf = esmini_state_to_carla_transform(
            state,
            apply_center_offset=self.apply_center_offset,
            carla_bbox_location_x=carla_bbox_x,
            carla_bbox_location_y=carla_bbox_y,
            walker_half_height=walker_hh,
            z_clearance=0.0,
            carla_mod=self.carla_mod,
        )
        actor.set_transform(exact_tf)

        info = ManagedActorInfo(
            esmini_id=esmini_id,
            name=name,
            object_type=state.objectType,
            object_category=state.objectCategory,
            is_ego=False,
            spawned_by_runner=True,
            carla_actor=actor,
            carla_actor_id=int(actor.id),
            center_offset_x=float(state.centerOffsetX),
            center_offset_y=float(state.centerOffsetY),
            center_offset_z=float(state.centerOffsetZ),
            carla_bbox_x=carla_bbox_x,
            carla_bbox_y=carla_bbox_y,
            walker_half_height=walker_hh,
            properties=props,
        )
        self.actors[esmini_id] = info
        return info

    def _extract_actor_geometry(
        self, actor: Any, state: SEScenarioObjectState
    ) -> tuple[float, float, float | None]:
        """Extract CARLA actor bounding box XY offset and pedestrian capsule half-height."""
        bbox_x = 0.0
        bbox_y = 0.0
        walker_hh: float | None = None
        if actor is not None and hasattr(actor, "bounding_box"):
            bb = actor.bounding_box
            if hasattr(bb, "location"):
                bbox_x = float(bb.location.x)
                bbox_y = float(bb.location.y)
            if state.objectType == SEObjectType.PEDESTRIAN and hasattr(bb, "extent"):
                if float(bb.extent.z) > 0.0:
                    walker_hh = float(bb.extent.z)
        if walker_hh is None and state.objectType == SEObjectType.PEDESTRIAN:
            walker_hh = 0.5 * state.height if state.height > 0.0 else 0.95
        return bbox_x, bbox_y, walker_hh

    def resolve_blueprint(
        self,
        esmini_id: int,
        state: SEScenarioObjectState,
        props: dict[str, str],
        explicit_blueprint: str | None = None,
        is_ego: bool = False,
    ) -> Any:
        """Select and configure the best matching CARLA ActorBlueprint for an esmini entity."""
        if self._blueprint_library is None:
            raise RuntimeError("CARLA BlueprintLibrary is not available.")

        type_name = self.esmini.get_object_type_name(esmini_id)
        model_file = Path(self.esmini.get_object_model_filename(esmini_id)).stem
        obj_name = self.esmini.get_object_name(esmini_id)

        # 1. Check explicit blueprint overrides (CLI or .xosc <Property>)
        candidate_ids: list[str] = []
        if explicit_blueprint:
            candidate_ids.append(explicit_blueprint)
        for prop_key in ("carla_blueprint", "blueprint", "model", "type"):
            if props.get(prop_key):
                candidate_ids.append(props[prop_key])
        for name_hint in (type_name, model_file, obj_name):
            if name_hint and (
                name_hint.startswith("vehicle.")
                or name_hint.startswith("walker.")
                or name_hint.startswith("static.prop.")
            ):
                candidate_ids.append(name_hint)

        bp = None
        for cid in candidate_ids:
            bp = self._find_blueprint_by_id_or_filter(cid)
            if bp is not None:
                break

        # 2. Category-based resolution
        if bp is None:
            if state.objectType == SEObjectType.PEDESTRIAN:
                walkers = list(self._blueprint_library.filter("walker.pedestrian.*"))
                if walkers:
                    bp = walkers[esmini_id % len(walkers)]
            elif state.objectType == SEObjectType.MISC_OBJECT:
                props_list = list(self._blueprint_library.filter("static.prop.*"))
                for preferred in (
                    "static.prop.streetbarrier",
                    "static.prop.trafficcone01",
                    "static.prop.box01",
                ):
                    found = self._find_blueprint_by_id_or_filter(preferred)
                    if found is not None:
                        bp = found
                        break
                if bp is None and props_list:
                    bp = props_list[0]
            else:
                # Vehicle category matching
                patterns = VEHICLE_CATEGORY_PATTERNS.get(
                    state.objectCategory,
                    VEHICLE_CATEGORY_PATTERNS[SEVehicleCategory.CAR],
                )
                for pat in patterns:
                    found = self._find_blueprint_by_id_or_filter(pat)
                    if found is not None:
                        bp = found
                        break
                if bp is None:
                    vehicles = list(self._blueprint_library.filter("vehicle.*"))
                    if vehicles:
                        bp = vehicles[0]

        if bp is None:
            raise RuntimeError(
                f"Could not find any suitable CARLA blueprint for esmini object {esmini_id} ({obj_name})"
            )

        # 3. Configure color if supported
        if bp.has_attribute("color"):
            rgb = self._resolve_color_rgb(props, type_name, model_file, obj_name)
            if rgb:
                bp.set_attribute("color", rgb)

        if bp.has_attribute("is_invincible") and state.objectType == SEObjectType.PEDESTRIAN:
            bp.set_attribute("is_invincible", "false")

        return bp

    def _find_blueprint_by_id_or_filter(self, pattern: str) -> Any | None:
        if self._blueprint_library is None or not pattern:
            return None
        try:
            exact = self._blueprint_library.find(pattern)
            if exact is not None:
                return exact
        except Exception:
            pass
        try:
            matches = list(self._blueprint_library.filter(pattern))
            if matches:
                return matches[0]
            matches_wild = list(self._blueprint_library.filter(f"*{pattern}*"))
            if matches_wild:
                return matches_wild[0]
        except Exception:
            pass
        return None

    @staticmethod
    def _resolve_color_rgb(
        props: dict[str, str], *name_hints: str
    ) -> str | None:
        raw_color = props.get("color", "").strip()
        if raw_color:
            if raw_color.startswith("#") and len(raw_color) == 7:
                try:
                    r = int(raw_color[1:3], 16)
                    g = int(raw_color[3:5], 16)
                    b = int(raw_color[5:7], 16)
                    return f"{r},{g},{b}"
                except ValueError:
                    pass
            if "," in raw_color:
                return raw_color
            if raw_color.lower() in NAMED_COLORS_RGB:
                return NAMED_COLORS_RGB[raw_color.lower()]

        for hint in name_hints:
            lower = hint.lower()
            for color_name, rgb in NAMED_COLORS_RGB.items():
                if f"_{color_name}" in lower or f"-{color_name}" in lower or lower == color_name:
                    return rgb
        return None

    # ------------------------------------------------------------------
    # Ego State Reporting (CARLA -> esminiLib)
    # ------------------------------------------------------------------

    def report_ego_to_esmini(self, dt: float = 0.0) -> None:
        """Read the CARLA Ego vehicle's live state and report it to ``esminiLib``."""
        if self.ego_esmini_id is None:
            return
        info = self.actors.get(self.ego_esmini_id)
        if info is None or info.carla_actor is None:
            return

        actor = info.carla_actor
        tf = actor.get_transform()
        vel = actor.get_velocity()
        ang_vel = actor.get_angular_velocity()

        x, y, z, h, p, r = carla_transform_to_esmini_pose(
            tf,
            center_offset_x=info.center_offset_x,
            center_offset_y=info.center_offset_y,
            carla_bbox_location_x=info.carla_bbox_x,
            carla_bbox_location_y=info.carla_bbox_y,
        )
        vx, vy, vz = carla_velocity_to_esmini(vel)
        hr, pr, rr = carla_angular_velocity_to_esmini(ang_vel)

        # Compute signed longitudinal speed along heading
        fwd = tf.get_forward_vector()
        forward_dot = float(vel.x) * float(fwd.x) + float(vel.y) * float(fwd.y) + float(vel.z) * float(fwd.z)
        speed_mag = math.sqrt(vx * vx + vy * vy + vz * vz)
        signed_speed = -speed_mag if forward_dot < -0.1 else speed_mag

        self.esmini.report_object_pos(self.ego_esmini_id, x, y, z, h, p, r)
        self.esmini.report_object_speed(self.ego_esmini_id, signed_speed)
        self.esmini.report_object_vel(self.ego_esmini_id, vx, vy, vz)
        self.esmini.report_object_angular_vel(self.ego_esmini_id, hr, pr, rr)

        if hasattr(actor, "get_acceleration"):
            acc = actor.get_acceleration()
            ax, ay, az = carla_velocity_to_esmini(acc)
            self.esmini.report_object_acc(self.ego_esmini_id, ax, ay, az)

        if hasattr(actor, "get_control"):
            ctrl = actor.get_control()
            steer = getattr(ctrl, "steer", 0.0)
            # Approximate max front wheel steering angle (~0.6 rad) and negate for RH frame
            wheel_angle = -float(steer) * 0.6
            if dt > 0.0:
                info.wheel_rotation = math.fmod(
                    info.wheel_rotation + (signed_speed / 0.35) * dt, 2.0 * math.pi
                )
            self.esmini.report_object_wheel_status(
                self.ego_esmini_id, info.wheel_rotation, wheel_angle
            )

    def _on_ego_esmini_callback(self, _state: SEScenarioObjectState) -> None:
        """Invoked by esmini inside SE_StepDT right after default controllers run."""
        self.report_ego_to_esmini(dt=0.0)

    # ------------------------------------------------------------------
    # NPC State Synchronization (esminiLib -> CARLA Batch Commands)
    # ------------------------------------------------------------------

    def sync_npcs_from_esmini(self) -> list[Any]:
        """Synchronize all non-ego entities from esmini to CARLA and return the RPC batch commands."""
        current_ids = set(self.esmini.get_object_ids())

        # 1. Destroy NPCs that were deleted from esmini (e.g. DeleteEntityAction)
        removed_ids = [
            oid for oid in list(self.actors.keys())
            if oid not in current_ids and oid != self.ego_esmini_id
        ]
        for oid in removed_ids:
            self._destroy_actor_entry(oid)

        # 2. Spawn newly added NPCs or update existing NPCs
        commands: list[Any] = []
        for idx, oid in enumerate(self.esmini.get_object_ids()):
            if oid == self.ego_esmini_id:
                continue
            state = self.esmini.get_object_state(oid)
            if state is None:
                continue

            if not self._is_object_active_in_sim(state):
                if oid in self.actors:
                    self._destroy_actor_entry(oid)
                continue

            info = self.actors.get(oid)
            if info is None:
                props = self.esmini.get_object_properties(idx)
                info = self._spawn_npc_actor(oid, state, props)
                if info is None:
                    continue

            commands.extend(self._build_single_npc_commands(info, state))

        if self.client is not None and commands:
            self.client.apply_batch(commands)

        if self.draw_ghost and self.ego_esmini_id is not None:
            self._draw_ego_ghost()

        return commands

    def _build_single_npc_commands(
        self, info: ManagedActorInfo, state: SEScenarioObjectState
    ) -> list[Any]:
        """Build CARLA batch RPC commands for a single NPC actor at the current frame."""
        if info.carla_actor_id < 0:
            return []

        tf = esmini_state_to_carla_transform(
            state,
            apply_center_offset=self.apply_center_offset,
            carla_bbox_location_x=info.carla_bbox_x,
            carla_bbox_location_y=info.carla_bbox_y,
            walker_half_height=info.walker_half_height,
            carla_mod=self.carla_mod,
        )

        cmd_mod = getattr(self.carla_mod, "command", None) if self.carla_mod else None
        if cmd_mod is None:
            # Direct actor API fallback when carla.command is not available
            if info.carla_actor is not None:
                info.carla_actor.set_transform(tf)
            return []

        cmds: list[Any] = []

        if state.objectType == SEObjectType.PEDESTRIAN:
            if hasattr(cmd_mod, "ApplyWalkerState"):
                cmds.append(
                    cmd_mod.ApplyWalkerState(
                        info.carla_actor_id, tf, float(max(0.0, state.speed))
                    )
                )
            else:
                cmds.append(cmd_mod.ApplyTransform(info.carla_actor_id, tf))
            return cmds

        # Vehicle or MiscObject transform
        cmds.append(cmd_mod.ApplyTransform(info.carla_actor_id, tf))

        # Linear and angular velocity for Doppler radar & motion blur
        vel_xyz = self.esmini.get_object_velocity_global_xyz(info.esmini_id)
        if vel_xyz is not None:
            vx, vy, vz = vel_xyz
        else:
            vx = state.speed * math.cos(state.h) * math.cos(state.p)
            vy = state.speed * math.sin(state.h) * math.cos(state.p)
            vz = -state.speed * math.sin(state.p)

        carla_vel = esmini_velocity_to_carla(vx, vy, vz, carla_mod=self.carla_mod)
        if hasattr(cmd_mod, "ApplyTargetVelocity"):
            cmds.append(cmd_mod.ApplyTargetVelocity(info.carla_actor_id, carla_vel))

        ang_xyz = self.esmini.get_object_angular_velocity(info.esmini_id)
        if ang_xyz is not None and hasattr(cmd_mod, "ApplyTargetAngularVelocity"):
            carla_ang = esmini_angular_velocity_to_carla(
                ang_xyz[0], ang_xyz[1], ang_xyz[2], carla_mod=self.carla_mod
            )
            cmds.append(
                cmd_mod.ApplyTargetAngularVelocity(info.carla_actor_id, carla_ang)
            )

        # Vehicle light states
        if state.objectType == SEObjectType.VEHICLE and hasattr(
            cmd_mod, "SetVehicleLightState"
        ):
            light_mask = self.compute_vehicle_light_mask(info.esmini_id, state)
            if light_mask != info.last_light_mask:
                info.last_light_mask = light_mask
                # Always pass integer actor_id to SetVehicleLightState
                cmds.append(
                    cmd_mod.SetVehicleLightState(int(info.carla_actor_id), int(light_mask))
                )

        return cmds

    def compute_vehicle_light_mask(
        self, esmini_id: int, state: SEScenarioObjectState
    ) -> int:
        """Query esmini light states for a vehicle and pack them into CARLA's VehicleLightState bitmask."""
        mask = CARLA_LIGHT_NONE
        for esmini_light, carla_bits in ESMINI_TO_CARLA_LIGHT_BITS:
            ls = self.esmini.get_object_light_state(esmini_id, esmini_light)
            if ls is not None and (
                ls.emitting or ls.light_mode in (SELightMode.ON, SELightMode.FLASHING)
            ):
                mask |= carla_bits

        if state.speed < -0.05:
            mask |= CARLA_LIGHT_REVERSE
        return mask

    # ------------------------------------------------------------------
    # Optional Ghost Visualization & Spectator Camera
    # ------------------------------------------------------------------

    def _draw_ego_ghost(self) -> None:
        if (
            self.world is None
            or not hasattr(self.world, "debug")
            or self.ego_esmini_id is None
        ):
            return
        if not self.esmini.object_has_ghost(self.ego_esmini_id):
            return
        ghost_state = self.esmini.get_object_ghost_state(self.ego_esmini_id)
        if ghost_state is None:
            return
        tf = esmini_state_to_carla_transform(
            ghost_state,
            apply_center_offset=self.apply_center_offset,
            z_clearance=0.5 * max(1.5, ghost_state.height),
            carla_mod=self.carla_mod,
        )
        if self.carla_mod is not None and hasattr(self.carla_mod, "BoundingBox"):
            extent = self.carla_mod.Vector3D(
                0.5 * max(4.0, ghost_state.length),
                0.5 * max(1.8, ghost_state.width),
                0.5 * max(1.5, ghost_state.height),
            )
            box = self.carla_mod.BoundingBox(tf.location, extent)
            color = self.carla_mod.Color(0, 255, 180, 60)
            self.world.debug.draw_box(box, tf.rotation, 0.08, color, 0.06)

    def update_spectator(
        self, distance: float = 8.5, height: float = 3.5, pitch: float = -15.0
    ) -> None:
        """Position the CARLA spectator camera behind the Ego vehicle."""
        if self.world is None or self.ego_esmini_id is None:
            return
        info = self.actors.get(self.ego_esmini_id)
        if info is None or info.carla_actor is None:
            return

        ego_tf = info.carla_actor.get_transform()
        yaw_rad = math.radians(ego_tf.rotation.yaw)
        cam_loc = make_location(
            x=ego_tf.location.x - distance * math.cos(yaw_rad),
            y=ego_tf.location.y - distance * math.sin(yaw_rad),
            z=ego_tf.location.z + height,
            carla_mod=self.carla_mod,
        )
        spectator = self.world.get_spectator()
        if spectator is not None:
            from .coordinates import make_rotation, make_transform

            cam_rot = make_rotation(
                pitch=pitch, yaw=ego_tf.rotation.yaw, roll=0.0, carla_mod=self.carla_mod
            )
            spectator.set_transform(
                make_transform(cam_loc, cam_rot, carla_mod=self.carla_mod)
            )

    # ------------------------------------------------------------------
    # Teardown / Cleanup
    # ------------------------------------------------------------------

    def _destroy_actor_entry(self, esmini_id: int) -> None:
        info = self.actors.pop(esmini_id, None)
        if info is None or info.carla_actor is None:
            return
        if info.is_ego and (not info.spawned_by_runner or self.keep_ego_on_exit):
            return
        for sensor in info.attached_sensors:
            try:
                sensor.destroy()
            except Exception:
                pass
        info.attached_sensors.clear()
        try:
            info.carla_actor.destroy()
        except Exception:
            pass

    def cleanup(self) -> None:
        """Destroy all runner-spawned CARLA actors cleanly."""
        if self.ego_esmini_id is not None:
            try:
                self.esmini.register_object_callback(self.ego_esmini_id, None)
            except Exception:
                pass

        for oid in list(self.actors.keys()):
            self._destroy_actor_entry(oid)
        self.actors.clear()
