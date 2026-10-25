# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Coordinate frame conversions (OpenDRIVE/esmini <-> CARLA) and OpenDRIVE map sync."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

from .esmini_lib import SEObjectType, SEScenarioObjectState

try:
    import carla as _carla_module
except ImportError:
    _carla_module = None


# ==============================================================================
# -- Lightweight Fallback Geometry Types (used when carla module isn't loaded) -
# ==============================================================================


@dataclass
class FallbackVector3D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


@dataclass
class FallbackLocation(FallbackVector3D):
    pass


@dataclass
class FallbackRotation:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    def get_forward_vector(self) -> FallbackVector3D:
        cp = math.cos(math.radians(self.pitch))
        sp = math.sin(math.radians(self.pitch))
        cy = math.cos(math.radians(self.yaw))
        sy = math.sin(math.radians(self.yaw))
        return FallbackVector3D(cp * cy, cp * sy, sp)


@dataclass
class FallbackTransform:
    location: FallbackLocation = None  # type: ignore[assignment]
    rotation: FallbackRotation = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.location is None:
            self.location = FallbackLocation()
        if self.rotation is None:
            self.rotation = FallbackRotation()

    def get_forward_vector(self) -> FallbackVector3D:
        return self.rotation.get_forward_vector()


def make_location(x: float = 0.0, y: float = 0.0, z: float = 0.0, carla_mod: Any = None) -> Any:
    mod = carla_mod if carla_mod is not None else _carla_module
    if mod is not None:
        return mod.Location(x=float(x), y=float(y), z=float(z))
    return FallbackLocation(x=float(x), y=float(y), z=float(z))


def make_rotation(
    pitch: float = 0.0, yaw: float = 0.0, roll: float = 0.0, carla_mod: Any = None
) -> Any:
    mod = carla_mod if carla_mod is not None else _carla_module
    if mod is not None:
        return mod.Rotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll))
    return FallbackRotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll))


def make_transform(location: Any = None, rotation: Any = None, carla_mod: Any = None) -> Any:
    mod = carla_mod if carla_mod is not None else _carla_module
    loc = location if location is not None else make_location(carla_mod=mod)
    rot = rotation if rotation is not None else make_rotation(carla_mod=mod)
    if mod is not None:
        return mod.Transform(loc, rot)
    return FallbackTransform(loc, rot)


def make_vector3d(x: float = 0.0, y: float = 0.0, z: float = 0.0, carla_mod: Any = None) -> Any:
    mod = carla_mod if carla_mod is not None else _carla_module
    if mod is not None:
        return mod.Vector3D(x=float(x), y=float(y), z=float(z))
    return FallbackVector3D(x=float(x), y=float(y), z=float(z))


# ==============================================================================
# -- Core OpenDRIVE (Right-Handed, rad) <-> CARLA (Left-Handed, deg) Math ------
# ==============================================================================


def normalize_angle_rad(angle_rad: float) -> float:
    """Wrap an angle in radians to ``[0, 2*pi)`` as used by esmini."""
    two_pi = 2.0 * math.pi
    res = math.fmod(angle_rad, two_pi)
    if res < 0.0:
        res += two_pi
    return res


def normalize_angle_deg(angle_deg: float) -> float:
    """Wrap an angle in degrees to ``[-180, 180)``."""
    res = math.fmod(angle_deg + 180.0, 360.0)
    if res < 0.0:
        res += 360.0
    return res - 180.0


def apply_local_offset_in_esmini_frame(
    x: float,
    y: float,
    z: float,
    h: float,
    p: float,
    r: float,
    offset_x: float,
    offset_y: float,
    offset_z: float = 0.0,
) -> tuple[float, float, float]:
    """Translate an OpenDRIVE pose ``(x, y, z, h, p, r)`` by a local offset ``(offset_x, offset_y, offset_z)``.

    Uses the standard ZYX (heading-pitch-roll) intrinsic rotation matrix in right-handed
    OpenDRIVE space so that translating from the rear-axle reference point to the vehicle's
    geometric center is exact on flat, pitched, or banked roads.
    """
    if offset_x == 0.0 and offset_y == 0.0 and offset_z == 0.0:
        return (x, y, z)

    ch, sh = math.cos(h), math.sin(h)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    dx = (
        offset_x * (ch * cp)
        + offset_y * (ch * sp * sr - sh * cr)
        + offset_z * (ch * sp * cr + sh * sr)
    )
    dy = (
        offset_x * (sh * cp)
        + offset_y * (sh * sp * sr + ch * cr)
        + offset_z * (sh * sp * cr - ch * sr)
    )
    dz = (
        offset_x * (-sp)
        + offset_y * (cp * sr)
        + offset_z * (cp * cr)
    )
    return (x + dx, y + dy, z + dz)


def esmini_pose_to_carla_transform(
    x: float,
    y: float,
    z: float,
    h: float,
    p: float,
    r: float,
    center_offset_x: float = 0.0,
    center_offset_y: float = 0.0,
    z_offset: float = 0.0,
    carla_mod: Any = None,
) -> Any:
    """Convert an esmini/OpenDRIVE pose ``(x, y, z, h, p, r)`` (m, rad) to a CARLA ``Transform`` (m, deg).

    Handedness reflection across the XZ plane (``S = diag(1, -1, 1)``):
    - ``x_carla = x_esmini``
    - ``y_carla = -y_esmini``
    - ``z_carla = z_esmini + z_offset``
    - ``roll_carla = degrees(r_esmini)``
    - ``pitch_carla = -degrees(p_esmini)``
    - ``yaw_carla = -degrees(h_esmini)``
    """
    wx, wy, wz = apply_local_offset_in_esmini_frame(
        x, y, z, h, p, r, center_offset_x, center_offset_y, 0.0
    )
    loc = make_location(
        x=wx,
        y=-wy,
        z=wz + z_offset,
        carla_mod=carla_mod,
    )
    rot = make_rotation(
        pitch=-math.degrees(p),
        yaw=-math.degrees(h),
        roll=math.degrees(r),
        carla_mod=carla_mod,
    )
    return make_transform(loc, rot, carla_mod=carla_mod)


def esmini_state_to_carla_transform(
    state: SEScenarioObjectState,
    apply_center_offset: bool = True,
    carla_bbox_location_x: float = 0.0,
    carla_bbox_location_y: float = 0.0,
    walker_half_height: float | None = None,
    z_clearance: float = 0.0,
    carla_mod: Any = None,
) -> Any:
    """Convert an ``SEScenarioObjectState`` directly to a CARLA ``Transform``.

    - For vehicles: optionally shifts from esmini's rear-axle reference point by
      ``(state.centerOffsetX - carla_bbox_location_x, state.centerOffsetY + carla_bbox_location_y)``
      so the CARLA vehicle's bounding box center aligns with esmini's bounding box center.
    - For pedestrians (``SEObjectType.PEDESTRIAN``): adds the pedestrian capsule's
      half-height to ``z`` because CARLA walker origins are at the center of the capsule.
    """
    offset_x = 0.0
    offset_y = 0.0
    z_offset = z_clearance

    if state.objectType == SEObjectType.PEDESTRIAN:
        if walker_half_height is not None and walker_half_height > 0.0:
            z_offset += walker_half_height
        elif state.height > 0.0:
            z_offset += 0.5 * state.height
        else:
            z_offset += 0.95
    elif apply_center_offset and state.objectType == SEObjectType.VEHICLE:
        offset_x = float(state.centerOffsetX) - float(carla_bbox_location_x)
        offset_y = float(state.centerOffsetY) + float(carla_bbox_location_y)

    return esmini_pose_to_carla_transform(
        x=state.x,
        y=state.y,
        z=state.z,
        h=state.h,
        p=state.p,
        r=state.r,
        center_offset_x=offset_x,
        center_offset_y=offset_y,
        z_offset=z_offset,
        carla_mod=carla_mod,
    )


def carla_transform_to_esmini_pose(
    transform: Any,
    center_offset_x: float = 0.0,
    center_offset_y: float = 0.0,
    carla_bbox_location_x: float = 0.0,
    carla_bbox_location_y: float = 0.0,
    z_offset: float = 0.0,
) -> tuple[float, float, float, float, float, float]:
    """Convert a CARLA ``Transform`` (m, deg) to an esmini pose ``(x, y, z, h, p, r)`` (m, rad).

    Applies the exact inverse of ``esmini_pose_to_carla_transform``, including translating
    from the CARLA actor origin back to the esmini rear-axle reference point if
    ``center_offset_x`` / ``center_offset_y`` are provided.
    """
    loc = transform.location
    rot = transform.rotation

    h = normalize_angle_rad(math.radians(-rot.yaw))
    p = math.radians(-rot.pitch)
    r = math.radians(rot.roll)

    cx = float(loc.x)
    cy = -float(loc.y)
    cz = float(loc.z) - float(z_offset)

    dx_local = float(center_offset_x) - float(carla_bbox_location_x)
    dy_local = float(center_offset_y) + float(carla_bbox_location_y)

    rx, ry, rz = apply_local_offset_in_esmini_frame(
        cx, cy, cz, h, p, r, -dx_local, -dy_local, 0.0
    )
    return (rx, ry, rz, h, p, r)


def esmini_velocity_to_carla(
    vx: float, vy: float, vz: float, carla_mod: Any = None
) -> Any:
    """Convert global linear velocity ``(vx, vy, vz)`` (m/s) from esmini to CARLA ``Vector3D``."""
    return make_vector3d(x=vx, y=-vy, z=vz, carla_mod=carla_mod)


def carla_velocity_to_esmini(velocity: Any) -> tuple[float, float, float]:
    """Convert CARLA ``Vector3D`` velocity (m/s) to esmini ``(vx, vy, vz)`` (m/s)."""
    return (float(velocity.x), -float(velocity.y), float(velocity.z))


def esmini_angular_velocity_to_carla(
    h_rate: float, p_rate: float, r_rate: float, carla_mod: Any = None
) -> Any:
    """Convert esmini angular rates ``(h_rate, p_rate, r_rate)`` (rad/s) to CARLA ``Vector3D`` (deg/s)."""
    return make_vector3d(
        x=math.degrees(r_rate),
        y=-math.degrees(p_rate),
        z=-math.degrees(h_rate),
        carla_mod=carla_mod,
    )


def carla_angular_velocity_to_esmini(ang_vel: Any) -> tuple[float, float, float]:
    """Convert CARLA angular velocity ``Vector3D`` (deg/s) to esmini ``(h_rate, p_rate, r_rate)`` (rad/s)."""
    r_rate = math.radians(float(ang_vel.x))
    p_rate = -math.radians(float(ang_vel.y))
    h_rate = -math.radians(float(ang_vel.z))
    return (h_rate, p_rate, r_rate)


# ==============================================================================
# -- OpenDRIVE (.xodr) Map Discovery & Synchronization -------------------------
# ==============================================================================


def parse_xosc_logic_file(xosc_path: str | Path) -> str | None:
    """Extract the ``<RoadNetwork><LogicFile filepath="..."/>`` attribute from an ``.xosc`` file."""
    try:
        tree = ET.parse(str(xosc_path))
        root = tree.getroot()
        logic_el = root.find(".//RoadNetwork/LogicFile")
        if logic_el is not None:
            return logic_el.attrib.get("filepath")
    except Exception:
        pass
    return None


def resolve_xodr_path(
    xosc_path: str | Path,
    logic_filepath: str,
    additional_search_paths: list[str | Path] | None = None,
) -> Path | None:
    """Check if ``logic_filepath`` already exists on disk relative to ``xosc_path`` or search paths."""
    if not logic_filepath:
        return None

    raw = Path(logic_filepath)
    if raw.is_absolute() and raw.is_file():
        return raw.resolve()

    xosc_dir = Path(xosc_path).resolve().parent
    candidates: list[Path] = [
        xosc_dir / raw,
        xosc_dir.parent / "xodr" / raw.name,
    ]
    if additional_search_paths:
        for sp in additional_search_paths:
            sp_path = Path(sp).resolve()
            candidates.append(sp_path / raw)
            candidates.append(sp_path / raw.name)
            candidates.append(sp_path / "xodr" / raw.name)

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None


def infer_carla_town_name(logic_filepath: str | None) -> str | None:
    """Infer a CARLA map/town name if ``logic_filepath`` names a known CARLA Town (e.g. ``Town01``)."""
    if not logic_filepath:
        return None
    stem = Path(logic_filepath).stem
    if stem.lower().startswith("town"):
        return stem
    return None


def sync_opendrive_map(
    xosc_path: str | Path,
    esmini: Any,
    carla_client: Any = None,
    carla_world: Any = None,
    reload_carla_world: bool = False,
    additional_search_paths: list[str | Path] | None = None,
) -> tuple[Any, Path | None]:
    """Synchronize the OpenDRIVE road network between ``.xosc``, ``esminiLib``, and CARLA.

    1. Adds the ``.xosc`` directory (and ``../xodr``, ``../models``, ``../catalogs``) to
       ``esmini``'s search paths via ``SE_AddPath``.
    2. Inspects ``<RoadNetwork><LogicFile filepath="..."/></RoadNetwork>``.
    3. If ``carla_world`` is provided:
       - Optionally loads the matching CARLA town or generates an OpenDRIVE world when
         ``reload_carla_world=True``.
       - If the ``.xodr`` file referenced by ``LogicFile`` is not on disk (e.g. ``filepath="Town10HD.xodr"``
         or ``filepath="Town04"``), exports ``carla_world.get_map().to_opendrive()`` into a
         temporary directory under the exact filename expected by ``esmini`` and registers
         that directory via ``esmini.add_path()``.
    """
    xosc_file = Path(xosc_path).resolve()
    xosc_dir = xosc_file.parent

    # Register standard relative OpenSCENARIO directories with esmini
    default_dirs = [
        xosc_dir,
        xosc_dir.parent / "xodr",
        xosc_dir.parent / "models",
        xosc_dir.parent / "catalogs",
        xosc_dir.parent / "Catalogs",
    ]
    if additional_search_paths:
        default_dirs.extend(Path(p).resolve() for p in additional_search_paths)

    for d in default_dirs:
        if d.is_dir():
            esmini.add_path(d)

    logic_fp = parse_xosc_logic_file(xosc_file)
    existing_xodr = (
        resolve_xodr_path(xosc_file, logic_fp, additional_search_paths)
        if logic_fp
        else None
    )

    temp_xodr_dir: Path | None = None

    if carla_world is not None:
        carla_map = carla_world.get_map()
        current_map_name = Path(carla_map.name).name if getattr(carla_map, "name", None) else ""
        desired_town = infer_carla_town_name(logic_fp)

        if reload_carla_world and carla_client is not None:
            if desired_town and desired_town.lower() not in current_map_name.lower():
                available = carla_client.get_available_maps()
                matching = [
                    m for m in available if Path(m).name.lower() == desired_town.lower()
                ]
                target_map = matching[0] if matching else desired_town
                carla_world = carla_client.load_world(target_map)
                carla_map = carla_world.get_map()
            elif existing_xodr is not None and not desired_town:
                xodr_text = existing_xodr.read_text(encoding="utf-8")
                if hasattr(carla_client, "generate_opendrive_world"):
                    carla_world = carla_client.generate_opendrive_world(xodr_text)
                    carla_map = carla_world.get_map()

        # If the .xosc references an OpenDRIVE file that is not on disk (e.g. a CARLA town's XODR),
        # export CARLA's active OpenDRIVE string so esmini uses the exact same road geometry.
        if logic_fp and existing_xodr is None and hasattr(carla_map, "to_opendrive"):
            xodr_content = carla_map.to_opendrive()
            if xodr_content:
                temp_xodr_dir = Path(tempfile.mkdtemp(prefix="carla_esmini_xodr_"))
                target_rel = Path(logic_fp).name
                if not target_rel.endswith(".xodr"):
                    target_file = temp_xodr_dir / f"{target_rel}.xodr"
                    # Also write exact name in case LogicFile has no extension
                    (temp_xodr_dir / target_rel).write_text(xodr_content, encoding="utf-8")
                else:
                    target_file = temp_xodr_dir / target_rel
                target_file.write_text(xodr_content, encoding="utf-8")
                esmini.add_path(temp_xodr_dir)

    return carla_world, temp_xodr_dir

