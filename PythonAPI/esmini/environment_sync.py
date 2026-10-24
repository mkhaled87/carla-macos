# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Synchronizes OpenSCENARIO EnvironmentAction and TrafficSignalAction with CARLA."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

from .esmini_lib import (
    EsminiLib,
    SEStoryBoardElementState,
    SEStoryBoardElementType,
)

try:
    import carla as _carla_module
except ImportError:
    _carla_module = None


CLOUD_STATE_MAP: dict[str, float] = {
    "free": 0.0,
    "clear": 0.0,
    "cloudy": 50.0,
    "overcast": 85.0,
    "rainy": 90.0,
    "skyoff": 100.0,
}

OKTAS_CLOUD_MAP: dict[str, float] = {
    "zerooktas": 0.0,
    "oneoktas": 12.5,
    "twooktas": 25.0,
    "threeoktas": 37.5,
    "fouroktas": 50.0,
    "fiveoktas": 62.5,
    "sixoktas": 75.0,
    "sevenoktas": 87.5,
    "eightoktas": 100.0,
    "nineoktas": 100.0,
}


@dataclass
class WeatherSpec:
    """Normalized weather parameters in CARLA's percentage/angle conventions."""

    cloudiness: float | None = None
    precipitation: float | None = None
    precipitation_deposits: float | None = None
    wind_intensity: float | None = None
    sun_azimuth_angle: float | None = None
    sun_altitude_angle: float | None = None
    fog_density: float | None = None
    fog_distance: float | None = None
    wetness: float | None = None


@dataclass
class TrafficSignalCommand:
    """Normalized traffic signal state transition."""

    signal_id: str
    state: str


class EnvironmentSynchronizer:
    """Parses OpenSCENARIO EnvironmentAction and TrafficSignalAction and applies them to CARLA."""

    def __init__(
        self,
        esmini: EsminiLib,
        carla_world: Any = None,
        xosc_path: str | Path | None = None,
        carla_mod: Any = None,
    ) -> None:
        self.esmini = esmini
        self.world = carla_world
        self.xosc_path = Path(xosc_path).resolve() if xosc_path else None
        self.carla_mod = carla_mod if carla_mod is not None else _carla_module

        self.init_weather_specs: list[WeatherSpec] = []
        self.init_signal_commands: list[TrafficSignalCommand] = []

        # Keyed by action name and full storyboard path
        self.action_weather_specs: dict[str, WeatherSpec] = {}
        self.action_signal_commands: dict[str, list[TrafficSignalCommand]] = {}

        # Controller phase maps: (controller_name, phase_name) -> list[TrafficSignalCommand]
        self.controller_phases: dict[tuple[str, str], list[TrafficSignalCommand]] = {}

        self._initial_carla_weather: Any = None
        self._traffic_lights_by_odr_id: dict[str, Any] = {}
        self._frozen_traffic_lights: list[Any] = []

    # ------------------------------------------------------------------
    # Initialization & XML Indexing
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        """Index the active OpenSCENARIO XML, cache CARLA traffic lights, and apply Init actions."""
        if self.world is not None:
            try:
                self._initial_carla_weather = self.world.get_weather()
            except Exception:
                self._initial_carla_weather = None
            self._index_carla_traffic_lights()

        xml_str = self.esmini.get_xml_string()
        root: ET.Element | None = None
        if xml_str:
            try:
                root = ET.fromstring(xml_str)
            except ET.ParseError:
                root = None
        if root is None and self.xosc_path is not None and self.xosc_path.is_file():
            try:
                root = ET.parse(str(self.xosc_path)).getroot()
            except Exception:
                root = None

        if root is not None:
            self._parse_xosc_root(root)

        # Apply any <Init> global EnvironmentAction / TrafficSignalAction immediately
        for wspec in self.init_weather_specs:
            self.apply_weather_spec(wspec)
        for sig_cmd in self.init_signal_commands:
            self.apply_traffic_signal_command(sig_cmd)

    def _index_carla_traffic_lights(self) -> None:
        self._traffic_lights_by_odr_id.clear()
        if self.world is None:
            return
        try:
            lights = self.world.get_actors().filter("traffic.traffic_light*")
            for tl in lights:
                odr_id = (
                    str(tl.get_opendrive_id())
                    if hasattr(tl, "get_opendrive_id")
                    else ""
                )
                if odr_id:
                    self._traffic_lights_by_odr_id[odr_id] = tl
                self._traffic_lights_by_odr_id[str(tl.id)] = tl
        except Exception:
            pass

    def _parse_xosc_root(self, root: ET.Element) -> None:
        # 1. Index TrafficSignalControllers in <RoadNetwork><TrafficSignals>
        for ctrl_el in root.findall(".//RoadNetwork/TrafficSignals/TrafficSignalController"):
            ctrl_name = ctrl_el.attrib.get("name", "")
            for phase_el in ctrl_el.findall("Phase"):
                phase_name = phase_el.attrib.get("name", "")
                cmds: list[TrafficSignalCommand] = []
                for state_el in phase_el.findall("TrafficSignalState"):
                    sig_id = state_el.attrib.get("trafficSignalId", "")
                    st_val = state_el.attrib.get("state", "")
                    if sig_id and st_val:
                        cmds.append(TrafficSignalCommand(signal_id=sig_id, state=st_val))
                if ctrl_name and phase_name and cmds:
                    self.controller_phases[(ctrl_name, phase_name)] = cmds

        # 2. Index <Init><Actions><GlobalAction>
        for global_act in root.findall(".//Init/Actions/GlobalAction"):
            env_el = global_act.find(".//Environment")
            if env_el is not None:
                self.init_weather_specs.append(self.parse_environment_element(env_el))
            for sig_cmd in self._extract_signal_commands(global_act):
                self.init_signal_commands.append(sig_cmd)

        # 3. Walk Storyboard hierarchy and index <Action> elements by name and path
        storyboard = root.find("Storyboard")
        if storyboard is not None:
            self._walk_storyboard(storyboard, [])

    def _walk_storyboard(self, element: ET.Element, path_stack: list[str]) -> None:
        name = element.attrib.get("name", "")
        current_stack = [*path_stack, name] if name else path_stack

        if element.tag == "Action":
            full_path = "/".join(current_stack)
            env_el = element.find(".//EnvironmentAction/Environment")
            if env_el is not None:
                wspec = self.parse_environment_element(env_el)
                if name:
                    self.action_weather_specs[name] = wspec
                if full_path:
                    self.action_weather_specs[full_path] = wspec

            sig_cmds = self._extract_signal_commands(element)
            if sig_cmds:
                if name:
                    self.action_signal_commands[name] = sig_cmds
                if full_path:
                    self.action_signal_commands[full_path] = sig_cmds

        for child in element:
            self._walk_storyboard(child, current_stack)

    def _extract_signal_commands(self, container: ET.Element) -> list[TrafficSignalCommand]:
        cmds: list[TrafficSignalCommand] = []
        for state_act in container.findall(".//TrafficSignalStateAction"):
            sig_name = state_act.attrib.get("name", "")
            sig_state = state_act.attrib.get("state", "")
            if sig_name and sig_state:
                cmds.append(TrafficSignalCommand(signal_id=sig_name, state=sig_state))

        for ctrl_act in container.findall(".//TrafficSignalControllerAction"):
            ctrl_ref = ctrl_act.attrib.get("trafficSignalControllerRef", "")
            phase = ctrl_act.attrib.get("phase", "")
            phase_cmds = self.controller_phases.get((ctrl_ref, phase), [])
            cmds.extend(phase_cmds)

        return cmds

    # ------------------------------------------------------------------
    # Parsing <Environment> XML into WeatherSpec
    # ------------------------------------------------------------------

    @staticmethod
    def parse_environment_element(env_el: ET.Element) -> WeatherSpec:
        spec = WeatherSpec()

        weather_el = env_el.find("Weather")
        if weather_el is not None:
            cloud_state = weather_el.attrib.get("cloudState", "").strip().lower()
            if cloud_state in CLOUD_STATE_MAP:
                spec.cloudiness = CLOUD_STATE_MAP[cloud_state]

            frac_cloud = weather_el.attrib.get("fractionalCloudCover", "").strip().lower()
            if frac_cloud in OKTAS_CLOUD_MAP:
                spec.cloudiness = OKTAS_CLOUD_MAP[frac_cloud]

            sun_el = weather_el.find("Sun")
            if sun_el is not None:
                if "azimuth" in sun_el.attrib:
                    az = float(sun_el.attrib["azimuth"])
                    # OpenSCENARIO specifies radians; if > 2*pi, treat as degrees
                    az_deg = az if abs(az) > 2.0 * math.pi + 0.1 else math.degrees(az)
                    spec.sun_azimuth_angle = az_deg % 360.0
                if "elevation" in sun_el.attrib:
                    el = float(sun_el.attrib["elevation"])
                    el_deg = el if abs(el) > math.pi + 0.1 else math.degrees(el)
                    spec.sun_altitude_angle = max(-90.0, min(90.0, el_deg))

            fog_el = weather_el.find("Fog")
            if fog_el is not None and "visualRange" in fog_el.attrib:
                vis_range = max(1.0, float(fog_el.attrib["visualRange"]))
                if vis_range >= 10000.0:
                    spec.fog_density = 0.0
                    spec.fog_distance = 1000.0
                else:
                    # Map visualRange [10m .. 2000m] to fog_density [100% .. 0%]
                    density = max(0.0, min(100.0, 100.0 * (1.0 - math.log10(max(10.0, vis_range) / 10.0) / 2.3)))
                    spec.fog_density = density
                    spec.fog_distance = min(1000.0, vis_range * 0.5)

            precip_el = weather_el.find("Precipitation")
            if precip_el is not None:
                ptype = precip_el.attrib.get("precipitationType", "dry").strip().lower()
                if ptype == "dry":
                    spec.precipitation = 0.0
                    spec.precipitation_deposits = 0.0
                    spec.wetness = 0.0
                else:
                    raw_int = float(
                        precip_el.attrib.get(
                            "intensity",
                            precip_el.attrib.get("precipitationIntensity", "0.5"),
                        )
                    )
                    # OpenSCENARIO 1.0 intensity is [0..1]; 1.1+ precipitationIntensity is mm/h
                    pct = raw_int * 100.0 if raw_int <= 1.0 else min(100.0, raw_int * 5.0)
                    spec.precipitation = max(0.0, min(100.0, pct))
                    spec.precipitation_deposits = max(0.0, min(100.0, pct * 0.8))
                    spec.wetness = max(0.0, min(100.0, pct))

            wind_el = weather_el.find("Wind")
            if wind_el is not None and "speed" in wind_el.attrib:
                wind_speed = max(0.0, float(wind_el.attrib["speed"]))
                # Map 0..25 m/s to 0..100%
                spec.wind_intensity = min(100.0, wind_speed * 4.0)

        return spec

    # ------------------------------------------------------------------
    # Applying Weather & Traffic Signals in CARLA
    # ------------------------------------------------------------------

    def on_storyboard_state_change(
        self,
        name: str,
        elem_type: SEStoryBoardElementType,
        state: SEStoryBoardElementState,
        full_path: str,
    ) -> None:
        """Callback invoked when an esmini Storyboard element changes state."""
        if state != SEStoryBoardElementState.RUNNING:
            return
        if elem_type not in (
            SEStoryBoardElementType.ACTION,
            SEStoryBoardElementType.UNDEFINED_ELEMENT_TYPE,
        ):
            return

        wspec = self.action_weather_specs.get(full_path) or self.action_weather_specs.get(name)
        if wspec is not None:
            self.apply_weather_spec(wspec)

        sig_cmds = (
            self.action_signal_commands.get(full_path)
            or self.action_signal_commands.get(name)
        )
        if sig_cmds:
            for cmd in sig_cmds:
                self.apply_traffic_signal_command(cmd)

    def apply_weather_spec(self, spec: WeatherSpec) -> Any:
        """Apply a ``WeatherSpec`` to ``carla_world`` and return the updated weather object."""
        if self.world is None:
            return spec

        try:
            weather = self.world.get_weather()
        except Exception:
            if self.carla_mod is not None and hasattr(self.carla_mod, "WeatherParameters"):
                weather = self.carla_mod.WeatherParameters()
            else:
                return spec

        for field_name in (
            "cloudiness",
            "precipitation",
            "precipitation_deposits",
            "wind_intensity",
            "sun_azimuth_angle",
            "sun_altitude_angle",
            "fog_density",
            "fog_distance",
            "wetness",
        ):
            val = getattr(spec, field_name)
            if val is not None and hasattr(weather, field_name):
                setattr(weather, field_name, float(val))

        self.world.set_weather(weather)
        return weather

    def apply_traffic_signal_command(self, cmd: TrafficSignalCommand) -> bool:
        """Set the state of a CARLA traffic light matching ``cmd.signal_id``."""
        tl = self._traffic_lights_by_odr_id.get(str(cmd.signal_id))
        if tl is None:
            # Strip common prefixes such as "id=" or "pos="
            cleaned = str(cmd.signal_id).split("=")[-1].strip()
            tl = self._traffic_lights_by_odr_id.get(cleaned)

        if tl is None:
            return False

        carla_state = self.map_signal_state_string(cmd.state, self.carla_mod)
        if carla_state is None:
            return False

        if hasattr(tl, "freeze") and tl not in self._frozen_traffic_lights:
            tl.freeze(True)
            self._frozen_traffic_lights.append(tl)

        if hasattr(tl, "set_state"):
            tl.set_state(carla_state)
        return True

    @staticmethod
    def map_signal_state_string(state_str: str, carla_mod: Any = None) -> Any:
        """Convert an OpenSCENARIO TrafficSignal state string into ``carla.TrafficLightState``.

        Supports both esmini's 3-lamp semicolon syntax (``"on;off;off"`` = Red,
        ``"off;on;off"`` = Yellow, ``"off;off;on"`` = Green, ``"off;off;off"`` = Off)
        and semantic names (``"red"``, ``"yellow"``, ``"green"``, ``"1;0;0"``, etc.).
        """
        s = state_str.strip().lower()
        tl_enum = getattr(carla_mod, "TrafficLightState", None) if carla_mod else None

        if ";" in s:
            parts = [p.strip() for p in s.split(";")]
            active = [p in ("on", "flashing", "1", "true") for p in parts]
            if len(active) >= 3:
                if active[2]:
                    return tl_enum.Green if tl_enum is not None else "Green"
                if active[1]:
                    return tl_enum.Yellow if tl_enum is not None else "Yellow"
                if active[0]:
                    return tl_enum.Red if tl_enum is not None else "Red"
                return tl_enum.Off if tl_enum is not None else "Off"

        if s in ("red", "stop", "r"):
            return tl_enum.Red if tl_enum is not None else "Red"
        if s in ("yellow", "amber", "y"):
            return tl_enum.Yellow if tl_enum is not None else "Yellow"
        if s in ("green", "go", "g"):
            return tl_enum.Green if tl_enum is not None else "Green"
        if s in ("off", "none", "flashing"):
            return tl_enum.Off if tl_enum is not None else "Off"
        return None

    def cleanup(self) -> None:
        """Unfreeze any scenario-controlled traffic lights and optionally restore initial weather."""
        for tl in self._frozen_traffic_lights:
            try:
                tl.freeze(False)
            except Exception:
                pass
        self._frozen_traffic_lights.clear()

        if self.world is not None and self._initial_carla_weather is not None:
            try:
                self.world.set_weather(self._initial_carla_weather)
            except Exception:
                pass
