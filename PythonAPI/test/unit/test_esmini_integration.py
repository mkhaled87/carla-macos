# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Unit and integration tests for CARLA's esminiLib OpenSCENARIO runner."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import tempfile
from typing import Any
import unittest

# Ensure PythonAPI/ is on sys.path
_PYTHON_API_DIR = Path(__file__).resolve().parents[2]
if str(_PYTHON_API_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_API_DIR))

from esmini import (  # noqa: E402
    CarlaEsminiRunner,
    EsminiLib,
    RunnerConfig,
    SEObjectType,
    SEScenarioObjectState,
    SEVehicleCategory,
    carla_angular_velocity_to_esmini,
    carla_transform_to_esmini_pose,
    carla_velocity_to_esmini,
    esmini_angular_velocity_to_carla,
    esmini_pose_to_carla_transform,
    esmini_state_to_carla_transform,
    esmini_velocity_to_carla,
)
from esmini.coordinates import (  # noqa: E402
    FallbackLocation,
    FallbackRotation,
    FallbackTransform,
    FallbackVector3D,
)


# ==============================================================================
# -- Sample OpenDRIVE (.xodr) & OpenSCENARIO (.xosc) Fixtures ------------------
# ==============================================================================

SAMPLE_STRAIGHT_XODR = """\
<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="5" name="TestStraightRoad" version="1.00" date="2026-01-01"/>
    <road name="Road1" length="500.0" id="1" junction="-1">
        <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="500.0">
                <line/>
            </geometry>
        </planView>
        <elevationProfile>
            <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
        </elevationProfile>
        <lateralProfile/>
        <lanes>
            <laneSection s="0.0">
                <left>
                    <lane id="1" type="driving" level="false">
                        <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
                    </lane>
                </left>
                <center>
                    <lane id="0" type="none" level="false"/>
                </center>
                <right>
                    <lane id="-1" type="driving" level="false">
                        <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
                    </lane>
                    <lane id="-2" type="driving" level="false">
                        <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
        <signals>
            <signal s="120.0" t="-4.0" id="101" name="Signal_101" dynamic="yes" orientation="+" country="OpenDRIVE" type="1000001"/>
        </signals>
    </road>
</OpenDRIVE>
"""

SAMPLE_SCENARIO_XOSC_TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
    <FileHeader revMajor="1" revMinor="2" date="2026-01-01T00:00:00" description="CARLA esmini integration test" author="CARLA"/>
    <ParameterDeclarations>
        <ParameterDeclaration name="TargetSpeed" parameterType="double" value="15.0"/>
        <ParameterDeclaration name="TriggerDist" parameterType="double" value="25.0"/>
    </ParameterDeclarations>
    <CatalogLocations/>
    <RoadNetwork>
        <LogicFile filepath="{xodr_filename}"/>
    </RoadNetwork>
    <Entities>
        <ScenarioObject name="hero">
            <Vehicle name="car_ego" vehicleCategory="car">
                <BoundingBox>
                    <Center x="1.4" y="0.0" z="0.75"/>
                    <Dimensions width="2.0" length="4.8" height="1.5"/>
                </BoundingBox>
                <Performance maxSpeed="60.0" maxAcceleration="10.0" maxDeceleration="10.0"/>
                <Axles>
                    <FrontAxle maxSteering="0.5" wheelDiameter="0.7" trackWidth="1.6" positionX="2.8" positionZ="0.35"/>
                    <RearAxle maxSteering="0.0" wheelDiameter="0.7" trackWidth="1.6" positionX="0.0" positionZ="0.35"/>
                </Axles>
                <Properties>
                    <Property name="carla_blueprint" value="vehicle.lincoln.mkz_2020"/>
                    <Property name="color" value="#143CC8"/>
                </Properties>
            </Vehicle>
        </ScenarioObject>
        <ScenarioObject name="lead_npc">
            <Vehicle name="car_red" vehicleCategory="car">
                <BoundingBox>
                    <Center x="1.35" y="0.0" z="0.75"/>
                    <Dimensions width="1.9" length="4.5" height="1.5"/>
                </BoundingBox>
                <Performance maxSpeed="60.0" maxAcceleration="10.0" maxDeceleration="10.0"/>
                <Axles>
                    <FrontAxle maxSteering="0.5" wheelDiameter="0.7" trackWidth="1.6" positionX="2.7" positionZ="0.35"/>
                    <RearAxle maxSteering="0.0" wheelDiameter="0.7" trackWidth="1.6" positionX="0.0" positionZ="0.35"/>
                </Axles>
                <Properties>
                    <Property name="color" value="#C81414"/>
                </Properties>
            </Vehicle>
        </ScenarioObject>
        <ScenarioObject name="crossing_walker">
            <Pedestrian name="walker_01" pedestrianCategory="pedestrian" model="walker" mass="75.0">
                <BoundingBox>
                    <Center x="0.0" y="0.0" z="0.9"/>
                    <Dimensions width="0.5" length="0.6" height="1.8"/>
                </BoundingBox>
                <Properties/>
            </Pedestrian>
        </ScenarioObject>
    </Entities>
    <Storyboard>
        <Init>
            <Actions>
                <GlobalAction>
                    <EnvironmentAction>
                        <Environment name="InitWeather">
                            <TimeOfDay animation="false" dateTime="2026-05-01T12:00:00"/>
                            <Weather cloudState="cloudy">
                                <Sun illuminance="100000.0" azimuth="0.5" elevation="0.8"/>
                                <Fog visualRange="10000.0"/>
                                <Precipitation precipitationType="dry" precipitationIntensity="0.0"/>
                            </Weather>
                            <RoadCondition frictionScaleFactor="1.0"/>
                        </Environment>
                    </EnvironmentAction>
                </GlobalAction>
                <GlobalAction>
                    <InfrastructureAction>
                        <TrafficSignalAction>
                            <TrafficSignalStateAction name="101" state="on;off;off"/>
                        </TrafficSignalAction>
                    </InfrastructureAction>
                </GlobalAction>
                <Private entityRef="hero">
                    <PrivateAction>
                        <TeleportAction>
                            <Position>
                                <LanePosition roadId="1" laneId="-1" offset="0.0" s="10.0"/>
                            </Position>
                        </TeleportAction>
                    </PrivateAction>
                </Private>
                <Private entityRef="lead_npc">
                    <PrivateAction>
                        <TeleportAction>
                            <Position>
                                <LanePosition roadId="1" laneId="-1" offset="0.0" s="50.0"/>
                            </Position>
                        </TeleportAction>
                    </PrivateAction>
                    <PrivateAction>
                        <LongitudinalAction>
                            <SpeedAction>
                                <SpeedActionDynamics dynamicsShape="step" value="0.0" dynamicsDimension="time"/>
                                <SpeedActionTarget>
                                    <AbsoluteTargetSpeed value="5.0"/>
                                </SpeedActionTarget>
                            </SpeedAction>
                        </LongitudinalAction>
                    </PrivateAction>
                </Private>
                <Private entityRef="crossing_walker">
                    <PrivateAction>
                        <TeleportAction>
                            <Position>
                                <LanePosition roadId="1" laneId="-2" offset="0.0" s="80.0"/>
                            </Position>
                        </TeleportAction>
                    </PrivateAction>
                    <PrivateAction>
                        <LongitudinalAction>
                            <SpeedAction>
                                <SpeedActionDynamics dynamicsShape="step" value="0.0" dynamicsDimension="time"/>
                                <SpeedActionTarget>
                                    <AbsoluteTargetSpeed value="1.5"/>
                                </SpeedActionTarget>
                            </SpeedAction>
                        </LongitudinalAction>
                    </PrivateAction>
                </Private>
            </Actions>
        </Init>
        <Story name="MainStory">
            <Act name="MainAct">
                <ManeuverGroup maximumExecutionCount="1" name="NpcManeuverGroup">
                    <Actors selectTriggeringEntities="false">
                        <EntityRef entityRef="lead_npc"/>
                    </Actors>
                    <Maneuver name="NpcAccelerateManeuver">
                        <Event name="EgoApproachesNpcEvent" priority="overwrite">
                            <Action name="AccelerateNpcAction">
                                <PrivateAction>
                                    <LongitudinalAction>
                                        <SpeedAction>
                                            <SpeedActionDynamics dynamicsShape="linear" value="5.0" dynamicsDimension="rate"/>
                                            <SpeedActionTarget>
                                                <AbsoluteTargetSpeed value="$TargetSpeed"/>
                                            </SpeedActionTarget>
                                        </SpeedAction>
                                    </LongitudinalAction>
                                </PrivateAction>
                            </Action>
                            <Action name="SwitchLightGreenAction">
                                <GlobalAction>
                                    <InfrastructureAction>
                                        <TrafficSignalAction>
                                            <TrafficSignalStateAction name="101" state="off;off;on"/>
                                        </TrafficSignalAction>
                                    </InfrastructureAction>
                                </GlobalAction>
                            </Action>
                            <Action name="RainWeatherAction">
                                <GlobalAction>
                                    <EnvironmentAction>
                                        <Environment name="RainEnv">
                                            <TimeOfDay animation="false" dateTime="2026-05-01T14:00:00"/>
                                            <Weather cloudState="rainy">
                                                <Sun illuminance="60000.0" azimuth="1.0" elevation="0.5"/>
                                                <Fog visualRange="400.0"/>
                                                <Precipitation precipitationType="rain" precipitationIntensity="15.0"/>
                                            </Weather>
                                        </Environment>
                                    </EnvironmentAction>
                                </GlobalAction>
                            </Action>
                            <StartTrigger>
                                <ConditionGroup>
                                    <Condition name="EgoCloseToNpcCondition" delay="0.0" conditionEdge="rising">
                                        <ByEntityCondition>
                                            <TriggeringEntities triggeringEntitiesRule="any">
                                                <EntityRef entityRef="hero"/>
                                            </TriggeringEntities>
                                            <EntityCondition>
                                                <RelativeDistanceCondition entityRef="lead_npc"
                                                                           relativeDistanceType="longitudinal"
                                                                           value="$TriggerDist"
                                                                           freespace="false"
                                                                           rule="lessThan"/>
                                            </EntityCondition>
                                        </ByEntityCondition>
                                    </Condition>
                                </ConditionGroup>
                            </StartTrigger>
                        </Event>
                    </Maneuver>
                </ManeuverGroup>
                <StartTrigger>
                    <ConditionGroup>
                        <Condition name="ActStartCondition" delay="0.0" conditionEdge="none">
                            <ByValueCondition>
                                <SimulationTimeCondition value="0.0" rule="greaterOrEqual"/>
                            </ByValueCondition>
                        </Condition>
                    </ConditionGroup>
                </StartTrigger>
            </Act>
        </Story>
        <StopTrigger>
            <ConditionGroup>
                <Condition name="ScenarioStopTime" delay="0.0" conditionEdge="rising">
                    <ByValueCondition>
                        <SimulationTimeCondition value="10.0" rule="greaterThan"/>
                    </ByValueCondition>
                </Condition>
            </ConditionGroup>
        </StopTrigger>
    </Storyboard>
</OpenSCENARIO>
"""


# ==============================================================================
# -- Mock CARLA Classes for Offline Full-Pipeline Verification -----------------
# ==============================================================================


class MockCarlaTrafficLightState:
    Red = "Red"
    Yellow = "Yellow"
    Green = "Green"
    Off = "Off"


@dataclass
class MockWeatherParameters:
    cloudiness: float = 0.0
    precipitation: float = 0.0
    precipitation_deposits: float = 0.0
    wind_intensity: float = 0.0
    sun_azimuth_angle: float = 0.0
    sun_altitude_angle: float = 45.0
    fog_density: float = 0.0
    fog_distance: float = 0.0
    wetness: float = 0.0


@dataclass
class MockWorldSettings:
    synchronous_mode: bool = False
    fixed_delta_seconds: float | None = None


@dataclass
class MockBoundingBox:
    location: FallbackLocation = field(default_factory=FallbackLocation)
    extent: FallbackVector3D = field(
        default_factory=lambda: FallbackVector3D(2.4, 1.0, 0.75)
    )


class MockBlueprint:
    def __init__(self, bp_id: str) -> None:
        self.id = bp_id
        self.attributes: dict[str, str] = {
            "role_name": "default",
            "ros_name": "",
            "ros2_autoware_control": "false",
            "color": "255,255,255",
            "is_invincible": "true",
        }

    def has_attribute(self, key: str) -> bool:
        return key in self.attributes

    def set_attribute(self, key: str, value: str) -> None:
        self.attributes[key] = str(value)


class MockBlueprintLibrary:
    def __init__(self) -> None:
        self._bps = [
            MockBlueprint("vehicle.lincoln.mkz_2020"),
            MockBlueprint("vehicle.tesla.model3"),
            MockBlueprint("vehicle.mercedes.sprinter"),
            MockBlueprint("vehicle.carlamotors.carlacola"),
            MockBlueprint("walker.pedestrian.0001"),
            MockBlueprint("static.prop.streetbarrier"),
        ]

    def find(self, bp_id: str) -> MockBlueprint | None:
        for bp in self._bps:
            if bp.id == bp_id:
                return MockBlueprint(bp.id)
        return None

    def filter(self, pattern: str) -> list[MockBlueprint]:
        import fnmatch

        return [
            MockBlueprint(bp.id)
            for bp in self._bps
            if fnmatch.fnmatch(bp.id, pattern)
        ]


class MockActor:
    def __init__(
        self,
        actor_id: int,
        type_id: str,
        transform: FallbackTransform,
        attributes: dict[str, str] | None = None,
    ) -> None:
        self.id = actor_id
        self.type_id = type_id
        self._transform = transform
        self._velocity = FallbackVector3D()
        self._angular_velocity = FallbackVector3D()
        self._acceleration = FallbackVector3D()
        self.attributes = dict(attributes or {})
        self.simulate_physics = True
        self.ros_enabled = False
        self.destroyed = False
        self.light_state = 0
        self.walker_speed = 0.0
        if type_id.startswith("walker."):
            self.bounding_box = MockBoundingBox(
                FallbackLocation(0.0, 0.0, 0.0),
                FallbackVector3D(0.3, 0.3, 0.9),
            )
        else:
            self.bounding_box = MockBoundingBox(
                FallbackLocation(0.0, 0.0, 0.75),
                FallbackVector3D(2.4, 1.0, 0.75),
            )

    def get_transform(self) -> FallbackTransform:
        return self._transform

    def set_transform(self, tf: FallbackTransform) -> None:
        self._transform = tf

    def get_velocity(self) -> FallbackVector3D:
        return self._velocity

    def set_target_velocity(self, vel: FallbackVector3D) -> None:
        self._velocity = vel

    def get_angular_velocity(self) -> FallbackVector3D:
        return self._angular_velocity

    def set_target_angular_velocity(self, ang_vel: FallbackVector3D) -> None:
        self._angular_velocity = ang_vel

    def get_acceleration(self) -> FallbackVector3D:
        return self._acceleration

    def set_simulate_physics(self, enabled: bool) -> None:
        self.simulate_physics = bool(enabled)

    def enable_for_ros(self) -> None:
        self.ros_enabled = True

    def destroy(self) -> bool:
        self.destroyed = True
        return True


class MockTrafficLight(MockActor):
    def __init__(self, actor_id: int, opendrive_id: str) -> None:
        super().__init__(actor_id, "traffic.traffic_light", FallbackTransform())
        self._opendrive_id = opendrive_id
        self.state = MockCarlaTrafficLightState.Red
        self.frozen = False

    def get_opendrive_id(self) -> str:
        return self._opendrive_id

    def set_state(self, state: Any) -> None:
        self.state = state

    def freeze(self, freeze: bool) -> None:
        self.frozen = bool(freeze)


class MockActorList(list):
    def filter(self, pattern: str) -> list[MockActor]:
        import fnmatch

        return [
            a
            for a in self
            if not a.destroyed and fnmatch.fnmatch(a.type_id, pattern)
        ]


class MockMap:
    def __init__(self, name: str = "Town01", xodr_content: str = SAMPLE_STRAIGHT_XODR) -> None:
        self.name = name
        self._xodr = xodr_content

    def to_opendrive(self) -> str:
        return self._xodr


class MockWorld:
    def __init__(self, xodr_content: str = SAMPLE_STRAIGHT_XODR) -> None:
        self._settings = MockWorldSettings()
        self._weather = MockWeatherParameters()
        self._map = MockMap("TownTest", xodr_content)
        self._bp_lib = MockBlueprintLibrary()
        self._actors = MockActorList()
        self._next_id = 100
        self._spectator = MockActor(1, "spectator", FallbackTransform())
        self.ticks = 0

        # Add a traffic light with OpenDRIVE ID "101"
        self.traffic_light_101 = MockTrafficLight(50, "101")
        self._actors.append(self.traffic_light_101)

    def get_settings(self) -> MockWorldSettings:
        return MockWorldSettings(
            synchronous_mode=self._settings.synchronous_mode,
            fixed_delta_seconds=self._settings.fixed_delta_seconds,
        )

    def apply_settings(self, settings: MockWorldSettings) -> int:
        self._settings = MockWorldSettings(
            synchronous_mode=settings.synchronous_mode,
            fixed_delta_seconds=settings.fixed_delta_seconds,
        )
        return self.ticks

    def get_weather(self) -> MockWeatherParameters:
        return MockWeatherParameters(**self._weather.__dict__)

    def set_weather(self, weather: MockWeatherParameters) -> None:
        self._weather = MockWeatherParameters(**weather.__dict__)

    def get_map(self) -> MockMap:
        return self._map

    def get_blueprint_library(self) -> MockBlueprintLibrary:
        return self._bp_lib

    def get_actors(self) -> MockActorList:
        return MockActorList([a for a in self._actors if not a.destroyed])

    def get_spectator(self) -> MockActor:
        return self._spectator

    def spawn_actor(self, bp: MockBlueprint, transform: FallbackTransform) -> MockActor:
        actor = MockActor(self._next_id, bp.id, transform, bp.attributes)
        self._next_id += 1
        self._actors.append(actor)
        return actor

    def try_spawn_actor(
        self, bp: MockBlueprint, transform: FallbackTransform
    ) -> MockActor | None:
        return self.spawn_actor(bp, transform)

    def tick(self) -> int:
        self.ticks += 1
        return self.ticks

    def wait_for_tick(self) -> int:
        self.ticks += 1
        return self.ticks


class MockCommandNamespace:
    @dataclass
    class ApplyTransform:
        actor_id: int
        transform: FallbackTransform

    @dataclass
    class ApplyTargetVelocity:
        actor_id: int
        velocity: FallbackVector3D

    @dataclass
    class ApplyTargetAngularVelocity:
        actor_id: int
        angular_velocity: FallbackVector3D

    @dataclass
    class ApplyWalkerState:
        actor_id: int
        transform: FallbackTransform
        speed: float

    @dataclass
    class SetVehicleLightState:
        actor_id: int
        light_state: int


class MockCarlaModule:
    Location = FallbackLocation
    Rotation = FallbackRotation
    Transform = FallbackTransform
    Vector3D = FallbackVector3D
    WeatherParameters = MockWeatherParameters
    TrafficLightState = MockCarlaTrafficLightState
    command = MockCommandNamespace


class MockClient:
    def __init__(self, world: MockWorld) -> None:
        self.world = world
        self.batches: list[list[Any]] = []

    def set_timeout(self, timeout: float) -> None:
        pass

    def get_world(self) -> MockWorld:
        return self.world

    def get_available_maps(self) -> list[str]:
        return ["/Game/Carla/Maps/TownTest"]

    def apply_batch(self, commands: list[Any]) -> None:
        self.batches.append(list(commands))
        by_id = {a.id: a for a in self.world.get_actors()}
        for cmd in commands:
            actor = by_id.get(getattr(cmd, "actor_id", -1))
            if actor is None:
                continue
            if isinstance(cmd, MockCommandNamespace.ApplyTransform):
                actor.set_transform(cmd.transform)
            elif isinstance(cmd, MockCommandNamespace.ApplyTargetVelocity):
                actor.set_target_velocity(cmd.velocity)
            elif isinstance(cmd, MockCommandNamespace.ApplyTargetAngularVelocity):
                actor.set_target_angular_velocity(cmd.angular_velocity)
            elif isinstance(cmd, MockCommandNamespace.ApplyWalkerState):
                actor.set_transform(cmd.transform)
                actor.walker_speed = cmd.speed
            elif isinstance(cmd, MockCommandNamespace.SetVehicleLightState):
                if not isinstance(cmd.actor_id, int):
                    raise TypeError("SetVehicleLightState actor_id must be int")
                actor.light_state = cmd.light_state


# ==============================================================================
# -- Test Cases ----------------------------------------------------------------
# ==============================================================================


class TestCoordinateConversions(unittest.TestCase):
    """Verify right-handed OpenDRIVE/esmini <-> left-handed CARLA coordinate conversions."""

    def test_pose_roundtrip_with_center_offsets(self) -> None:
        test_poses = [
            (10.0, -3.5, 0.2, 0.0, 0.0, 0.0, 1.4, 0.0),
            (125.4, 42.8, 3.1, 0.75, 0.05, -0.03, 1.35, -0.05),
            (-50.0, -120.0, -1.5, 3.8, -0.08, 0.04, 1.6, 0.1),
        ]
        for x, y, z, h, p, r, cx, cy in test_poses:
            tf = esmini_pose_to_carla_transform(
                x, y, z, h, p, r, center_offset_x=cx, center_offset_y=cy
            )
            rx, ry, rz, rh, rp, rr = carla_transform_to_esmini_pose(
                tf, center_offset_x=cx, center_offset_y=cy
            )
            self.assertAlmostEqual(x, rx, places=6)
            self.assertAlmostEqual(y, ry, places=6)
            self.assertAlmostEqual(z, rz, places=6)
            self.assertAlmostEqual(h, rh, places=6)
            self.assertAlmostEqual(p, rp, places=6)
            self.assertAlmostEqual(r, rr, places=6)

    def test_handedness_sign_conventions(self) -> None:
        # In OpenDRIVE: +Y is North (left of +X East), +heading is CCW around +Z.
        # In CARLA: +Y is South (right of +X East), +yaw is CW around +Z.
        tf = esmini_pose_to_carla_transform(
            x=20.0, y=5.0, z=1.0, h=math.pi / 4.0, p=0.1, r=-0.2
        )
        self.assertAlmostEqual(tf.location.x, 20.0, places=6)
        self.assertAlmostEqual(tf.location.y, -5.0, places=6)
        self.assertAlmostEqual(tf.location.z, 1.0, places=6)
        self.assertAlmostEqual(tf.rotation.yaw, -45.0, places=6)
        self.assertAlmostEqual(tf.rotation.pitch, -math.degrees(0.1), places=6)
        self.assertAlmostEqual(tf.rotation.roll, math.degrees(-0.2), places=6)

    def test_pedestrian_capsule_z_offset(self) -> None:
        state = SEScenarioObjectState()
        state.objectType = int(SEObjectType.PEDESTRIAN)
        state.x = 15.0
        state.y = -2.0
        state.z = 1.0
        state.height = 1.8

        tf = esmini_state_to_carla_transform(state)
        self.assertAlmostEqual(tf.location.x, 15.0, places=6)
        self.assertAlmostEqual(tf.location.y, 2.0, places=6)
        # z should be road z (1.0) + half of pedestrian height (0.9) = 1.9
        self.assertAlmostEqual(tf.location.z, 1.9, places=6)

    def test_velocity_and_angular_velocity_roundtrip(self) -> None:
        c_vel = esmini_velocity_to_carla(12.5, -3.2, 0.4)
        self.assertAlmostEqual(c_vel.x, 12.5)
        self.assertAlmostEqual(c_vel.y, 3.2)
        self.assertAlmostEqual(c_vel.z, 0.4)
        vx, vy, vz = carla_velocity_to_esmini(c_vel)
        self.assertAlmostEqual(vx, 12.5)
        self.assertAlmostEqual(vy, -3.2)
        self.assertAlmostEqual(vz, 0.4)

        c_ang = esmini_angular_velocity_to_carla(0.3, -0.1, 0.05)
        hr, pr, rr = carla_angular_velocity_to_esmini(c_ang)
        self.assertAlmostEqual(hr, 0.3, places=6)
        self.assertAlmostEqual(pr, -0.1, places=6)
        self.assertAlmostEqual(rr, 0.05, places=6)


class TestEsminiCarlaCoSimulation(unittest.TestCase):
    """End-to-end integration tests with the real libesminiLib binary and Mock CARLA world."""

    def setUp(self) -> None:
        self.temp_dir = Path(tempfile.mkdtemp(prefix="test_carla_esmini_"))
        self.xodr_path = self.temp_dir / "test_straight.xodr"
        self.xodr_path.write_text(SAMPLE_STRAIGHT_XODR, encoding="utf-8")

        self.xosc_path = self.temp_dir / "test_scenario.xosc"
        self.xosc_path.write_text(
            SAMPLE_SCENARIO_XOSC_TEMPLATE.format(xodr_filename="test_straight.xodr"),
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        import shutil

        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_esmini_library_loads_and_parses_scenario(self) -> None:
        with EsminiLib() as esmini:
            esmini.set_log_file_path("")
            esmini.log_to_console(False)
            esmini.add_path(self.temp_dir)
            rc = esmini.init(self.xosc_path)
            self.assertEqual(rc, 0)
            self.assertEqual(esmini.get_number_of_objects(), 3)

            ego_id = esmini.get_id_by_name("hero")
            npc_id = esmini.get_id_by_name("lead_npc")
            walker_id = esmini.get_id_by_name("crossing_walker")
            self.assertGreaterEqual(ego_id, 0)
            self.assertGreaterEqual(npc_id, 0)
            self.assertGreaterEqual(walker_id, 0)

            ego_state = esmini.get_object_state(ego_id)
            self.assertIsNotNone(ego_state)
            assert ego_state is not None
            self.assertAlmostEqual(ego_state.s, 10.0, places=2)
            self.assertEqual(ego_state.laneId, -1)
            self.assertEqual(ego_state.objectType, SEObjectType.VEHICLE)
            self.assertEqual(ego_state.objectCategory, SEVehicleCategory.CAR)

    def test_spawn_ego_and_external_ad_trigger_execution(self) -> None:
        """Verify that external Ego motion in CARLA triggers esmini conditions and updates NPCs/weather/signals."""
        world = MockWorld()
        client = MockClient(world)

        config = RunnerConfig(
            xosc_path=self.xosc_path,
            delta_seconds=0.05,
            time_scale=0.0,
            max_sim_time=3.0,
            spawn_ego=True,
            autoware_mode=True,
            follow_ego=True,
            disable_esmini_log=True,
            parameters={"TargetSpeed": 22.0, "TriggerDist": 30.0},
        )

        runner = CarlaEsminiRunner(
            config=config,
            carla_client=client,
            carla_world=world,
            carla_mod=MockCarlaModule,
        )

        runner.setup()
        try:
            self.assertIsNotNone(runner.actor_manager)
            assert runner.actor_manager is not None
            ego_id = runner.actor_manager.ego_esmini_id
            self.assertEqual(ego_id, 0)

            ego_info = runner.actor_manager.actors[ego_id]
            ego_actor = ego_info.carla_actor
            self.assertIsNotNone(ego_actor)

            # Verify Autoware / ROS 2 attributes were configured on the spawned Ego
            self.assertEqual(ego_actor.attributes.get("role_name"), "ego")
            self.assertEqual(ego_actor.attributes.get("ros_name"), "ego")
            self.assertEqual(
                ego_actor.attributes.get("ros2_autoware_control"), "true"
            )
            self.assertTrue(ego_actor.ros_enabled)
            self.assertTrue(ego_actor.simulate_physics)

            # Verify NPCs have physics disabled and pedestrian was spawned
            npc_info = runner.actor_manager.actors[1]
            walker_info = runner.actor_manager.actors[2]
            self.assertFalse(npc_info.carla_actor.simulate_physics)
            self.assertTrue(walker_info.carla_actor.type_id.startswith("walker."))

            # Verify Init global actions set initial weather (cloudy=50%) and TrafficLight 101 to Red
            self.assertAlmostEqual(world.get_weather().cloudiness, 50.0, places=1)
            self.assertEqual(
                world.traffic_light_101.state, MockCarlaTrafficLightState.Red
            )
            self.assertTrue(world.traffic_light_101.frozen)

            # Simulate an external AD stack driving the Ego vehicle forward at 25 m/s in CARLA
            ego_speed = 25.0
            for _ in range(45):
                tf = ego_actor.get_transform()
                tf.location.x += ego_speed * config.delta_seconds
                ego_actor.set_transform(tf)
                ego_actor.set_target_velocity(FallbackVector3D(ego_speed, 0.0, 0.0))
                if not runner.step(config.delta_seconds):
                    break

            # Because Ego drove faster (25 m/s) than lead_npc (5 m/s), the longitudinal gap
            # dropped below TriggerDist (30m) and triggered EgoCloseToNpcCondition!
            cond_names = [c[0] for c in runner.triggered_conditions]
            self.assertIn("EgoCloseToNpcCondition", cond_names)

            # Verify that the triggered event:
            # 1) Accelerated lead_npc toward TargetSpeed (22.0 m/s)
            npc_state = runner.esmini.get_object_state(1)
            self.assertIsNotNone(npc_state)
            assert npc_state is not None
            self.assertGreater(npc_state.speed, 6.0)

            # 2) Switched TrafficLight 101 to Green via TrafficSignalStateAction
            self.assertEqual(
                world.traffic_light_101.state, MockCarlaTrafficLightState.Green
            )

            # 3) Applied RainWeatherAction (precipitation = 75%)
            self.assertAlmostEqual(world.get_weather().precipitation, 75.0, places=1)
        finally:
            runner.cleanup()

        # Verify cleanup restored traffic light freeze state and destroyed spawned actors
        self.assertFalse(world.traffic_light_101.frozen)
        self.assertTrue(ego_actor.destroyed)
        self.assertTrue(npc_info.carla_actor.destroyed)
        self.assertTrue(walker_info.carla_actor.destroyed)

    def test_wait_for_external_ego_and_opendrive_export_from_carla_map(self) -> None:
        """Verify --wait-for-ego mode and automatic export of CARLA's in-memory OpenDRIVE."""
        # Create an .xosc referencing a CARLA map ('TownVirtual.xodr') that does NOT exist on disk
        virtual_xosc = self.temp_dir / "virtual_town_scenario.xosc"
        virtual_xosc.write_text(
            SAMPLE_SCENARIO_XOSC_TEMPLATE.format(xodr_filename="TownVirtual.xodr"),
            encoding="utf-8",
        )

        world = MockWorld(xodr_content=SAMPLE_STRAIGHT_XODR)
        client = MockClient(world)

        # Pre-spawn an external Ego vehicle at an arbitrary location (x=999, y=999)
        ext_bp = MockBlueprint("vehicle.tesla.model3")
        ext_bp.set_attribute("role_name", "hero")
        external_ego = world.spawn_actor(
            ext_bp,
            FallbackTransform(FallbackLocation(999.0, 999.0, 5.0), FallbackRotation()),
        )

        config = RunnerConfig(
            xosc_path=virtual_xosc,
            delta_seconds=0.05,
            time_scale=0.0,
            max_sim_time=0.5,
            spawn_ego=False,
            wait_for_ego=True,
            teleport_external_ego=True,
            disable_esmini_log=True,
        )

        with CarlaEsminiRunner(
            config=config,
            carla_client=client,
            carla_world=world,
            carla_mod=MockCarlaModule,
        ) as runner:
            # Verify runner attached to external_ego and teleported it to the .xosc initial pose (s=10.0 + centerOffsetX=1.4 -> x=11.4, y=1.75)
            self.assertAlmostEqual(
                external_ego.get_transform().location.x, 11.4, places=2
            )
            self.assertAlmostEqual(
                external_ego.get_transform().location.y, 1.75, places=2
            )
            runner.step(0.05)

        # External ego must NOT be destroyed when the scenario finishes
        self.assertFalse(external_ego.destroyed)


if __name__ == "__main__":
    unittest.main()
