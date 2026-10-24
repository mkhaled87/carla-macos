# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""CARLA esminiLib integration package for running ASAM OpenSCENARIO 1.x (.xosc) scenarios."""

from .actor_manager import ActorManager, ManagedActorInfo
from .coordinates import (
    carla_angular_velocity_to_esmini,
    carla_transform_to_esmini_pose,
    carla_velocity_to_esmini,
    esmini_angular_velocity_to_carla,
    esmini_pose_to_carla_transform,
    esmini_state_to_carla_transform,
    esmini_velocity_to_carla,
    sync_opendrive_map,
)
from .environment_sync import EnvironmentSynchronizer, TrafficSignalCommand, WeatherSpec
from .esmini_lib import (
    EsminiLib,
    SEControllerType,
    SELightMode,
    SELightState,
    SEObjectType,
    SEParameterType,
    SEPositionDiff,
    SEPositionMode,
    SEPositionModeType,
    SERelativeDistanceType,
    SERoadInfo,
    SERouteInfo,
    SEScenarioObjectState,
    SEStoryBoardElementState,
    SEStoryBoardElementType,
    SEVehicleCategory,
    SEVehicleLightType,
    SEVisibilityMask,
)
from .fetch_esmini import download_esmini, find_esmini_library, find_esmini_root
from .runner import CarlaEsminiRunner, RunnerConfig

__all__ = [
    "ActorManager",
    "CarlaEsminiRunner",
    "EnvironmentSynchronizer",
    "EsminiLib",
    "ManagedActorInfo",
    "RunnerConfig",
    "SEControllerType",
    "SELightMode",
    "SELightState",
    "SEObjectType",
    "SEParameterType",
    "SEPositionDiff",
    "SEPositionMode",
    "SEPositionModeType",
    "SERelativeDistanceType",
    "SERoadInfo",
    "SERouteInfo",
    "SEScenarioObjectState",
    "SEStoryBoardElementState",
    "SEStoryBoardElementType",
    "SEVehicleCategory",
    "SEVehicleLightType",
    "SEVisibilityMask",
    "TrafficSignalCommand",
    "WeatherSpec",
    "carla_angular_velocity_to_esmini",
    "carla_transform_to_esmini_pose",
    "carla_velocity_to_esmini",
    "download_esmini",
    "esmini_angular_velocity_to_carla",
    "esmini_pose_to_carla_transform",
    "esmini_state_to_carla_transform",
    "esmini_velocity_to_carla",
    "find_esmini_library",
    "find_esmini_root",
    "sync_opendrive_map",
]

