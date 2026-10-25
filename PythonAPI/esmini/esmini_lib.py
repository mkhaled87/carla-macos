# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Complete ctypes bindings and Pythonic wrapper for esminiLib (v3.10+)."""

from __future__ import annotations

import ctypes
from ctypes import (
    CDLL,
    CFUNCTYPE,
    POINTER,
    Structure,
    byref,
    c_bool,
    c_char_p,
    c_double,
    c_int,
    c_ubyte,
    c_uint,
    c_uint32,
    c_ulonglong,
    c_void_p,
)
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import Callable

from .fetch_esmini import find_esmini_library, find_esmini_root

c_id_t = c_uint32
SE_ID_UNDEFINED = 0xFFFFFFFF
SE_IDX_UNDEFINED = 0xFFFFFFFF
SE_PARAM_NAME_SIZE = 32


# ==============================================================================
# -- Enumerations --------------------------------------------------------------
# ==============================================================================


class SEObjectType(IntEnum):
    """Matches esmini scenarioengine::Object::Type (Entities.hpp)."""

    TYPE_NONE = 0
    VEHICLE = 1
    PEDESTRIAN = 2
    MISC_OBJECT = 3


class SEVehicleCategory(IntEnum):
    """Matches esmini scenarioengine::Vehicle::Category (Entities.hpp)."""

    CAR = 0
    VAN = 1
    TRUCK = 2
    SEMI_TRAILER = 3
    TRAILER = 4
    BUS = 5
    MOTORBIKE = 6
    BICYCLE = 7
    TRAIN = 8
    TRAM = 9


class SEPedestrianCategory(IntEnum):
    """Matches esmini scenarioengine::Pedestrian::Category (Entities.hpp)."""

    PEDESTRIAN = 0
    WHEELCHAIR = 1
    ANIMAL = 2


class SEMiscObjectCategory(IntEnum):
    """Matches esmini scenarioengine::MiscObject::Category (Entities.hpp)."""

    NONE = 0
    OBSTACLE = 1
    POLE = 2
    TREE = 3
    VEGETATION = 4
    BARRIER = 5
    BUILDING = 6
    PARKING_SPACE = 7
    PATCH = 8
    RAILING = 9
    TRAFFIC_ISLAND = 10
    CROSSWALK = 11
    STREET_LAMP = 12
    GANTRY = 13
    SOUND_BARRIER = 14
    WIND = 15
    ROAD_MARK = 16


class SEControllerType(IntEnum):
    """Matches esmini scenarioengine::Controller::Type."""

    DEFAULT_CONTROLLER = 0
    EXTERNAL = 1


class SEVisibilityMask(IntFlag):
    """Matches esmini scenarioengine::Object::Visibility bitmask."""

    NONE = 0
    GRAPHICS = 1
    TRAFFIC = 2
    SENSORS = 4
    ALL = 7


class SEPositionMode(IntFlag):
    """Bitmask modes for interpreting Z, Heading, Pitch, Roll as default/abs/rel."""

    SE_Z_SET = 1
    SE_Z_DEFAULT = 1
    SE_Z_ABS = 3
    SE_Z_REL = 7
    SE_Z_MASK = 7
    SE_H_SET = 1 << 4
    SE_H_DEFAULT = 1 << 4
    SE_H_ABS = 3 << 4
    SE_H_REL = 7 << 4
    SE_H_MASK = 7 << 4
    SE_P_SET = 1 << 8
    SE_P_DEFAULT = 1 << 8
    SE_P_ABS = 3 << 8
    SE_P_REL = 7 << 8
    SE_P_MASK = 7 << 8
    SE_R_SET = 1 << 12
    SE_R_DEFAULT = 1 << 12
    SE_R_ABS = 3 << 12
    SE_R_REL = 7 << 12
    SE_R_MASK = 7 << 12
    SE_SNAP_TO_ROUTE_SET = 1 << 16
    SE_SNAP_TO_ROUTE_DEFAULT = 1 << 16
    SE_SNAP_TO_ROUTE_OFF = 3 << 16
    SE_SNAP_TO_ROUTE_ON = 7 << 16
    SE_SNAP_TO_ROUTE_MASK = 7 << 16


class SEPositionModeType(IntEnum):
    """Matches SE_PositionModeType in esminiLib.hpp."""

    SE_SET = 1
    SE_UPDATE = 2


class SEGhostTrailReturnCode(IntEnum):
    """Matches SE_GhostTrailReturnCode in esminiLib.hpp."""

    SE_GHOST_TRAIL_OK = 0
    SE_GHOST_TRAIL_ERROR = -1
    SE_GHOST_TRAIL_NO_VERTICES = -2
    SE_GHOST_TRAIL_TIME_PRIOR = -3
    SE_GHOST_TRAIL_TIME_PAST = -4
    SE_GHOST_TRAIL_DIST_PAST = -5


class SERelativeDistanceType(IntEnum):
    """Matches SE_RelativeDistanceType in esminiLib.hpp."""

    REL_DIST_UNDEFINED = 0
    REL_DIST_LATERAL = 1
    REL_DIST_LONGITUDINAL = 2
    REL_DIST_CARTESIAN = 3
    REL_DIST_EUCLIDIAN = 4


class SEVehicleLightType(IntEnum):
    """Matches SE_VehicleLightType in esminiLib.hpp."""

    DAYTIME_RUNNING_LIGHTS = 0
    LOW_BEAM = 1
    HIGH_BEAM = 2
    FOG_LIGHTS = 3
    FOG_LIGHTS_FRONT = 4
    FOG_LIGHTS_REAR = 5
    BRAKE_LIGHTS = 6
    WARNING_LIGHTS = 7
    INDICATOR_LEFT = 8
    INDICATOR_RIGHT = 9
    REVERSING_LIGHTS = 10
    TAIL_LIGHTS = 11
    LICENSE_PLATE_ILLUMINATION = 12
    SPECIAL_PURPOSE_LIGHTS = 13


class SELightMode(IntEnum):
    """Matches light_mode values in SE_LightState."""

    OFF = 0
    ON = 1
    FLASHING = 2
    UNKNOWN = 3


class SEOSIStaticReportMode(IntEnum):
    """Matches SE_OSIStaticReportMode in esminiLib.hpp."""

    DEFAULT = 0
    API = 1
    API_AND_LOG = 2


class SEStoryBoardElementType(IntEnum):
    """Matches StoryBoardElement::ElementType in StoryBoardElement.hpp."""

    UNDEFINED_ELEMENT_TYPE = 0
    STORY_BOARD = 1
    STORY = 2
    ACT = 3
    MANEUVER_GROUP = 4
    MANEUVER = 5
    EVENT = 6
    ACTION = 7


class SEStoryBoardElementState(IntEnum):
    """Matches StoryBoardElement::State in StoryBoardElement.hpp."""

    UNDEFINED_ELEMENT_STATE = 0
    STANDBY = 1
    RUNNING = 2
    COMPLETE = 3


class SEParameterType(IntEnum):
    """Matches OSCParameterDeclarations::ParameterType."""

    PARAM_TYPE_INTEGER = 1
    PARAM_TYPE_DOUBLE = 2
    PARAM_TYPE_STRING = 3
    PARAM_TYPE_BOOL = 4


# ==============================================================================
# -- C Structures --------------------------------------------------------------
# ==============================================================================


class SEScenarioObjectState(Structure):
    """Matches SE_ScenarioObjectState in esminiLib.hpp (28 fields)."""

    _fields_ = [
        ("id", c_int),
        ("model_id", c_int),
        ("ctrl_type", c_int),
        ("timestamp", c_double),
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("h", c_double),
        ("p", c_double),
        ("r", c_double),
        ("roadId", c_id_t),
        ("junctionId", c_id_t),
        ("t", c_double),
        ("laneId", c_int),
        ("laneOffset", c_double),
        ("s", c_double),
        ("speed", c_double),
        ("centerOffsetX", c_double),
        ("centerOffsetY", c_double),
        ("centerOffsetZ", c_double),
        ("width", c_double),
        ("length", c_double),
        ("height", c_double),
        ("objectType", c_int),
        ("objectCategory", c_int),
        ("wheel_angle", c_double),
        ("wheel_rot", c_double),
        ("visibilityMask", c_int),
    ]


class SEWheelData(Structure):
    """Matches SE_WheelData in esminiLib.hpp."""

    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("h", c_double),
        ("p", c_double),
        ("wheel_radius", c_double),
        ("friction_coefficient", c_double),
        ("axle", c_int),
        ("index", c_int),
    ]


class SELightState(Structure):
    """Matches SE_LightState in esminiLib.hpp."""

    _fields_ = [
        ("light_mode", c_int),
        ("emitting", c_bool),
    ]


class SERoadInfo(Structure):
    """Matches SE_RoadInfo in esminiLib.hpp."""

    _fields_ = [
        ("global_pos_x", c_double),
        ("global_pos_y", c_double),
        ("global_pos_z", c_double),
        ("local_pos_x", c_double),
        ("local_pos_y", c_double),
        ("local_pos_z", c_double),
        ("angle", c_double),
        ("road_heading", c_double),
        ("road_pitch", c_double),
        ("road_roll", c_double),
        ("trail_heading", c_double),
        ("curvature", c_double),
        ("speed_limit", c_double),
        ("roadId", c_id_t),
        ("junctionId", c_id_t),
        ("laneId", c_int),
        ("laneOffset", c_double),
        ("s", c_double),
        ("t", c_double),
        ("road_type", c_int),
        ("road_rule", c_int),
        ("lane_type", c_int),
        ("trail_wheel_angle", c_double),
    ]


class SERouteInfo(Structure):
    """Matches SE_RouteInfo in esminiLib.hpp."""

    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("h", c_double),
        ("roadId", c_id_t),
        ("junctionId", c_id_t),
        ("laneId", c_int),
        ("osiLaneId", c_int),
        ("laneOffset", c_double),
        ("s", c_double),
        ("t", c_double),
    ]


class SELaneBoundaryId(Structure):
    """Matches SE_LaneBoundaryId in esminiLib.hpp."""

    _fields_ = [
        ("far_left_lb_id", c_id_t),
        ("left_lb_id", c_id_t),
        ("right_lb_id", c_id_t),
        ("far_right_lb_id", c_id_t),
    ]


class SEPositionDiff(Structure):
    """Matches SE_PositionDiff in esminiLib.hpp."""

    _fields_ = [
        ("ds", c_double),
        ("dt", c_double),
        ("dLaneId", c_int),
        ("dx", c_double),
        ("dy", c_double),
        ("oppositeLanes", c_bool),
    ]


class SESimpleVehicleState(Structure):
    """Matches SE_SimpleVehicleState in esminiLib.hpp."""

    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("h", c_double),
        ("p", c_double),
        ("speed", c_double),
        ("wheel_rotation", c_double),
        ("wheel_angle", c_double),
    ]


class SEParameter(Structure):
    """Matches SE_Parameter in esminiLib.hpp."""

    _fields_ = [
        ("name", c_char_p),
        ("value", c_void_p),
    ]


class SEVariable(Structure):
    """Matches SE_Variable in esminiLib.hpp."""

    _fields_ = [
        ("name", c_char_p),
        ("value", c_void_p),
    ]


class SEOverrideActionStatusGear(Structure):
    _fields_ = [
        ("active", c_bool),
        ("type", c_int),
        ("number", c_int),
        ("value_type", c_int),
    ]


class SEOverrideActionStatusBrake(Structure):
    _fields_ = [
        ("active", c_bool),
        ("type", c_int),
        ("value", c_double),
        ("maxRate", c_double),
        ("value_type", c_int),
    ]


class SEOverrideActionStatusPedals(Structure):
    _fields_ = [
        ("active", c_bool),
        ("value", c_double),
        ("maxRate", c_double),
    ]


class SEOverrideActionStatusSteering(Structure):
    _fields_ = [
        ("active", c_bool),
        ("value", c_double),
        ("maxRate", c_double),
        ("maxTorque", c_double),
    ]


class SEOverrideActionList(Structure):
    """Matches SE_OverrideActionList in esminiLib.hpp."""

    _fields_ = [
        ("throttle", SEOverrideActionStatusPedals),
        ("brake", SEOverrideActionStatusBrake),
        ("clutch", SEOverrideActionStatusPedals),
        ("parkingBrake", SEOverrideActionStatusBrake),
        ("steeringWheel", SEOverrideActionStatusSteering),
        ("gear", SEOverrideActionStatusGear),
    ]


class SERoadSign(Structure):
    """Matches SE_RoadSign in esminiLib.hpp."""

    _fields_ = [
        ("id", c_int),
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("z_offset", c_double),
        ("h", c_double),
        ("roadId", c_int),
        ("s", c_double),
        ("t", c_double),
        ("name", c_char_p),
        ("orientation", c_int),
        ("length", c_double),
        ("height", c_double),
        ("width", c_double),
    ]


class SERoadObjValidity(Structure):
    """Matches SE_RoadObjValidity in esminiLib.hpp."""

    _fields_ = [
        ("fromLane", c_int),
        ("toLane", c_int),
    ]


class SEImage(Structure):
    """Matches SE_Image in esminiLib.hpp."""

    _fields_ = [
        ("width", c_int),
        ("height", c_int),
        ("pixelSize", c_int),
        ("pixelFormat", c_int),
        ("data", POINTER(c_ubyte)),
    ]


class SECenter(Structure):
    """Matches SE_Center in esminiLib.hpp."""

    _fields_ = [
        ("x_", c_double),
        ("y_", c_double),
        ("z_", c_double),
    ]


class SEDimensions(Structure):
    """Matches SE_Dimensions in esminiLib.hpp."""

    _fields_ = [
        ("width_", c_double),
        ("length_", c_double),
        ("height_", c_double),
    ]


class SEOSCBoundingBox(Structure):
    """Matches SE_OSCBoundingBox in esminiLib.hpp."""

    _fields_ = [
        ("center_", SECenter),
        ("dimensions_", SEDimensions),
    ]


class SESpeedActionStruct(Structure):
    """Matches SE_SpeedActionStruct in esminiLib.hpp."""

    _fields_ = [
        ("id", c_int),
        ("speed", c_double),
        ("transition_shape", c_int),
        ("transition_dim", c_int),
        ("transition_value", c_double),
    ]


class SELaneChangeActionStruct(Structure):
    """Matches SE_LaneChangeActionStruct in esminiLib.hpp."""

    _fields_ = [
        ("id", c_int),
        ("mode", c_int),
        ("target", c_int),
        ("transition_shape", c_int),
        ("transition_dim", c_int),
        ("transition_value", c_double),
    ]


class SELaneOffsetActionStruct(Structure):
    """Matches SE_LaneOffsetActionStruct in esminiLib.hpp."""

    _fields_ = [
        ("id", c_int),
        ("offset", c_double),
        ("maxLateralAcc", c_double),
        ("transition_shape", c_int),
    ]


# ==============================================================================
# -- C Callback Signatures -----------------------------------------------------
# ==============================================================================

ParamDeclCallbackType = CFUNCTYPE(None, c_void_p)
ObjectCallbackType = CFUNCTYPE(None, POINTER(SEScenarioObjectState), c_void_p)
ConditionCallbackType = CFUNCTYPE(None, c_char_p, c_double)
StoryBoardElementStateChangeCallbackType = CFUNCTYPE(None, c_char_p, c_int, c_int, c_char_p)
ImageCallbackType = CFUNCTYPE(None, POINTER(SEImage), c_void_p)

# Persistent module-level callback trampolines.
# Because libesminiLib is a process-wide C++ singleton, pinning the CFUNCTYPE
# trampolines at module scope guarantees C++ never invokes a garbage-collected
# Python function pointer or a NULL pointer across multiple SE_Init / SE_Close runs.
_ACTIVE_ESMINI_INSTANCE: EsminiLib | None = None


@ParamDeclCallbackType
def _GLOBAL_PARAM_DECL_TRAMPOLINE(_user_data: c_void_p) -> None:
    inst = _ACTIVE_ESMINI_INSTANCE
    if inst is not None and inst._py_param_decl_cb is not None:
        inst._py_param_decl_cb()


@ConditionCallbackType
def _GLOBAL_CONDITION_TRAMPOLINE(name_bytes: bytes, timestamp: float) -> None:
    inst = _ACTIVE_ESMINI_INSTANCE
    if inst is not None and inst._py_condition_cb is not None:
        name = name_bytes.decode("utf-8", errors="replace") if name_bytes else ""
        inst._py_condition_cb(name, float(timestamp))


@StoryBoardElementStateChangeCallbackType
def _GLOBAL_STORYBOARD_TRAMPOLINE(
    name_b: bytes, elem_type: int, state: int, full_path_b: bytes
) -> None:
    inst = _ACTIVE_ESMINI_INSTANCE
    if inst is not None and inst._py_storyboard_cb is not None:
        name = name_b.decode("utf-8", errors="replace") if name_b else ""
        full_path = full_path_b.decode("utf-8", errors="replace") if full_path_b else ""
        try:
            t_enum = SEStoryBoardElementType(elem_type)
        except ValueError:
            t_enum = SEStoryBoardElementType.UNDEFINED_ELEMENT_TYPE
        try:
            s_enum = SEStoryBoardElementState(state)
        except ValueError:
            s_enum = SEStoryBoardElementState.UNDEFINED_ELEMENT_STATE
        inst._py_storyboard_cb(name, t_enum, s_enum, full_path)


@ObjectCallbackType
def _GLOBAL_OBJECT_TRAMPOLINE(
    state_ptr: POINTER(SEScenarioObjectState), _user_data: c_void_p
) -> None:
    inst = _ACTIVE_ESMINI_INSTANCE
    if inst is not None and state_ptr:
        contents = state_ptr.contents
        cb = inst._py_object_cbs.get(int(contents.id))
        if cb is not None:
            cb(contents)



# ==============================================================================
# -- High-Level EsminiLib Wrapper ----------------------------------------------
# ==============================================================================


class EsminiLib:
    """Pythonic ctypes wrapper around ``libesminiLib``.

    Manages library loading, function prototype declarations, C-string encoding,
    callback lifetime pinning (preventing Python garbage collection of active
    C callbacks), and clean scenario teardown.
    """

    def __init__(
        self,
        lib_path: str | Path | None = None,
        auto_download: bool = True,
    ) -> None:
        resolved_path = find_esmini_library(
            explicit_path=lib_path,
            auto_download=auto_download,
        )
        if resolved_path is None:
            raise FileNotFoundError(
                "Could not locate libesminiLib. Run `python3 -m PythonAPI.esmini.fetch_esmini` "
                "or set ESMINI_LIB_PATH / ESMINI_ROOT."
            )

        self.lib_path: Path = resolved_path.resolve()
        self.esmini_root: Path | None = find_esmini_root(self.lib_path)
        self._dll: CDLL = ctypes.CDLL(str(self.lib_path))
        self._initialized: bool = False

        # Python callables invoked by the persistent module-level C trampolines
        self._py_param_decl_cb: Callable[[], None] | None = None
        self._py_condition_cb: Callable[[str, float], None] | None = None
        self._py_storyboard_cb: (
            Callable[[str, SEStoryBoardElementType, SEStoryBoardElementState, str], None]
            | None
        ) = None
        self._py_object_cbs: dict[int, Callable[[SEScenarioObjectState], None]] = {}

        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self

        self._bind_functions()

    def _bind(self, name: str, argtypes: list, restype=c_int) -> None:
        fn = getattr(self._dll, name, None)
        if fn is not None:
            fn.argtypes = argtypes
            fn.restype = restype

    def _bind_functions(self) -> None:
        # Basic configuration & lifecycle
        self._bind("SE_AddPath", [c_char_p], c_int)
        self._bind("SE_ClearPaths", [], None)
        self._bind("SE_SetLogFilePath", [c_char_p], None)
        self._bind("SE_SetDatFilePath", [c_char_p], None)
        self._bind("SE_GetSeed", [], c_uint)
        self._bind("SE_SetSeed", [c_uint], None)
        self._bind("SE_SetOption", [c_char_p], c_int)
        self._bind("SE_UnsetOption", [c_char_p], c_int)
        self._bind("SE_SetOptionValue", [c_char_p, c_char_p], c_int)
        self._bind("SE_SetOptionPersistent", [c_char_p], c_int)
        self._bind("SE_SetOptionValuePersistent", [c_char_p, c_char_p], c_int)
        self._bind("SE_GetOptionValue", [c_char_p], c_char_p)
        self._bind("SE_GetOptionValueByEnum", [c_uint], c_char_p)
        self._bind("SE_GetOptionValuesCount", [c_char_p], c_int)
        self._bind("SE_GetOptionValueByIndex", [c_char_p, c_uint], c_char_p)
        self._bind("SE_GetOptionSet", [c_char_p], c_bool)
        self._bind("SE_SetWindowPosAndSize", [c_int, c_int, c_int, c_int], None)
        self._bind(
            "SE_RegisterParameterDeclarationCallback",
            [c_void_p, c_void_p],
            None,
        )
        self._bind("SE_SetOSITolerances", [c_double, c_double], c_int)
        self._bind("SE_SetParameterDistribution", [c_char_p], c_int)
        self._bind("SE_ResetParameterDistribution", [], None)
        self._bind("SE_GetNumberOfPermutations", [], c_uint)
        self._bind("SE_SelectPermutation", [c_uint], c_int)
        self._bind("SE_GetPermutationIndex", [], c_int)

        # Init & Step
        self._bind("SE_Init", [c_char_p, c_int, c_int, c_int, c_int], c_int)
        self._bind("SE_InitWithString", [c_char_p, c_int, c_int, c_int, c_int], c_int)
        self._bind("SE_InitWithArgs", [c_int, POINTER(c_char_p)], c_int)
        self._bind("SE_GetXMLString", [], c_char_p)
        self._bind("SE_StepDT", [c_double], c_int)
        self._bind("SE_Step", [], c_int)
        self._bind("SE_Close", [], None)
        self._bind("SE_LogToConsole", [c_bool], None)
        self._bind("SE_CollisionDetection", [c_bool], None)
        self._bind("SE_GetSimulationTime", [], c_double)
        self._bind("SE_GetSimTimeStep", [], c_double)
        self._bind("SE_GetQuitFlag", [], c_int)
        self._bind("SE_GetPauseFlag", [], c_int)
        self._bind("SE_GetODRFilename", [], c_char_p)
        self._bind("SE_GetSceneGraphFilename", [], c_char_p)

        # Parameters, Variables, Properties
        self._bind("SE_GetNumberOfParameters", [], c_int)
        self._bind("SE_GetParameterName", [c_int, POINTER(c_int)], c_char_p)
        self._bind("SE_GetNumberOfVariables", [], c_int)
        self._bind("SE_GetVariableName", [c_int, POINTER(c_int)], c_char_p)
        self._bind("SE_GetNumberOfProperties", [c_int], c_int)
        self._bind("SE_GetObjectPropertyName", [c_int, c_int], c_char_p)
        self._bind("SE_GetObjectPropertyValue", [c_int, c_char_p], c_char_p)
        self._bind("SE_SetParameter", [SEParameter], c_int)
        self._bind("SE_GetParameter", [POINTER(SEParameter)], c_int)
        self._bind("SE_GetParameterInt", [c_char_p, POINTER(c_int)], c_int)
        self._bind("SE_GetParameterDouble", [c_char_p, POINTER(c_double)], c_int)
        self._bind("SE_GetParameterString", [c_char_p, POINTER(c_char_p)], c_int)
        self._bind("SE_GetParameterBool", [c_char_p, POINTER(c_bool)], c_int)
        self._bind("SE_SetParameterInt", [c_char_p, c_int], c_int)
        self._bind("SE_SetParameterDouble", [c_char_p, c_double], c_int)
        self._bind("SE_SetParameterString", [c_char_p, c_char_p], c_int)
        self._bind("SE_SetParameterBool", [c_char_p, c_bool], c_int)
        self._bind("SE_SetVariable", [SEVariable], c_int)
        self._bind("SE_GetVariable", [POINTER(SEVariable)], c_int)
        self._bind("SE_GetVariableInt", [c_char_p, POINTER(c_int)], c_int)
        self._bind("SE_GetVariableDouble", [c_char_p, POINTER(c_double)], c_int)
        self._bind("SE_GetVariableString", [c_char_p, POINTER(c_char_p)], c_int)
        self._bind("SE_GetVariableBool", [c_char_p, POINTER(c_bool)], c_int)
        self._bind("SE_SetVariableInt", [c_char_p, c_int], c_int)
        self._bind("SE_SetVariableDouble", [c_char_p, c_double], c_int)
        self._bind("SE_SetVariableString", [c_char_p, c_char_p], c_int)
        self._bind("SE_SetVariableBool", [c_char_p, c_bool], c_int)

        # Object management & state reporting
        self._bind("SE_SetObjectPositionMode", [c_int, c_int, c_int], None)
        self._bind("SE_SetObjectPositionModeDefault", [c_int, c_int], None)
        self._bind(
            "SE_AddObject",
            [c_char_p, c_int, c_int, c_int, c_int, c_char_p],
            c_int,
        )
        self._bind(
            "SE_AddObjectWithBoundingBox",
            [c_char_p, c_int, c_int, c_int, c_int, c_char_p, SEOSCBoundingBox, c_int],
            c_int,
        )
        self._bind("SE_DeleteObject", [c_int], c_int)
        self._bind(
            "SE_ReportObjectPos",
            [c_int, c_double, c_double, c_double, c_double, c_double, c_double],
            c_int,
        )
        self._bind(
            "SE_ReportObjectPosMode",
            [c_int, c_double, c_double, c_double, c_double, c_double, c_double, c_int],
            c_int,
        )
        self._bind("SE_ReportObjectPosXYH", [c_int, c_double, c_double, c_double], c_int)
        self._bind(
            "SE_ReportObjectRoadPos",
            [c_int, c_id_t, c_int, c_double, c_double],
            c_int,
        )
        self._bind("SE_ReportObjectSpeed", [c_int, c_double], c_int)
        self._bind("SE_ReportObjectLateralPosition", [c_int, c_double], c_int)
        self._bind("SE_ReportObjectLateralLanePosition", [c_int, c_int, c_double], c_int)
        self._bind("SE_ReportObjectVel", [c_int, c_double, c_double, c_double], c_int)
        self._bind(
            "SE_ReportObjectAngularVel",
            [c_int, c_double, c_double, c_double],
            c_int,
        )
        self._bind("SE_ReportObjectAcc", [c_int, c_double, c_double, c_double], c_int)
        self._bind(
            "SE_ReportObjectAngularAcc",
            [c_int, c_double, c_double, c_double],
            c_int,
        )
        self._bind("SE_ReportObjectWheelStatus", [c_int, c_double, c_double], c_int)
        self._bind("SE_SetSnapLaneTypes", [c_int, c_int], c_int)
        self._bind("SE_SetLockOnLane", [c_int, c_bool], c_int)

        # Object queries
        self._bind("SE_GetNumberOfObjects", [], c_int)
        self._bind("SE_GetId", [c_int], c_int)
        self._bind("SE_GetIdByName", [c_char_p], c_int)
        self._bind("SE_GetObjectState", [c_int, POINTER(SEScenarioObjectState)], c_int)
        self._bind("SE_GetGeoPos", [c_int, POINTER(c_double), POINTER(c_double)], c_int)
        self._bind("SE_GetObjectLightState", [c_int, c_int, POINTER(SELightState)], c_int)
        self._bind("SE_GetObjectRouteStatus", [c_int], c_int)
        self._bind("SE_GetObjectInLaneType", [c_int], c_int)
        self._bind("SE_ObjectCanChangeLanes", [c_int], c_int)
        self._bind("SE_GetOverrideActionStatus", [c_int, POINTER(SEOverrideActionList)], c_int)
        self._bind("SE_GetObjectTypeName", [c_int], c_char_p)
        self._bind("SE_GetObjectName", [c_int], c_char_p)
        self._bind("SE_GetObjectModelFileName", [c_int], c_char_p)
        self._bind("SE_ObjectHasGhost", [c_int], c_int)
        self._bind("SE_GetObjectGhostId", [c_int], c_int)
        self._bind("SE_GetObjectGhostState", [c_int, POINTER(SEScenarioObjectState)], c_int)
        self._bind("SE_GetObjectNumberOfCollisions", [c_int], c_int)
        self._bind("SE_GetObjectCollision", [c_int, c_int], c_int)
        self._bind("SE_GetObjectOdometer", [c_int], c_double)
        self._bind(
            "SE_GetObjectVelocityGlobalXYZ",
            [c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind(
            "SE_GetObjectAngularVelocity",
            [c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind(
            "SE_GetObjectAngularAcceleration",
            [c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind("SE_GetObjectAcceleration", [c_int], c_double)
        self._bind(
            "SE_GetObjectAccelerationGlobalXYZ",
            [c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind(
            "SE_GetObjectAccelerationLocalLatLong",
            [c_int, POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind("SE_GetObjectNumberOfWheels", [c_int], c_int)
        self._bind("SE_GetObjectWheelData", [c_int, c_int, POINTER(SEWheelData)], c_int)

        # Road & distance queries
        self._bind("SE_GetSpeedUnit", [], c_int)
        self._bind(
            "SE_GetRoadInfoAtDistance",
            [c_int, c_double, POINTER(SERoadInfo), c_int, c_bool],
            c_int,
        )
        self._bind(
            "SE_GetRoadInfoAlongRoute",
            [c_int, c_double, POINTER(SERoadInfo), c_int, c_bool],
            c_int,
        )
        self._bind(
            "SE_GetRoadInfoAlongGhostTrail",
            [c_int, c_double, POINTER(SERoadInfo), POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind(
            "SE_GetRoadInfoGhostTrailTime",
            [c_int, c_double, POINTER(SERoadInfo), POINTER(c_double)],
            c_int,
        )
        self._bind(
            "SE_GetDistanceToObject",
            [c_int, c_int, c_bool, POINTER(SEPositionDiff)],
            c_int,
        )
        self._bind(
            "SE_SimpleGetDistanceToObject",
            [c_int, c_int, c_int, c_double, POINTER(c_double), POINTER(c_double)],
            c_int,
        )
        self._bind(
            "SE_GetTimeToCollision",
            [c_int, c_int, c_int, c_int, c_bool, POINTER(c_double)],
            c_int,
        )

        # Callbacks
        self._bind("SE_RegisterObjectCallback", [c_int, c_void_p, c_void_p], None)
        self._bind("SE_RegisterConditionCallback", [c_void_p], None)
        self._bind("SE_RegisterStoryBoardElementStateChangeCallback", [c_void_p], None)

        # Road signs & IDs
        self._bind("SE_GetNumberOfRoadSigns", [c_id_t], c_uint)
        self._bind("SE_GetRoadSign", [c_id_t, c_uint, POINTER(SERoadSign)], c_int)
        self._bind("SE_GetRoadIdString", [c_id_t], c_char_p)
        self._bind("SE_GetRoadIdFromString", [c_char_p], c_id_t)
        self._bind("SE_GetJunctionIdString", [c_id_t], c_char_p)
        self._bind("SE_GetJunctionIdFromString", [c_char_p], c_id_t)

        # Routes & Action Injection
        self._bind("SE_GetNumberOfRoutePoints", [c_int], c_int)
        self._bind("SE_GetRoutePoint", [c_int, c_uint, POINTER(SERouteInfo)], c_int)
        self._bind("SE_GetRouteTotalLength", [c_int], c_double)
        self._bind("SE_InjectSpeedAction", [POINTER(SESpeedActionStruct)], None)
        self._bind("SE_InjectLaneChangeAction", [POINTER(SELaneChangeActionStruct)], None)
        self._bind("SE_InjectLaneOffsetAction", [POINTER(SELaneOffsetActionStruct)], None)
        self._bind("SE_InjectedActionOngoing", [c_int], c_bool)

    # ------------------------------------------------------------------
    # Search paths, options & logging
    # ------------------------------------------------------------------

    def add_path(self, path: str | Path) -> int:
        return int(self._dll.SE_AddPath(str(path).encode("utf-8")))

    def clear_paths(self) -> None:
        self._dll.SE_ClearPaths()

    def set_log_file_path(self, path: str | Path) -> None:
        self._dll.SE_SetLogFilePath(str(path).encode("utf-8"))

    def set_dat_file_path(self, path: str | Path) -> None:
        self._dll.SE_SetDatFilePath(str(path).encode("utf-8"))

    def log_to_console(self, enabled: bool) -> None:
        self._dll.SE_LogToConsole(bool(enabled))

    def set_seed(self, seed: int) -> None:
        self._dll.SE_SetSeed(int(seed))

    def get_seed(self) -> int:
        return int(self._dll.SE_GetSeed())

    def set_option(self, name: str) -> int:
        return int(self._dll.SE_SetOption(name.encode("utf-8")))

    def unset_option(self, name: str) -> int:
        return int(self._dll.SE_UnsetOption(name.encode("utf-8")))

    def set_option_value(self, name: str, value: str) -> int:
        return int(self._dll.SE_SetOptionValue(name.encode("utf-8"), str(value).encode("utf-8")))

    def set_option_persistent(self, name: str) -> int:
        return int(self._dll.SE_SetOptionPersistent(name.encode("utf-8")))

    def set_option_value_persistent(self, name: str, value: str) -> int:
        return int(
            self._dll.SE_SetOptionValuePersistent(
                name.encode("utf-8"), str(value).encode("utf-8")
            )
        )

    def get_option_value(self, name: str) -> str | None:
        res = self._dll.SE_GetOptionValue(name.encode("utf-8"))
        return res.decode("utf-8") if res else None

    def collision_detection(self, enabled: bool) -> None:
        self._dll.SE_CollisionDetection(bool(enabled))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def register_parameter_declaration_callback(
        self, callback: Callable[[], None] | None
    ) -> None:
        """Register a callback invoked after ParameterDeclarations are parsed and before Init."""
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        self._py_param_decl_cb = callback
        if callback is None:
            self._dll.SE_RegisterParameterDeclarationCallback(None, None)
        else:
            self._dll.SE_RegisterParameterDeclarationCallback(
                ctypes.cast(_GLOBAL_PARAM_DECL_TRAMPOLINE, c_void_p), None
            )

    def register_object_callback(
        self,
        object_id: int,
        callback: Callable[[SEScenarioObjectState], None] | None,
    ) -> None:
        """Register a callback invoked for ``object_id`` inside each step after default controllers."""
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        if callback is None:
            self._py_object_cbs.pop(int(object_id), None)
            return

        self._py_object_cbs[int(object_id)] = callback
        self._dll.SE_RegisterObjectCallback(
            int(object_id), ctypes.cast(_GLOBAL_OBJECT_TRAMPOLINE, c_void_p), None
        )

    def register_condition_callback(
        self, callback: Callable[[str, float], None] | None
    ) -> None:
        """Register a callback ``(condition_name, timestamp)`` invoked whenever a condition triggers."""
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        self._py_condition_cb = callback
        if callback is not None:
            self._dll.SE_RegisterConditionCallback(
                ctypes.cast(_GLOBAL_CONDITION_TRAMPOLINE, c_void_p)
            )

    def register_storyboard_element_state_change_callback(
        self,
        callback: Callable[[str, SEStoryBoardElementType, SEStoryBoardElementState, str], None]
        | None,
    ) -> None:
        """Register a callback ``(name, element_type, state, full_path)`` for Storyboard transitions."""
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        self._py_storyboard_cb = callback
        if callback is not None:
            self._dll.SE_RegisterStoryBoardElementStateChangeCallback(
                ctypes.cast(_GLOBAL_STORYBOARD_TRAMPOLINE, c_void_p)
            )

    # ------------------------------------------------------------------
    # Initialization, Stepping & Shutdown
    # ------------------------------------------------------------------

    def init(
        self,
        osc_filename: str | Path,
        disable_ctrls: bool = False,
        use_viewer: int = 0,
        threads: int = 0,
        record: bool = False,
    ) -> int:
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        res = int(
            self._dll.SE_Init(
                str(osc_filename).encode("utf-8"),
                int(disable_ctrls),
                int(use_viewer),
                int(threads),
                int(record),
            )
        )
        self._initialized = res == 0
        return res

    def init_with_string(
        self,
        osc_xml: str,
        disable_ctrls: bool = False,
        use_viewer: int = 0,
        threads: int = 0,
        record: bool = False,
    ) -> int:
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        res = int(
            self._dll.SE_InitWithString(
                osc_xml.encode("utf-8"),
                int(disable_ctrls),
                int(use_viewer),
                int(threads),
                int(record),
            )
        )
        self._initialized = res == 0
        return res

    def init_with_args(self, args: list[str]) -> int:
        global _ACTIVE_ESMINI_INSTANCE
        _ACTIVE_ESMINI_INSTANCE = self
        argv_list = ["esmini", *args]
        c_argv = (c_char_p * len(argv_list))(*[a.encode("utf-8") for a in argv_list])
        res = int(self._dll.SE_InitWithArgs(len(argv_list), c_argv))
        self._initialized = res == 0
        return res

    def step_dt(self, dt: float) -> int:
        return int(self._dll.SE_StepDT(float(dt)))

    def step(self) -> int:
        return int(self._dll.SE_Step())

    def close(self) -> None:
        self._py_param_decl_cb = None
        self._py_condition_cb = None
        self._py_storyboard_cb = None
        self._py_object_cbs.clear()
        self._dll.SE_RegisterParameterDeclarationCallback(None, None)
        if self._initialized:
            self._dll.SE_Close()
            self._initialized = False

    def __enter__(self) -> EsminiLib:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Simulation Status & Metadata
    # ------------------------------------------------------------------

    def get_simulation_time(self) -> float:
        return float(self._dll.SE_GetSimulationTime())

    def get_sim_time_step(self) -> float:
        return float(self._dll.SE_GetSimTimeStep())

    def get_quit_flag(self) -> int:
        return int(self._dll.SE_GetQuitFlag())

    def get_pause_flag(self) -> int:
        return int(self._dll.SE_GetPauseFlag())

    def get_odr_filename(self) -> str:
        res = self._dll.SE_GetODRFilename()
        return res.decode("utf-8", errors="replace") if res else ""

    def get_scene_graph_filename(self) -> str:
        res = self._dll.SE_GetSceneGraphFilename()
        return res.decode("utf-8", errors="replace") if res else ""

    def get_xml_string(self) -> str:
        res = self._dll.SE_GetXMLString()
        return res.decode("utf-8", errors="replace") if res else ""

    # ------------------------------------------------------------------
    # Object Queries
    # ------------------------------------------------------------------

    def get_number_of_objects(self) -> int:
        return int(self._dll.SE_GetNumberOfObjects())

    def get_id(self, index: int) -> int:
        return int(self._dll.SE_GetId(int(index)))

    def get_id_by_name(self, name: str) -> int:
        return int(self._dll.SE_GetIdByName(name.encode("utf-8")))

    def get_object_ids(self) -> list[int]:
        count = self.get_number_of_objects()
        if count <= 0:
            return []
        return [self.get_id(i) for i in range(count)]

    def get_object_state(self, object_id: int) -> SEScenarioObjectState | None:
        state = SEScenarioObjectState()
        res = int(self._dll.SE_GetObjectState(int(object_id), byref(state)))
        if res != 0:
            return None
        return state

    def get_all_object_states(self) -> list[SEScenarioObjectState]:
        states: list[SEScenarioObjectState] = []
        for obj_id in self.get_object_ids():
            st = self.get_object_state(obj_id)
            if st is not None:
                states.append(st)
        return states

    def get_object_name(self, object_id: int) -> str:
        res = self._dll.SE_GetObjectName(int(object_id))
        return res.decode("utf-8", errors="replace") if res else ""

    def get_object_type_name(self, object_id: int) -> str:
        res = self._dll.SE_GetObjectTypeName(int(object_id))
        return res.decode("utf-8", errors="replace") if res else ""

    def get_object_model_filename(self, object_id: int) -> str:
        res = self._dll.SE_GetObjectModelFileName(int(object_id))
        return res.decode("utf-8", errors="replace") if res else ""

    def get_object_properties(self, index: int) -> dict[str, str]:
        """Return all ``<Properties><Property name="..." value="..."/>`` for object at ``index``."""
        count = int(self._dll.SE_GetNumberOfProperties(int(index)))
        if count <= 0:
            return {}
        props: dict[str, str] = {}
        for i in range(count):
            name_b = self._dll.SE_GetObjectPropertyName(int(index), i)
            if not name_b:
                continue
            val_b = self._dll.SE_GetObjectPropertyValue(int(index), name_b)
            name = name_b.decode("utf-8", errors="replace")
            val = val_b.decode("utf-8", errors="replace") if val_b else ""
            props[name] = val
        return props

    def get_object_property_value(self, index: int, prop_name: str) -> str:
        res = self._dll.SE_GetObjectPropertyValue(int(index), prop_name.encode("utf-8"))
        return res.decode("utf-8", errors="replace") if res else ""

    def get_object_light_state(
        self, object_id: int, light_type: SEVehicleLightType | int
    ) -> SELightState | None:
        ls = SELightState()
        res = int(
            self._dll.SE_GetObjectLightState(
                int(object_id), int(light_type), byref(ls)
            )
        )
        if res != 0:
            return None
        return ls

    def get_object_velocity_global_xyz(self, object_id: int) -> tuple[float, float, float] | None:
        vx, vy, vz = c_double(), c_double(), c_double()
        res = int(
            self._dll.SE_GetObjectVelocityGlobalXYZ(
                int(object_id), byref(vx), byref(vy), byref(vz)
            )
        )
        if res != 0:
            return None
        return (vx.value, vy.value, vz.value)

    def get_object_angular_velocity(self, object_id: int) -> tuple[float, float, float] | None:
        hr, pr, rr = c_double(), c_double(), c_double()
        res = int(
            self._dll.SE_GetObjectAngularVelocity(
                int(object_id), byref(hr), byref(pr), byref(rr)
            )
        )
        if res != 0:
            return None
        return (hr.value, pr.value, rr.value)

    def get_object_acceleration_global_xyz(
        self, object_id: int
    ) -> tuple[float, float, float] | None:
        ax, ay, az = c_double(), c_double(), c_double()
        res = int(
            self._dll.SE_GetObjectAccelerationGlobalXYZ(
                int(object_id), byref(ax), byref(ay), byref(az)
            )
        )
        if res != 0:
            return None
        return (ax.value, ay.value, az.value)

    def object_has_ghost(self, object_id: int) -> bool:
        return int(self._dll.SE_ObjectHasGhost(int(object_id))) == 1

    def get_object_ghost_id(self, object_id: int) -> int:
        return int(self._dll.SE_GetObjectGhostId(int(object_id)))

    def get_object_ghost_state(self, object_id: int) -> SEScenarioObjectState | None:
        state = SEScenarioObjectState()
        res = int(self._dll.SE_GetObjectGhostState(int(object_id), byref(state)))
        if res != 0:
            return None
        return state

    def get_object_collisions(self, object_id: int) -> list[int]:
        count = int(self._dll.SE_GetObjectNumberOfCollisions(int(object_id)))
        if count <= 0:
            return []
        return [
            int(self._dll.SE_GetObjectCollision(int(object_id), i))
            for i in range(count)
        ]

    # ------------------------------------------------------------------
    # Object State Reporting (External Ego / External Entities)
    # ------------------------------------------------------------------

    def report_object_pos(
        self,
        object_id: int,
        x: float,
        y: float,
        z: float,
        h: float,
        p: float,
        r: float,
    ) -> int:
        return int(
            self._dll.SE_ReportObjectPos(
                int(object_id),
                float(x),
                float(y),
                float(z),
                float(h),
                float(p),
                float(r),
            )
        )

    def report_object_pos_mode(
        self,
        object_id: int,
        x: float,
        y: float,
        z: float,
        h: float,
        p: float,
        r: float,
        mode: int,
    ) -> int:
        return int(
            self._dll.SE_ReportObjectPosMode(
                int(object_id),
                float(x),
                float(y),
                float(z),
                float(h),
                float(p),
                float(r),
                int(mode),
            )
        )

    def report_object_pos_xyh(
        self, object_id: int, x: float, y: float, h: float
    ) -> int:
        return int(
            self._dll.SE_ReportObjectPosXYH(
                int(object_id), float(x), float(y), float(h)
            )
        )

    def report_object_road_pos(
        self,
        object_id: int,
        road_id: int,
        lane_id: int,
        lane_offset: float,
        s: float,
    ) -> int:
        return int(
            self._dll.SE_ReportObjectRoadPos(
                int(object_id),
                int(road_id),
                int(lane_id),
                float(lane_offset),
                float(s),
            )
        )

    def report_object_speed(self, object_id: int, speed: float) -> int:
        return int(self._dll.SE_ReportObjectSpeed(int(object_id), float(speed)))

    def report_object_vel(
        self, object_id: int, vx: float, vy: float, vz: float
    ) -> int:
        return int(
            self._dll.SE_ReportObjectVel(
                int(object_id), float(vx), float(vy), float(vz)
            )
        )

    def report_object_angular_vel(
        self, object_id: int, h_rate: float, p_rate: float, r_rate: float
    ) -> int:
        return int(
            self._dll.SE_ReportObjectAngularVel(
                int(object_id), float(h_rate), float(p_rate), float(r_rate)
            )
        )

    def report_object_acc(
        self, object_id: int, ax: float, ay: float, az: float
    ) -> int:
        return int(
            self._dll.SE_ReportObjectAcc(
                int(object_id), float(ax), float(ay), float(az)
            )
        )

    def report_object_angular_acc(
        self, object_id: int, h_acc: float, p_acc: float, r_acc: float
    ) -> int:
        return int(
            self._dll.SE_ReportObjectAngularAcc(
                int(object_id), float(h_acc), float(p_acc), float(r_acc)
            )
        )

    def report_object_wheel_status(
        self, object_id: int, rotation: float, angle: float
    ) -> int:
        return int(
            self._dll.SE_ReportObjectWheelStatus(
                int(object_id), float(rotation), float(angle)
            )
        )

    def set_object_position_mode(
        self, object_id: int, mode_type: SEPositionModeType | int, mode: int
    ) -> None:
        self._dll.SE_SetObjectPositionMode(int(object_id), int(mode_type), int(mode))

    def set_object_position_mode_default(
        self, object_id: int, mode_type: SEPositionModeType | int
    ) -> None:
        self._dll.SE_SetObjectPositionModeDefault(int(object_id), int(mode_type))

    # ------------------------------------------------------------------
    # Parameters & Variables
    # ------------------------------------------------------------------

    def get_parameter_names(self) -> list[tuple[str, SEParameterType]]:
        count = int(self._dll.SE_GetNumberOfParameters())
        if count <= 0:
            return []
        result: list[tuple[str, SEParameterType]] = []
        for i in range(count):
            ptype = c_int()
            name_b = self._dll.SE_GetParameterName(i, byref(ptype))
            if name_b:
                name = name_b.decode("utf-8", errors="replace")
                result.append((name, SEParameterType(ptype.value)))
        return result

    def set_parameter_auto(self, name: str, value: str | int | float | bool) -> int:
        """Set a scenario parameter by automatically matching its declared OpenSCENARIO type."""
        name_b = name.encode("utf-8")
        param_types = dict(self.get_parameter_names())
        declared_type = param_types.get(name)

        if declared_type == SEParameterType.PARAM_TYPE_INTEGER:
            return int(self._dll.SE_SetParameterInt(name_b, int(value)))
        if declared_type == SEParameterType.PARAM_TYPE_DOUBLE:
            return int(self._dll.SE_SetParameterDouble(name_b, float(value)))
        if declared_type == SEParameterType.PARAM_TYPE_BOOL:
            if isinstance(value, str):
                bval = value.strip().lower() in ("true", "1", "yes")
            else:
                bval = bool(value)
            return int(self._dll.SE_SetParameterBool(name_b, bval))
        if declared_type == SEParameterType.PARAM_TYPE_STRING:
            return int(self._dll.SE_SetParameterString(name_b, str(value).encode("utf-8")))

        # Fallback: try double -> int -> bool -> string
        if isinstance(value, bool):
            return int(self._dll.SE_SetParameterBool(name_b, value))
        if isinstance(value, int):
            return int(self._dll.SE_SetParameterInt(name_b, value))
        if isinstance(value, float):
            return int(self._dll.SE_SetParameterDouble(name_b, value))

        sval = str(value)
        if sval.lower() in ("true", "false"):
            if self._dll.SE_SetParameterBool(name_b, sval.lower() == "true") == 0:
                return 0
        try:
            ival = int(sval)
            if self._dll.SE_SetParameterInt(name_b, ival) == 0:
                return 0
        except ValueError:
            pass
        try:
            fval = float(sval)
            if self._dll.SE_SetParameterDouble(name_b, fval) == 0:
                return 0
        except ValueError:
            pass
        return int(self._dll.SE_SetParameterString(name_b, sval.encode("utf-8")))
