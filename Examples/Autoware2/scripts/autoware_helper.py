#!/usr/bin/env python3
"""Container-side ROS 2 helper for CARLA <-> Autoware 2.1.0 closed-loop workflows.

Subcommands:
  check-connection   Verify live native ROS 2 topics streamed from CARLA on macOS host.
  init-localization  Wait for map/sensors and initialize GNSS + NDT localization via ADAPI.
  drive              Set downstream route goal and engage Autoware Autonomous mode via ADAPI.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
    RouteState,
)
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
    SetRoutePoints,
)
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, PointCloud2
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport


BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

TRANSIENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def spin_until(node: Node, predicate, timeout_sec: float, step_sec: float = 0.05) -> bool:
    """Spin node in small increments until predicate() is True or timeout expires."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=step_sec)
        if predicate():
            return True
    return predicate()


def call_service(node: Node, client, request, timeout_sec: float = 15.0):
    """Wait for service availability and synchronously await one response."""
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service {client.srv_name} not available within {timeout_sec}s")
    future = client.call_async(request)
    if not spin_until(node, future.done, timeout_sec=timeout_sec):
        raise RuntimeError(f"Service {client.srv_name} call timed out after {timeout_sec}s")
    return future.result()


def cmd_check_connection(args: argparse.Namespace) -> int:
    """Verify CARLA -> Autoware DDS topic delivery across all core sensor streams."""
    rclpy.init()
    node = rclpy.create_node("carla_connection_check")
    try:
        topics = [
            ("/clock", Clock, True),
            ("/sensing/lidar/top/pointcloud_before_sync", PointCloud2, True),
            ("/sensing/imu/tamagawa/imu_raw", Imu, True),
            ("/sensing/gnss/pose_with_covariance", PoseWithCovarianceStamped, True),
            ("/sensing/camera/CAM_FRONT/image", Image, False),
            ("/vehicle/status/velocity_status", VelocityReport, True),
            ("/vehicle/status/steering_status", SteeringReport, True),
        ]
        counts: dict[str, int] = {topic: 0 for topic, _, _ in topics}
        details: dict[str, str] = {}

        def make_cb(topic_name: str):
            def _cb(msg) -> None:
                counts[topic_name] += 1
                if topic_name in details:
                    return
                if isinstance(msg, Clock):
                    details[topic_name] = f"t={msg.clock.sec}.{msg.clock.nanosec:09d}"
                elif isinstance(msg, PointCloud2):
                    pts = msg.width * msg.height
                    details[topic_name] = f"frame='{msg.header.frame_id}', points={pts}"
                elif isinstance(msg, PoseWithCovarianceStamped):
                    p = msg.pose.pose.position
                    details[topic_name] = (
                        f"frame='{msg.header.frame_id}', pos=({p.x:.1f}, {p.y:.1f}, {p.z:.1f})"
                    )
                elif isinstance(msg, Image):
                    details[topic_name] = (
                        f"frame='{msg.header.frame_id}', {msg.width}x{msg.height}"
                    )
                elif isinstance(msg, Imu):
                    details[topic_name] = f"frame='{msg.header.frame_id}'"
                elif isinstance(msg, VelocityReport):
                    details[topic_name] = f"vx={msg.longitudinal_velocity:.2f} m/s"
                elif isinstance(msg, SteeringReport):
                    details[topic_name] = f"steer={msg.steering_tire_angle:.3f} rad"
            return _cb

        subs = [
            node.create_subscription(msg_type, topic, make_cb(topic), BEST_EFFORT_QOS)
            for topic, msg_type, _ in topics
        ]
        _ = subs

        required = [topic for topic, _, is_req in topics if is_req]
        spin_until(
            node,
            lambda: all(counts[t] >= 2 for t in counts),
            timeout_sec=args.timeout,
        )

        print("\nCARLA -> Autoware Native ROS 2 Topic Check:")
        missing_required = []
        for topic, _, is_req in topics:
            cnt = counts[topic]
            status = "OK" if cnt > 0 else ("MISSING" if is_req else "OPTIONAL (0 msgs)")
            info = details.get(topic, "-")
            print(f"  [{status:<7}] {topic:<46} msgs={cnt:<4} ({info})")
            if is_req and cnt == 0:
                missing_required.append(topic)

        if missing_required:
            print(
                "\n[FAIL] Required topics did not arrive from CARLA:",
                ", ".join(missing_required),
                file=sys.stderr,
            )
            return 1
        print("\n[OK] Bidirectional DDS bridge verified; all required CARLA streams active.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _dist_2d(p1: object, p2: object) -> float:
    dx = p1.x - p2.x  # type: ignore[attr-defined]
    dy = p1.y - p2.y  # type: ignore[attr-defined]
    return (dx * dx + dy * dy) ** 0.5


def _ensure_localized(node, timeout_sec: float = 120.0, retries: int = 10):
    """Ensure Autoware localization is initialized and aligned within 3 m of GNSS."""
    holder: dict[str, object] = {"loc_state": None, "odom": None, "gnss": None}

    sub_loc = node.create_subscription(
        LocalizationInitializationState,
        "/api/localization/initialization_state",
        lambda m: holder.__setitem__("loc_state", m.state),
        TRANSIENT_QOS,
    )
    sub_odom = node.create_subscription(
        Odometry,
        "/localization/kinematic_state",
        lambda m: holder.__setitem__("odom", m),
        RELIABLE_QOS,
    )
    sub_gnss = node.create_subscription(
        PoseWithCovarianceStamped,
        "/sensing/gnss/pose_with_covariance",
        lambda m: holder.__setitem__("gnss", m),
        RELIABLE_QOS,
    )

    def _aligned() -> bool:
        return (
            holder["loc_state"] == LocalizationInitializationState.INITIALIZED
            and holder["odom"] is not None
            and holder["gnss"] is not None
            and _dist_2d(
                holder["odom"].pose.pose.position,  # type: ignore[union-attr]
                holder["gnss"].pose.pose.position,  # type: ignore[union-attr]
            )
            < 3.0
        )

    try:
        client = node.create_client(InitializeLocalization, "/api/localization/initialize")
        print("[init-localization] Waiting for /api/localization/initialize & GNSS...")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            print("[FAIL] Localization initialize service timed out.", file=sys.stderr)
            return None
        if not spin_until(node, lambda: holder["gnss"] is not None, timeout_sec=30.0):
            print("[FAIL] No GNSS pose received from CARLA.", file=sys.stderr)
            return None

        # Allow transient initialization_state and latest kinematic_state to arrive before deciding.
        spin_until(
            node,
            lambda: holder["loc_state"] is not None and holder["odom"] is not None,
            timeout_sec=2.0,
        )
        if _aligned():
            return holder["odom"]

        if holder["loc_state"] == LocalizationInitializationState.INITIALIZED:
            # Ego respawned/teleported since last run; clear stale route before re-initializing.
            clear_cli = node.create_client(ClearRoute, "/api/routing/clear_route")
            if clear_cli.wait_for_service(timeout_sec=1.0):
                call_service(node, clear_cli, ClearRoute.Request(), timeout_sec=10.0)
            print("[init-localization] Ego pose shifted >3 m from EKF; re-initializing to GNSS...")

        for attempt in range(1, retries + 1):
            if _aligned():
                break
            holder["odom"] = None
            req = InitializeLocalization.Request()
            if holder["loc_state"] == LocalizationInitializationState.INITIALIZED:
                req.pose = [holder["gnss"]]  # type: ignore[list-item]
            res = call_service(node, client, req, timeout_sec=30.0)
            if res.status.success:
                break
            print(f"[init-localization] Attempt {attempt}/{retries}: {res.status.message}")
            time.sleep(2.0)

        if not spin_until(node, _aligned, timeout_sec=30.0):
            print(
                f"[FAIL] Localization state={holder['loc_state']} did not converge near GNSS.",
                file=sys.stderr,
            )
            return None
        return holder["odom"]
    finally:
        for sub in (sub_loc, sub_odom, sub_gnss):
            node.destroy_subscription(sub)


def cmd_init_localization(args: argparse.Namespace) -> int:
    """Wait for ADAPI + sensors and initialize Autoware GNSS/NDT localization."""
    rclpy.init()
    node = rclpy.create_node("carla_init_localization")
    try:
        odom = _ensure_localized(node, timeout_sec=args.timeout, retries=args.retries)
        if odom is None:
            return 1
        p = odom.pose.pose.position
        print(f"[OK] Localization INITIALIZED at map pose ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_drive(args: argparse.Namespace) -> int:
    """Set route goal and engage Autoware Autonomous mode."""
    rclpy.init()
    node = rclpy.create_node("carla_autoware_driver")
    try:
        if _ensure_localized(node, timeout_sec=30.0, retries=10) is None:
            print("[FAIL] Autoware localization is not initialized yet.", file=sys.stderr)
            return 1

        state = {
            "route": None,
            "mode": None,
            "autoware_ctrl": False,
        }
        sub_route = node.create_subscription(
            RouteState,
            "/api/routing/state",
            lambda m: state.__setitem__("route", m.state),
            TRANSIENT_QOS,
        )
        sub_mode = node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            lambda m: (
                state.__setitem__("mode", m.mode),
                state.__setitem__("autoware_ctrl", m.is_autoware_control_enabled),
            ),
            TRANSIENT_QOS,
        )
        _ = (sub_route, sub_mode)

        clear_cli = node.create_client(ClearRoute, "/api/routing/clear_route")
        call_service(node, clear_cli, ClearRoute.Request(), timeout_sec=15.0)

        goal_pose = Pose()
        goal_pose.position.x = args.goal[0]
        goal_pose.position.y = args.goal[1]
        goal_pose.position.z = args.goal[2]
        goal_pose.orientation.x = args.goal[3]
        goal_pose.orientation.y = args.goal[4]
        goal_pose.orientation.z = args.goal[5]
        goal_pose.orientation.w = args.goal[6]

        set_cli = node.create_client(SetRoutePoints, "/api/routing/set_route_points")
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.header.stamp = node.get_clock().now().to_msg()
        req.goal = goal_pose
        req.waypoints = []
        res = call_service(node, set_cli, req, timeout_sec=20.0)
        if not res.status.success:
            print(f"[FAIL] SetRoutePoints rejected: {res.status.message}", file=sys.stderr)
            return 1

        if not spin_until(node, lambda: state["route"] == RouteState.SET, timeout_sec=15.0):
            print(f"[FAIL] Route did not enter SET state (got {state['route']}).", file=sys.stderr)
            return 1
        print(
            f"[drive] Route SET -> goal ({args.goal[0]:.2f}, {args.goal[1]:.2f}, {args.goal[2]:.2f})"
        )

        # Allow planning + trajectory validator + MRM diagnostics a short moment to settle.
        time.sleep(2.0)

        mode_cli = node.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )
        engaged = False
        for attempt in range(1, 11):
            m_res = call_service(node, mode_cli, ChangeOperationMode.Request(), timeout_sec=15.0)
            if m_res.status.success:
                engaged = True
                break
            print(f"[drive] Waiting to engage AUTONOMOUS ({attempt}/10): {m_res.status.message}")
            spin_until(node, lambda: False, timeout_sec=1.5)

        if not engaged:
            print("[FAIL] Unable to transition Autoware to AUTONOMOUS mode.", file=sys.stderr)
            return 1

        spin_until(
            node,
            lambda: state["mode"] == OperationModeState.AUTONOMOUS and state["autoware_ctrl"],
            timeout_sec=10.0,
        )
        print("[OK] Autoware engaged in AUTONOMOUS mode.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_chk = sub.add_parser("check-connection", help="Check CARLA native ROS 2 topics")
    p_chk.add_argument("--timeout", type=float, default=12.0)

    p_init = sub.add_parser("init-localization", help="Initialize GNSS + NDT localization")
    p_init.add_argument("--timeout", type=float, default=120.0)
    p_init.add_argument("--retries", type=int, default=10)

    p_drv = sub.add_parser("drive", help="Set route goal and engage autonomous driving")
    p_drv.add_argument("--goal", nargs=7, type=float, required=True, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "check-connection":
        return cmd_check_connection(args)
    if args.command == "init-localization":
        return cmd_init_localization(args)
    if args.command == "drive":
        return cmd_drive(args)
    return 1


if __name__ == "__main__":
    sys.exit(main())
