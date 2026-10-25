#!/usr/bin/env python3

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Autoware ADAPI Automation Helper (runs inside the Autoware container).

Provides programmatic service clients and publishers for:
1. wait_adapi: Waits for ADAPI services to become online.
2. init_localization: Requests GNSS auto-initialization via /api/localization/initialize.
3. send_goal: Publishes target goal pose on /planning/mission_planning/goal.
4. engage_autonomous: Requests mode change via /api/operation_mode/change_to_autonomous.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import (
    AccelWithCovarianceStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.srv import InitializeLocalization, ChangeOperationMode, SetRoutePoints, AcceptStart
from autoware_localization_msgs.srv import InitializeLocalization as DirectInitializeLocalization
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from autoware_system_msgs.srv import ChangeOperationMode as SystemChangeOperationMode, ChangeAutowareControl
from autoware_vehicle_msgs.msg import VelocityReport, Engage


class AutowareAutomationHelper(Node):
    def __init__(self):
        super().__init__(
            "autoware_automation_helper",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        self.init_loc_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self.direct_loc_client = self.create_client(
            DirectInitializeLocalization, "/localization/initialize"
        )
        self.set_route_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )
        self.change_mode_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )
        self.sys_mode_client = self.create_client(
            SystemChangeOperationMode, "/system/operation_mode/change_operation_mode"
        )
        self.sys_control_client = self.create_client(
            ChangeAutowareControl, "/system/operation_mode/change_autoware_control"
        )
        self.accept_start_client = self.create_client(
            AcceptStart, "/api/motion/accept_start"
        )
        self.engage_pub = self.create_publisher(
            Engage, "/autoware/engage", 10
        )
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", qos_transient
        )

    def wait_adapi(self, timeout_sec: float = 60.0) -> bool:
        start = time.time()
        self.get_logger().info("Waiting for ADAPI change_to_autonomous service...")
        while time.time() - start < timeout_sec:
            if self.change_mode_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("ADAPI change_to_autonomous service is ONLINE.")
                return True
            time.sleep(1.0)
        return False

    def init_localization(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = 0.0,
        yaw_deg: float | None = None,
        max_attempts: int = 10,
        delay: float = 4.0,
    ) -> bool:
        if not self.init_loc_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("InitializeLocalization service not available!")
            return False

        for attempt in range(1, max_attempts + 1):
            req = InitializeLocalization.Request()
            if x is not None and y is not None and yaw_deg is not None:
                pose_cov = PoseWithCovarianceStamped()
                pose_cov.header.stamp = self.get_clock().now().to_msg()
                pose_cov.header.frame_id = "map"
                pose_cov.pose.pose.position.x = float(x)
                pose_cov.pose.pose.position.y = float(y)
                pose_cov.pose.pose.position.z = float(z if z is not None else 0.0)
                yaw_rad = math.radians(yaw_deg)
                pose_cov.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
                pose_cov.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
                pose_cov.pose.covariance[0] = 1.0
                pose_cov.pose.covariance[7] = 1.0
                pose_cov.pose.covariance[14] = 0.01
                pose_cov.pose.covariance[21] = 0.01
                pose_cov.pose.covariance[28] = 0.01
                pose_cov.pose.covariance[35] = 0.2
                req.pose = [pose_cov]
            future = self.init_loc_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.done():
                res = future.result()
                if res is not None and res.status.success:
                    self.get_logger().info(
                        f"Localization initialized successfully on attempt {attempt}: {res.status.message}"
                    )
                    return True
                else:
                    msg = res.status.message if res else "No response"
                    code = res.status.code if res else -1
                    self.get_logger().warn(
                        f"Attempt {attempt}/{max_attempts} failed (code={code}): {msg}"
                    )
            time.sleep(delay)
        return False

    def init_localization_direct(
        self, x: float, y: float, z: float, yaw_deg: float
    ) -> bool:
        if not self.direct_loc_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("/localization/initialize service not available!")
            return False

        req = DirectInitializeLocalization.Request()
        req.method = DirectInitializeLocalization.Request.DIRECT
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header.stamp = self.get_clock().now().to_msg()
        pose_cov.header.frame_id = "map"
        pose_cov.pose.pose.position.x = float(x)
        pose_cov.pose.pose.position.y = float(y)
        pose_cov.pose.pose.position.z = float(z)
        yaw_rad = math.radians(yaw_deg)
        pose_cov.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose_cov.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        pose_cov.pose.covariance[0] = 0.1
        pose_cov.pose.covariance[7] = 0.1
        pose_cov.pose.covariance[14] = 0.01
        pose_cov.pose.covariance[21] = 0.01
        pose_cov.pose.covariance[28] = 0.01
        pose_cov.pose.covariance[35] = 0.05
        req.pose_with_covariance = [pose_cov]

        future = self.direct_loc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            res = future.result()
            if res is not None and res.status.success:
                self.get_logger().info(
                    f"Direct localization initialized successfully: {res.status.message}"
                )
                return True
            else:
                msg = res.status.message if res else "No response"
                self.get_logger().error(f"Direct localization failed: {msg}")
        return False

    def get_kinematic_state(self, timeout_sec: float = 10.0) -> dict | None:
        received_msg = []
        def cb(msg):
            received_msg.append(msg)

        sub = self.create_subscription(Odometry, "/localization/kinematic_state", cb, 10)
        start = time.time()
        while time.time() - start < timeout_sec and not received_msg:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(sub)
        if received_msg:
            msg = received_msg[0]
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = math.atan2(2 * ori.w * ori.z, 1 - 2 * ori.z**2)
            return {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z,
                "yaw_deg": math.degrees(yaw)
            }
        return None

    def set_route(self, x: float, y: float, z: float, yaw_deg: float) -> bool:
        if not self.set_route_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("/api/routing/set_route_points service not available!")
            return False

        req = SetRoutePoints.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = "map"
        req.option.allow_goal_modification = True
        req.goal.position.x = float(x)
        req.goal.position.y = float(y)
        req.goal.position.z = float(z)

        yaw_rad = math.radians(yaw_deg)
        req.goal.orientation.z = math.sin(yaw_rad / 2.0)
        req.goal.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info(
            f"Calling /api/routing/set_route_points to map=({x:.2f}, {y:.2f}), yaw={yaw_deg:.2f}°..."
        )
        future = self.set_route_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.done():
            res = future.result()
            if res is not None and (res.status.success or res.status.code == 1):
                self.get_logger().info(f"Route set successfully: {res.status.message}")
                return True
            else:
                msg = res.status.message if res else "No response"
                code = res.status.code if res else -1
                self.get_logger().error(f"SetRoutePoints failed (code={code}): {msg}")
        return False

    def send_goal(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Publish multiple times to ensure delivery
        for _ in range(5):
            self.goal_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info(
            f"Published Goal Pose: map=({x:.2f}, {y:.2f}, {z:.2f}), yaw={yaw_deg:.2f}°"
        )

    def engage_autonomous(self, max_attempts: int = 15, delay: float = 6.0) -> bool:
        # First attempt direct system-level engage (bypasses UI/ADAPI state machine restrictions)
        if self.sys_control_client.wait_for_service(timeout_sec=5.0):
            req_ctrl = ChangeAutowareControl.Request()
            req_ctrl.autoware_control = True
            fut_ctrl = self.sys_control_client.call_async(req_ctrl)
            rclpy.spin_until_future_complete(self, fut_ctrl, timeout_sec=5.0)
            if fut_ctrl.done() and fut_ctrl.result():
                self.get_logger().info(f"System ChangeAutowareControl: {fut_ctrl.result().status.message}")

        if self.sys_mode_client.wait_for_service(timeout_sec=5.0):
            req_mode = SystemChangeOperationMode.Request()
            req_mode.mode = SystemChangeOperationMode.Request.AUTONOMOUS
            fut_mode = self.sys_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self, fut_mode, timeout_sec=5.0)
            if fut_mode.done() and fut_mode.result() and fut_mode.result().status.success:
                self.get_logger().info("System ChangeOperationMode AUTONOMOUS succeeded!")
                self.trigger_start()
                return True

        if not self.change_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("ChangeOperationMode service not available!")
            return False

        for attempt in range(1, max_attempts + 1):
            req = ChangeOperationMode.Request()
            future = self.change_mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.done():
                res = future.result()
                if res is not None and res.status.success:
                    self.get_logger().info(
                        f"Autonomous mode ENGAGED successfully on attempt {attempt}: {res.status.message}"
                    )
                    self.trigger_start()
                    return True
                else:
                    msg = res.status.message if res else "No response"
                    code = res.status.code if res else -1
                    self.get_logger().warn(
                        f"Attempt {attempt}/{max_attempts} engage failed (code={code}): {msg}"
                    )
            time.sleep(delay)
        return False

    def trigger_start(self):
        # 1. Publish /autoware/engage
        engage_msg = Engage()
        engage_msg.stamp = self.get_clock().now().to_msg()
        engage_msg.engage = True
        for _ in range(3):
            self.engage_pub.publish(engage_msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().info("Published /autoware/engage (engage=True)")

        # 2. Call /api/motion/accept_start
        if self.accept_start_client.wait_for_service(timeout_sec=2.0):
            req_start = AcceptStart.Request()
            fut_start = self.accept_start_client.call_async(req_start)
            rclpy.spin_until_future_complete(self, fut_start, timeout_sec=3.0)
            if fut_start.done() and fut_start.result():
                self.get_logger().info(f"AcceptStart: {fut_start.result().status.message}")

    def init_localization_direct(
        self, x: float, y: float, z: float, yaw_deg: float
    ) -> bool:
        if not self.direct_loc_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("/localization/initialize service not available!")
            return False

        req = DirectInitializeLocalization.Request()
        req.method = DirectInitializeLocalization.Request.DIRECT
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header.stamp = self.get_clock().now().to_msg()
        pose_cov.header.frame_id = "map"
        pose_cov.pose.pose.position.x = float(x)
        pose_cov.pose.pose.position.y = float(y)
        pose_cov.pose.pose.position.z = float(z)
        yaw_rad = math.radians(yaw_deg)
        pose_cov.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose_cov.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        pose_cov.pose.covariance[0] = 0.1
        pose_cov.pose.covariance[7] = 0.1
        pose_cov.pose.covariance[14] = 0.01
        pose_cov.pose.covariance[21] = 0.01
        pose_cov.pose.covariance[28] = 0.01
        pose_cov.pose.covariance[35] = 0.05
        req.pose_with_covariance = [pose_cov]

        future = self.direct_loc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            res = future.result()
            if res is not None and res.status.success:
                self.get_logger().info(
                    f"Direct localization initialized successfully: {res.status.message}"
                )
                return True
            else:
                msg = res.status.message if res else "No response"
                self.get_logger().error(f"Direct localization failed: {msg}")
        return False

    def get_kinematic_state(self, timeout_sec: float = 10.0) -> dict | None:
        received_msg = []
        def cb(msg):
            received_msg.append(msg)

        sub = self.create_subscription(Odometry, "/localization/kinematic_state", cb, 10)
        start = time.time()
        while time.time() - start < timeout_sec and not received_msg:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(sub)
        if received_msg:
            msg = received_msg[0]
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = math.atan2(2 * ori.w * ori.z, 1 - 2 * ori.z**2)
            return {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z,
                "yaw_deg": math.degrees(yaw)
            }
        return None


class CarlaStateRelay(Node):
    """
    Lightweight Ground-Truth Localization Relay for CARLA <-> Autoware integration.

    Subscribes to CARLA's native ROS 2 topics:
      - /sensing/gnss/pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped)
      - /vehicle/status/velocity_status (autoware_vehicle_msgs/VelocityReport)

    Publishes Autoware localization inputs:
      - /localization/kinematic_state (nav_msgs/Odometry) in map frame
      - /localization/acceleration (geometry_msgs/AccelWithCovarianceStamped)
      - /localization/initialization_state (autoware_adapi_v1_msgs/LocalizationInitializationState)
      - /tf (map -> base_link transform)

    Enables full Autoware planning and control stack execution without heavy
    CPU/GPU-bound NDT scan matching on systems without dedicated CUDA hardware.
    """

    def __init__(self, publish_rate_hz: float = 20.0):
        super().__init__(
            "carla_state_relay",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        self.publish_rate = publish_rate_hz

        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self.gnss_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self.on_gnss,
            qos_volatile,
        )
        self.vel_sub = self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.on_vel,
            qos_volatile,
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 10
        )
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.init_state_pub = self.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            qos_transient,
        )
        self.api_init_state_pub = self.create_publisher(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            qos_transient,
        )
        self.objects_pub = self.create_publisher(
            PredictedObjects,
            "/perception/object_recognition/objects",
            10,
        )
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid,
            "/perception/occupancy_grid_map/map",
            10,
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            "/perception/obstacle_segmentation/pointcloud",
            10,
        )
        self.traffic_signals_pub = self.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            10,
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Direct host UDP telemetry receiver (port 9099)
        import socket, struct
        self._struct = struct
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        try:
            self.sock.bind(("0.0.0.0", 9099))
            self.get_logger().info("CarlaStateRelay: UDP telemetry socket bound on port 9099.")
        except Exception as e:
            self.get_logger().warn(f"CarlaStateRelay: Could not bind UDP port 9099: {e}")

        self.last_pose: PoseWithCovarianceStamped | None = None
        self.last_vel: VelocityReport | None = None
        self.udp_telemetry: tuple | None = None
        self.prev_speed = 0.0
        self.prev_speed_time = time.time()
        self.current_accel = 0.0
        self.publish_count = 0

        from rclpy.clock import Clock, ClockType
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_state, clock=self._steady_clock
        )
        self.get_logger().info(
            f"CarlaStateRelay active at {self.publish_rate} Hz (UDP/GNSS -> /localization/kinematic_state & TF)"
        )

    def on_gnss(self, msg: PoseWithCovarianceStamped):
        self.last_pose = msg

    def on_vel(self, msg: VelocityReport):
        self.last_vel = msg

    def poll_udp_telemetry(self):
        try:
            while True:
                data, _ = self.sock.recvfrom(1024)
                if len(data) == 64:
                    self.udp_telemetry = self._struct.unpack("!2I7d", data)
                elif len(data) == 56:
                    vals = self._struct.unpack("!7d", data)
                    self.udp_telemetry = (0, 0) + vals
        except (BlockingIOError, OSError):
            pass

    def publish_state(self):
        self.poll_udp_telemetry()

        from geometry_msgs.msg import Quaternion
        from builtin_interfaces.msg import Time as MsgTime

        if self.udp_telemetry is not None:
            sec, nanosec, base_x, base_y, base_z, map_yaw, vx, vy, wz = self.udp_telemetry
            if sec > 0 or nanosec > 0:
                stamp = MsgTime(sec=int(sec), nanosec=int(nanosec))
            else:
                stamp = self.get_clock().now().to_msg()
            orient = Quaternion()
            orient.x = 0.0
            orient.y = 0.0
            orient.z = math.sin(map_yaw / 2.0)
            orient.w = math.cos(map_yaw / 2.0)
            yaw = map_yaw
        elif self.last_pose is not None:
            stamp = self.last_pose.header.stamp
            gnss_x = self.last_pose.pose.pose.position.x
            gnss_y = self.last_pose.pose.pose.position.y
            gnss_z = self.last_pose.pose.pose.position.z
            orient = self.last_pose.pose.pose.orientation

            # Heading angle from quaternion
            siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
            cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # GNSS receiver is mounted at x=+0.8m ahead and z=+1.8m above base_link (rear axle)
            base_x = gnss_x - 0.8 * math.cos(yaw)
            base_y = gnss_y - 0.8 * math.sin(yaw)
            base_z = max(0.0, gnss_z - 1.8)

            vx = self.last_vel.longitudinal_velocity if self.last_vel else 0.0
            vy = self.last_vel.lateral_velocity if self.last_vel else 0.0
            wz = self.last_vel.heading_rate if self.last_vel else 0.0
        else:
            return

        now = time.time()
        dt = now - self.prev_speed_time
        if dt > 0.01:
            raw_accel = (vx - self.prev_speed) / dt
            self.current_accel = 0.8 * self.current_accel + 0.2 * raw_accel
            self.prev_speed = vx
            self.prev_speed_time = now

        # 1. Publish Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = base_x
        odom.pose.pose.position.y = base_y
        odom.pose.pose.position.z = base_z
        odom.pose.pose.orientation = orient
        # Low covariance for high-confidence localization
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.001
        odom.pose.covariance[28] = 0.001
        odom.pose.covariance[35] = 0.005

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # 2. Publish Acceleration
        accel = AccelWithCovarianceStamped()
        accel.header.stamp = stamp
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = self.current_accel
        self.accel_pub.publish(accel)

        # 3. Broadcast TF map -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = base_x
        tf_msg.transform.translation.y = base_y
        tf_msg.transform.translation.z = base_z
        tf_msg.transform.rotation = orient
        self.tf_broadcaster.sendTransform(tf_msg)

        # 4. Publish LocalizationInitializationState
        init_msg = LocalizationInitializationState()
        init_msg.stamp = stamp
        init_msg.state = LocalizationInitializationState.INITIALIZED
        self.init_state_pub.publish(init_msg)
        self.api_init_state_pub.publish(init_msg)

        # 5. Publish perception stubs (objects, occupancy grid, pointcloud, traffic signals)
        # to unlock Autoware planning modules (behavior_path_planner, behavior_velocity_planner, AEB)
        objs_msg = PredictedObjects()
        objs_msg.header.stamp = stamp
        objs_msg.header.frame_id = "map"
        self.objects_pub.publish(objs_msg)

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = "map"
        grid_msg.info.map_load_time = stamp
        grid_msg.info.resolution = 0.5
        grid_msg.info.width = 40
        grid_msg.info.height = 40
        grid_msg.info.origin.position.x = base_x - 10.0
        grid_msg.info.origin.position.y = base_y - 10.0
        grid_msg.info.origin.position.z = base_z
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = [0] * (40 * 40)
        self.occupancy_pub.publish(grid_msg)

        pc_msg = PointCloud2()
        pc_msg.header.stamp = stamp
        pc_msg.header.frame_id = "base_link"
        pc_msg.height = 1
        pc_msg.width = 0
        pc_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12
        pc_msg.row_step = 0
        pc_msg.data = b""
        pc_msg.is_dense = True
        self.pointcloud_pub.publish(pc_msg)

        tl_msg = TrafficLightGroupArray()
        tl_msg.stamp = stamp
        self.traffic_signals_pub.publish(tl_msg)

        self.publish_count += 1
        if self.publish_count % 100 == 0:
            self.get_logger().info(
                f"[Relay #{self.publish_count}] base_link=({base_x:.2f}, {base_y:.2f}) speed={vx*3.6:.1f} km/h yaw={math.degrees(yaw):.1f}°"
            )


def main():
    parser = argparse.ArgumentParser(description="Autoware ADAPI Automation Helper")
    subparsers = parser.add_subparsers(dest="action", required=True)

    subparsers.add_parser("wait_adapi")
    subparsers.add_parser("state_relay")
    init_parser = subparsers.add_parser("init_localization")
    init_parser.add_argument("--direct", action="store_true", default=False)
    init_parser.add_argument("--x", type=float, default=None)
    init_parser.add_argument("--y", type=float, default=None)
    init_parser.add_argument("--z", type=float, default=0.0)
    init_parser.add_argument("--yaw", type=float, default=None)

    goal_parser = subparsers.add_parser("send_goal")
    goal_parser.add_argument("--x", type=float, required=True)
    goal_parser.add_argument("--y", type=float, required=True)
    goal_parser.add_argument("--z", type=float, default=0.0)
    goal_parser.add_argument("--yaw", type=float, required=True)

    subparsers.add_parser("engage_autonomous")
    subparsers.add_parser("get_kinematic_state")

    args = parser.parse_args()

    rclpy.init()

    if args.action == "state_relay":
        relay = CarlaStateRelay(publish_rate_hz=20.0)
        try:
            rclpy.spin(relay)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            try:
                relay.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
        sys.exit(0)

    node = AutowareAutomationHelper()
    success = False

    try:
        if args.action == "wait_adapi":
            success = node.wait_adapi(timeout_sec=90.0)
        elif args.action == "init_localization":
            if args.direct or (args.x is not None and args.y is not None and args.yaw is not None):
                node.get_logger().info(f"Attempting direct localization initialization at ({args.x}, {args.y}, yaw={args.yaw})...")
                success = node.init_localization_direct(
                    x=args.x, y=args.y, z=args.z, yaw_deg=args.yaw
                )
                if not success and not args.direct:
                    node.get_logger().warn("Direct init failed; attempting ADAPI initialization...")
                    success = node.init_localization(
                        x=args.x, y=args.y, z=args.z, yaw_deg=args.yaw, max_attempts=6, delay=3.0
                    )
            else:
                success = node.init_localization(
                    x=args.x, y=args.y, z=args.z, yaw_deg=args.yaw, max_attempts=12, delay=3.0
                )
        elif args.action == "send_goal":
            node.send_goal(args.x, args.y, args.z, args.yaw)
            success = node.set_route(args.x, args.y, args.z, args.yaw)
        elif args.action == "engage_autonomous":
            success = node.engage_autonomous(max_attempts=15, delay=6.0)
        elif args.action == "get_kinematic_state":
            state = node.get_kinematic_state(timeout_sec=10.0)
            if state is not None:
                print(json.dumps(state))
                success = True
            else:
                success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
