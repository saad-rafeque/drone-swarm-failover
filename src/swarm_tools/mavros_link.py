"""Test-harness handle for one drone's MAVROS namespace (not onboard code).

Interfaces verified on the running system (Phase 0: ros2 topic/service list, mavros_msgs/*.msg):
  <ns>/mavros/state (mavros_msgs/State), <ns>/mavros/extended_state (mavros_msgs/ExtendedState),
  <ns>/mavros/local_position/pose (geometry_msgs/PoseStamped, ENU local frame),
  <ns>/mavros/global_position/global (sensor_msgs/NavSatFix),
  <ns>/mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped, ENU),
  <ns>/mavros/cmd/arming (mavros_msgs/CommandBool), <ns>/mavros/set_mode (mavros_msgs/SetMode).
"""
from __future__ import annotations

import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

REQUEST_PERIOD_S = 1.0  # at most one mode/arm request per drone per second


class MavrosLink:
    def __init__(self, node: Node, drone_id: int, ns: str) -> None:
        self.node = node
        self.drone_id = drone_id
        self.state = State()
        self.landed_state = ExtendedState.LANDED_STATE_UNDEFINED
        self.local: tuple[float, float, float] | None = None
        self.fix: NavSatFix | None = None
        self.cmd = (0.0, 0.0, 0.0)
        self._last_request = 0.0
        qos = qos_profile_sensor_data
        node.create_subscription(State, f"{ns}/mavros/state", self._on_state, qos)
        node.create_subscription(ExtendedState, f"{ns}/mavros/extended_state", self._on_ext, qos)
        node.create_subscription(PoseStamped, f"{ns}/mavros/local_position/pose", self._on_pose, qos)
        node.create_subscription(NavSatFix, f"{ns}/mavros/global_position/global", self._on_fix, qos)
        self._sp_pub = node.create_publisher(TwistStamped, f"{ns}/mavros/setpoint_velocity/cmd_vel", 10)
        self._arm_cli = node.create_client(CommandBool, f"{ns}/mavros/cmd/arming")
        self._mode_cli = node.create_client(SetMode, f"{ns}/mavros/set_mode")

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_ext(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.local = (p.x, p.y, p.z)

    def _on_fix(self, msg: NavSatFix) -> None:
        self.fix = msg

    def publish_setpoint(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.cmd
        self._sp_pub.publish(msg)

    def _may_request(self) -> bool:
        now = time.time()
        if now - self._last_request < REQUEST_PERIOD_S:
            return False
        self._last_request = now
        return True

    def request_mode(self, mode: str) -> None:
        """Fire-and-forget SET_MODE (rate-limited); success is observed on mavros/state."""
        if self.state.mode != mode and self._mode_cli.service_is_ready() and self._may_request():
            req = SetMode.Request()
            req.custom_mode = mode
            self._mode_cli.call_async(req)

    def request_arm(self) -> None:
        if not self.state.armed and self._arm_cli.service_is_ready() and self._may_request():
            req = CommandBool.Request()
            req.value = True
            self._arm_cli.call_async(req)

    @property
    def offboard_armed(self) -> bool:
        return self.state.mode == "OFFBOARD" and self.state.armed
