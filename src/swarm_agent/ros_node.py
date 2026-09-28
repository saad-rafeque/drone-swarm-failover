"""ROS 2 node for ONE drone: wraps AgentCore and talks to this drone's own MAVROS (docs/SPECIFICATION.md §6).

There is no central controller; this node is the only thing that commands its drone, and the same
code is meant to run onboard. It runs in namespace /uav<ID>.

Interfaces (verified on the running system in Phases 0/1, see reports):
  subscribes  mavros/state (mavros_msgs/State), mavros/extended_state (mavros_msgs/ExtendedState),
              mavros/global_position/global (sensor_msgs/NavSatFix),
              mavros/local_position/pose (geometry_msgs/PoseStamped, ENU),
              mavros/local_position/velocity_local (geometry_msgs/TwistStamped, ENU),
              mavros/battery (sensor_msgs/BatteryState, percentage 0..1),
              hb_in (std_msgs/UInt8MultiArray, heartbeats from the link emulator / radio)
  publishes   mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped, ENU, setpoint rate),
              hb_out (std_msgs/UInt8MultiArray, 50-byte heartbeat, 5 Hz),
              agent_state (std_msgs/String, JSON, logging only - never used for control)
  services    mavros/set_mode (mavros_msgs/SetMode), mavros/cmd/arming (mavros_msgs/CommandBool)

Position: east/north from the global fix converted to the shared ENU frame (never the local
frame); height from the local pose z, i.e. above the drone's own take-off point. In simulation
every home shares one ground elevation, so that height is also above the origin's ground level.
If the autopilot data goes stale (fcu_timeout_s) the drone counts as dead: no heartbeats.
A drone whose autopilot link is a telemetry radio (--radio-link; the Phase 6 stand-in) uses the
radio_standin settings instead: slower autopilot streams that fit the radio, and a longer fcu_timeout_s.

Run: scripts/ros_env.sh python3 -m swarm_agent.ros_node --id 3 [--radio-link]
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, MessageInterval, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import String, UInt8MultiArray

from .agent_core import AgentCore, Command, FlightMode, OwnState
from .config import MAVLINK_MSG_IDS, Config, default_config_path, load_config
from .geometry import ZERO, EnuFrame, GeoPoint, Vec3
from .heartbeat import decode, encode

REQUEST_PERIOD_S = 1.0
MODE_OFFBOARD, MODE_LAND = "OFFBOARD", "AUTO.LAND"
STREAM_RETRY_S = 2.0
# MAVLink common message IDs (verified against pymavlink.dialects.v20.common, tests/test_ros_constants.py)


class SwarmAgentNode(Node):
    def __init__(self, cfg: Config, drone_id: int, radio_link: bool = False) -> None:
        super().__init__("swarm_agent", namespace=f"/uav{drone_id}")
        self.cfg, self.drone_id = cfg, drone_id
        self.radio_link = radio_link
        self.fcu_timeout_s = cfg.radio_standin.fcu_timeout_s if radio_link else cfg.heartbeat.fcu_timeout_s
        streams = cfg.radio_standin.autopilot_streams_hz if radio_link else cfg.autopilot_streams_hz
        self.frame = EnuFrame(cfg.origin_geo)
        self.core: AgentCore | None = None
        self.state = State()
        self.landed_state = ExtendedState.LANDED_STATE_UNDEFINED
        self.en: tuple[float, float] | None = None
        self.z: float | None = None
        self.vel: Vec3 = ZERO
        self.battery_pct = 100.0
        self.fix_ok = False
        self._rx = {"fix": -math.inf, "pose": -math.inf}
        self._last_request = {"mode": -math.inf, "arm": -math.inf}
        self.last_cmd: Command | None = None
        self.fcu_ok = False
        self.hb_sent = self.hb_received = 0

        q = qos_profile_sensor_data
        self.create_subscription(State, "mavros/state", self._on_state, q)
        self.create_subscription(ExtendedState, "mavros/extended_state", self._on_ext, q)
        self.create_subscription(NavSatFix, "mavros/global_position/global", self._on_fix, q)
        self.create_subscription(PoseStamped, "mavros/local_position/pose", self._on_pose, q)
        self.create_subscription(TwistStamped, "mavros/local_position/velocity_local", self._on_vel, q)
        self.create_subscription(BatteryState, "mavros/battery", self._on_battery, q)
        self.create_subscription(UInt8MultiArray, "hb_in", self._on_hb, q)
        self.sp_pub = self.create_publisher(TwistStamped, "mavros/setpoint_velocity/cmd_vel", 10)
        self.hb_pub = self.create_publisher(UInt8MultiArray, "hb_out", q)
        self.state_pub = self.create_publisher(String, "agent_state", 10)
        self.mode_cli = self.create_client(SetMode, "mavros/set_mode")
        self.arm_cli = self.create_client(CommandBool, "mavros/cmd/arming")
        self.rate_cli = self.create_client(MessageInterval, "mavros/set_message_interval")
        self._streams_pending = {MAVLINK_MSG_IDS[k]: v for k, v in streams.items()}
        self._streams_last_try = -math.inf
        self.create_timer(1.0 / cfg.setpoints.rate_hz, self._tick)
        self.create_timer(1.0 / cfg.logging.rate_hz, self._publish_state)

    # ------------------------------------------------------------------ helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_ext(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state

    def _on_fix(self, msg: NavSatFix) -> None:
        e, n, _ = self.frame.to_enu(GeoPoint(msg.latitude, msg.longitude, self.cfg.origin.alt_m))
        self.en = (e, n)
        self.fix_ok = msg.status.status >= NavSatStatus.STATUS_FIX
        self._rx["fix"] = self.now()

    def _on_pose(self, msg: PoseStamped) -> None:
        self.z = msg.pose.position.z
        self._rx["pose"] = self.now()

    def _on_vel(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self.vel = (v.x, v.y, v.z)

    def _on_battery(self, msg: BatteryState) -> None:
        if not math.isnan(msg.percentage):
            self.battery_pct = 100.0 * msg.percentage

    def _on_hb(self, msg: UInt8MultiArray) -> None:
        if self.core is None or not self.fcu_ok:
            return
        try:
            hb = decode(bytes(msg.data))
        except ValueError:
            return
        self.hb_received += 1
        self.core.on_heartbeat(hb, self.now())

    def _landed(self) -> bool:
        if self.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            return True
        return self.landed_state == ExtendedState.LANDED_STATE_UNDEFINED and not self.state.armed

    def _may_request(self, kind: str, now: float) -> bool:
        if now - self._last_request[kind] < REQUEST_PERIOD_S:
            return False
        self._last_request[kind] = now
        return True

    def _set_mode(self, mode: str, now: float) -> None:
        if self.state.mode != mode and self.mode_cli.service_is_ready() and self._may_request("mode", now):
            req = SetMode.Request()
            req.custom_mode = mode
            self.mode_cli.call_async(req)

    def _arm(self, now: float) -> None:
        if not self.state.armed and self.arm_cli.service_is_ready() and self._may_request("arm", now):
            req = CommandBool.Request()
            req.value = True
            self.arm_cli.call_async(req)

    def _publish_setpoint(self, v: Vec3) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v
        self.sp_pub.publish(msg)

    def _request_streams(self, now: float) -> None:
        """Ask the autopilot for the configured MAVLink stream rates until each one is accepted."""
        if not self._streams_pending or now - self._streams_last_try < STREAM_RETRY_S:
            return
        if not (self.state.connected and self.rate_cli.service_is_ready()):
            return
        self._streams_last_try = now
        for msg_id, rate in list(self._streams_pending.items()):
            req = MessageInterval.Request()
            req.message_id, req.message_rate = msg_id, rate
            fut = self.rate_cli.call_async(req)
            fut.add_done_callback(lambda f, m=msg_id: self._on_stream_set(m, f))

    def _on_stream_set(self, msg_id: int, fut) -> None:
        res = fut.result()
        if res is not None and res.success:
            self._streams_pending.pop(msg_id, None)

    # ------------------------------------------------------------------ control loop
    def _tick(self) -> None:
        now = self.now()
        self._request_streams(now)
        fresh = self.state.connected and self.en is not None and self.z is not None and all(
            now - t < self.fcu_timeout_s for t in self._rx.values())
        self.fcu_ok = fresh
        if not fresh:
            self._publish_setpoint(ZERO)  # keep PX4's offboard proof-of-life; no heartbeats
            return
        pos = (self.en[0], self.en[1], self.z)
        if self.core is None:
            self.core = AgentCore(self.cfg, self.drone_id, (pos[0], pos[1], 0.0), now)
        own = OwnState(pos=pos, vel=self.vel, battery_pct=self.battery_pct,
                       ready=self.state.connected and self.fix_ok, landed=self._landed())
        cmd, hb = self.core.step(own, now)
        self.last_cmd = cmd
        if cmd.mode == FlightMode.OFFBOARD:
            self._publish_setpoint(cmd.vel)
            if self.state.mode != MODE_OFFBOARD:
                self._set_mode(MODE_OFFBOARD, now)
            elif not self.state.armed:
                self._arm(now)
        else:
            self._publish_setpoint(ZERO)
            if cmd.mode == FlightMode.LAND and self.state.armed:
                self._set_mode(MODE_LAND, now)
        if hb is not None:
            self.hb_pub.publish(UInt8MultiArray(data=encode(hb)))
            self.hb_sent += 1

    def _publish_state(self) -> None:
        core, cmd = self.core, self.last_cmd
        d: dict = {"t": round(self.now(), 3), "id": self.drone_id, "fcu_ok": self.fcu_ok,
                   "streams_pending": len(self._streams_pending),
                   "armed": self.state.armed, "mode": self.state.mode, "landed": self._landed(),
                   "battery": round(self.battery_pct, 1), "hb_sent": self.hb_sent, "hb_rx": self.hb_received}
        if self.en is not None and self.z is not None:
            d["pos"] = [round(self.en[0], 3), round(self.en[1], 3), round(self.z, 3)]
            d["pos_t"] = round(self._rx["fix"], 3)  # when that fix arrived (metrics extrapolate from here)
            d["vel"] = [round(x, 3) for x in self.vel]
        if core is not None:
            e = core.election
            d.update(role=e.role.name, term=e.term, master=e.master_id, phase=core.phase.name,
                     heading=round(core.heading, 5), orphan=core.orphan, transit=core.transit,
                     retire=core.retire_stage, handover_to=e.handover_to)
            if e.role.name == "MASTER":
                d["members"] = sorted(e.members(self.now()))
            # exact moments for the Phase 4 metrics (scripts/phase4_metrics.py): when this drone last became
            # master, and when its battery handover started (the first "retire" event)
            claims = [ev.t for ev in e.events if ev.kind == "claim"]
            if claims:
                d["claim_t"] = round(claims[-1], 3)
            retire_t = next((ev.t for ev in e.events if ev.kind == "retire"), None)
            if retire_t is not None:
                d["retire_t"] = round(retire_t, 3)
        if cmd is not None:
            d.update(reason=cmd.reason, cmd=[round(x, 3) for x in cmd.vel], cmd_mode=cmd.mode.name)
        self.state_pub.publish(String(data=json.dumps(d, separators=(",", ":"))))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="swarm agent for one drone")
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--config", default=str(default_config_path()))
    ap.add_argument("--num-drones", type=int, default=0, help="override swarm.num_drones")
    ap.add_argument("--radio-link", action="store_true",
                    help="this drone's autopilot link is a telemetry radio: use the radio_standin settings")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    if args.num_drones:
        cfg = cfg.with_num_drones(args.num_drones)
    if args.id not in cfg.drone_ids:
        print(f"drone id {args.id} not in configured ids {cfg.drone_ids}", file=sys.stderr)
        return 2
    rclpy.init()
    node = SwarmAgentNode(cfg, args.id, radio_link=args.radio_link)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
