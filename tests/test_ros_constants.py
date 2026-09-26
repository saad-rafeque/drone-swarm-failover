"""Constants the ROS node relies on, checked against their sources (skipped without ROS/pymavlink)."""
from __future__ import annotations

import pytest


def test_mavlink_message_ids_match_pymavlink():
    common = pytest.importorskip("pymavlink.dialects.v20.common")
    ros_node = pytest.importorskip("swarm_agent.ros_node")  # needs rclpy + mavros_msgs
    for name, msg_id in ros_node.MAVLINK_MSG_IDS.items():
        assert getattr(common, f"MAVLINK_MSG_ID_{name}") == msg_id


def test_configured_streams_are_known(cfg):
    ros_node = pytest.importorskip("swarm_agent.ros_node")
    assert set(cfg.autopilot_streams_hz) <= set(ros_node.MAVLINK_MSG_IDS)
    assert cfg.autopilot_streams_hz["GLOBAL_POSITION_INT"] >= cfg.setpoints.rate_hz
