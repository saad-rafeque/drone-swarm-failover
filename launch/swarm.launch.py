"""ROS 2 launch file: link emulator + logger + one swarm_agent process per drone.

The agents run as separate processes (one per drone, like onboard computers); PX4 SIH + MAVROS
are started beforehand by scripts/run_mission.py (simulation tooling, src/swarm_tools/sim_launch.py).

Usage (with ROS sourced and src/ on PYTHONPATH, e.g. via scripts/ros_env.sh):
  ros2 launch "<repo>/launch/swarm.launch.py" num_drones:=10 run_dir:=/path/to/run
Optional: loss_pct:=10 latency_ms:=150 jitter_ms:=30 (override config/swarm.yaml link_emulator)
          radio_ids:=1,3 (these drones' autopilot links are telemetry radios: their agents get --radio-link)
"""
from __future__ import annotations

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _processes(context, *args, **kwargs):
    n = int(LaunchConfiguration("num_drones").perform(context))
    first_id = int(LaunchConfiguration("first_id").perform(context))
    run_dir = LaunchConfiguration("run_dir").perform(context)
    py = sys.executable
    radio = {int(x) for x in LaunchConfiguration("radio_ids").perform(context).split(",") if x.strip()}
    emulator = [py, "-m", "swarm_tools.link_emulator", "--num-drones", str(n), "--run-dir", run_dir]
    for opt in ("loss_pct", "latency_ms", "jitter_ms"):
        value = LaunchConfiguration(opt).perform(context)
        if value:
            emulator += [f"--{opt.replace('_', '-')}", value]
    actions = [
        ExecuteProcess(cmd=emulator, name="link_emulator", output="log"),
        ExecuteProcess(cmd=[py, "-m", "swarm_tools.logger_node", "--num-drones", str(n), "--run-dir", run_dir],
                       name="swarm_logger", output="log"),
    ]
    for drone_id in range(first_id, first_id + n):
        agent = [py, "-m", "swarm_agent.ros_node", "--id", str(drone_id), "--num-drones", str(n)]
        if drone_id in radio:
            agent.append("--radio-link")
        actions.append(ExecuteProcess(cmd=agent, name=f"swarm_agent_{drone_id}", output="log"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", description="number of drones (ids first_id..)"),
        DeclareLaunchArgument("first_id", default_value="1"),
        DeclareLaunchArgument("run_dir", description="output folder for logs"),
        DeclareLaunchArgument("loss_pct", default_value=""),
        DeclareLaunchArgument("latency_ms", default_value=""),
        DeclareLaunchArgument("jitter_ms", default_value=""),
        DeclareLaunchArgument("radio_ids", default_value=""),
        OpaqueFunction(function=_processes),
    ])
