# ROS 2 launch

`swarm.launch.py` starts the link emulator, the logger and one `swarm_agent` process per drone, each
like a separate onboard computer. PX4 SIH and MAVROS must already be running; `scripts/run_mission.py`
starts them (through `src/swarm_tools/sim_launch.py`) and then this launch file, so normally you run
`run_mission.py` rather than this file directly.

Direct use, with ROS 2 sourced and `src/` on `PYTHONPATH`:

```bash
scripts/ros_env.sh ros2 launch "$PWD/launch/swarm.launch.py" num_drones:=10 run_dir:=/tmp/swarm_run
```

Optional arguments override the link emulator settings in `config/swarm.yaml`: `loss_pct:=10`,
`latency_ms:=150`, `jitter_ms:=30`.
