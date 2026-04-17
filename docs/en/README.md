# Docs (English summary)

The authoritative docs are Korean and live one level up
([SETUP.md](../SETUP.md), [ARCHITECTURE.md](../ARCHITECTURE.md),
[TOPICS.md](../TOPICS.md), [ROBOTS.md](../ROBOTS.md)). This file is a
brief English digest for non-Korean readers.

## What this repo is

`newton-bridge` is a **standalone Newton Physics ↔ ROS 2 Jazzy** bridge.
A single Python process (`newton_bridge` package) embeds a Newton solver
and an `rclpy` node that exposes the robot over standard ROS 2 topics and
services. The container runs with `network_mode: host` so the DDS
participant joins the host domain — `ros2 topic list` on the host sees
`/clock`, `/joint_states`, `/joint_command` directly.

## Sync modes

| `SYNC_MODE=` | Behavior | When to use |
|---|---|---|
| `freerun` (default) | Sim auto-steps at `FREERUN_RATE=realtime\|max` | Observation, loose-sync control, demos |
| `handshake` | Sim advances only on `/sim/step` service call | Deterministic RL rollout, repro tests |

## Topics & services

- **pub** `/clock` — `rosgraph_msgs/Clock`, every physics step.
- **pub** `/joint_states` — `sensor_msgs/JointState`, rate-limited in
  freerun, one-per-step in handshake.
- **sub** `/joint_command` — `sensor_msgs/JointState`, position targets
  (rad). Partial name match is tolerated.
- **srv** (handshake only) `/sim/step`, `/sim/reset` — `std_srvs/Trigger`.

## Robot packs

Each `robots/<name>/` has:

- `robot.yaml` — tracked in git; declares joints, home pose, drive gains,
  solver choice, ROS topic config.
- `models/` — gitignored; URDF or MJCF + meshes/assets fetched by
  `scripts/host/fetch_assets.sh`.

Switching robot: `ROBOT=franka ./scripts/host/run.sh sim`.

Adding a new robot: see [ROBOTS.md](../ROBOTS.md) (Korean) — the pack
contract (`robot.yaml` schema + `models/` layout) is stable.

## Quick start

```bash
./scripts/host/install.sh           # docker + compose + nvidia-container-toolkit
./scripts/host/fetch_assets.sh      # mujoco_menagerie + UR5e URDF
./scripts/host/build.sh             # docker image (5–15 min)
./scripts/host/run.sh verify        # in-container smoke test
./scripts/host/run.sh sim           # default: ur5e, freerun, headless
ENABLE_VIEWER=1 ./scripts/host/run.sh sim   # open Newton GL viewer
```

Host terminal (separate):

```bash
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 examples/controller_demo.py --mode freerun --robot ur5e
```
