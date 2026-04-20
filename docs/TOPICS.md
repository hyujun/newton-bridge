# Topic / Service Contract

모든 토픽은 표준 msg 타입만 사용 (커스텀 msg 패키지 없음).

## Topics

| Direction | Topic | Type | Rate | QoS | Note |
|---|---|---|---|---|---|
| pub | `/clock` | `rosgraph_msgs/Clock` | physics_hz 또는 publish 시점 | Reliable, depth=10 | 외부 노드는 `use_sim_time: true` 로 구독 |
| pub | `/joint_states` | `sensor_msgs/JointState` | `publish_rate_hz` (freerun) / per-step (handshake) | Reliable, depth=10 | `name` 순서는 `robot.yaml: joint_names`. position/velocity/effort 3필드 모두 채움 |
| pub | `/tf` | `tf2_msgs/TFMessage` | `/joint_states` 와 동일 시점 | Reliable, depth=10 | `ros.publish_tf` (default `true`) 로 on/off. 각 body 를 `tf_root_frame` 의 child 로 퍼블리시 |
| sub | `/joint_command` | `sensor_msgs/JointState` | 외부 publish rate | Reliable, depth=10 | position/velocity/effort 필드 각각 드라이브 채널로 매핑 (아래 §) |

### Joint conventions

- **단위**: position = radian (revolute), velocity = rad/s, effort = N·m.
- **이름**: `robot.yaml: joint_names` 가 authoritative. 컨트롤러가 일부만 보내도 sim 은 매칭되는 것만 반영하고 나머지는 마지막 target 유지.
- **효과 시점**: freerun 에서는 다음 `world.step()` 직전에 반영. handshake 에서는 `/sim/step` 콜 시점의 latest-wins.

### `/joint_command` 4채널 해석

`sensor_msgs/JointState` 의 `position` / `velocity` / `effort` 필드를 각각의 제어 채널로 매핑합니다. **비어 있는 배열은 "건드리지 않음"** 을 의미 (길이가 `name`과 일치해야 반영).

| msg 필드 | Newton control | 작동 조건 |
|---|---|---|
| `position` | `control.joint_target_pos` | pack 의 joint mode 가 `position` 또는 `position_velocity` 일 때 PD 로 추종 |
| `velocity` | `control.joint_target_vel` | mode 가 `velocity` 또는 `position_velocity` 일 때 PD 의 vel setpoint 로 사용 |
| `effort` | `control.joint_f` | mode 가 `effort` 일 때 solver 가 직접 해당 토크를 적용 |

pack 의 drive mode 는 joint 별로 다르게 설정할 수 있습니다 ([ROBOTS.md — per-joint drive](ROBOTS.md) 참조). 메시지에 세 필드를 동시에 실어 보내도 안전 — 해당 joint 의 mode 가 그 중 하나만 수용합니다.

### `/joint_command` 예시

```bash
# position control
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint']
position: [0.5]
"

# velocity control
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint']
velocity: [0.2]
"

# effort (torque) control
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint']
effort: [5.0]
"
```

## `/tf` (Phase 4)

각 body 의 world-frame pose 를 `tf_root_frame` (default `world`) → `<body_label>` transform 으로 퍼블리시합니다. publish rate 은 `/joint_states` 와 동일 시점 (같은 `header.stamp`).

pack yaml `ros:` 필드:

```yaml
ros:
  publish_tf: true           # default true (결정 D)
  tf_root_frame: world       # TF 트리의 루트 프레임 이름
  publish_frames: []         # [] = 전체 (root 제외), 또는 화이트리스트 ["tool0", "wrist_3_link"]
```

**주의**: Newton 이 world-frame pose 만 제공하므로, 현재 구현은 킨매틱 트리가 아닌 **평탄한 world → each-body** 구조를 퍼블리시합니다. `robot_state_publisher` 와 호환되는 parent→child 체인은 URDF 를 재파싱해야 하므로 별도 phase 대상.

## Services (handshake mode only)

| Service | Type | Semantics |
|---|---|---|
| `/sim/step` | `std_srvs/Trigger` | apply latest `/joint_command` → step 1 → publish state → return `success=true, message='sim_time=...'` |
| `/sim/reset` | `std_srvs/Trigger` | restore `home_pose`, zero velocities, publish state |

### 사용 예

```bash
# 한 스텝
ros2 service call /sim/step std_srvs/srv/Trigger "{}"

# 복귀
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
```

스크립트 예시: [examples/controller_demo.py](../examples/controller_demo.py).

## Extension: gripper topic (미구현)

Franka 같은 로봇의 gripper 를 arm joint 와 분리해서 제어하고 싶을 때, 현재 스키마를 다음과 같이 확장할 예정 (Phase 2):

```yaml
# robots/franka_gripper/robot.yaml
ros:
  joint_states_topic:   /joint_states
  joint_command_topic:  /joint_command
  extra_topics:
    - name: /gripper_command
      type: std_msgs/Float64          # width in m
      target_joints: [finger_joint1, finger_joint2]  # mimic coupled
```

지금은 arm-only (franka 7 DoF) 로 통일. gripper 필요해지면 `extra_topics:` 스키마 + 스트림 분기 로직을 [src/newton_bridge/node.py](../src/newton_bridge/node.py) 에 추가.

## Extension: per-step srv (미구현)

`std_srvs/Trigger` 는 인자가 없어서 "N-step per call" 이 불가. 필요해지면 `newton_bridge_msgs/srv/SimStep.srv` 를 도입 (uint32 steps + optional command, response에 state 포함). 트리거는 [ARCHITECTURE.md](ARCHITECTURE.md#extension-multi-step-srv-미구현) 참조.

## Sync mode 요약

| | freerun | handshake |
|---|---|---|
| sim step | 자율 | `/sim/step` 콜 |
| `/clock` publish | publish 시점마다 | 매 step |
| `/joint_states` | `publish_rate_hz` | 매 step |
| 사용 케이스 | 관찰/로깅, loose-sync 제어 | RL rollout, deterministic 테스트 |

env var 로 선택: `SYNC_MODE=freerun` (기본) | `SYNC_MODE=handshake`.
