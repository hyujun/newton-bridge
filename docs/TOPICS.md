# Topic / Service Contract

모든 토픽은 표준 msg 타입만 사용 (커스텀 msg 패키지 없음).

## Topics

| Direction | Topic | Type | Rate | QoS | Note |
|---|---|---|---|---|---|
| pub | `/clock` | `rosgraph_msgs/Clock` | physics_hz 또는 publish 시점 | Reliable, depth=10 | 외부 노드는 `use_sim_time: true` 로 구독 |
| pub | `/joint_states` | `sensor_msgs/JointState` | `publish_rate_hz` (freerun) / per-step (sync) + `sync_timeout_ms` idle republish | Reliable, depth=10 | `name` 순서는 `robot.yaml: joint_names`. position/velocity/effort 3필드 모두 채움 |
| pub | `/tf` | `tf2_msgs/TFMessage` | `/joint_states` 와 동일 시점 | Reliable, depth=10 | `ros.publish_tf` (default `true`) 로 on/off. 각 body 를 `tf_root_frame` 의 child 로 퍼블리시 |
| sub | `/joint_command` | `sensor_msgs/JointState` | 외부 publish rate | Reliable, depth=10 | position/velocity/effort 필드 각각 드라이브 채널로 매핑 (아래 §) |
| sub | `/sim/set_gravity` | `geometry_msgs/Vector3` | latest-wins | Reliable, depth=10 | 런타임 gravity 변경; Phase 6a. 단위 m/s² |

### Joint conventions

- **단위**: position = radian (revolute), velocity = rad/s, effort = N·m.
- **이름**: `robot.yaml: joint_names` 가 authoritative. 컨트롤러가 일부만 보내도 sim 은 매칭되는 것만 반영하고 나머지는 마지막 target 유지.
- **효과 시점**: freerun 에서는 다음 `world.step()` 직전에 반영. sync 에서는 `/joint_command` 수신 콜백이 직접 1 step 을 실행 (publish = step trigger).

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

## Sensors (Phase 5)

`scene.yaml` 의 `sensors:` 블록으로 contact / IMU 센서를 선언하면, 각 센서마다 **표준 ROS msg** 토픽이 자동 등록됩니다. 커스텀 msg 패키지는 사용하지 않습니다 (결정 E).

```yaml
sensors:
  contact:
    - label: ee
      bodies: ["*wrist_3_link*"]   # fnmatch glob (body_label 매칭)
      # shapes: [...]               # 또는 shape 기준 (택일)
      measure_total: true            # default true (aggregate over matched bodies)
      topic: /contact_wrenches/ee    # optional (default: /contact_wrenches/<label>)
      frame_id: wrist_3_link
  imu:
    - label: base_imu
      sites: [base_site]             # MJCF 의 site 라벨 (URDF 는 site 가 없음)
      topic: /imu/base
      frame_id: base_link
```

| 센서 | 토픽 타입 | 내용 |
|---|---|---|
| contact | `geometry_msgs/WrenchStamped` | `force` = `SensorContact.total_force` 합산; `torque` 는 0 (Newton 1.1.0 은 vec3 만 제공) |
| imu | `sensor_msgs/Imu` | `linear_acceleration`, `angular_velocity` 채움; `orientation_covariance[0]=-1` (orientation 미제공) |

**주의**: `SensorIMU` 는 Newton의 **site** 개념이 필요하므로 MJCF 소스 pack 에서만 의미 있습니다. URDF pack 에 붙이려면 로더 레벨에서 `builder.add_site(...)` 를 수동으로 호출해야 합니다.

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

## Services

| Service | Type | Semantics |
|---|---|---|
| `/sim/reset` | `std_srvs/Trigger` | restore `home_pose`, zero velocities, publish state. 두 모드 모두 available |

`/sim/step` 은 제거되었습니다 — sync 모드에서 step 트리거 역할을 `/joint_command` publish 가 대신합니다.

### 사용 예

```bash
# 복귀 (freerun / sync 공통)
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"

# sync 모드에서 1 step 진행 (= /joint_command publish 1회)
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: [], position: [], velocity: [], effort: []}"
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

## Sync mode 요약

| | freerun | sync |
|---|---|---|
| sim step | 자율 | `/joint_command` publish 수신 시 |
| `/clock` publish | publish 시점마다 | 매 step + watchdog idle republish |
| `/joint_states` | `publish_rate_hz` | 매 step + `sync_timeout_ms` idle republish |
| 뷰어 렌더 | `sim.viewer_hz` 로 제한 | `sim.viewer_hz` 로 제한 |
| 사용 케이스 | 관찰/로깅, loose-sync 제어 | RL rollout, deterministic 테스트 |

env var 로 선택: `SYNC_MODE=freerun` (기본) | `SYNC_MODE=sync`. legacy `handshake` 값은 deprecation 경고 후 `sync` 로 treat.
