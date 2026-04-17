# Topic / Service Contract

모든 토픽은 표준 msg 타입만 사용 (커스텀 msg 패키지 없음).

## Topics

| Direction | Topic | Type | Rate | QoS | Note |
|---|---|---|---|---|---|
| pub | `/clock` | `rosgraph_msgs/Clock` | physics_hz 또는 publish 시점 | Reliable, depth=10 | 외부 노드는 `use_sim_time: true` 로 구독 |
| pub | `/joint_states` | `sensor_msgs/JointState` | `publish_rate_hz` (freerun) / per-step (handshake) | Reliable, depth=10 | `name` 순서는 `robot.yaml: joint_names` |
| sub | `/joint_command` | `sensor_msgs/JointState` | 외부 publish rate | Reliable, depth=10 | position targets (rad); 부분 name 매칭 OK |

### Joint conventions

- **단위**: position = radian (revolute). velocity / effort 는 읽기만 되며 현재 퍼블리시에는 position 만 채웁니다.
- **이름**: `robot.yaml: joint_names` 가 authoritative. 컨트롤러가 일부만 보내도 sim 은 매칭되는 것만 반영하고 나머지는 마지막 target 유지.
- **효과 시점**: freerun 에서는 다음 `world.step()` 직전에 반영. handshake 에서는 `/sim/step` 콜 시점의 latest-wins.

### `/joint_command` 예시

```bash
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint']
position: [0.5]
"
```

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

스크립트 예시: [scripts/controller_demo.py](../scripts/controller_demo.py).

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

지금은 arm-only (franka 7 DoF) 로 통일. gripper 필요해지면 `extra_topics:` 스키마 + 스트림 분기 로직을 `sim_node.py` 에 추가.

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
