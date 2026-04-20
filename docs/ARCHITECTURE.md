# Architecture

## 경계 요약

| 레이어 | 위치 | 역할 |
|---|---|---|
| Newton Physics (Warp GPU) | 컨테이너 내부 Python 프로세스 | 물리 스텝, differentiable |
| rclpy bridge | 동일 프로세스 (`newton_bridge` 패키지) | DDS 토픽/서비스 ↔ Newton state |
| ROS 2 Jazzy | 호스트 (또는 다른 컨테이너) | 외부 custom controller |

단일 프로세스가 `newton.Model` 과 `rclpy.Node` 를 동시에 보유해서, 같은 GIL 아래에서 shared-memory 로 state 에 접근합니다. 별도 extension / sidechannel 레이어 없음.

## 통신 경로

```
┌─ Container (newton-bridge) ────────────────────────────────┐
│                                                            │
│   python -m newton_bridge                                  │
│     ├─ NewtonWorld      (src/newton_bridge/world.py)       │
│     │    ├─ ModelBuilder + parse_urdf/mjcf                 │
│     │    ├─ state_0, state_1 (double-buffer)               │
│     │    └─ solver.step (XPBD | MuJoCo | Featherstone)     │
│     │                                                      │
│     └─ SimBridgeNode    (src/newton_bridge/node.py)        │
│          ├─ pub  /clock         rosgraph_msgs/Clock        │
│          ├─ pub  /joint_states  sensor_msgs/JointState     │
│          ├─ sub  /joint_command sensor_msgs/JointState     │
│          └─ srv  /sim/reset     std_srvs/Trigger           │
│                                                            │
└─── network_mode: host, FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ───┘
                              │
                              ▼ (DDS UDPv4)
                         host ROS 2 graph
```

## Sync 모델

### freerun (default)

- sim 자체 루프가 `world.step()` 을 주도, `FREERUN_RATE=realtime` 이면 wall-clock 에 맞춰 `time.sleep` 으로 페이싱, `FREERUN_RATE=max` 면 가능한 빠르게.
- `/clock` 은 매 퍼블리시 주기마다 송출 (상태 퍼블리시와 함께).
- `/joint_states` 는 `robot.yaml: ros.publish_rate_hz` 기준 (기본 100Hz).
- `/joint_command` 는 latest-wins. 스텝 진입 직전에 한 번만 drive target 에 반영.
- 외부 컨트롤러가 `use_sim_time: true` 를 켜면 `/clock` 기준으로 타임스탬프 맞음.

### sync

- sim 은 **자동으로 step 하지 않음**. 외부가 `/joint_command` 를 publish 해야 1 step 진행.
- 한 번의 `/joint_command` 수신 내부 순서 (`SimBridgeNode._on_cmd` 안에서 인라인):
  1. 메시지 필드를 `_latest_cmd` 로 버퍼링.
  2. `_apply_latest_cmd()` — drive target 에 write.
  3. `world.step()` (internal substep 은 `sim.substeps` 수대로).
  4. `/joint_states` + `/clock` 퍼블리시.
  5. `sim.viewer_hz` 기준 누산이 차면 viewer 렌더 1프레임.
- 결정성(reproducibility) 우선. 다수 step 은 publish 를 N 번 반복 (`examples/controller_demo.py --mode sync` 참고).
- **Idle watchdog** — `/joint_command` 가 `ros.sync_timeout_ms` (기본 100ms) 동안 안 오면, main loop 가 현재 상태를 `/joint_states` 로 재퍼블리시. step 은 하지 않으므로 sim_time 은 멈춘 채, 구독자만 살아 있게 유지.
- **Render decoupling** — physics 500Hz 라도 `sim.viewer_hz=60` 이면 뷰어 draw 는 60Hz. `viewer_hz=0` 이면 매 step 렌더.
- `/sim/reset` 은 home_pose 로 복귀 + 상태 1회 퍼블리시 (두 모드 공통).

## 시간 모델

| 클럭 | 기본값 | 출처 |
|---|---|---|
| `physics_hz` | 400Hz | `robot.yaml: sim.physics_hz` |
| `substeps` | 1 | `robot.yaml: sim.substeps` (solver 안정성용) |
| `/clock` | publish_rate_hz 에 동기 | `newton_bridge.node` |
| `/joint_states` | 100Hz (freerun) / step 당 1회 + watchdog idle republish (sync) | `newton_bridge.node` |
| viewer 렌더 | `sim.viewer_hz` (기본 60Hz, `physics_hz` 와 독립) | `RenderTicker` in `ticks.py` |
| 호스트 `use_sim_time` | `/clock` 구독 | 호스트 launch/node 설정 |

## Robot pack 계약

`robots/<name>/robot.yaml` 한 파일로 pack 정의. `newton_bridge` 패키지는 로봇을 모른 채 pack 만 읽어 바인딩합니다. 에셋은 `robots/<name>/models/` 아래에 둡니다 (URDF / MJCF 구분 없이 통일).

```yaml
robot:
  source: urdf | mjcf
  source_rel: <path relative to pack dir>
  base_position: [x, y, z]

sim:
  physics_hz: <Hz>
  substeps: <int>
  solver: xpbd | mujoco | featherstone
  ground_plane: bool

joint_names: [...]           # ROS JointState 순서의 authoritative source
home_pose: { name: rad, ... }
drive:
  mode: position
  stiffness: float
  damping: float
ros:
  joint_states_topic: /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz: int
```

에셋(URDF/MJCF/STL)은 gitignore. `scripts/host/fetch_assets.sh` 가 외부에서 끌어옴. 새 pack 을 추가하거나 외부 `*_description` 패키지(URDF/xacro/MJCF)를 붙이는 절차는 [ROBOTS.md](ROBOTS.md) 참조.

## 왜 이 설계인가

- **단일 프로세스**: Newton Warp kernel 과 rclpy callback 이 같은 GIL 아래 있으므로 shared-memory 로 state 접근 — 별도 IPC 계층 없음.
- **표준 메시지만 사용**: 초기 스캐폴드는 `sensor_msgs/JointState`, `rosgraph_msgs/Clock`, `std_srvs/Trigger` 만으로 구성. 커스텀 msg 패키지 없음 → colcon build 없음 → 이미지 빌드 속도 빠름.
- **pack 계약**: `robots/<name>/robot.yaml` 만 바꾸면 로봇 swap 이 단순 env var 하나로 가능.

## 알려진 이슈

- **Newton API 안정성**: Newton 1.x 는 빠르게 움직입니다. [src/newton_bridge/world.py](../src/newton_bridge/world.py) 의 `# -- NEWTON API SURFACE --` 블록이 수정 포인트. API 변경 시 그 블록만 고치면 나머지는 재활용.
- **joint 이름 매핑**: Newton 의 `model.joint_name` / `model.joint_q_start` 는 importer 구현에 따라 URDF/MJCF 의 이름을 그대로 보존하거나 접두사를 붙일 수 있습니다. 미스매치 나면 `robot.yaml: joint_names` 를 실제 값에 맞춰 교정 (`scripts/container/verify.sh` 가 "joints in robot.yaml not found" 로 알려줌).
- **MuJoCo solver + URDF**: URDF 는 actuator 블록이 없어서 `SolverMuJoCo` 와 곧바로 안 맞을 수 있음. URDF 는 `SolverXPBD` / `SolverFeatherstone` 로 쓰기를 권장 (pack 에 기본값 반영됨).
