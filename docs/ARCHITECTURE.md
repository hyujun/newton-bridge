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
│   python -m newton_bridge   (process)                      │
│   ┌── main thread ─────────────────────────────────────┐   │
│   │  NewtonWorld       (src/newton_bridge/world.py)    │   │
│   │   ├─ ModelBuilder + parse_urdf/mjcf                │   │
│   │   ├─ state_0, state_1 (solver double-buffer)       │   │
│   │   └─ solver.step (XPBD|MuJoCo|Featherstone|…)      │   │
│   │  SimBridgeNode     (src/newton_bridge/node.py)     │   │
│   │   ├─ pub  /clock           rosgraph_msgs/Clock     │   │
│   │   ├─ pub  /joint_states    sensor_msgs/JointState  │   │
│   │   ├─ pub  /sim/diagnostics diagnostic_msgs/...     │   │
│   │   ├─ sub  /joint_command   sensor_msgs/JointState  │   │
│   │   └─ srv  /sim/reset       std_srvs/Trigger        │   │
│   │  TelemetryRegistry + StatusLogger                  │   │
│   └────┬───────────────────────────────────────────────┘   │
│        │ StateSnapshot.publish(state, sim_time, telemetry) │
│        ▼ (lock-free double-buffer; state.assign() memcpy)  │
│   ┌── viewer daemon thread ────────────────────────────┐   │
│   │  ViewerThread     (src/newton_bridge/viewer_thread)│   │
│   │   ├─ RenderTicker (viewer_hz wall-clock)           │   │
│   │   ├─ build_viewer() inside thread → owns GL ctx    │   │
│   │   └─ begin_frame / log_state / end_frame           │   │
│   └────────────────────────────────────────────────────┘   │
│        ▲ closed_event / paused_event / step_q / reset_q    │
│        └── viewer→main signals (ESC, SPACE, '.', Reset btn)│
│                                                            │
└─── network_mode: host, RMW=rmw_cyclonedds_cpp (default) ───┘
                              │
                              ▼ (DDS / UDP)
                         host ROS 2 graph
```

### Thread 분리

물리/ROS 와 viewer 가 한 thread 에 있으면 viewer 의 GPU readback 이나 GL swap 이 sim step rate 를 직접 떨어뜨립니다 (특히 sync 모드에서 cmd 처리 latency 에 viewer 비용이 더해짐). 그래서 viewer 만 별도 daemon thread 에 분리하고, 둘 사이는 `StateSnapshot` 이중 버퍼로만 통신:

- main 은 step 마다 `snapshot.publish(state_0, sim_time, telemetry)` 를 호출 → 사용 중이 아닌 슬롯에 `state.assign()` (in-place GPU memcpy) 후 인덱스만 atomic 으로 swap. Lock 은 인덱스/메타 swap 짧은 구간에만.
- viewer thread 는 `consume()` 으로 슬롯을 빌리고, 들고 있는 동안 producer 는 다른 슬롯에 쓰므로 tear 없음. 다음 publish 까지 컨슈머가 못 따라가면 그냥 frame 을 skip.
- viewer thread 는 `build_viewer()` 를 자기 thread 내부에서 호출 — GL OpenGL context 가 viewer thread 소유여야 안전 (`ViewerGL` 은 main thread 가정이 깨졌을 때 segfault 보고 있음).
- 입력 채널 (ESC/창닫기/SPACE/`.`/Reset) 은 `threading.Event` + `queue.Queue(maxsize=1)` 로 main 에 전달. UI Reset 버튼은 직접 `world.reset()` 을 부르지 않고, queue → main 이 기존 `/sim/reset` 경로를 재사용해 reset 로직 일원화.

```
main:    [step] → snapshot.publish() ──┐
                                       ▼
                              [back buf state.assign(live)]
                                       │  swap front/back idx (lock 짧게)
                              ┌────────┴───────┐
                              │                │
viewer:  consume() → render → release()
```

### Telemetry

Viewer 가 별도 thread 가 되면 "viewer 가 그려지고 있다" 가 더 이상 "physics 가 살아 있다" 를 의미하지 않습니다. `TelemetryRegistry` 가 sliding-window rate meter 4개 (step/cmd/pub/render) 와 state 분류기 (INIT/IDLE/STALL/RUNNING) 를 묶어 1Hz 로 세 곳에 fan-out:
1. `StatusLogger` 가 stderr 에 status 라인 (STALL 시 WARN).
2. `/sim/diagnostics` (`diagnostic_msgs/DiagnosticArray`).
3. (PR#3 후속에서 viewer overlay 도 — GL stats 패널 / Rerun scalar chart)

자세한 분류 룰은 [CONFIGURATION.md §Telemetry & 진단](CONFIGURATION.md#telemetry--진단).

## Sync 모델

### freerun (default)

- sim 자체 루프가 `world.step()` 을 주도, `FREERUN_RATE=realtime` 이면 wall-clock 에 맞춰 `time.sleep` 으로 페이싱, `FREERUN_RATE=max` 면 가능한 빠르게.
- `/clock`, `/joint_states`, `/tf` 는 매 step (substeps 포함한 한 번의 `world.step()` 완료 직후) 함께 송출. rate-limit 없음.
- `/joint_command` 는 latest-wins. 스텝 진입 직전에 한 번만 drive target 에 반영.
- 외부 컨트롤러가 `use_sim_time: true` 를 켜면 `/clock` 기준으로 타임스탬프 맞음.

### sync

- sim 은 **자동으로 step 하지 않음**. 외부가 `/joint_command` 를 publish 해야 1 step 진행.
- 한 번의 `/joint_command` 수신 내부 순서 (`SimBridgeNode._on_cmd` 안에서 인라인):
  1. 메시지 필드를 `_latest_cmd` 로 버퍼링 + `telemetry.note_cmd()`.
  2. `_apply_latest_cmd()` — drive target 에 write.
  3. `world.step()` (internal substep 은 `sim.substeps` 수대로) + `telemetry.note_step()`.
  4. `/joint_states` + `/clock` 퍼블리시 + `snapshot.publish()` (viewer thread 가 다음 tick 에 그림).
- 결정성(reproducibility) 우선. 다수 step 은 publish 를 N 번 반복 (`examples/controller_demo.py --mode sync` 참고).
- **Idle watchdog** — `/joint_command` 가 `ros.sync_timeout_ms` (기본 100ms) 동안 안 오면, main loop 가 현재 상태를 `/joint_states` 로 재퍼블리시. step 은 하지 않으므로 sim_time 은 멈춘 채, 구독자만 살아 있게 유지. Telemetry 는 이 상태를 `state=IDLE` 로 분류 (정상).
- **Render decoupling** — 렌더는 viewer thread 가 `sim.viewer_hz` 로 wall-clock pacing. physics_hz / cmd rate 와 무관. `viewer_hz=0` 이면 throttle 없이 매 tick 렌더 (단, busy-loop 방지 위해 0.5ms `MIN_SLEEP` 강제).
- `/sim/reset` 은 home_pose 로 복귀 + 상태 1회 퍼블리시 + snapshot publish (두 모드 공통). UI Reset 버튼도 동일 경로로 dispatch — viewer thread 가 직접 reset 하지 않음.

## 시간 모델

| 클럭 | 기본값 | 출처 |
|---|---|---|
| `physics_hz` | 400Hz | `robot.yaml: sim.physics_hz` |
| `substeps` | 1 | `robot.yaml: sim.substeps` (solver 안정성용) |
| `/clock` | 매 physics step | `newton_bridge.node` |
| `/joint_states` | 매 physics step (sync 는 추가로 watchdog idle republish) | `newton_bridge.node` |
| viewer 렌더 | `sim.viewer_hz` (기본 60Hz, wall-clock, physics/커맨드 rate 와 독립; viewer thread 에서 pacing) | `RenderTicker` in `ticks.py` (driven by `ViewerThread`) |
| 상태 라인 | 1Hz ROS 로거 (`STATUS_LOG_HZ` 로 조절) | `StatusLogger` (rclpy logger 로 라우팅) |
| `/sim/diagnostics` | 1Hz (고정) | `SimBridgeNode._publish_diagnostics` |
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
