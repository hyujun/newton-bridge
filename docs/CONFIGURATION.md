# Configuration Reference

env var · `robot.yaml` / `scene.yaml` 스키마 · ROS 토픽/서비스 계약 · viewer 모드를 한 곳에. 새 pack 추가 절차는 [ROBOTS.md](ROBOTS.md), 일상 워크플로우는 [USAGE.md](USAGE.md).

---

## Environment Variables

우선순위: `run.sh` 명령줄 export > `.env` > compose.yml 기본값 > 애플리케이션 fallback.

### 로봇 pack

| 변수 | 기본 | 출처 | 설명 |
|---|---|---|---|
| `ROBOT` | `ur5e` | `run.sh` | 컨테이너 경로 `/workspace/robots/$ROBOT` 로 확장 |
| `ROBOT_PACK` | `/workspace/robots/ur5e` | `__main__.py` | 컨테이너 내 pack 경로 (직접 지정하면 `ROBOT` 무시) |
| `EXTERNAL_PACK_HOST` | (unset) | `run.sh` | 호스트의 외부 robot_description 폴더 절대경로. 설정되면 `/workspace/external_pack` 으로 ro 마운트되고 `ROBOT_PACK` 도 자동으로 거기를 가리킴. URDF only — 자세한 절차는 [ROBOTS.md §Path D](ROBOTS.md#path-d-호스트의-외부-폴더-사용-urdf-only) |

### Sync + pacing

| 변수 | 기본 | 허용값 | 설명 |
|---|---|---|---|
| `SYNC_MODE` | `freerun` | `freerun` \| `sync` | sim step trigger 방식. `sync` 는 `/joint_command` 수신마다 1 step. legacy `handshake` 는 deprecation 경고 후 `sync` 로 treat |
| `FREERUN_RATE` | `realtime` | `realtime` \| `max` | freerun 전용. `max` 는 wall-clock sleep 제거 |

### DDS / ROS 2

**컨테이너와 호스트가 동일 값이어야 함**. 불일치 시 silent discovery failure.

| 변수 | 기본 | 설명 |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | DDS domain (0~101) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | `rmw_cyclonedds_cpp` (기본, 검증됨) \| `rmw_fastrtps_cpp` (opt-in). 이미지에 두 패키지 모두 설치됨 |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` (FastDDS 선택 시) | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 일 때 `run.sh` 가 자동으로 export. SHM 은 root/user UID 경계에서 깨지므로 FastDDS 에선 필수. Cyclone 에선 무시됨 |
| `CYCLONEDDS_URI` | (unset) | Cyclone DDS XML 설정 파일 경로. 멀티캐스트 차단된 네트워크 / 특정 인터페이스 고정 등 고급 케이스에서만 필요 ([TROUBLESHOOTING.md](TROUBLESHOOTING.md) 참조) |

### Viewer

| 변수 | 기본 | 허용값 | 설명 |
|---|---|---|---|
| `VIEWER` | `rerun` | `rerun` \| `gl` \| `usd` \| `file` \| `null` \| `none` | [§Viewer](#viewer-모드) 참조 |
| `VIEWER_WIDTH` | `1280` | int | `gl` 창 너비 |
| `VIEWER_HEIGHT` | `720` | int | `gl` 창 높이 |
| `VIEWER_FPS` | `60` | int | `usd` 녹화 frame rate |
| `VIEWER_UP_AXIS` | `Z` | `X` \| `Y` \| `Z` | `usd` 녹화 up axis |
| `VIEWER_OUTPUT_PATH` | `workspace/runs/sim_<ts>.<ext>` | path | `usd` / `file` 녹화 경로 오버라이드 |
| `VIEWER_OUTPUT_DIR` | `/workspace/workspace/runs` | dir | 타임스탬프 파일의 부모 디렉토리 |
| `RERUN_APP_ID` | `newton_bridge` | str | Rerun app id |
| `RERUN_WEB_PORT` | `9090` | int | Rerun 웹 뷰어 포트 (호스트 `:9090`) |
| `RERUN_GRPC_PORT` | `9876` | int | Rerun gRPC 포트 |
| `RERUN_RECORD_TO` | (unset) | path | `.rrd` 녹화 경로. 설정 시 웹 뷰어와 동시 녹화 |
| `ENABLE_VIEWER` | (deprecated) | — | 설정되면 에러. `VIEWER` 로 대체 |

### Telemetry / 진단

| 변수 | 기본 | 허용값 | 설명 |
|---|---|---|---|
| `STATUS_LOG_HZ` | `1.0` | float | 1Hz 기본 상태 라인 cadence (ROS 로거로 emit). `0` 이면 비활성. STALL 상태일 때 자동으로 WARN 레벨로 승격됩니다. |
| `LOG_LEVEL` | `INFO` | `DEBUG`/`INFO`/`WARNING`/`ERROR` | stdlib `logging` root 레벨. `DEBUG` 로 내리면 viewer thread 의 진단 메시지까지 보입니다. |

`/sim/diagnostics` (`diagnostic_msgs/DiagnosticArray`) 는 항상 1Hz 로 게시되며, 비활성화 환경 변수는 없습니다. 자세한 토픽 내용은 [§Telemetry & 진단](#telemetry--진단) 참조.

### GPU / X11

| 변수 | 기본 | 설명 |
|---|---|---|
| `NVIDIA_VISIBLE_DEVICES` | `all` | `0` / `0,1` / `GPU-<uuid>` 로 격리 |
| `NVIDIA_DRIVER_CAPABILITIES` | `compute,utility,graphics,video,display` | GL 렌더링 필요 |
| `DISPLAY` | `:0` | `gl` 뷰어용 X 소켓 |
| `XAUTHORITY` | `~/.Xauthority` | `gl` 뷰어 X 인증 |

### UID 매칭

| 변수 | 기본 | 설명 |
|---|---|---|
| `HOST_UID` | `$(id -u)` | 컨테이너 유저 UID. `sudo` 로 run.sh 돌리면 0 이 되니 주의 |
| `HOST_GID` | `$(id -g)` | 컨테이너 유저 GID |

### Jupyter

| 변수 | 기본 | 설명 |
|---|---|---|
| `JUPYTER_TOKEN` | `newton` | `run.sh jupyter` 의 `--ServerApp.token` |

전체 기본값 스냅샷: [`.env.example`](../.env.example). 복사해서 `.env` 로 저장하면 `docker compose` 가 자동 로드.

---

## Pack YAML 스키마

`robots/<name>/` (또는 `EXTERNAL_PACK_HOST` 의 호스트 폴더) 에 다음 중 하나:
- `scene.yaml` — canonical (멀티-articulation 지원)
- `robot.yaml` — legacy, loader 가 scene 으로 자동 승격

로더 동작은 [src/newton_bridge/robot_pack.py](../src/newton_bridge/robot_pack.py) 참조. 승격 시 `robot.yaml` 의 필드는 한 world 한 articulation (`label=<pack_dir_name>`) 의 scene 으로 1:1 변환됩니다.

### `scene.yaml` (canonical)

```yaml
sim:
  physics_hz: 400                    # 물리 step rate (Hz). 내부 적분은 substeps 분할
  substeps: 1                        # step 당 solver sub-iteration
  solver: mujoco                     # xpbd | mujoco | featherstone | semi_implicit | style3d | vbd
  solver_params:                     # solver ctor 에 **kwargs 로 forward
    iterations: 100                  # (XPBD) solver iterations
    angular_damping: 0.1             # solver 별 키/값은 Newton 문서 참조
  ground_plane: true                 # add_ground_plane()
  gravity: [0.0, 0.0, -9.81]         # m/s², 월드 기본값
  viewer_hz: 60                      # 뷰어 렌더 rate (wall-clock). 0 이면 throttle 없음.
                                     # physics_hz / 커맨드 rate 와 무관 — 500Hz 물리 또는
                                     # sync 모드 저주기 커맨드여도 항상 초당 60프레임 목표

worlds:
  - label: env0                      # 고유 label (현재 1개만 허용)
    gravity: [0.0, 0.0, -9.81]       # optional, sim.gravity override
    articulations:
      - label: arm                   # 고유, ros.primary_articulation 이 가리킴
        source: urdf                 # urdf | xacro | mjcf
        source_rel: models/ur5e.urdf # pack dir 기준 상대경로
        source_args:                 # xacro 전용. xacro.process_file 의 mappings= 로 전달.
          ur_type: ur5e              #   모든 값은 문자열로 캐스팅됨.
          name: ur5e
        xform:
          pos: [0.0, 0.0, 0.0]       # 월드 좌표
          rot: [0.0, 0.0, 0.0, 1.0]  # quaternion xyzw
        articulation_pattern: "*"    # ArticulationView fnmatch glob
        joint_names:                 # ROS /joint_states 노출 순서 (authoritative)
          - shoulder_pan_joint
          - shoulder_lift_joint
          # ...
        home_pose:                   # reset 시 복귀, rad
          shoulder_pan_joint: 0.0
          shoulder_lift_joint: -1.5708
        drive:                       # 전 DOF 의 기본 drive
          mode: position             # position | velocity | effort | position_velocity | none
          stiffness: 10000.0         # PD Kp (target_ke)
          damping: 100.0             # PD Kd (target_kd)
        joints:                      # per-joint override (optional)
          shoulder_pan_joint:
            drive: {mode: velocity, damping: 5.0}
            effort_limit: 120.0      # N·m
            velocity_limit: 3.14     # rad/s
            armature: 0.01           # 관성 부가
            friction: 0.1            # Coulomb 마찰
            limit_ke: 10000.0        # joint limit penalty Kp
            limit_kd: 100.0          # joint limit penalty Kd
        softbodies:                  # optional, per-articulation (in progress)
          - name: left_fingertip_pad
            asset_rel: softbody/pad.npz   # pack-relative .npz (vertices + tet_indices)
            pos: [0.0, 0.0, 0.0]          # 월드 spawn 위치 (default 원점)
            rot: [0.0, 0.0, 0.0, 1.0]     # quaternion xyzw, default identity
            scale: 1.0
            material:
              density: 100.0
              k_mu:     1.0e6             # FEM shear modulus
              k_lambda: 1.0e6             # FEM Lamé λ
              k_damp:   1.0e-6
            particle_radius: 0.003        # optional
            attach:                       # required — 파티클을 link 에 핀
              body: left_fingertip        # URDF/MJCF link name
              vertex_indices: [0, 1, 2]   # mesh vertex 인덱스 (mass=0 으로 고정)
            contact:                      # optional, 마지막 spec 이 winner (Newton 1.1.0 글로벌)
              ke: 2.0e6
              kd: 1.0e-7
              mu: 0.5

sensors:                             # optional
  contact:
    - label: ee
      bodies: ["*wrist_3_link*"]     # fnmatch, body_label 매칭
      measure_total: true
      topic: /contact_wrenches/ee    # default /contact_wrenches/<label>
      frame_id: wrist_3_link
  imu:
    - label: base_imu
      sites: [base_site]             # MJCF site label (URDF 는 수동 add_site 필요)
      topic: /imu/base               # default /imu/<label>
      frame_id: base_link

ros:
  primary_articulation: arm          # /joint_states 에 퍼블리시될 articulation label
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
  sync_timeout_ms:     100           # sync 모드 전용. /joint_command 가 이 시간 동안
                                     # 안 오면 현재 상태를 /joint_states 로 재퍼블리시
                                     # (step 없음). 구독자가 굶지 않게 하기 위함
  publish_tf:          true          # /tf on/off (loader default true; pack 별로 false 도 가능)
  tf_root_frame:       world         # /tf 트리 루트 프레임
  publish_frames:      []            # [] = 전체 (root 제외), 또는 whitelist
```

### `robot.yaml` (legacy)

로더가 scene 으로 승격하므로 동일한 필드를 flat 하게 놓으면 됩니다:

```yaml
robot:
  source: urdf                       # urdf | xacro | mjcf
  source_rel: models/ur5e.urdf       # xacro 일 땐 `.xacro` 파일 경로
  source_args:                       # xacro 전용, 미지정 시 빈 dict
    ur_type: ur5e
  base_position: [0.0, 0.0, 0.0]

sim:
  physics_hz: 400
  substeps: 1
  solver: mujoco
  ground_plane: true

joint_names:
  - shoulder_pan_joint
  # ...

home_pose:
  shoulder_pan_joint: 0.0

drive:
  mode: position
  stiffness: 10000.0
  damping: 100.0

joints:                              # per-joint override (optional)
  shoulder_pan_joint:
    drive: {mode: velocity, damping: 5.0}
    effort_limit: 150.0

articulation_pattern: "*"            # optional, default "*"

softbodies: []                       # optional, scene.yaml 의 articulation.softbodies 와 동일 스키마

ros:
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
```

승격 규칙 ([`_promote_robot_yaml`](../src/newton_bridge/robot_pack.py#L51)):
- `robot.base_position` → articulation `xform.pos`
- `joint_names`/`home_pose`/`drive`/`joints`/`articulation_pattern`/`softbodies` → 단일 articulation 에 그대로
- `sim.gravity` → world gravity
- `sensors:` 있으면 scene 로 carry-through
- `ros.primary_articulation` 기본값 = `<pack_dir 이름>` (예: `ur5e`)

### Softbody 모드 (in progress)

`softbodies:` 가 비어있지 않으면 NewtonWorld 가 hybrid 모드로 빌드됩니다 — rigid solver (`sim.solver`) 옆에 별도 `SolverVBD` 와 `CollisionPipeline` 이 함께 생성되고, 각 spec 의 `.npz` (`vertices`, `tet_indices`) 가 `builder.add_soft_mesh` 로 추가됩니다. `attach.vertex_indices` 의 파티클은 `model.particle_mass=0` 으로 핀 됩니다.

현재 PR2 시점에서는 **build path + attach metadata 까지만** 동작합니다. 매 substep 에서 핀 파티클을 link transform 으로 따라가게 하는 warp kernel 과 hybrid step 순서 (`rigid.step → attach kernel → soft.step`) 는 PR3 에서 추가됩니다. 그때까지 softbodies 가 있어도 step kernel 은 rigid-only.

---

## Solver 선택 매트릭스

| 소스 | 권장 solver | 이유 |
|---|---|---|
| URDF (actuator 없음) | `mujoco` 또는 `xpbd` / `featherstone` | MuJoCo 가 빌더-레벨 gain 을 존중, URDF 에도 잘 맞음 (실측 Newton 1.1.0 에서 URDF + `xpbd` 는 drive 가 joint 에 도달 안 함) |
| MJCF (with `<actuator>`) | `mujoco` | actuator 블록의 gain 이 solver 로 그대로 전달됨 |
| MJCF (no `<actuator>`) | `xpbd` / `featherstone` | `drive.stiffness/damping` 이 사용됨 |

실제 pack 들:
- `ur5e` — xacro → URDF + `mujoco`. `$(find ur_description)` 은 apt `ros-jazzy-ur-description` 이 제공
- `franka` — MJCF + `mujoco` (`panda.xml` 에 actuator 포함)
- `kuka_iiwa_14` — MJCF + `mujoco`

불일치는 `./scripts/host/run.sh verify` 섹션 6 에서 잡힘.

---

## Drive Mode 의미

| Mode | `/joint_command` 에서 사용되는 필드 | Newton 동작 |
|---|---|---|
| `position` | `position` | `control.joint_target_pos` 추종 (PD) |
| `velocity` | `velocity` | `control.joint_target_vel` 추종 (PD) |
| `effort` | `effort` | `control.joint_f` 직접 토크 적용 |
| `position_velocity` | `position` + `velocity` | position PD, velocity setpoint 병행 |
| `none` | — | 비구동 joint (센서 용) |

Per-joint `joints.<name>.drive.mode` 로 섞을 수 있음. 메시지에 세 필드 다 실어도 안전 — joint 의 mode 가 그 중 하나만 수용.

---

## ROS Topics & Services

모든 토픽은 표준 msg 타입만 사용 (커스텀 msg 패키지 없음).

### Topics

| Direction | Topic | Type | Rate | QoS | Note |
|---|---|---|---|---|---|
| pub | `/clock` | `rosgraph_msgs/Clock` | 매 physics step | Reliable, depth=10 | 외부 노드는 `use_sim_time: true` 로 구독 |
| pub | `/joint_states` | `sensor_msgs/JointState` | 매 physics step (sync 는 추가로 `sync_timeout_ms` idle republish) | Reliable, depth=10 | `name` 순서는 pack 의 `joint_names`. position/velocity/effort 3필드 모두 채움 |
| pub | `/tf` | `tf2_msgs/TFMessage` | `/joint_states` 와 동일 시점 | Reliable, depth=10 | yaml `ros.publish_tf` / CLI / env / `/sim/set_publish_tf` 서비스로 on/off — 자세한 우선순위는 §/tf 설정. 각 body 를 `tf_root_frame` 의 child 로 퍼블리시 |
| sub | `/joint_command` | `sensor_msgs/JointState` | 외부 publish rate | Reliable, depth=10 | position/velocity/effort 필드 각각 드라이브 채널로 매핑 (위 §Drive Mode) |
| sub | `/sim/set_gravity` | `geometry_msgs/Vector3` | latest-wins | Reliable, depth=10 | 런타임 gravity 변경. 단위 m/s² |

### Services

| Service | Type | Semantics |
|---|---|---|
| `/sim/reset` | `std_srvs/Trigger` | restore `home_pose`, zero velocities, publish state. 두 모드 모두 available |
| `/sim/set_publish_tf` | `std_srvs/SetBool` | `/tf` 퍼블리싱 런타임 on/off. publisher 자체는 항상 살아 있고 emit 만 게이트 — QoS/discovery 영향 없음 |

`/sim/step` 은 제거되었습니다 — sync 모드에서 step 트리거 역할을 `/joint_command` publish 가 대신합니다.

### Joint conventions

- **단위**: position = radian (revolute), velocity = rad/s, effort = N·m.
- **이름**: pack 의 `joint_names` 가 authoritative. 컨트롤러가 일부만 보내도 sim 은 매칭되는 것만 반영하고 나머지는 마지막 target 유지.
- **효과 시점**: freerun 에서는 다음 `world.step()` 직전에 반영. sync 에서는 `/joint_command` 수신 콜백이 직접 1 step 을 실행 (publish = step trigger).
- **빈 배열의 의미**: `position` / `velocity` / `effort` 중 비어 있는 배열 = "이 채널 건드리지 않음". 길이가 `name` 과 일치해야 반영.

### `/joint_command` 예시

```bash
# position 한 joint
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint'], position: [0.5]}"

# velocity 한 joint (pack 이 velocity 모드일 때)
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint'], velocity: [0.2]}"

# effort (torque) 한 joint
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint'], effort: [5.0]}"

# sync 모드에서 빈 publish = "현재 target 그대로 1 step 진행"
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: [], position: [], velocity: [], effort: []}"

# home 복귀 (freerun / sync 공통)
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
```

### Sync mode 요약

| | freerun | sync |
|---|---|---|
| sim step | 자율 | `/joint_command` publish 수신 시 |
| `/clock` publish | 매 step | 매 step + watchdog idle republish |
| `/joint_states` | 매 step | 매 step + `sync_timeout_ms` idle republish |
| 뷰어 렌더 | `sim.viewer_hz` 로 제한, **viewer thread 가 독립적으로 pacing** | 동일 — sync 모드여도 viewer 는 viewer_hz 로 계속 그림 (idle 일 땐 마지막 snapshot 보여줌) |
| 사용 케이스 | 관찰/로깅, loose-sync 제어 | RL rollout, deterministic 테스트 |

> **Thread 분리 (2026-05 update)**: 물리/ROS 는 main thread, 렌더링은 별도 daemon thread (`viewer_thread.ViewerThread`). main 은 step 마다 `StateSnapshot.publish()` (이중 버퍼 GPU memcpy) 만 호출하고 즉시 다음 작업으로 넘어갑니다. 느린 viewer 가 step rate 를 떨어뜨리지 못하며, sync 모드에서 cmd 가 안 들어와도 viewer 는 viewer_hz 로 계속 마지막 frame 을 grab/render 합니다. 자세한 설계 노트는 [ARCHITECTURE.md](ARCHITECTURE.md) 참조.

---

## Sensor 설정

`pack.sensors.contact[]`:

| 필드 | 타입 | 기본 | 설명 |
|---|---|---|---|
| `label` | str | (required) | 고유 이름 |
| `bodies` | list[str] | — | fnmatch glob, body_label 매칭. `shapes` 와 택일 |
| `shapes` | list[str] | — | shape 기준. `bodies` 와 택일 |
| `measure_total` | bool | `true` | 매칭 body 들의 총 force 합 |
| `topic` | str | `/contact_wrenches/<label>` | 퍼블리시 토픽 |
| `frame_id` | str | `world` | msg.header.frame_id |

`pack.sensors.imu[]`:

| 필드 | 타입 | 설명 |
|---|---|---|
| `label` | str | 고유 이름 |
| `site` / `sites` | str / list[str] | MJCF site label (URDF 는 수동 `builder.add_site` 필요) |
| `topic` | str | `/imu/<label>` 기본 |
| `frame_id` | str | `world` 기본 |

| 센서 | 토픽 타입 | 내용 |
|---|---|---|
| contact | `geometry_msgs/WrenchStamped` | `force` = `SensorContact.total_force` 합산; `torque` 는 0 (Newton 1.1.0 은 vec3 만 제공) |
| imu | `sensor_msgs/Imu` | `linear_acceleration`, `angular_velocity` 채움; `orientation_covariance[0]=-1` (orientation 미제공) |

**주의**: `SensorIMU` 는 Newton의 **site** 개념이 필요하므로 MJCF 소스 pack 에서만 의미 있습니다. URDF pack 에 붙이려면 로더 레벨에서 `builder.add_site(...)` 를 수동으로 호출해야 합니다.

---

## Telemetry & 진단

Viewer 가 별도 thread 에서 도는 환경에서 "viewer 가 그려지고 있다 ≠ physics 가 살아 있다" 입니다. 그래서 모든 채널의 rate / 상태를 명시적으로 노출합니다. 구현은 [src/newton_bridge/telemetry.py](../src/newton_bridge/telemetry.py).

### 메트릭

| 메트릭 | 의미 |
|---|---|
| `step_hz` | physics step rate (1초 슬라이딩 윈도) |
| `rt_factor` | `step_hz × physics_dt` — 1.0 이면 realtime, <1 이면 sim 이 wall-clock 보다 느림 |
| `cmd_hz` | `/joint_command` 수신 rate |
| `pub_hz` | `/joint_states` publish rate |
| `render_hz` | viewer thread 가 실제로 그린 frame rate |
| `time_since_cmd` | 마지막 `/joint_command` 후 경과 (sec) |
| `time_since_step` | 마지막 step 후 경과 (sec) |
| `state` | `INIT` / `RUNNING` / `IDLE` / `STALL` |

### state 분류

| state | 조건 |
|---|---|
| `INIT` | step 이 한 번도 없음 |
| `IDLE` | sync 모드 + `time_since_cmd > sync_timeout_s` + `cmd_hz==0`. 정상 대기 |
| `STALL` | `time_since_step > max(0.5s, 50 × physics_dt)` 또는 sync 모드에서 cmd_hz>0 인데 step_hz==0 이 1초 넘게 지속. 비정상 |
| `RUNNING` | 그 외 |

### 출력 채널 (모두 기본 ON)

1. **Terminal** (1Hz, ROS 로거 경유) — `STATUS_LOG_HZ` 로 cadence 조절. STALL 일 때 자동으로 WARN 레벨로 emit:
   ```
   [INFO] [newton_bridge]: [newton_bridge] sim 12.34s | step 240Hz (rt 1.00) | cmd 100Hz | pub 100Hz | render 60Hz | state=RUNNING
   ```
   안 보이면 `LOG_LEVEL` (root stdlib logger) 가 `INFO` 위인지, 또는 `STATUS_LOG_HZ=0` 으로 꺼져있는지 확인하세요.
2. **Viewer overlay**
   - **GL** (`VIEWER=gl`): imgui 통계 패널에 sim/step/cmd/pub/render Hz + state 6줄. STALL 빨강 / IDLE 노랑 / RUNNING 초록 컬러 코딩.
   - **Rerun** (`VIEWER=rerun`): `telemetry/{step,cmd,pub,render}_hz` 와 `telemetry/realtime_factor` scalar 시계열. 웹 UI 의 timeseries 패널에 자동 표시.
   - 다른 모드 (`usd`/`file`/`null`/`none`): 오버레이 없음 (지원 surface 없음).
3. **ROS topic** `/sim/diagnostics` (`diagnostic_msgs/DiagnosticArray`, 1Hz) — 위 메트릭 전부를 `KeyValue` 로. STALL 시 `level=WARN`. 확인:
   ```bash
   ros2 topic echo /sim/diagnostics
   ```

---

## /tf 설정

`/tf` 퍼블리싱은 **3가지 레이어**로 토글 가능. 우선순위 (높음 → 낮음):

1. **런타임 서비스** `/sim/set_publish_tf` (`std_srvs/SetBool`) — 시뮬 실행 중 on/off
   ```bash
   ros2 service call /sim/set_publish_tf std_srvs/srv/SetBool "{data: false}"  # OFF
   ros2 service call /sim/set_publish_tf std_srvs/srv/SetBool "{data: true}"   # ON
   ```
2. **CLI 플래그** `--publish-tf` / `--no-publish-tf` — 시작 시점 override
   ```bash
   python -m newton_bridge --no-publish-tf
   ```
3. **환경 변수** `NEWTON_BRIDGE_PUBLISH_TF=0` (또는 `1/true/false/on/off`)
4. **`ros.publish_tf`** (yaml, 기본 `true`)

publisher 객체는 항상 생성되어 토픽이 `ros2 topic list` 에 항상 보임 — 토글은 emit-side gate 라 QoS/discovery 매칭이 흔들리지 않음. 끄면 `/tf` 페이로드 작성과 publish 호출 자체를 skip 하므로 CPU 절감 효과는 그대로.

`ros.publish_frames`:
- `[]` (기본) — 루트(`tf_root_frame`) 를 제외한 **모든** body 퍼블리시
- `["tool0", "wrist_3_link"]` — whitelist (런타임 변경은 미지원, yaml only)

Newton 은 world-frame pose 만 제공하므로 현재 구현은 평탄한 `world → each-body` 구조. `robot_state_publisher` 호환 parent→child 체인은 URDF 재파싱 필요 (별도 phase).

---

## Viewer 모드

`VIEWER` env var 하나로 디스패치. 디스패치 코드는 [src/newton_bridge/viewer.py](../src/newton_bridge/viewer.py).

| `VIEWER=` | 출력 | X11 | GL 드라이버 | 비용 | 용도 |
|---|---|---|---|---|---|
| `rerun` (기본) | 웹 UI @ `http://localhost:9090` | 불필요 | 불필요 | 중 | 원격/Dev-container, 스크러빙 |
| `gl` | 호스트 X 창 | 필요 | 필요 | 높음 | 로컬 워크스테이션, 인터랙션 (카메라 drag) |
| `usd` | `workspace/runs/sim_<ts>.usd` | 불필요 | 불필요 | 저 (녹화) | Omniverse 로 옮겨서 분석 |
| `file` | `workspace/runs/sim_<ts>.nvpr` | 불필요 | 불필요 | 저 (녹화) | Newton 자체 replay |
| `null` | 무출력 (팩토리만 dispatch) | 불필요 | 불필요 | ~0 | 벤치마크 (pure sim cost 측정) |
| `none` | viewer 완전 비활성 | 불필요 | 불필요 | 0 | 프로덕션, CI |

`none` 과 `null` 의 차이: `null` 은 매 frame `ViewerNull.log_state()` 가 호출됨 (작은 overhead). `none` 은 viewer 객체 자체를 만들지 않음.

### `rerun` (기본) — 웹 UI

```bash
./scripts/host/run.sh sim       # 브라우저: http://localhost:9090
```

브라우저로 `http://localhost:9090` 접속 → wasm 로드 후 **"Connection URL" 입력 화면**이 뜨면 다음을 입력:

```
rerun+http://localhost:9876/proxy
```

ViewerRerun 의 web UI(9090) 와 데이터 스트림(gRPC 9876) 이 별도 포트라, 웹 UI 가 자동으로 gRPC 에 붙지 않고 한 번 수동 입력이 필요합니다. 연결되면 좌측 트리에 `world/...` (로봇 메시) 와 `telemetry/...` (step_hz, render_hz, realtime_factor) 가 나타납니다.

- `network_mode: host` 덕에 포트 매핑 불필요
- 원격 머신이면 SSH tunnel: `ssh -L 9090:localhost:9090 -L 9876:localhost:9876 user@remote` (web + gRPC 둘 다 필요)
- `VIEWER_WIDTH` / `VIEWER_HEIGHT` 는 무시 (웹은 클라이언트가 뷰포트 소유)
- 동시 녹화: `RERUN_RECORD_TO=/workspace/workspace/runs/session.rrd ./scripts/host/run.sh sim` → 호스트에서 `rerun workspace/runs/session.rrd` 로 재생
- 포트 충돌: `RERUN_WEB_PORT=9091 RERUN_GRPC_PORT=9877 ./scripts/host/run.sh sim` → Connection URL 도 `rerun+http://localhost:9877/proxy` 로 맞춰서 입력

### `gl` — 네이티브 X11 창

```bash
VIEWER=gl ./scripts/host/run.sh sim
VIEWER=gl VIEWER_WIDTH=1920 VIEWER_HEIGHT=1080 ./scripts/host/run.sh sim
```

prereq: 호스트 X 서버 (Wayland 도 XWayland 로 OK), `DISPLAY` 세팅, nvidia GL + `NVIDIA_DRIVER_CAPABILITIES=...graphics,display`. `run.sh` 가 `xhost +local:docker` 자동 실행.

- **창을 닫으면 sim 이 종료됩니다** (viewer thread 가 `closed_event` 를 set, main 이 읽고 shutdown 진행).
- **GL 뷰어 입력**

  | 입력 | 효과 |
  |---|---|
  | `Space` | 일시정지/재개. sim step 은 멈추지만 rclpy spin 은 계속 → `/joint_command` 는 받아서 pending. 재개 시 반영 |
  | `.` (period) | 일시정지 상태에서 한 frame 만 single-step. queue maxsize=1 이므로 빠르게 여러 번 눌러도 정확히 1 step 씩 처리 |
  | `Esc` / 창 닫기 | sim 종료 |
  | UI Reset 버튼 | `/sim/reset` 서비스 호출과 동일한 경로 (queue 로 신호 → main 이 reset 수행). viewer 가 직접 `world.reset()` 을 부르지 않음 |
  | WASD / 마우스 | 카메라 이동 / 회전 (Newton ViewerGL 내부 처리) |
  | 우클릭 | body picking (Newton ViewerGL 내부 처리) |

- 실패 모드: X socket 접근 불가 → [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 참조.

### `usd` / `file` — 녹화

```bash
VIEWER=usd ./scripts/host/run.sh sim    # workspace/runs/sim_<UTC>.usd
VIEWER=file ./scripts/host/run.sh sim   # workspace/runs/sim_<UTC>.nvpr (Newton 네이티브)
```

| 변수 | 기본 | 설명 |
|---|---|---|
| `VIEWER_OUTPUT_PATH` | `workspace/runs/sim_<ts>.<ext>` | 고정 경로로 저장 |
| `VIEWER_OUTPUT_DIR` | `/workspace/workspace/runs` | 타임스탬프 파일 부모 |
| `VIEWER_FPS` | `60` | 녹화 frame rate (`usd` only) |
| `VIEWER_UP_AXIS` | `Z` | USD up axis (`usd` only) |

USD 는 Omniverse / Isaac Sim / USD Composer 호환. `.nvpr` 은 `./scripts/host/run.sh example player --viewer gl --file <path>` 로 재생.

### `null` / `none` — 헤드리스

```bash
VIEWER=null FREERUN_RATE=max ./scripts/host/run.sh sim    # 벤치마크
VIEWER=none ./scripts/host/run.sh sim                     # 완전 비활성 (CI)
```

### Sync 모드에서의 viewer

sync 모드에서는 sim step 자체가 외부 구동이므로 **state 는 `/joint_command` 또는 `/sim/reset` 시점에만 변합니다**. 다만 viewer 는 별도 thread 에서 viewer_hz 로 계속 마지막 snapshot 을 다시 그리므로 창 자체가 "얼어붙어" 보이지 않습니다 — 카메라 회전, GL UI 버튼, Rerun timeline 모두 동작.

`__main__.py` 가 다음 안내를 띄웁니다 (rerun/gl 한정):

```
[newton_bridge] note: sync mode advances only on /joint_command or /sim/reset;
                     the viewer will appear frozen until a controller publishes commands.
```

연속 프레임을 보고 싶으면 `controller_demo.py --mode sync` 로 루프 publish.

### 녹화 파일 cleanup

`workspace/runs/` 은 `.gitignore` 에 걸려 있어도 디스크에는 쌓임:

```bash
ls -lh workspace/runs/
rm workspace/runs/sim_2025*.usd
```

---

## 참고

- [INSTALL.md](INSTALL.md) — `.env` 복사 위치
- [USAGE.md](USAGE.md) — env var 의 실전 조합
- [ROBOTS.md](ROBOTS.md) — 새 pack 추가 시 schema 예제
- [ARCHITECTURE.md](ARCHITECTURE.md) — 왜 이런 스키마인지 (컨트리뷰터용)
