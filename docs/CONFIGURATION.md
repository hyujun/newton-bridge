# Configuration Reference

env var + robot.yaml / scene.yaml 모든 knob 을 한 곳에. 토픽/서비스 스키마는 [TOPICS.md](TOPICS.md), 새 pack 추가 절차는 [ROBOTS.md](ROBOTS.md).

---

## Environment Variables

우선순위: `run.sh` 명령줄 export > `.env` > compose.yml 기본값 > 애플리케이션 fallback.

### 로봇 pack

| 변수 | 기본 | 출처 | 설명 |
|---|---|---|---|
| `ROBOT` | `ur5e` | `run.sh` | 컨테이너 경로 `/workspace/robots/$ROBOT` 로 확장 |
| `ROBOT_PACK` | `/workspace/robots/ur5e` | `__main__.py` | 컨테이너 내 pack 경로 (직접 지정하면 `ROBOT` 무시) |

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
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | 다른 rmw 는 미검증 |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | **반드시 UDPv4**. SHM 은 root/user UID 경계에서 깨짐 |

### Viewer

| 변수 | 기본 | 허용값 | 설명 |
|---|---|---|---|
| `VIEWER` | `rerun` | `rerun` \| `gl` \| `usd` \| `file` \| `null` \| `none` | [VIEWER.md](VIEWER.md) 참조 |
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
| `ENABLE_VIEWER` | (deprecated) | — | 설정되면 에러. Phase 7 에서 `VIEWER` 로 대체 |

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

`robots/<name>/` 에 다음 중 하나:
- `scene.yaml` — canonical (Phase 2+)
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
  viewer_hz: 60                      # 뷰어 렌더 rate. 0 이면 물리 step 마다 렌더.
                                     # physics_hz 와 독립 — 500Hz 물리 + 60Hz 뷰어 가능

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

sensors:                             # optional (Phase 5)
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
  publish_rate_hz:     100           # freerun 의 state 퍼블리시 rate
  sync_timeout_ms:     100           # sync 모드 전용. /joint_command 가 이 시간 동안
                                     # 안 오면 현재 상태를 /joint_states 로 재퍼블리시
                                     # (step 없음). 구독자가 굶지 않게 하기 위함
  publish_tf:          true          # Phase 4. default true
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

ros:
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz:     100
```

승격 규칙 ([`_promote_robot_yaml`](../src/newton_bridge/robot_pack.py#L51)):
- `robot.base_position` → articulation `xform.pos`
- `joint_names`/`home_pose`/`drive`/`joints`/`articulation_pattern` → 단일 articulation 에 그대로
- `sim.gravity` → world gravity
- `sensors:` 있으면 scene 로 carry-through
- `ros.primary_articulation` 기본값 = `<pack_dir 이름>` (예: `ur5e`)

---

## Solver 선택 매트릭스

| 소스 | 권장 solver | 이유 |
|---|---|---|
| URDF (actuator 없음) | `mujoco` 또는 `xpbd` / `featherstone` | MuJoCo 가 빌더-레벨 gain 을 존중, URDF 에도 잘 맞음 (실측 Newton 1.1.0 에서 URDF + `xpbd` 는 drive 가 joint 에 도달 안 함) |
| MJCF (with `<actuator>`) | `mujoco` | actuator 블록의 gain 이 solver 로 그대로 전달됨 |
| MJCF (no `<actuator>`) | `xpbd` / `featherstone` | `drive.stiffness/damping` 이 사용됨 |

실제 pack 들:
- `ur5e` — xacro → URDF + `mujoco`. `$(find ur_description)` 은 apt `ros-jazzy-ur-description` 이 제공. `package://` 메쉬는 `resolve-robotics-uri-py` 가 `AMENT_PREFIX_PATH` 로 해석 (실험 결과 XPBD 불일치)
- `franka` — MJCF + `mujoco` (`panda.xml` 에 actuator 포함)
- `kuka_iiwa_14` — MJCF + `mujoco`

불일치는 `./scripts/host/run.sh verify` 섹션 6 에서 잡힘.

**Deformable solver** (`style3d` / `vbd`) 는 [DEFERRED_WORK.md Phase 6b](DEFERRED_WORK.md) 에서 다루는 미구현 범위 — 현재 pack 스키마로는 cloth/soft/particle 을 선언할 수 없습니다.

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

출력 msg 타입은 [TOPICS.md §Sensors](TOPICS.md#sensors-phase-5) 참조.

---

## /tf 설정

`ros.publish_tf` 를 `false` 로 하면 `/tf` 퍼블리셔 자체가 생성되지 않음 (CPU 절감).

`ros.publish_frames`:
- `[]` (기본) — 루트(`tf_root_frame`) 를 제외한 **모든** body 퍼블리시
- `["tool0", "wrist_3_link"]` — whitelist

Newton 은 world-frame pose 만 제공하므로 현재 구현은 평탄한 `world → each-body` 구조. `robot_state_publisher` 호환 parent→child 체인은 URDF 재파싱 필요 (별도 phase).

---

## 참고

- [INSTALL.md](INSTALL.md) — `.env` 복사 위치
- [USAGE.md](USAGE.md) — env var 의 실전 조합
- [ROBOTS.md](ROBOTS.md) — 새 pack 추가 시 schema 예제
- [TOPICS.md](TOPICS.md) — ROS 토픽/서비스 전체
- [ARCHITECTURE.md](ARCHITECTURE.md) — 왜 이런 스키마인지
