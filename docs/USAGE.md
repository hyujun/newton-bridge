# Usage

설치가 끝난 뒤 일상 작업 매뉴얼. 설치/빌드는 [INSTALL.md](INSTALL.md), env var · 토픽 계약 · viewer 상세는 [CONFIGURATION.md](CONFIGURATION.md), 새 로봇 추가는 [ROBOTS.md](ROBOTS.md).

## `run.sh` 하위명령 한눈에

모든 진입점은 `scripts/host/run.sh <mode> [args...]` — 내부적으로 `docker compose` 호출을 감쌉니다.

| Mode | 용도 | 기본값 |
|---|---|---|
| `sim` / (생략) | sim 프로세스 기동 (`python -m newton_bridge`) | `ROBOT=ur5e`, `SYNC_MODE=freerun`, `VIEWER=rerun` |
| `shell` | 컨테이너 안 bash (up -d 후 exec) | — |
| `example <name>` | `python -m newton.examples <name> ...` | viewer 는 인자로 지정 |
| `jupyter` | Jupyter notebook @ host:8888 | `JUPYTER_TOKEN=newton`, `/workspace/workspace/notebooks/` |
| `verify` | `scripts/container/verify.sh` — 11-섹션 smoke test | — |
| `upd` | `docker compose up -d` (백그라운드) | — |
| `logs` | `docker compose logs -f` | — |
| `down` | `docker compose down` | 볼륨/이미지 보존 |

선택은 env var:

```bash
ROBOT=franka ./scripts/host/run.sh sim                            # pack 만 변경
SYNC_MODE=sync ./scripts/host/run.sh sim                          # sync mode 변경
VIEWER=gl ./scripts/host/run.sh sim                               # 네이티브 X11 창
ROBOT=kuka_iiwa_14 SYNC_MODE=sync VIEWER=none \
    ./scripts/host/run.sh sim                                     # 전부 조합
```

---

## 표준 워크플로우

### A. Freerun 관찰 (가장 흔함)

```bash
# 터미널 1 — sim
./scripts/host/run.sh sim

# 터미널 2 — 호스트에서 관찰
source /opt/ros/jazzy/setup.bash
ros2 topic list                        # /clock /joint_states /joint_command /tf
ros2 topic hz /joint_states            # ~100Hz
ros2 topic echo --once /joint_states
```

브라우저로 `http://localhost:9090` → Rerun 뷰어.

종료는 터미널 1 에서 `Ctrl-C`. 컨테이너는 `--rm` 으로 자동 제거.

### B. External controller — `controller_demo.py`

호스트에서 돌면서 `/joint_command` 에 home 자세 기준 sine wave 를 퍼블리시, `/joint_states` 를 구독해서 마지막 자세를 출력하는 데모.

```bash
# 터미널 1 — sim
./scripts/host/run.sh sim

# 터미널 2 — 커스텀 컨트롤러 (호스트)
source /opt/ros/jazzy/setup.bash
python3 examples/controller_demo.py --mode freerun --robot ur5e --duration 10
```

```
controller_demo.py (host)                sim (container)
        │                                       │
        ├─ load robots/ur5e/robot.yaml          │
        ├─ home = [0, -1.57, 1.57, ...]         │
        ├─ create Node                          │
        │                                       │
        │   pub /joint_command @ 50Hz           │
        │   target[i] = home[i] + 0.2*sin(t+i)  │
        │ ─────────────────────────────────▶    │ sub → control.joint_target_pos
        │                                       │ step(dt) @ 400Hz
        │                                       │ pub /joint_states @ 100Hz
        │ ◀─────────────────────────────────    │
        │   sub /joint_states                   │
        │   store msg as latest                 │
        └─ duration 초 후 마지막 positions 출력  │
```

외부 컨트롤러는 `use_sim_time: true` 를 켜고 `/clock` 을 구독하면 sim 시간 기준으로 스탬프가 맞습니다. 로봇 교체는 `ROBOT=franka` + `--robot franka` 를 양쪽에 동일하게.

### C. Sync (deterministic, externally driven)

sync 모드는 **`/joint_command` publish 1회 = 1 step**. 시간을 전적으로 외부가 소유합니다.

```bash
# 터미널 1 — sim (sync)
SYNC_MODE=sync ./scripts/host/run.sh sim

# 터미널 2
source /opt/ros/jazzy/setup.bash

# target 변경 + 1 step 진행
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint'], position: [0.3]}"

# home 복귀 (언제든 가능)
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"

# 또는 demo 스크립트 (N 번 publish + /joint_states 로 진행 확인)
python3 examples/controller_demo.py --mode sync --robot ur5e --steps 200
```

각 iteration:
1. target = `home + 0.2*sin(i * 0.01)` 계산
2. `/joint_command` 퍼블리시 — sim 측 `_on_cmd` 콜백이 이를 받아 즉시 1 step 실행
3. `/joint_states` 의 새 `header.stamp` 로 step 완료 확인

200 번 publish → 총 `200 * physics_dt = 200/400 = 0.5s` 의 sim time 경과.

동작 요약:
- `/joint_command` 수신 시점에만 `world.step()` 1회 + `/joint_states` + `/clock` 퍼블리시
- `/joint_command` 가 끊기면 `ros.sync_timeout_ms` (기본 100ms) 후 현재 상태만 재퍼블리시 (step 없음) — 구독자가 굶지 않도록
- `sim.viewer_hz` (기본 60) 는 wall-clock 뷰어 FPS 목표 — physics rate / 커맨드 rate 와 독립

Legacy `SYNC_MODE=handshake` 도 동작하지만 deprecation 경고 후 `sync` 로 treat.

### D. 로봇 전환

```bash
ROBOT=franka ./scripts/host/run.sh sim
ROBOT=kuka_iiwa_14 ./scripts/host/run.sh sim
```

호스트의 외부 robot_description 폴더를 직접 쓰려면:

```bash
EXTERNAL_PACK_HOST=$HOME/my_robot_description ./scripts/host/run.sh sim
```

자세한 절차는 [ROBOTS.md §Path D](ROBOTS.md#path-d-호스트의-외부-폴더-사용-urdf-only).

### E. 벤치마크 (max-rate sim throughput)

```bash
FREERUN_RATE=max VIEWER=null ./scripts/host/run.sh sim
```

- `FREERUN_RATE=max` — wall-clock sleep 제거, 가능한 빠르게 step
- `VIEWER=null` — viewer dispatch 는 타되 render cost 제로

다른 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /clock
# physics_hz 대비 실제 sim rate 측정. UR5e@400Hz 는 RTX 3070 Ti 에서 대략 ~4000Hz
```

`/clock` 퍼블리시 rate 이 실제 sim step rate 의 상한. `publish_rate_hz: 100` 이면 state/tf 는 100Hz 로 토픽 rate 제한됨 (sim 은 빠르게 돌지만 ROS 노출은 rate-limit).

---

## `/joint_command` 보내는 법

`sensor_msgs/JointState` 의 3 개 배열이 각각 다른 제어 채널:

| 배열 | Newton `control` | 활성 조건 |
|---|---|---|
| `position` | `joint_target_pos` | pack drive mode `position` / `position_velocity` |
| `velocity` | `joint_target_vel` | `velocity` / `position_velocity` |
| `effort` | `joint_f` | `effort` |

**빈 배열 = "이 채널 건드리지 않음"**. 길이가 `name` 과 일치해야 반영. 부분 joint (일부만) 보내도 OK — 매칭되는 것만 업데이트.

```bash
# position 한 joint
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint'], position: [0.5]}"

# 전체 joint position (UR5e 6 DoF)
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
position: [0.0, -1.2, 1.5, -1.57, -1.57, 0.0]
"
```

타이밍:
- freerun: 다음 `world.step()` 직전에 반영 (latest-wins)
- sync: `/joint_command` 수신 시점에 1 step 실행 (publish = step trigger)

토픽/서비스 전체 계약은 [CONFIGURATION.md §ROS Topics & Services](CONFIGURATION.md#ros-topics--services).

---

## 런타임 gravity 변경

```bash
# 무중력
ros2 topic pub -1 /sim/set_gravity geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"

# 달 중력
ros2 topic pub -1 /sim/set_gravity geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -1.62}"
```

latest-wins, 단위 m/s².

---

## 시뮬 건강 모니터링

Sync 모드에서는 viewer 가 그려지고 있어도 physics 가 실제로 돌고 있는지 한눈에 보기 어렵습니다. Bridge 는 세 채널로 상태를 노출합니다.

**Terminal**: 1Hz ROS 로그 라인. 평소엔 INFO, STALL 일 땐 WARN 으로 자동 승격:
```
[INFO] [newton_bridge]: [newton_bridge] sim 12.34s | step 240Hz (rt 1.00) | cmd 100Hz | pub 100Hz | render 60Hz | state=RUNNING
```
끄려면 `STATUS_LOG_HZ=0`. 한 줄도 안 보이면 `LOG_LEVEL=INFO` (기본) 인지, root logger handler 가 살아있는지 확인.

**Viewer**: GL 모드는 좌측 통계 패널, Rerun 모드는 web UI 의 timeseries 패널 (`telemetry/*`) 에 동일 메트릭이 실시간으로 그려집니다.

**ROS**: `/sim/diagnostics` (`diagnostic_msgs/DiagnosticArray`, 1Hz):
```bash
ros2 topic echo /sim/diagnostics
```
RViz `Diagnostics` 패널, `rqt_runtime_monitor`, 또는 사용자 dashboard 에 직접 연결 가능. STALL 시 `status[].level=WARN`.

state 분류와 메트릭 정의는 [CONFIGURATION.md §Telemetry & 진단](CONFIGURATION.md#telemetry--진단) 참조.

---

## /tf 트리 확인

```bash
./scripts/host/run.sh sim
# 다른 터미널
source /opt/ros/jazzy/setup.bash
ros2 run tf2_tools view_frames          # frames.pdf 생성
# 또는
ros2 topic echo /tf --once
```

현재는 **flat `world → <body>`** 구조 — parent/child chain 은 별도 phase. 전체 body 퍼블리시 (기본) 또는 whitelist:

```yaml
# robot.yaml
ros:
  publish_tf: true
  tf_root_frame: world
  publish_frames: [tool0, wrist_3_link]   # 이 두 개만
```

RViz2 와 연동 (호스트):

```bash
source /opt/ros/jazzy/setup.bash
rviz2
# Add → TF, Fixed Frame: world
```

---

## 센서 추가 — contact 워크스루

`robots/ur5e/robot.yaml` 에 `sensors:` 블록 추가 → `/contact_wrenches/ee` 토픽 자동 생성.

```yaml
# robots/ur5e/robot.yaml 끝에 추가
sensors:
  contact:
    - label: ee
      bodies: ["*wrist_3_link*"]
      measure_total: true
      topic: /contact_wrenches/ee
      frame_id: wrist_3_link
```

```bash
./scripts/host/run.sh sim
# 로그에 "sensors: 1 contact, 0 imu"

source /opt/ros/jazzy/setup.bash
ros2 topic echo /contact_wrenches/ee
```

UR5e 가 home 자세에 있는 상태에서는 wrist_3_link 가 ground 에 닿지 않아 force 가 대부분 `(0, 0, 0)`. 떨어뜨려서 접촉시키려면 중력을 강하게:

```bash
ros2 topic pub -1 /sim/set_gravity geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -30.0}"
```

IMU 는 Newton 의 site 개념이 필요해서 MJCF pack 에서만 직접 — URDF 는 수동 `builder.add_site()` 필요. 전체 sensor 스키마는 [CONFIGURATION.md §Sensor 설정](CONFIGURATION.md#sensor-설정).

---

## `shell` 모드 — 컨테이너 내부 접근

```bash
./scripts/host/run.sh shell
# 안에서:
python3 -c "import newton; print(newton.__version__)"
python3 -m newton.examples basic_pendulum --viewer null --num-frames 50
```

이미 떠 있는 서비스에 붙으므로 `run.sh sim` 이 돌고 있으면 동일 프로세스 공간에 들어감. 새로 뜨면 service up.

독립 컨테이너 필요하면:

```bash
docker compose -f docker/compose.yml run --rm newton-bridge bash
```

---

## `example` 모드 — Newton 공식 예제

```bash
./scripts/host/run.sh example basic_pendulum --viewer gl
./scripts/host/run.sh example robot_anymal_c_walk --viewer null --num-frames 200
```

- `newton.examples` 서브모듈 전부 실행 가능 (`verify.sh §3` 가 레지스트리 확인)
- viewer 인자는 Newton CLI 가 받음 (`--viewer gl|null|usd`). 본 repo 의 `VIEWER` env 와 독립

레지스트리는 `./scripts/host/run.sh shell` 에서:

```bash
python -c "from newton import examples; import pkgutil; \
  [print(m.name) for m in pkgutil.walk_packages(examples.__path__, examples.__name__+'.')]"
```

---

## `jupyter` 모드 — 노트북

```bash
./scripts/host/run.sh jupyter
# 브라우저: http://localhost:8888/?token=newton
```

- `/workspace/workspace/notebooks/` 에 연결됨 — 호스트의 `workspace/notebooks/` 와 bind
- `JUPYTER_TOKEN` 은 `.env` 또는 env var 로 변경
- `network_mode: host` 라 포트 매핑 불필요 (호스트 `:8888` 직접 노출)
- 파일은 호스트 UID 로 써짐 (`compose.yml` 의 `user: ${HOST_UID}:${HOST_GID}`)

새 노트북에서 NewtonWorld 직접 만지기:

```python
import warp as wp
import newton
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld

wp.init()
pack = load_pack(Path("/workspace/robots/ur5e"))
world = NewtonWorld(pack)

j0 = world.exposed_joint_names[0]
q0 = world.read_joint_positions()[j0]
world.set_joint_targets([j0], positions=[q0 + 0.3])
for _ in range(200):
    world.step()

world.read_joint_positions()
```

`SimBridgeNode` 를 노트북에서 띄우면 호스트 `ros2 topic list` 와 도메인이 같으면 간섭하니 주의.

---

## Host ↔ Container 공유 경로

| 호스트 | 컨테이너 | 용도 | Access |
|---|---|---|---|
| `src/` | `/workspace/newton-bridge/src` | newton_bridge 패키지 (live edit) | ro |
| `robots/` | `/workspace/robots` | 내장 pack (예시용) | ro |
| `${EXTERNAL_PACK_HOST}` | `/workspace/external_pack` | 호스트의 외부 robot_description | ro |
| `scripts/container/` | `/workspace/scripts` | verify.sh, rl_smoketest.py (**flattened**) | ro |
| `workspace/` | `/workspace/workspace` | outputs, notebooks, models (RW) | rw |

> 주의: `scripts/container/*` 는 컨테이너에서 `/workspace/scripts/` 로 평탄화됩니다. 호스트 경로 그대로 컨테이너에서 찾으면 안 됨.

---

## 종료 / cleanup

```bash
# sim 종료 (Ctrl-C) 후 잔여 컨테이너 확인
docker ps -a | grep newton-bridge

# 남아 있으면 제거
./scripts/host/run.sh down

# 볼륨까지 (pip 캐시 초기화)
docker compose -f docker/compose.yml down -v

# 이미지 제거
docker image rm newton-bridge:latest
```

---

## VSCode Container Tools 연동

`ms-azuretools.vscode-containers` 확장 사이드바에 아무것도 안 뜨면 대부분 docker 그룹 문제. [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 참조.

스택이 떠 있어야 트리에 컨테이너가 보입니다:

```bash
./scripts/host/run.sh sim       # foreground
./scripts/host/run.sh upd       # 또는 백그라운드
```

---

## 관련 문서

- [CONFIGURATION.md](CONFIGURATION.md) — 전체 env var · pack yaml · 토픽/서비스 · viewer
- [ROBOTS.md](ROBOTS.md) — 새 pack 추가 (외부 폴더 포함)
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 실행 중 문제
