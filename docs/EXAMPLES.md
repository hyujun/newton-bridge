# Examples / Walkthroughs

`examples/` 와 `scripts/container/` 에 있는 샘플 코드 + 흔한 워크플로우의 step-by-step. 설치 완료 + `run.sh verify` 통과 상태를 전제합니다.

## 카탈로그

| 경로 | 위치 | 용도 |
|---|---|---|
| [examples/controller_demo.py](../examples/controller_demo.py) | 호스트 | freerun/sync E2E 데모 (sine 타겟) |
| [scripts/container/verify.sh](../scripts/container/verify.sh) | 컨테이너 | 11-섹션 smoke test |
| [scripts/container/rl_smoketest.py](../scripts/container/rl_smoketest.py) | 컨테이너 | torch-cu12 + CUDA 검증 |
| [scripts/host/verify_ros.sh](../scripts/host/verify_ros.sh) | 호스트 | sim 과 host ROS 2 간 토픽 흐름 확인 |

---

## 1. External controller — freerun sine

`controller_demo.py` 는 호스트에서 돌면서 `/joint_command` 에 home 자세 기준 sine wave 를 퍼블리시, `/joint_states` 를 구독해서 마지막 자세를 출력합니다.

### 실행

```bash
# 터미널 1 — sim (기본: ur5e, freerun, rerun viewer)
./scripts/host/run.sh sim

# 터미널 2 — 호스트 controller
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 examples/controller_demo.py --mode freerun --robot ur5e --duration 10
```

### 동작 이해

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
        │                                       │
        └─ duration 초 후 마지막 positions 출력  │
```

- pack yaml 을 직접 읽어 home pose 를 확보 (sim 과 동일 값 사용)
- joint 이름 순서는 pack yaml 의 `joint_names` 를 그대로 씀 — sim 이 퍼블리시하는 `/joint_states.name` 과 일치
- sine 진폭 0.2 rad, joint 별 phase shift 로 각 joint 의 움직임이 눈에 보이도록

### 로봇 교체

```bash
ROBOT=franka ./scripts/host/run.sh sim
# 다른 터미널
python3 examples/controller_demo.py --mode freerun --robot franka
```

---

## 2. Sync — deterministic step loop

```bash
# 터미널 1
SYNC_MODE=sync ./scripts/host/run.sh sim

# 터미널 2
python3 examples/controller_demo.py --mode sync --robot ur5e --steps 200
```

각 iteration:
1. target = `home + 0.2*sin(i * 0.01)` 계산
2. `/joint_command` 퍼블리시 — sim 측 `_on_cmd` 콜백이 이를 받아 즉시 1 step 실행
3. `/joint_states` 의 새 `header.stamp` 로 step 완료 확인

200 번 publish → 총 `200 * physics_dt = 200/400 = 0.5s` 의 sim time 경과.

**실패 시**:
```
[demo] no /joint_states received (is sim running?)
[demo] step N timed out waiting for /joint_states
```
→ sim 이 안 떠 있거나 DDS 매치 실패 (UDP transport / domain id). `SYNC_MODE=sync` 로 기동되어 있는지 확인.

---

## 3. 새 pack 30-분 튜토리얼

UR5e URDF 를 "myarm" 이라는 새 pack 으로 복제해서 작동 확인. (실무에서는 [ROBOTS.md](ROBOTS.md) 의 A/B/C 경로를 따름)

```bash
# 1) pack 디렉토리
mkdir -p robots/myarm/models

# 2) 기존 UR5e URDF 재사용 (튜토리얼 목적)
cp -r robots/ur5e/models/* robots/myarm/models/

# 3) robot.yaml 작성
cat > robots/myarm/robot.yaml <<'YAML'
robot:
  source: urdf
  source_rel: models/ur5e.urdf
  base_position: [0.0, 0.0, 0.0]
sim:
  physics_hz: 400
  substeps: 1
  solver: mujoco
  ground_plane: true
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
home_pose:
  shoulder_pan_joint:  0.0
  shoulder_lift_joint: -1.0      # UR5e 와 다른 값 — 구분되는지 확인용
  elbow_joint:          1.5
  wrist_1_joint:       -1.57
  wrist_2_joint:       -1.57
  wrist_3_joint:        0.0
drive:
  mode: position
  stiffness: 10000.0
  damping: 100.0
ros:
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz:     100
YAML

# 4) verify.sh 섹션 6 에서 PASS 인지
./scripts/host/run.sh verify 2>&1 | grep "pack myarm"

# 5) 실제 기동
ROBOT=myarm ./scripts/host/run.sh sim

# 6) 호스트에서 확인
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic echo --once /joint_states
# positions 가 myarm 의 home_pose 와 일치해야 함
```

- **bind-mount**: `robots/` 는 ro 로 bind 되어 있어 호스트 편집이 컨테이너에 즉시 반영. 재빌드 불필요
- **asset 분리**: 신규 pack 은 `fetch_assets.sh` 에 자체 블록을 추가해야 재실행 안전성을 유지. 위 튜토리얼은 기존 ur5e mesh 를 copy 만 했으므로 `clean` 시 사라짐

새 URDF/MJCF 소스를 연결하려면 [ROBOTS.md — Path A/B/C](ROBOTS.md) 참조.

---

## 4. 센서 추가 (contact) — sensor walkthrough

`robots/ur5e/robot.yaml` 에 `sensors:` 블록 추가 → `/contact_wrenches/ee` 토픽 자동 생성.

### 설정

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

### 실행 + 확인

```bash
# 1) sim 재기동
./scripts/host/run.sh sim

# 2) 로그에 센서 생성 확인
#   [INFO]: sensors: 1 contact, 0 imu

# 3) 호스트에서 구독
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic echo /contact_wrenches/ee
```

UR5e 가 home 자세에 있는 상태에서는 wrist_3_link 가 ground 에 닿지 않아 force 가 대부분 `(0, 0, 0)`. 떨어뜨려서 접촉시키려면:

```bash
# 중력을 강하게
ros2 topic pub -1 /sim/set_gravity geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -30.0}"

# 또는 gains 낮춰서 arm 무너뜨림
# (pack 의 drive.stiffness 를 100 정도로 낮춘 뒤 재기동)
```

IMU 센서는 MJCF pack 에서만 직접 — URDF 는 site 개념이 없어 수동 `builder.add_site()` 필요 ([sensors.py](../src/newton_bridge/sensors.py)).

전체 sensor 스키마는 [CONFIGURATION.md §Sensor 설정](CONFIGURATION.md#sensor-설정).

---

## 5. /tf 트리 확인

```bash
./scripts/host/run.sh sim
# 다른 터미널
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run tf2_tools view_frames          # frames.pdf 생성
# 또는
ros2 topic echo /tf --once
```

현재는 **flat `world → <body>`** 구조 — parent/child chain 은 Phase 4 범위 외. 전체 body 퍼블리시 (기본) 또는 whitelist:

```yaml
# robot.yaml
ros:
  publish_tf: true
  tf_root_frame: world
  publish_frames: [tool0, wrist_3_link]   # 이 두 개만
```

### RViz2 와 연동 (호스트)

```bash
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
rviz2
# Add → TF
# Fixed Frame: world
```

---

## 6. Jupyter notebook 에서 Newton 만지기

```bash
./scripts/host/run.sh jupyter
# 브라우저: http://localhost:8888/?token=newton
```

`/workspace/workspace/notebooks/` 에 새 notebook 생성 후:

```python
import warp as wp
import newton
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld

wp.init()
pack = load_pack(Path("/workspace/robots/ur5e"))
world = NewtonWorld(pack)

# home pose + 약간의 offset 으로 200 step
j0 = world.exposed_joint_names[0]
q0 = world.read_joint_positions()[j0]
world.set_joint_targets([j0], positions=[q0 + 0.3])
for _ in range(200):
    world.step()

world.read_joint_positions()
```

- `rclpy` 는 Jupyter 모드에서도 import 가능하지만, `SimBridgeNode` 를 띄우면 호스트 `ros2 topic list` 와 간섭할 수 있음 (같은 `ROS_DOMAIN_ID`)
- 인터랙티브 Rerun 연동:
  ```python
  import rerun as rr
  rr.init("nb", spawn=False)
  rr.serve_web(web_port=9091)   # 9090 은 sim 이 쓸 수 있으니 피함
  ```

---

## 7. RL smoketest — torch-cu12 검증

```bash
docker compose -f docker/compose.yml run --rm newton-bridge \
    python3 /workspace/scripts/rl_smoketest.py
```

컨테이너 안에서:
1. `torch` import + CUDA 가시성 확인
2. policy-shape MLP (48→256→256→12) 를 GPU 에 올림
3. 배치 4096 forward pass

출력:
```
[rl-smoke] torch 2.x.x+cu128
[rl-smoke] cuda device: NVIDIA GeForce RTX 3070 Ti
[rl-smoke] forward OK: in=(4096, 48) -> out=(4096, 12)
```

실제 RL 파이프라인은 [DEFERRED_WORK.md — Phase 9](DEFERRED_WORK.md) 에서 별도 서브프로젝트로.

Newton 공식 RL 예제는:
```bash
./scripts/host/run.sh example robot_anymal_c_walk --viewer null --num-frames 200
```

---

## 8. 벤치마크 — max-rate sim throughput

```bash
FREERUN_RATE=max VIEWER=null SYNC_MODE=freerun ./scripts/host/run.sh sim
```

- `FREERUN_RATE=max` — wall-clock sleep 제거
- `VIEWER=null` — render cost ~0
- `SYNC_MODE=freerun` — sync 외부-driven 왕복 latency 제외

다른 터미널:

```bash
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic hz /clock
# physics_hz 대비 실제 sim rate 측정. UR5e@400Hz 는 RTX 3070 Ti 에서 대략 ~4000Hz
```

`/clock` 퍼블리시 rate 이 실제 sim step rate 의 상한. `publish_rate_hz: 100` 이면 state/tf 는 100Hz 로 토픽 rate 제한됨 (sim 은 빠르게 돌지만 ROS 노출은 rate-limit).

---

## 9. 녹화 → 재생

### USD 녹화

```bash
VIEWER=usd ./scripts/host/run.sh sim
# Ctrl-C 로 종료하면 workspace/runs/sim_<UTC>.usd 생성됨
ls workspace/runs/
```

파일을 Omniverse / IsaacSim / USD Composer 에 열면 애니메이션이 sim 시간 기준으로 재생됨.

### Rerun 녹화

```bash
RERUN_RECORD_TO=/workspace/workspace/runs/session.rrd ./scripts/host/run.sh sim
# 웹 뷰어 + .rrd 파일 동시 출력
```

재생은 호스트에서:

```bash
pip install rerun-sdk
rerun workspace/runs/session.rrd
```

---

## 관련 문서

- [USAGE.md](USAGE.md) — run.sh 하위명령 레퍼런스
- [TOPICS.md](TOPICS.md) — 토픽/서비스 스키마
- [ROBOTS.md](ROBOTS.md) — 새 pack 추가 절차
- [VIEWER.md](VIEWER.md) — 뷰어 상세
- [CONFIGURATION.md](CONFIGURATION.md) — env var + pack yaml
