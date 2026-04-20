# Usage

설치가 끝난 뒤 일상 작업 매뉴얼. 설치/빌드는 [INSTALL.md](INSTALL.md), env var 상세는 [CONFIGURATION.md](CONFIGURATION.md), viewer 는 [VIEWER.md](VIEWER.md).

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
SYNC_MODE=handshake ./scripts/host/run.sh sim                     # sync mode 변경
VIEWER=gl ./scripts/host/run.sh sim                               # 네이티브 X11 창
ROBOT=kuka_iiwa_14 SYNC_MODE=handshake VIEWER=none \
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
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic list                        # /clock /joint_states /joint_command /tf
ros2 topic hz /joint_states            # ~100Hz
ros2 topic echo --once /joint_states
```

브라우저로 `http://localhost:9090` → Rerun 뷰어.

종료는 터미널 1 에서 `Ctrl-C`. 컨테이너는 `--rm` 으로 자동 제거.

### B. External controller + sim

```bash
# 터미널 1 — sim
./scripts/host/run.sh sim

# 터미널 2 — 커스텀 컨트롤러 (호스트)
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 examples/controller_demo.py --mode freerun --robot ur5e --duration 10
```

외부 컨트롤러는 `use_sim_time: true` 를 켜고 `/clock` 을 구독하면 sim 시간 기준으로 스탬프가 맞습니다.

### C. Handshake (deterministic)

```bash
# 터미널 1 — sim (handshake)
SYNC_MODE=handshake ./scripts/host/run.sh sim

# 터미널 2
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 service call /sim/step std_srvs/srv/Trigger "{}"     # 1 step
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"    # home 복귀

# 또는 demo 스크립트
python3 examples/controller_demo.py --mode handshake --robot ur5e --steps 200
```

handshake 에서는 `/sim/step` 호출 전까지 시간이 멈춰 있고, viewer 도 frozen. 콜 시점에만 1 step 진행 + `/joint_states` + `/clock` 퍼블리시 + viewer 렌더.

### D. 로봇 전환

```bash
ROBOT=franka ./scripts/host/run.sh sim
ROBOT=kuka_iiwa_14 ./scripts/host/run.sh sim
```

`ROBOT_PACK=/workspace/robots/<name>` 로 컨테이너 경로를 직접 주어도 됩니다. 새 pack 추가는 [ROBOTS.md](ROBOTS.md).

### E. 벤치마크 (max-rate)

```bash
FREERUN_RATE=max VIEWER=null ./scripts/host/run.sh sim
```

- `FREERUN_RATE=max` — wall-clock sleep 제거, 가능한 빠르게 step
- `VIEWER=null` — viewer dispatch 는 타되 render cost 제로 (benchmark 용)

다른 터미널에서 `ros2 topic hz /clock` 으로 sim_rate 측정.

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
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint']
position: [0.5]
"

# velocity 한 joint (pack 이 velocity 모드일 때)
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint']
velocity: [0.2]
"

# 전체 joint position (UR5e 6 DoF)
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
position: [0.0, -1.2, 1.5, -1.57, -1.57, 0.0]
"
```

타이밍:
- freerun: 다음 `world.step()` 직전에 반영 (latest-wins)
- handshake: `/sim/step` 호출 시점의 latest-wins

토픽/서비스 전체 계약은 [TOPICS.md](TOPICS.md).

---

## 런타임 gravity 변경

```bash
# 무중력
ros2 topic pub -1 /sim/set_gravity geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"

# 달 중력
ros2 topic pub -1 /sim/set_gravity geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -1.62}"
```

latest-wins, 단위 m/s². Phase 6a.

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

`list` 옵션은 예제마다 다르니 레지스트리는 `./scripts/host/run.sh shell` 에서 `python -c "from newton import examples; import pkgutil; [print(m.name) for m in pkgutil.walk_packages(examples.__path__, examples.__name__+'.')]"`.

---

## `jupyter` 모드 — 노트북

```bash
./scripts/host/run.sh jupyter
# 브라우저: http://localhost:8888/?token=newton
```

- `/workspace/workspace/notebooks/` 에 연결됨 — 호스트의 `workspace/notebooks/` 와 bind
- `JUPYTER_TOKEN` 은 `.env` 또는 env var 로 변경
- `network_mode: host` 라 포트 매핑 불필요 (호스트 `:8888` 직접 노출)

파일은 컨테이너가 host UID 로 써서 `sudo chown` 불필요 (`compose.yml` 의 `user: ${HOST_UID}:${HOST_GID}`).

---

## Host ↔ Container 공유 경로

| 호스트 | 컨테이너 | 용도 | Access |
|---|---|---|---|
| `src/` | `/workspace/newton-bridge/src` | newton_bridge 패키지 (live edit) | ro |
| `robots/` | `/workspace/robots` | pack + models (bind) | ro |
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
```

다른 도구에서 같은 이미지를 쓰고 있지 않은지 확인 후:

```bash
docker image rm newton-bridge:latest
```

---

## Viewer 선택 요약

| `VIEWER=` | 출력 | X11 필요? |
|---|---|---|
| `rerun` (기본) | 웹 UI @ `http://localhost:9090` | 아니오 |
| `gl` | 호스트 X 창 (닫으면 sim 종료) | 예 |
| `usd` | `workspace/runs/sim_<ts>.usd` | 아니오 |
| `file` | `workspace/runs/sim_<ts>.nvpr` | 아니오 |
| `null` | 무출력 (벤치마크) | 아니오 |
| `none` | viewer 완전 비활성 (init 비용 0) | 아니오 |

상세는 [VIEWER.md](VIEWER.md).

---

## VSCode Container Tools 연동

`ms-azuretools.vscode-containers` 확장 사이드바에 아무것도 안 뜨면 대부분 docker 그룹 문제. [TROUBLESHOOTING.md #VSCode](TROUBLESHOOTING.md#vscode-container-tools-사이드바가-비어있음) 참조.

스택이 떠 있어야 트리에 컨테이너가 보입니다:

```bash
./scripts/host/run.sh sim       # foreground
./scripts/host/run.sh upd       # 또는 백그라운드
```

---

## 관련 문서

- [CONFIGURATION.md](CONFIGURATION.md) — 전체 env var + pack yaml 필드
- [TOPICS.md](TOPICS.md) — 토픽/서비스 계약 (단위, QoS, 센서)
- [VIEWER.md](VIEWER.md) — viewer 상세
- [EXAMPLES.md](EXAMPLES.md) — controller_demo / 센서 / Jupyter 워크플로우
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 실행 중 문제
