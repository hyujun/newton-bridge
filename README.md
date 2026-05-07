# newton-bridge

**Newton Physics (standalone)** ↔ **ROS 2 Jazzy** 브리지. 호스트의 custom controller 와 topic/service 기반으로 sync 되는 시뮬레이터.

`network_mode: host` 로 컨테이너의 DDS 참여자가 호스트와 동일 도메인에 속하므로, 호스트에서 `ros2 topic list` 로 바로 `/clock`, `/joint_states`, `/joint_command` 가 보입니다.

## 한눈에

```
┌─ Host: Ubuntu 24.04 + ROS 2 Jazzy ──────────────────┐
│                                                      │
│   Custom controller  ─ pub /joint_command            │
│                      ─ sub /joint_states (use_sim_time)
│                      ─ sub /tf (toggleable)          │
│                      ─ call /sim/reset / set_publish_tf (optional)
│                      ─ pub /sim/set_gravity (optional)│
│                             │                        │
│   ┌─────────────────────────┴──────────────────────┐ │
│   │ Container: newton-bridge                        │ │
│   │   python -m newton_bridge                       │ │
│   │     ├─ Newton: ModelBuilder + solver step       │ │
│   │     └─ rclpy: /clock, /joint_states/command,    │ │
│   │                /tf, /sim/{reset,set_publish_tf, │ │
│   │                set_gravity, diagnostics}        │ │
│   └─────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

## Quick start

```bash
./scripts/host/install.sh             # 호스트 prereq (base utils + docker + compose v2 + nvidia toolkit + ur_description + xacro)
./scripts/host/install.sh --with-ros  # + 전체 ROS 2 Jazzy Desktop (verify_ros.sh / controller_demo 용)
./scripts/host/fetch_assets.sh        # mujoco_menagerie + ur5e URDF 다운로드 (예시 pack 채우기)
./scripts/host/build.sh               # Docker 이미지 빌드 (5~15분)
./scripts/host/run.sh verify          # 컨테이너 스모크 테스트
./scripts/host/run.sh sim             # ROBOT=ur5e, freerun, VIEWER=rerun → http://localhost:9090
VIEWER=gl ./scripts/host/run.sh sim   # Newton GL viewer 창 (X11 필요)
VIEWER=none ./scripts/host/run.sh sim # headless

# 호스트의 외부 robot_description 폴더 직접 사용 (URDF only):
EXTERNAL_PACK_HOST=$HOME/my_robot_description ./scripts/host/run.sh sim
# 폴더 내부에 robot.yaml + URDF 가 필요. 절차는 docs/ROBOTS.md (Path D).
```

> 모든 `scripts/host/*.sh` 는 **재실행 안전 (idempotent)** — 이미 완료된 단계는 스킵합니다.

> `install.sh` 는 **fresh Ubuntu 24.04** 에서 base utils (git, rsync, curl, jq, xhost) +
> Docker Engine + compose v2 + (GPU 있으면) nvidia-container-toolkit + ROS 2 apt repo +
> `ros-jazzy-ur-description` + `ros-jazzy-xacro` 까지 설치합니다 (뒤 두 개는 `fetch_assets.sh`
> 가 ur5e mesh/config/xacro 를 robot pack 으로 복사할 때 필요). `--only-check` 로 현재 상태만
> 점검, `--with-ros` 로 호스트 전체 ROS 2 Jazzy Desktop (`verify_ros.sh` / `controller_demo.py` 용)
> 추가 설치. **NVIDIA driver 는 감지만 하고 설치하지 않음** — 없으면
> `sudo ubuntu-drivers autoinstall && sudo reboot` 후 재실행.

> **Viewer 선택** (`VIEWER` env): `rerun` (기본, 웹 UI @ `http://localhost:9090`,
> X11 불필요) · `gl` (X11 passthrough + nvidia GL 드라이버 필요, 창 닫으면 sim 종료) ·
> `usd` / `file` (`workspace/runs/<ts>.{usd,nvpr}` 로 녹화) · `null` (벤치마크) · `none`.
> `sync` 모드에서는 `/joint_command` 수신 시에만 step 이 진행됩니다. Viewer 는 별도 thread 에서 `sim.viewer_hz` (기본 60Hz) 로 wall-clock pacing — physics/cmd rate 와 독립이고, sync 모드 idle 중에도 마지막 snapshot 을 60Hz 로 계속 redraw 하므로 창이 얼어붙지 않습니다. 시뮬 헬스는 1Hz `[newton_bridge] step ... cmd ... state=...` stderr 라인 + `/sim/diagnostics` 토픽으로 노출.

별도 터미널에서 (호스트):

```bash
source /opt/ros/jazzy/setup.bash
# RMW 기본값은 Cyclone DDS (컨테이너와 자동 일치). FastDDS 쓰려면
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic hz /joint_states
python3 examples/controller_demo.py --mode freerun --robot ur5e
```

## Supported robots

| Pack | DoF | Source | Solver (기본) |
|---|---|---|---|
| `ur5e` | 6 | URDF (ur_description) | mujoco |
| `franka` | 7 | MJCF (mujoco_menagerie/franka_emika_panda) | mujoco |
| `kuka_iiwa_14` | 7 | MJCF (mujoco_menagerie/kuka_iiwa_14) | mujoco |

위 표의 pack 들은 **빌트인 예시** — 실제 프로젝트 로봇은 호스트의 robot_description 폴더에 `robot.yaml` 만 추가해서 `EXTERNAL_PACK_HOST` 로 직접 마운트하는 것을 권장합니다 (URDF only). 빌트인 pack 을 다른 걸로 전환하려면 `ROBOT=kuka_iiwa_14 ./scripts/host/run.sh sim`.

새 로봇 추가 (외부 폴더 / URDF / xacro / MJCF) 절차는 [docs/ROBOTS.md](docs/ROBOTS.md).

## Sync modes

| `SYNC_MODE=` | 동작 | 용도 |
|---|---|---|
| `freerun` (default) | sim 이 `FREERUN_RATE=realtime\|max` 로 자율 step | 관찰, loose-sync 제어, 데모 |
| `sync` | 외부가 `/joint_command` 를 publish 할 때마다 1 step. idle 시 `ros.sync_timeout_ms` 마다 현재 상태 재퍼블리시 | deterministic RL rollout, 결정성 테스트 |

## 설치된 Newton extras

- `examples` — sim + importers + viewer (GL/USD)
- `torch-cu12` — PyTorch CUDA 12.8 (RL policy 추론)
- `notebook` — Jupyter + Rerun
- `dev` — 테스트/린트

Jupyter 는 `./scripts/host/run.sh jupyter` 로 host:8888 에.

## 디렉토리 레이아웃

```
newton-bridge/
├── README.md                      ← 여기
├── pyproject.toml                 newton_bridge 패키지 메타 (editable install)
├── .env.example                   기본값 스냅샷
├── src/newton_bridge/             ★ Newton + rclpy 단일 프로세스 (모듈화)
│   ├── __main__.py                `python -m newton_bridge` entry point
│   ├── world.py                   NewtonWorld (ModelBuilder + solver)
│   ├── node.py                    SimBridgeNode (rclpy pubs/subs/services)
│   ├── snapshot.py                StateSnapshot (main → viewer 이중버퍼)
│   ├── viewer_thread.py           ViewerThread (별도 thread + viewer pacing)
│   ├── telemetry.py               RateMeter / TelemetryRegistry / StatusLogger
│   ├── robot_pack.py              robot.yaml loader
│   └── viewer.py                  optional Newton GL viewer 팩토리
├── docker/                        Docker 전부
│   ├── Dockerfile                 CUDA 12.9 + Ubuntu 24.04 + ROS 2 Jazzy + Newton[전체]
│   ├── compose.yml                GPU + X11 + network_mode:host + ROS env
│   └── entrypoint.sh              source /opt/ros/jazzy + exec
├── scripts/
│   ├── host/                      호스트에서 실행
│   │   ├── install.sh             docker / compose / nvidia toolkit 설치
│   │   ├── build.sh / run.sh      이미지 빌드 / 컨테이너 제어
│   │   ├── fetch_assets.sh        URDF/MJCF 수급
│   │   └── verify_ros.sh          호스트 ROS 2 연동 체크
│   └── container/                 컨테이너 내부
│       ├── verify.sh              스모크 테스트
│       └── rl_smoketest.py        torch-cu12 검증
├── examples/
│   └── controller_demo.py         sine-wave E2E 데모
├── robots/                        ← 빌트인 예시 pack (data only, docs/ROBOTS.md)
│   ├── ur5e/robot.yaml            6-DoF arm
│   ├── franka/robot.yaml          7-DoF arm
│   └── kuka_iiwa_14/robot.yaml    7-DoF arm
├── tests/                         pytest (host-side unit tests)
├── workspace/                     호스트↔컨테이너 공유 (outputs, notebooks)
├── assets/_cache/                 fetch_assets.sh 의 상류 클론 (gitignored, ~2 GB)
└── docs/
    ├── README.md                  docs 인덱스 (사용자 여정 기준)
    ├── INSTALL.md / USAGE.md      설치 → 일상 워크플로우
    ├── ROBOTS.md                  새 로봇 추가 (외부 폴더 / URDF / xacro / MJCF)
    ├── TROUBLESHOOTING.md         단계별 실패 모드
    ├── CONFIGURATION.md           env + pack yaml + 토픽 + viewer 레퍼런스
    └── ARCHITECTURE.md            (컨트리뷰터용) 레이어 경계, sync 모델
```

## 관련 문서

처음 사용자는 순서대로 — 인덱스는 [docs/README.md](docs/README.md).

1. [docs/INSTALL.md](docs/INSTALL.md) — 호스트 prereq · 이미지 빌드 · verify
2. [docs/USAGE.md](docs/USAGE.md) — `run.sh` 하위명령 + 일상 워크플로우 (controller_demo, Jupyter, 벤치)
3. [docs/ROBOTS.md](docs/ROBOTS.md) — 내 로봇 추가 (외부 폴더 / URDF / xacro / MJCF)
4. [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — 단계별 실패 모드 + 조치
5. [docs/CONFIGURATION.md](docs/CONFIGURATION.md) — env var · pack yaml · 토픽/서비스 · viewer 레퍼런스

컨트리뷰터/내부 작업: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md). Newton 공식: <https://newton-physics.github.io/newton/latest/>
