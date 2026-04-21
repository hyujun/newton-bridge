# newton-bridge

**Newton Physics (standalone)** ↔ **ROS 2 Jazzy** 브리지. 호스트의 custom controller 와 topic/service 기반으로 sync 되는 시뮬레이터.

`network_mode: host` 로 컨테이너의 DDS 참여자가 호스트와 동일 도메인에 속하므로, 호스트에서 `ros2 topic list` 로 바로 `/clock`, `/joint_states`, `/joint_command` 가 보입니다.

## 한눈에

```
┌─ Host: Ubuntu 24.04 + ROS 2 Jazzy ──────────────────┐
│                                                      │
│   Custom controller  ─ pub /joint_command            │
│                      ─ sub /joint_states (use_sim_time)
│                      ─ call /sim/reset  (optional)    │
│                             │                        │
│   ┌─────────────────────────┴──────────────────────┐ │
│   │ Container: newton-bridge                        │ │
│   │   python -m newton_bridge                       │ │
│   │     ├─ Newton: ModelBuilder + solver step       │ │
│   │     └─ rclpy: /clock, /joint_states/command,    │ │
│   │                /sim/reset                        │ │
│   └─────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

## Quick start

```bash
./scripts/host/install.sh             # 호스트 prereq (base utils + docker + compose v2 + nvidia toolkit)
./scripts/host/install.sh --with-ros  # + ROS 2 Jazzy Desktop + ur_description (verify_ros.sh 용)
./scripts/host/fetch_assets.sh        # mujoco_menagerie + ur5e URDF 다운로드
./scripts/host/build.sh               # Docker 이미지 빌드 (5~15분)
./scripts/host/run.sh verify          # 컨테이너 스모크 테스트
./scripts/host/run.sh sim             # ROBOT=ur5e, freerun, VIEWER=rerun → http://localhost:9090
VIEWER=gl ./scripts/host/run.sh sim   # Newton GL viewer 창 (X11 필요)
VIEWER=none ./scripts/host/run.sh sim # headless
```

> 모든 `scripts/host/*.sh` 는 **재실행 안전 (idempotent)** — 이미 완료된 단계는 스킵합니다.

> `install.sh` 는 **fresh Ubuntu 24.04** 에서 base utils (git, rsync, curl, jq, xhost) +
> Docker Engine + compose v2 + (GPU 있으면) nvidia-container-toolkit 까지 설치합니다.
> `--only-check` 로 현재 상태만 점검, `--with-ros` 로 호스트 ROS 2 Jazzy Desktop + `ur_description`
> 까지 설치 (`verify_ros.sh` / `controller_demo.py` 용). **NVIDIA driver 는 감지만 하고 설치하지 않음**
> — 없으면 `sudo ubuntu-drivers autoinstall && sudo reboot` 후 재실행.

> **Viewer 선택** (`VIEWER` env): `rerun` (기본, 웹 UI @ `http://localhost:9090`,
> X11 불필요) · `gl` (X11 passthrough + nvidia GL 드라이버 필요, 창 닫으면 sim 종료) ·
> `usd` / `file` (`workspace/runs/<ts>.{usd,nvpr}` 로 녹화) · `null` (벤치마크) · `none`.
> `sync` 모드에서는 `/joint_command` 수신 시에만 step 이 진행되며, 렌더는 `sim.viewer_hz` (기본 60Hz) 로 물리 step rate 와 독립.

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
| `ur5e` | 6 | URDF (ur_description) | xpbd |
| `franka` | 7 | MJCF (mujoco_menagerie/franka_emika_panda) | mujoco |
| `kuka_iiwa_14` | 7 | MJCF (mujoco_menagerie/kuka_iiwa_14) | mujoco |

전환은 env var 하나: `ROBOT=kuka_iiwa_14 ./scripts/host/run.sh sim`.

새 로봇 추가 또는 외부 `*_description` 패키지(URDF / xacro / MJCF) 연동은 [docs/ROBOTS.md](docs/ROBOTS.md) 참고.

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
│   ├── robot_pack.py              robot.yaml loader
│   └── viewer.py                  optional Newton GL viewer
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
├── robots/                        ← pack = robot.yaml + models/ (docs/ROBOTS.md)
│   ├── ur5e/robot.yaml            6-DoF arm
│   ├── franka/robot.yaml          7-DoF arm
│   └── kuka_iiwa_14/robot.yaml    7-DoF arm
├── tests/                         pytest (host-side unit tests)
├── workspace/                     호스트↔컨테이너 공유 (outputs, notebooks)
├── assets/_cache/                 fetch_assets.sh 의 상류 클론 (gitignored, ~2 GB)
└── docs/
    ├── README.md                  docs 인덱스 + 역할별 진입점
    ├── INSTALL.md / USAGE.md / CONFIGURATION.md / VIEWER.md
    ├── TROUBLESHOOTING.md / EXAMPLES.md
    └── ARCHITECTURE.md / TOPICS.md / ROBOTS.md / DEFERRED_WORK.md
```

## 관련 문서

전체 목록 + 역할별 진입점은 [docs/README.md](docs/README.md).

- [docs/INSTALL.md](docs/INSTALL.md) — 호스트 prereq · 이미지 빌드 · verify
- [docs/USAGE.md](docs/USAGE.md) — `run.sh` 하위명령 + 일상 워크플로우
- [docs/CONFIGURATION.md](docs/CONFIGURATION.md) — env var + pack yaml 레퍼런스
- [docs/VIEWER.md](docs/VIEWER.md) — viewer 모드 (rerun/gl/usd/file/null/none)
- [docs/EXAMPLES.md](docs/EXAMPLES.md) — controller_demo · 센서 · Jupyter · 벤치
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — 단계별 실패 모드 + 조치
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — 레이어 경계, sync/time 모델
- [docs/TOPICS.md](docs/TOPICS.md) — 토픽/서비스 계약, 확장 경로
- [docs/ROBOTS.md](docs/ROBOTS.md) — 새 robot pack 추가 (URDF/xacro/MJCF)
- Newton 공식: <https://newton-physics.github.io/newton/latest/>
