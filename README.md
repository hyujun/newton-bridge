# newton-bridge

**Newton Physics (standalone)** ↔ **ROS 2 Jazzy** 브리지. 호스트의 custom controller 와 topic/service 기반으로 sync 되는 시뮬레이터.

`network_mode: host` 로 컨테이너의 DDS 참여자가 호스트와 동일 도메인에 속하므로, 호스트에서 `ros2 topic list` 로 바로 `/clock`, `/joint_states`, `/joint_command` 가 보입니다.

## 한눈에

```
┌─ Host: Ubuntu 24.04 + ROS 2 Jazzy ──────────────────┐
│                                                      │
│   Custom controller  ─ pub /joint_command            │
│                      ─ sub /joint_states (use_sim_time)
│                      ─ call /sim/step  (handshake)   │
│                             │                        │
│   ┌─────────────────────────┴──────────────────────┐ │
│   │ Container: newton-bridge                        │ │
│   │   sim_node.py                                   │ │
│   │     ├─ Newton: ModelBuilder + solver step       │ │
│   │     └─ rclpy: /clock, /joint_states/command,    │ │
│   │                /sim/step, /sim/reset            │ │
│   └─────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

## Quick start

```bash
./scripts/fetch_assets.sh   # mujoco_menagerie + ur5e URDF 다운로드
./build.sh                  # Docker 이미지 빌드 (5~15분)
./run.sh verify             # 컨테이너 스모크 테스트
./run.sh sim                # ROBOT=ur5e, freerun, 기본
```

별도 터미널에서 (호스트):

```bash
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic hz /joint_states
python3 scripts/controller_demo.py --mode freerun --robot ur5e
```

## Supported robots

| Pack | DoF | Source | Solver (기본) |
|---|---|---|---|
| `ur5e` | 6 | URDF (ur_description) | xpbd |
| `franka` | 7 | MJCF (mujoco_menagerie/franka_emika_panda) | mujoco |
| `kuka_iiwa_14` | 7 | MJCF (mujoco_menagerie/kuka_iiwa_14) | mujoco |

전환은 env var 하나: `ROBOT=kuka_iiwa_14 ./run.sh sim`.

## Sync modes

| `SYNC_MODE=` | 동작 | 용도 |
|---|---|---|
| `freerun` (default) | sim 이 `FREERUN_RATE=realtime\|max` 로 자율 step | 관찰, loose-sync 제어, 데모 |
| `handshake` | 외부가 `/sim/step` 호출 시에만 1 step | deterministic RL rollout, 결정성 테스트 |

## 설치된 Newton extras (전체)

- `examples` — sim + importers + viewer (GL/USD)
- `torch-cu12` — PyTorch CUDA 12.8 (RL policy 추론)
- `notebook` — Jupyter + Rerun
- `dev` — 테스트/린트

Jupyter 는 `./run.sh jupyter` 로 host:8888 에.

## 디렉토리 레이아웃

```
newton-bridge/
├── README.md                    ← 여기
├── Dockerfile                   CUDA 12.4 + Ubuntu 24.04 + ROS 2 Jazzy + Newton[전체]
├── docker-compose.yml           GPU + X11 + network_mode:host + ROS env
├── docker/entrypoint.sh         source /opt/ros/jazzy + exec
├── .env.example                 기본값 스냅샷
├── build.sh / run.sh            이미지 빌드 / 컨테이너 제어
├── sim_node.py                  ★ Newton + rclpy 단일 프로세스
├── robots/
│   ├── ur5e/robot.yaml          6-DoF arm
│   ├── franka/robot.yaml        7-DoF arm
│   └── kuka_iiwa_14/robot.yaml  7-DoF arm
├── scripts/
│   ├── fetch_assets.sh          URDF/MJCF 수급
│   ├── verify.sh                컨테이너 내부 스모크
│   ├── verify_ros.sh            호스트 ROS 2 연동 체크
│   ├── controller_demo.py       sine-wave E2E 데모
│   └── rl_smoketest.py          torch-cu12 검증
├── workspace/                   호스트↔컨테이너 공유 (outputs, notebooks)
└── docs/
    ├── SETUP.md
    ├── ARCHITECTURE.md
    └── TOPICS.md
```

## 관련 문서

- [docs/SETUP.md](docs/SETUP.md) — 호스트 prereq, 최초 기동, DDS 설정
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — 레이어 경계, sync 모델, 시간 모델
- [docs/TOPICS.md](docs/TOPICS.md) — 토픽/서비스 계약, 확장 경로
- Newton 공식: <https://newton-physics.github.io/newton/latest/>

## sibling: sim-bridge

같은 `~/ros2_ws/` 안의 [sim-bridge](../sim-bridge/) 는 **Isaac Sim 컨테이너 내부**의 Newton 을 쓰는 경로입니다. newton-bridge 는 Isaac Sim 없이 standalone Newton 만 씀. Robot pack 계약은 두 repo 가 호환.
