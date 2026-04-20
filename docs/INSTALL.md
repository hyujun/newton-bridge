# Install / Build

호스트 prereq 설치부터 컨테이너 이미지 빌드까지의 one-shot 절차. 첫 사용자는 이 문서만 따라가면 `./scripts/host/run.sh verify` 통과 상태까지 도달합니다. 일상 사용법은 [USAGE.md](USAGE.md), 실행 중 문제는 [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## 전체 흐름

```
  [0] NVIDIA driver       수동 설치 (installer 가 건드리지 않음)
       │
  [1] install.sh          base utils + Docker + compose v2 + nvidia toolkit (+선택: ROS 2)
       │
  [2] fetch_assets.sh     URDF/MJCF 에셋 수급 (UR5e + mujoco_menagerie)
       │
  [3] build.sh            컨테이너 이미지 (5~15분)
       │
  [4] run.sh verify       11-섹션 스모크 테스트
```

모든 `scripts/host/*.sh` 는 **재실행 안전** (idempotent). 중간에 실패해도 문제 단계만 고쳐 재실행하면 됩니다.

---

## 0. Host prerequisites

Ubuntu **22.04 또는 24.04** 만 공식 지원. Python/ROS 조합 특성상 Debian 파생이 아니면 apt 스텝 수동 조정 필요.

| 요구 | 버전 | 확인 명령 | 출처 |
|---|---|---|---|
| NVIDIA driver | 545+ (권장 550+, `CUDA Version >= 12.4`) | `nvidia-smi` | **수동 설치** |
| Docker Engine | 최신 (24+) | `docker version` | install.sh |
| docker compose v2 | 최신 | `docker compose version` | install.sh |
| NVIDIA Container Toolkit | 최신 | `dpkg -s nvidia-container-toolkit` | install.sh |
| ROS 2 Jazzy (host) | Desktop | `dpkg -s ros-jazzy-desktop` | install.sh `--with-ros` |

**NVIDIA driver 는 installer 가 설치하지 않습니다.** 없으면 먼저:

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
# reboot 후
nvidia-smi   # CUDA Version: 12.4+ 확인
```

GPU 없는 머신은 본 repo 대부분의 기능이 동작하지 않습니다 (SDF collision / mesh-mesh contact / rerun 렌더링 / torch-cu12 전부 GPU 전용).

---

## 1. `install.sh` — 호스트 패키지 설치

```bash
cd /path/to/newton-bridge
./scripts/host/install.sh                  # 기본: docker + compose + (GPU 감지 시) toolkit
./scripts/host/install.sh --with-ros       # + ROS 2 Jazzy Desktop + ur_description
./scripts/host/install.sh --only-check     # 현재 상태만 출력, 변경 없음
./scripts/host/install.sh --no-nvidia      # GPU 있어도 toolkit 스킵
```

**설치 단계** (기존이면 스킵):

1. base utils — `git rsync ca-certificates curl gnupg lsb-release x11-xserver-utils jq`
2. Docker Engine + compose v2 — `download.docker.com` apt repo 등록 후 설치
3. `docker` 그룹 멤버십 — 현재 유저 추가 (재로그인 필요)
4. NVIDIA Container Toolkit — driver 있을 때만. `nvidia-ctk runtime configure --runtime=docker` 후 docker 재시작
5. (선택) ROS 2 Jazzy Desktop + `ur_description` — `packages.ros.org` 등록 후 설치

**`--with-ros` 를 언제?**
- 호스트에서 `verify_ros.sh`, `examples/controller_demo.py` 실행 예정이면 필요
- `fetch_assets.sh` 가 `ros2 pkg prefix ur_description` 로 UR5e URDF 경로를 찾으므로 UR5e 를 쓰려면 필요

### Docker 그룹 재로그인

`install.sh` 가 유저를 `docker` 그룹에 추가한 뒤 새 그룹은 **재로그인 후** 적용됩니다:

```bash
# 확인 — permission denied 면 아직 적용 안 됨
docker ps

# 재로그인 없이 즉시 적용 (현재 쉘만)
newgrp docker
docker ps   # 성공해야 함
```

> ⚠️ `docker` 그룹은 사실상 root (컨테이너로 호스트 FS 마운트 가능). 공용 호스트에서는 신중히.

### GPU 컨테이너 동작 검증

```bash
docker run --rm --gpus all nvcr.io/nvidia/cuda:12.4.0-base-ubuntu24.04 nvidia-smi
```

GPU 표가 나오면 OK. 실패하면 [TROUBLESHOOTING.md #GPU 런타임](TROUBLESHOOTING.md#gpu-런타임-오류).

---

## 2. `fetch_assets.sh` — 로봇 에셋 수급

URDF / MJCF / mesh 파일은 라이선스·크기 이유로 gitignore. 재실행 안전한 스크립트로 끌어옵니다.

```bash
./scripts/host/fetch_assets.sh
```

**무엇을 하는가**:

| Pack | 소스 | 배치 위치 |
|---|---|---|
| `ur5e` | apt `ros-jazzy-ur-description` → `/opt/ros/jazzy/share/ur_description` → `rsync` | `robots/ur5e/models/` |
| `franka` | `mujoco_menagerie/franka_emika_panda/` (shallow clone) → `cp` | `robots/franka/models/` |
| `kuka_iiwa_14` | `mujoco_menagerie/kuka_iiwa_14/` (shared clone) → `cp` | `robots/kuka_iiwa_14/models/` |

- `assets/_cache/mujoco_menagerie/` 에 shallow clone 캐시 (~2GB, gitignored). 재실행 시 이미 있으면 clone 스킵
- UR5e 는 `ros-jazzy-ur-description` 이 호스트에 있어야 함. 없으면 `install.sh --with-ros` 먼저

새 로봇 추가는 [ROBOTS.md](ROBOTS.md) — `fetch_assets.sh` 에 블록 추가하는 패턴.

---

## 3. `build.sh` — 컨테이너 이미지 빌드

```bash
./scripts/host/build.sh             # 일반 빌드 (레이어 캐시 사용, ~5분)
./scripts/host/build.sh --no-cache  # 완전 재빌드 (CUDA 런타임 다시 받음, ~15분)
```

**이미지 레이어** (`docker/Dockerfile`):

1. `nvidia/cuda:12.9.1-runtime-ubuntu24.04` base
2. locale + base tooling (curl, gnupg, sudo)
3. ROS 2 Jazzy apt repo 등록
4. ROS 2 ros-base + GL/EGL deps + python toolchain
5. Python venv (`/opt/newton-venv`, `--system-site-packages` 로 rclpy 공유)
6. `newton[examples,notebook,dev]` 설치
7. `newton[torch-cu12]` 설치 (CUDA 12.8 wheel 인덱스, ~2GB)
8. `pyproject.toml` + `src/` 복사 후 `pip install -e .` (editable)
9. entrypoint = `source /opt/ros/jazzy && exec CMD`

> `src/` 는 런타임에 `compose.yml` 이 bind-mount (ro) 로 덮어써서, 호스트 편집이 즉시 반영됩니다. Dockerfile 안의 `COPY src` 는 editable install 을 성립시키는 용도.

**HOST_UID 매칭**: `build.sh` 와 `run.sh` 가 `HOST_UID=$(id -u)` / `HOST_GID=$(id -g)` 를 자동 export. `sudo` 로 실행하면 root UID 로 고정되니 주의.

### 이미지 크기 대략

- CUDA runtime + ROS 2 base: ~2GB
- Newton + examples + notebook: ~1GB
- torch-cu12: ~2GB
- 총합 **~5GB** (재빌드 시 layer cache hit 시 2단계 pip 만 재실행)

---

## 4. `run.sh verify` — 빌드 검증

```bash
./scripts/host/run.sh verify
```

컨테이너 안에서 `scripts/container/verify.sh` 를 실행 (11 섹션):

| # | 확인 항목 |
|---|---|
| 1 | `warp.init()` + CUDA 장치 가시성 |
| 2 | Newton 모듈 import + 버전 |
| 3 | `newton.examples` 하위 모듈 walk |
| 4 | `ModelBuilder.finalize` + 10 step (revolute joint) |
| 5 | `rclpy` import (JointState/Trigger/Clock) |
| 6 | **모든 robot pack 파싱 + finalize + PD drive 반응** |
| 7 | `set_joint_targets` 가 pos/vel/effort 3채널을 `control` 버퍼에 write |
| 8 | per-joint drive override merge + `parse_drive_mode` |
| 9 | SensorContact 빌드 + NaN 없는 force 수집 |
| 10 | `VIEWER=null` 팩토리 round-trip |
| 11 | `sim.solver_params` pass-through + `set_gravity` |

**모두 PASS 이어야 다음 단계 (sim 기동) 로 넘어갑니다.** FAIL 시 섹션 이름이 힌트 — 가장 흔한 실패는 (a) driver/toolkit 미설치로 섹션 1 실패, (b) `fetch_assets.sh` 미실행으로 섹션 6 실패. [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 참조.

---

## 5. 첫 기동

```bash
# 터미널 A — sim 컨테이너
./scripts/host/run.sh sim

# 터미널 B — 호스트에서 ROS 2 토픽 확인
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4   # 필수 — SHM 금지
./scripts/host/verify_ros.sh
```

`ros2 topic hz /joint_states` 가 ~100Hz 로 뜨면 설치 + 빌드 성공. 이후 일상 사용은 [USAGE.md](USAGE.md).

### 호스트 ROS 2 환경 변수

컨테이너와 호스트가 DDS 토픽을 주고받으려면 **세 변수가 동일**해야 합니다. `.bashrc` 에 박아두면 편함:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4   # SHM 전송 금지 (UID 경계 때문)
```

`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 가 빠지면 **silent drop**: 컨테이너 안에서는 토픽 리스트가 보이는데 호스트는 안 보임.

---

## 6. `.env` 로 기본값 오버라이드 (선택)

`docker compose` 가 `.env.example` 을 참조하지 않고 `.env` 만 읽으므로 복사부터:

```bash
cp .env.example .env
${EDITOR:-nano} .env
```

자주 건드리는 변수:

| 변수 | 기본 | 용도 |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | 다른 도메인 쓰면 호스트/컨테이너 양쪽 맞추기 |
| `ROBOT_PACK` | `/workspace/robots/ur5e` | 기본 로봇 pack (셸에서 `ROBOT=franka` 가 더 간편) |
| `SYNC_MODE` | `freerun` | `handshake` 로 바꾸면 deterministic |
| `FREERUN_RATE` | `realtime` | `max` = 벤치마크 |
| `NVIDIA_VISIBLE_DEVICES` | `all` | `0` / `0,1` 로 GPU 격리 |
| `JUPYTER_TOKEN` | `newton` | `run.sh jupyter` 기본 토큰 |

전체 레퍼런스는 [CONFIGURATION.md](CONFIGURATION.md).

---

## 재실행 / 업데이트

### 코드 업데이트 (편집 중)

`src/` 는 bind-mount 이므로 호스트 편집이 즉시 반영 — **재빌드 불필요**. 컨테이너 프로세스만 재시작 (`Ctrl-C` 후 `run.sh sim`).

### 의존성 업데이트 (Dockerfile 수정)

```bash
./scripts/host/build.sh             # 수정된 레이어부터 재빌드
./scripts/host/run.sh verify        # smoke test 재실행
```

### 에셋 업데이트

```bash
./scripts/host/fetch_assets.sh      # 다른 pack 은 건드리지 않음
```

### 완전 초기화

```bash
./scripts/host/run.sh down                                # 컨테이너 제거
docker volume rm newton-bridge_newton-pip-cache 2>/dev/null  # pip 캐시
docker image rm newton-bridge:latest 2>/dev/null          # 이미지
rm -rf assets/_cache robots/*/models                      # 에셋 캐시 + 수급본
```

---

## 관련 문서

- [USAGE.md](USAGE.md) — 설치 후 일상 사용
- [CONFIGURATION.md](CONFIGURATION.md) — env var + pack yaml 레퍼런스
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 설치/빌드/실행 단계별 실패 모드
- [ARCHITECTURE.md](ARCHITECTURE.md) — 왜 이 구성인지 (단일 프로세스, network_mode:host)
