# Setup

## Host prerequisites (Ubuntu 24.04)

| | 요구 버전 | 확인 |
|---|---|---|
| NVIDIA driver | 545+ (권장 550+) — `CUDA Version >= 12.4` | `nvidia-smi` |
| Docker Engine | 최신 | `docker version` |
| NVIDIA Container Toolkit | 최신 | `dpkg -s nvidia-container-toolkit` |
| ROS 2 | Jazzy | `dpkg -s ros-jazzy-ros-base` |

### 한 번만 실행

```bash
# 1) NVIDIA driver 확인
nvidia-smi           # CUDA Version: 12.4+

# 2) Docker + NVIDIA Container Toolkit (sim-bridge 셋업 했다면 이미 되어 있음)
#    설치 방법은 Newton 공식 가이드 §2 참조.

# 3) ROS 2 Jazzy (호스트용)
#    https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-jazzy-ros-base

# 4) GPU 컨테이너 동작 검증
docker run --rm --gpus all nvcr.io/nvidia/cuda:12.4.0-base-ubuntu24.04 nvidia-smi
```

### 호스트 ROS 2 환경 변수

컨테이너와 호스트가 서로 DDS 토픽을 주고받으려면 **반드시 동일 값**이어야 합니다. 호스트 쉘에서 (`.bashrc` 에 박아두면 편함):

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4   # SHM 금지 (root 컨테이너 ↔ 일반 사용자 호스트)
```

`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 를 빼먹으면 컨테이너 내부에서 `ros2 topic list` 는 되는데 호스트에서는 토픽이 안 뜨는 **silent drop** 이 발생합니다.

## 저장소 셋업

```bash
cd ~/ros2_ws
git clone <this-repo> newton-bridge    # 또는 현재 위치에 이미 있음
cd newton-bridge

# (선택) 기본값 바꾸고 싶으면
cp .env.example .env
${EDITOR:-nano} .env

# 로봇 에셋 다운로드 (mujoco_menagerie + UR5e URDF)
./scripts/fetch_assets.sh

# 컨테이너 이미지 빌드 (5~15분)
./build.sh

# 빌드 검증
./run.sh verify
```

`verify` 가 5/6 섹션 PASS 로 끝나면 세팅 완료. FAIL 나오면 [TROUBLESHOOTING](ARCHITECTURE.md#알려진-이슈) 참고.

## 최초 실행

```bash
# 터미널 A — sim 기동 (freerun, ur5e 기본)
./run.sh sim

# 터미널 B — 호스트 ROS 2 로 연동 확인
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./scripts/verify_ros.sh
```

`ros2 topic hz /clock`, `ros2 topic hz /joint_states` 가 목표 rate 근처로 떠야 정상.

## 다른 로봇 / 다른 sync mode

```bash
ROBOT=franka ./run.sh sim                             # freerun, franka
ROBOT=kuka_iiwa_14 SYNC_MODE=handshake ./run.sh sim   # handshake, iiwa14
FREERUN_RATE=max ./run.sh sim                         # as-fast-as-possible
```

## GUI viewer (Newton 자체 뷰어)

ROS 2 연동 없이 Newton 샘플 씬을 띄우고 싶을 때:

```bash
./run.sh example basic_pendulum --viewer gl
```

`DISPLAY` 가 세팅돼 있으면 호스트 X 서버로 창이 뜸. 헤드리스면 `--viewer usd --output-path /workspace/workspace/outputs/xxx.usd` 로 파일 저장.

## Jupyter

```bash
./run.sh jupyter
# 브라우저로 http://localhost:8888/?token=newton
```

## 주의사항

- **Python 3.10 불가**: Newton 가이드 §9 의 `imgui_bundle` 빌드 이슈 때문. 이 repo 는 Ubuntu 24.04 + Python 3.12 로 통일.
- **GPU 전용 기능**: SDF collision, mesh-mesh contact, tiled camera sensor, implicit MPM 등은 NVIDIA GPU 필수. 호스트에 GPU 없으면 이 repo 의 대부분이 안 돕니다.
- **컨테이너 UID 매칭**: `docker-compose.yml` 의 `user: ${UID}:${GID}` 가 호스트 UID 를 따라가도록 되어 있음. `./build.sh` 와 `./run.sh` 는 자동으로 `UID=$(id -u)` 를 export. `sudo` 로 실행하면 루트 UID 로 세팅 파일이 만들어지니 주의.
