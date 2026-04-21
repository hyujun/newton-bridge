# Troubleshooting

설치/빌드/실행 단계별로 자주 만나는 문제와 조치. 카테고리별로 시작하는 증상만 훑어도 대체로 찾을 수 있습니다.

## 단계 인덱스

- [설치 (install.sh)](#설치-단계)
- [에셋 수급 (fetch_assets.sh)](#에셋-수급-단계)
- [이미지 빌드 (build.sh)](#이미지-빌드-단계)
- [컨테이너 기동 / verify](#컨테이너-기동-단계)
- [Sim 실행 + ROS 2 연동](#sim-실행-단계)
- [Viewer](#viewer-문제)
- [Robot pack](#robot-pack-문제)

---

## 설치 단계

### GPU 런타임 오류

**증상**: `docker run --rm --gpus all nvcr.io/.../cuda:*-base-ubuntu24.04 nvidia-smi` 실행 시:

```
docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
```

**원인**: NVIDIA Container Toolkit 미설치, 또는 `nvidia-ctk runtime configure` 가 docker daemon 에 반영 안 됨.

**조치**:
```bash
./scripts/host/install.sh --only-check   # 'nvidia docker runtime' 상태 확인
./scripts/host/install.sh                # 누락 시 자동 설치 + systemctl restart docker
# 그래도 실패하면 수동:
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker info | grep -i runtime            # nvidia 가 보여야 함
```

호스트에 driver 가 없으면 먼저:

```bash
nvidia-smi                               # command not found 면 driver 부재
sudo ubuntu-drivers autoinstall
sudo reboot
```

### `docker ps` → permission denied

**증상**: `permission denied while trying to connect to the docker API at unix:///var/run/docker.sock`

**원인**: 유저가 `docker` 그룹 멤버가 아니거나, 그룹 추가 후 재로그인 안 함.

**조치**:
```bash
id -nG | grep docker                     # 확인
sudo usermod -aG docker "$USER"          # 없으면 추가
newgrp docker                            # 현재 쉘 즉시 반영
docker ps                                # 성공해야 함
```

VSCode 등 GUI 툴은 **재로그인** (쉘 newgrp 만으로는 전달 안 됨).

### `install.sh` 가 NVIDIA driver 를 안 깜

**설계**: installer 는 driver 를 설치하지 않음 — Ubuntu 업데이트/재부팅 간 간섭을 피하기 위함. 수동 설치:

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
# 재부팅 후
nvidia-smi                               # CUDA Version: 12.4+ 확인
./scripts/host/install.sh                # 이제 toolkit 설치 진행
```

---

## 에셋 수급 단계

### `ros-jazzy-ur-description not installed`

**증상**: `fetch_assets.sh` 실행 시

```
[fetch] ros-jazzy-ur-description not installed.
  install it with:
    ./scripts/host/install.sh --with-ros
```

**조치**: `--with-ros` 플래그 재실행.

```bash
./scripts/host/install.sh --with-ros
./scripts/host/fetch_assets.sh
```

### `mujoco_menagerie` clone 실패

**증상**: git clone 단계에서 network error, 또는 `assets/_cache/mujoco_menagerie/` 가 불완전.

**조치**:
```bash
rm -rf assets/_cache/mujoco_menagerie    # 부분 clone 제거
./scripts/host/fetch_assets.sh           # 재시도
```

프록시 환경이면 `HTTP_PROXY`/`HTTPS_PROXY` export 후 재실행.

### mesh / xml 이 `robots/<name>/models/` 에 없음

**증상**: `fetch_assets.sh` 는 성공했는데 pack 빌드 시 mesh not found.

**조치**:
```bash
ls robots/ur5e/models/                   # *.urdf + meshes/ 가 있어야 함
ls robots/franka/models/                 # *.xml + assets/
ls robots/kuka_iiwa_14/models/           # *.xml + assets/
```

비어 있으면 `rm -rf robots/*/models/*` 후 `fetch_assets.sh` 재실행 — 해당 블록만 다시 배치.

---

## 이미지 빌드 단계

### `pip install newton` 실패

**증상**: `build.sh` 중간에 wheel 빌드 에러.

**가능성**:
- 인터넷 접근 문제 — `docker build` 의 `RUN pip install` 단계는 빌더 네트워크 필요
- Python 3.10 환경에서 `imgui_bundle` 빌드 충돌 — 본 repo 는 24.04 + Python 3.12 고정 (Dockerfile 에서)
- 메모리 부족 — `free -g` 가 4GB 미만이면 wheel 빌드 중 OOM 가능

**조치**:
```bash
./scripts/host/build.sh --no-cache       # 레이어 캐시 무시하고 재빌드
```

### `torch-cu12` 레이어가 매번 재다운로드

**증상**: `newton[torch-cu12]` 가 ~2GB 재다운로드.

**원인**: `pip` 캐시 볼륨 `newton-bridge_newton-pip-cache` 가 삭제됨.

**조치**: 볼륨이 살아 있는지 확인.

```bash
docker volume ls | grep newton-pip-cache
```

없어졌다면 다음 빌드 때 재생성되고, 이후 호출부터는 캐시 hit.

### 이미지가 너무 큼

**설계값**: ~5GB (CUDA + ROS + Newton full extras). 줄이려면 `newton[examples,notebook,dev]` 에서 불필요 extra 제거 — 단 `examples` 없이는 viewer 가 안 됨.

---

## 컨테이너 기동 단계

### `verify.sh` 섹션 1 FAIL — CUDA 안 보임

**증상**: `warp.init() + cuda devices` 에서 `no CUDA devices visible`.

**원인**: 컨테이너에 GPU 가 안 붙음.

**조치**:
```bash
docker compose -f docker/compose.yml run --rm newton-bridge nvidia-smi
# 결과 없으면 runtime: nvidia 가 적용 안 됨
```

- `compose.yml` 의 `runtime: nvidia` 가 살아 있는지 확인
- GPU 호스트 테스트 (위 "GPU 런타임 오류") 통과했는지
- `NVIDIA_VISIBLE_DEVICES=all` (또는 특정 idx) 가 `.env` 에서 빈 문자열로 덮이지 않았는지

### `verify.sh` 섹션 6 FAIL — `pack X parses + finalizes` 실패

**원인 후보**:

1. **에셋 누락** — `fetch_assets.sh` 미실행 또는 `models/` 비어 있음
   ```bash
   ls robots/*/models/
   ./scripts/host/fetch_assets.sh
   ```

2. **joint name 불일치** — `pack['joint_names']` 가 URDF/MJCF 의 실제 joint 와 안 맞음
   ```
   pack['joint_names'] references joints not in ArticulationView DOFs.
     missing:    ['shoulder_pan']
     actual:     ['shoulder_pan_joint', ...]
   ```
   `robot.yaml: joint_names` 를 "actual" 값으로 교정.

3. **URDF + wrong solver** — URDF + `SolverXPBD` 조합이 Newton 1.1.0 에서 drive 를 적용 못 함. pack 의 `sim.solver: mujoco` 로 변경.

4. **mesh `package://` 경로** — URDF 가 `package://ur_description/...` 를 그대로 참조. sed 로 상대경로 치환:
   ```bash
   sed -i 's|package://ur_description/meshes|meshes|g' robots/ur5e/models/ur5e.urdf
   ```

### `verify.sh` 섹션 9 FAIL — SensorContact

**증상**: `SensorContact builds + publishes without NaN` 실패.

**원인**: Newton 1.1.0 의 SensorContact API 는 dev build 간 시그니처 차이 있음. [sensors.py](../src/newton_bridge/sensors.py) 의 `_contact_targets` 가 `bodies`/`shapes` 키를 그대로 forward 하므로, Newton 쪽 deprecation 이 있으면 패치 필요.

조치: Newton 버전 확인 후 [src/newton_bridge/sensors.py](../src/newton_bridge/sensors.py) 업데이트. `SensorIMU` 도 같은 방식 — `_first_attr` 가 여러 attribute 이름을 시도함.

---

## Sim 실행 단계

### 호스트에서 `ros2 topic list` 에 sim 토픽이 안 뜸

**증상**: 컨테이너 안에서는 `/joint_states` 가 보이는데 호스트에서는 빈 리스트.

**원인 1 — RMW 불일치**: 컨테이너와 호스트의 `RMW_IMPLEMENTATION` 이 다르면 discovery 실패. 컨테이너 기본은 `rmw_cyclonedds_cpp`.

**조치 (호스트 쉘)**:
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

**원인 2 — FastDDS SHM 깨짐** (FastDDS 를 쓰는 경우에만): SHM 전송이 컨테이너/호스트 UID 경계에서 조용히 실패.

**조치**:
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4   # FastDDS 에서만 필수
ros2 topic list
```

**원인 3 — Cyclone 멀티캐스트 차단**: 네트워크가 멀티캐스트를 막으면 Cyclone discovery 도 실패 (host network mode 에선 드묾). `CYCLONEDDS_URI` 로 XML 설정을 주입해 loopback/unicast 로 고정:

```xml
<!-- ~/.cyclonedds.xml -->
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers><Peer address="localhost"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```
```bash
export CYCLONEDDS_URI=file:///home/$USER/.cyclonedds.xml
```
컨테이너와 호스트 양쪽에서 같은 파일을 참조해야 하며, compose.yml 의 `environment` 에 `- CYCLONEDDS_URI=${CYCLONEDDS_URI-}` 를 추가하고 XML 을 bind-mount 로 얹으면 됨.

`.bashrc` 에 env 를 박아두면 영구. 컨테이너 기본값은 `compose.yml` 에 이미 세팅되어 있음.

### 컨테이너에서 `ros2 topic hz /joint_states` 가 0Hz

**freerun 모드**: `publish_rate_hz` 가 호스트 ROS 2 노드 기준이므로 호스트에서 측정. 컨테이너 안에서는 `ros2 topic hz` 가 sub 를 뜨면서 DDS discovery 타이밍이 꼬일 수 있음.

**sync 모드**: `/joint_command` 가 한 번도 publish 되지 않았고 `sync_timeout_ms` (기본 100ms) 도 안 지났다면 초기 startup publish 1회만 보임. 주기적 command 를 내리거나 `controller_demo.py --mode sync` 로 루프.

### `/joint_command` 를 퍼블리시해도 로봇이 안 움직임

**가능성**:

1. **joint 이름 오타** — 메시지 `name` 이 pack `joint_names` 에 없으면 조용히 무시됨
   ```bash
   ros2 topic echo --once /joint_states   # name 배열 복사해서 사용
   ```

2. **drive mode 불일치** — `pack.drive.mode: position` 인데 `/joint_command.position` 이 비어 있음 (velocity 만 보냄)

3. **solver 가 drive 를 적용 안 함** — URDF + XPBD 조합. solver 를 `mujoco` 로.

4. **PD gain 이 너무 낮음** — `stiffness: 10.0` 같은 값은 중력이 이김. UR5e 기준 `10000.0`.

진단:
```bash
# 컨테이너 안에서
python3 - <<'PY'
import os; os.environ['ROBOT_PACK']='/workspace/robots/ur5e'
import warp as wp; wp.init()
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld
world = NewtonWorld(load_pack(Path(os.environ['ROBOT_PACK'])))
j0 = world.exposed_joint_names[0]
q0 = world.read_joint_positions()[j0]
world.set_joint_targets([j0], positions=[q0 + 0.5])
for _ in range(200): world.step()
print(j0, 'Δq =', world.read_joint_positions()[j0] - q0)
PY
```

Δq 가 `0.0` 에 가까우면 solver/drive 문제. 0.4+ 면 `/joint_command` 경로 문제 (DDS 또는 joint name).

### state 값이 NaN 또는 폭주

**원인**:
- PD gain 이 너무 높음 (`stiffness: 100000` 급)
- `physics_hz` 대비 `substeps` 부족 (400Hz + substeps=1 은 stiff system 에서 한계)
- solver-joint 미스매치 (URDF + XPBD)

**조치**:
```yaml
sim:
  physics_hz: 1000     # 더 세밀
  substeps: 2
drive:
  stiffness: 1000.0    # 10× 감소
```

### sync 에서 state 가 업데이트 안 됨

`/joint_command` publish 필요 (publish 1회 = 1 step):

```bash
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState \
  "{name: [], position: [], velocity: [], effort: []}"
```

빈 배열은 "channel 건드리지 않음" 으로 해석되어 target 은 그대로 두고 step 만 진행됩니다. `/joint_states` 가 새 stamp 로 돌아오는지 확인하세요.

`/joint_command` 가 끊겼는데도 `/joint_states` 가 `sync_timeout_ms` (기본 100ms) 주기로 오면 idle watchdog 동작 — step 은 안 일어나고 현재 상태만 재퍼블리시.

---

## Viewer 문제

### Rerun 웹뷰어가 `http://localhost:9090` 에서 안 뜸

**확인**:
1. sim 이 `VIEWER=rerun` (기본) 으로 기동되었는지 — `VIEWER=none` 이면 포트가 안 열림
2. 포트 충돌 — `ss -lntp | grep 9090`
3. 원격 머신이면 SSH tunnel 필요:
   ```bash
   ssh -L 9090:localhost:9090 user@remote
   ```
4. `network_mode: host` 라 포트 매핑 아니라 호스트에 직접 바인드

### GL viewer 가 안 뜸

**증상**: `VIEWER=gl` 로 기동 시 `Xlib.error.DisplayConnectionError` 또는 검은 화면.

**조치**:

1. 호스트에서 X 서버 동작 확인:
   ```bash
   echo "$DISPLAY"          # :0 또는 :1
   xhost                    # access control 상태
   xhost +local:docker      # run.sh 가 해주지만 수동 가능
   ```

2. `$XAUTHORITY` 가 유효한 파일인지:
   ```bash
   ls -l "$XAUTHORITY"      # 보통 ~/.Xauthority
   ```

3. Wayland 세션이면 XWayland 가 동작하는지. 안 되면 로그인 시 Xorg 세션 선택.

4. `NVIDIA_DRIVER_CAPABILITIES` 에 `graphics,display` 포함:
   ```bash
   docker compose -f docker/compose.yml run --rm newton-bridge \
       env | grep NVIDIA_DRIVER
   ```

로그 확인:
```bash
./scripts/host/run.sh sim 2>&1 | grep -i viewer
# "[newton_bridge] VIEWER=gl init failed: ..." 로 이유 출력
```

실패해도 sim 은 headless 로 계속 진행됩니다 ([__main__.py:62](../src/newton_bridge/__main__.py#L62)).

### Viewer 가 frozen (sync 모드)

**의도된 동작**: sync 는 `/joint_command` 수신 시에만 sim 이 진행하고 `sim.viewer_hz` 에 맞춰 프레임을 갱신. 다른 터미널에서 command 를 publish:

```bash
ros2 topic pub -r 60 /joint_command sensor_msgs/msg/JointState \
  "{name: [], position: [], velocity: [], effort: []}"
```

또는 freerun 으로 전환:

```bash
SYNC_MODE=freerun VIEWER=gl ./scripts/host/run.sh sim
```

### `ENABLE_VIEWER` 를 썼더니 exit 2

**설계**: Phase 7 에서 `ENABLE_VIEWER` 를 제거. 사용 시 `run.sh` 가 명시적 에러:

```
[run.sh] ERROR: ENABLE_VIEWER is deprecated. Use VIEWER=gl (or rerun/usd/file/null/none).
```

`VIEWER=gl` 로 대체.

---

## Robot pack 문제

| 증상 | 원인 | 조치 |
|---|---|---|
| `joints in robot.yaml not found in Newton model: [...]` | URDF/MJCF 실제 joint 와 이름 불일치 | `verify.sh §6` 출력의 "actual" 리스트로 교체 |
| `pack.primary_articulation=... not among world articulations` | `scene.yaml` 의 `ros.primary_articulation` 이 오타 | articulation label 과 일치시킴 |
| `multi-world scenes are not yet supported` | `worlds:` 에 2개 이상 | Phase 2b 구현 전까지 단일 world 로 축소. [DEFERRED_WORK.md](DEFERRED_WORK.md) |
| `multi-articulation worlds are not yet supported` | 한 world 에 articulation 2개 이상 | 위와 동일 |
| mesh 로드 실패 / SDF not found | URDF 에 `package://...` 잔존 | sed 로 상대경로 치환 ([ROBOTS.md §A.3](ROBOTS.md)) |
| `/joint_states` 는 뜨는데 로봇이 꿈틀대기만 (MJCF) | `solver: mujoco` 인데 MJCF 에 `<actuator>` 없음 | `solver: xpbd` + `drive.stiffness/damping` 설정 |
| `/joint_states` 값이 NaN / 폭주 | PD gain 높음, 또는 substeps 부족 | gain 10× 감소, `physics_hz: 1000`, `substeps: 2~4` |
| multi-DoF joint home_pose 틀림 | `_apply_home_pose` 가 첫 component 만 세팅 | [world.py](../src/newton_bridge/world.py) 에서 per-component 로 확장 |
| `solver_params` 관련 `ValueError` | 잘못된 키워드 | 에러 메시지의 kwargs 확인 후 제거 |
| SensorIMU `sites` 경고 | URDF 에는 site 개념 없음 | MJCF pack 에서만, 또는 수동 `builder.add_site()` |

---

## VSCode Container Tools 사이드바가 비어있음

**원인**: `ms-azuretools.vscode-containers` 확장이 docker socket 접근 실패.

**조치**:
```bash
docker ps                                # permission denied 면 그룹 문제
sudo usermod -aG docker "$USER"          # 없으면 추가
newgrp docker                            # 쉘 즉시 반영
# VSCode 완전 재시작 (Reload Window 로는 env 재적용 안 됨)
```

스택이 떠 있어야 트리에 컨테이너가 보입니다:

```bash
./scripts/host/run.sh sim                # 또는
./scripts/host/run.sh upd
```

---

## 리셋 / 완전 초기화

```bash
# 1) 컨테이너 제거
./scripts/host/run.sh down
docker ps -a | grep newton-bridge && docker rm -f $(docker ps -aq --filter name=newton-bridge)

# 2) 이미지 + 볼륨
docker image rm newton-bridge:latest 2>/dev/null
docker volume rm newton-bridge_newton-pip-cache 2>/dev/null

# 3) 에셋 캐시 + 수급본
rm -rf assets/_cache robots/*/models/*

# 4) 재수급 + 재빌드
./scripts/host/fetch_assets.sh
./scripts/host/build.sh
./scripts/host/run.sh verify
```

---

## 로그 수집 (버그 리포트용)

```bash
# 호스트 환경
uname -a > bug_report.txt
lsb_release -a >> bug_report.txt
nvidia-smi >> bug_report.txt
docker version >> bug_report.txt
docker info | grep -iE "runtime|nvidia" >> bug_report.txt

# 이미지 상태
docker image inspect newton-bridge:latest | head -40 >> bug_report.txt

# verify.sh 전체 출력
./scripts/host/run.sh verify > verify.log 2>&1

# sim 기동 로그 (30초)
timeout 30 ./scripts/host/run.sh sim > sim.log 2>&1 || true
```

---

## 추가 참고

- [ARCHITECTURE.md §알려진 이슈](ARCHITECTURE.md#알려진-이슈) — Newton API 안정성, joint 이름 매핑
- [ROBOTS.md #자주 만나는 실패 모드](ROBOTS.md#자주-만나는-실패-모드) — pack 작성 단계별
- [DEFERRED_WORK.md](DEFERRED_WORK.md) — 의도적 미구현 경로
