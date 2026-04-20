# Adding a Robot

새 로봇을 `robots/<name>/` pack 으로 추가하거나, 외부 `*_description` ROS 2 패키지의 URDF/xacro/MJCF 를 끌어와서 붙이는 절차를 다룹니다. `newton_bridge` 는 로봇을 모른 채 pack 만 읽으므로, **pack 을 하나 더 만들면 = 새 로봇 지원** 입니다.

개념 요약은 [ARCHITECTURE.md §Robot pack 계약](ARCHITECTURE.md#robot-pack-계약), 토픽 계약은 [TOPICS.md](TOPICS.md) 참조.

## Pack 의 해부

```
robots/<name>/
├── robot.yaml         ← 레거시 단일 로봇 (여전히 지원; 자동 shim)
├── scene.yaml         ← 또는 (우선) canonical 멀티-articulation 스키마
└── models/            ← URDF 또는 MJCF + meshes/assets (gitignored)
    ├── <name>.urdf    #   URDF 경로의 경우
    │   └── meshes/    #   URDF 가 상대경로로 참조
    ├── <name>.xml     #   MJCF 경로의 경우
    └── assets/        #   MJCF 가 상대경로로 참조
```

- **tracked vs fetched**: 에셋(URDF/MJCF/STL)은 [.gitignore](../.gitignore) 에 따라 저장소에 안 올라갑니다. 라이선스/크기 이유. `robot.yaml` / `scene.yaml` 만 커밋.
- **경로 규약**: `source_rel` 는 pack 디렉토리 기준 상대경로 (`models/<name>.urdf` 또는 `models/<name>.xml`). 컨테이너 안에서는 `/workspace/robots/<name>/` 으로 bind-mount 됩니다.
- **로더 우선순위**: `scene.yaml` 이 있으면 그걸 우선 사용, 없으면 `robot.yaml` 을 자동으로 scene 형태로 승격 (shim). 둘 다 없으면 에러.
- **URDF/MJCF 통일**: 이전에는 `urdf/` 와 `mjcf/` 로 갈려 있었으나 현재는 둘 다 `models/` 하나로 통일. 로봇을 다른 format 으로 마이그레이션할 때 상위 경로가 그대로라 편함.

## 최소 `scene.yaml` 스켈레톤 (canonical Phase 2+)

```yaml
# robots/<name>/scene.yaml — 멀티-articulation 지원의 canonical 스키마.
sim:
  physics_hz: 400
  substeps: 1
  solver: mujoco          # xpbd | mujoco | featherstone
  ground_plane: true
  gravity: [0, 0, -9.81]

worlds:
  - label: env0
    gravity: [0, 0, -9.81]    # optional; sim.gravity override
    articulations:
      - label: arm           # 고유 이름 (ROS primary_articulation 참조용)
        source: urdf         # urdf | mjcf
        source_rel: models/<name>.urdf
        xform:
          pos: [0, 0, 0]
          rot: [0, 0, 0, 1]  # quaternion xyzw
        articulation_pattern: "*"   # fnmatch glob (ArticulationView 매칭)
        joint_names: [j1, j2, ...]  # ROS 노출 서브셋
        home_pose: {j1: 0.0, ...}
        drive: {mode: position, stiffness: 10000, damping: 100}
        joints:                    # per-joint 오버라이드 (Phase 3)
          j1: {effort_limit: 150, friction: 0.1}

ros:
  primary_articulation: arm  # 어떤 articulation 이 /joint_states 에 퍼블리시될지
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz:     100
  publish_tf:          true      # Phase 4
  tf_root_frame:       world
  publish_frames:      []        # [] = 전체, 또는 whitelist
```

**현재 제한 (Phase 2 스코프)**: 하나의 world + 하나의 articulation 만 실행. 멀티-articulation 또는 멀티-world 정의는 로더가 `NotImplementedError` 로 명확히 거부.

## 최소 `robot.yaml` 스켈레톤

```yaml
# robots/<name>/robot.yaml
robot:
  source: urdf            # urdf | mjcf
  source_rel: models/<name>.urdf
  base_position: [0.0, 0.0, 0.0]

sim:
  physics_hz: 400
  substeps: 1
  solver: xpbd            # xpbd | mujoco | featherstone
  ground_plane: true

joint_names:              # /joint_states 의 name 순서 = authoritative
  - joint_1
  - joint_2
  # ...

home_pose:                # reset 시 복귀 위치 (rad)
  joint_1:  0.0
  joint_2: -1.5708

drive:                   # top-level defaults applied to every actuated DOF
  mode: position         # position | velocity | effort | position_velocity | none
  stiffness: 10000.0     # target_ke (PD stiffness)
  damping:    100.0      # target_kd (PD damping)

# Optional per-joint overrides. Keys match any actuated DOF (exposed via
# joint_names or not). Missing fields inherit from top-level `drive:`.
# Non-drive scalars below (armature, effort_limit, velocity_limit, friction,
# limit_ke, limit_kd) are pushed directly into the Newton builder joint slot.
joints:
  shoulder_pan_joint:
    drive:
      mode: velocity
      damping: 5.0       # stiffness inherits 10000.0
    effort_limit: 150.0
    velocity_limit: 3.14
    armature: 0.01
    friction: 0.1

ros:
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz:     100

# Optional. fnmatch glob against model.articulation_label. Defaults to "*"
# which matches every articulation. Override when multiple robots share the
# same model and you want this pack to bind to only one of them.
articulation_pattern: "*"
```

`joint_names` 는 `/joint_states` / `/joint_command` 로 **노출할** 관절 목록입니다 (ROS 계약). Newton 이 파싱한 실제 DOF 는 더 많을 수 있습니다 — 예: franka 의 `finger_joint1/2` 는 pack 에서 빼면 시뮬레이션은 되지만 ROS 에는 나타나지 않습니다. 없는 joint 를 명시하면 fatal 이고, 순서는 ROS 메시지의 authoritative 순서입니다.

스키마 정의는 [ARCHITECTURE.md](ARCHITECTURE.md#robot-pack-계약), 실제 예시는 [robots/ur5e/robot.yaml](../robots/ur5e/robot.yaml), [robots/franka/robot.yaml](../robots/franka/robot.yaml), [robots/kuka_iiwa_14/robot.yaml](../robots/kuka_iiwa_14/robot.yaml).

## Solver 선택 가이드

| Source | 권장 solver | 이유 |
|---|---|---|
| URDF | `xpbd` 또는 `featherstone` | URDF 에는 actuator 블록 없음 → `SolverMuJoCo` 는 drive target 을 못 읽음 |
| MJCF (with `<actuator>`) | `mujoco` | MJCF 의 actuator/gain 이 solver 에 그대로 전달됨 |
| MJCF (no `<actuator>`) | `xpbd` / `featherstone` | URDF 와 동일 취급. `robot.yaml: drive.stiffness/damping` 이 사용됨 |

불일치는 `./scripts/host/run.sh verify` 의 섹션 6 ("load every robot pack") 에서 잡힙니다.

---

## 경로 A: URDF 기반 새 로봇 (external `*_description` 패키지)

`ros-jazzy-<robot>-description` apt 패키지나 워크스페이스의 xacro 매크로를 쓰는 가장 흔한 경로.

### 1) pack 디렉토리 생성

```bash
mkdir -p robots/myarm/models
```

### 2) URDF 확보 — 세 가지 소스

**(a) 이미 flat URDF 파일이 있는 경우**

```bash
cp /path/to/myarm.urdf robots/myarm/models/myarm.urdf
# mesh 가 상대경로면 meshes 디렉토리도 복사
cp -r /path/to/meshes/ robots/myarm/models/meshes/
```

**(b) apt 의 `*_description` 패키지 (호스트에서 변환)**

```bash
# 호스트 셸 — 컨테이너 밖에서 실행
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-myarm-description   # 예시

SHARE="$(ros2 pkg prefix myarm_description)/share/myarm_description"
ls "${SHARE}/urdf"   # .urdf 인지 .urdf.xacro 인지 확인
```

- 순수 URDF: `cp "${SHARE}/urdf/myarm.urdf" robots/myarm/models/`
- xacro: 아래 (c) 경로로.
- mesh 참조(`package://myarm_description/meshes/...`)가 있다면 §3 참조.

**(c) xacro → URDF 변환 (호스트에서)**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run xacro xacro \
    "${SHARE}/urdf/myarm.urdf.xacro" \
    name:=myarm \
    > robots/myarm/models/myarm.urdf
```

- `name:=...` 같이 xacro 가 요구하는 arg 는 패키지마다 다름. `xacro --inorder <file>` 로 필수 arg 목록을 얻을 수 있음.
- xacro 는 **ROS 2 환경이 source 된 상태에서만** `package://` 를 resolve 함. 컨테이너 안은 `myarm_description` 이 없으므로 호스트에서 변환하는 것이 원칙.

### 3) mesh 경로 정리

Newton 의 URDF 파서는 아래 두 가지만 처리합니다.

- **상대 경로**: `<mesh filename="meshes/base.stl"/>` — URDF 파일 기준 상대경로. `robots/myarm/models/myarm.urdf` ↔ `robots/myarm/models/meshes/base.stl` 구조면 그대로 동작.
- **`file://` 절대경로**: 컨테이너 안의 절대경로로 바꿔 둘 것.

`package://myarm_description/...` 은 **컨테이너 안에서 resolve 안 됩니다**. sed 한 번으로 치환:

```bash
sed -i 's|package://myarm_description/meshes|meshes|g' \
    robots/myarm/models/myarm.urdf
```

변환 전/후 diff 를 훑어 남는 `package://` 가 없는지 확인.

### 4) `robot.yaml` 작성

joint 이름은 URDF 의 `<joint name="...">` 에서 그대로 옵니다. grep 으로 뽑기:

```bash
grep -oP '<joint name="\K[^"]+' robots/myarm/models/myarm.urdf
```

- `type="fixed"` 는 `joint_names` 에 넣지 말 것 (DoF 가 없음).
- `type="revolute"` / `continuous"` / `prismatic"` 만 리스트에 포함.
- 리스트 순서가 그대로 `/joint_states.name` 에 실림 → 호스트 컨트롤러와 합의된 순서로 정렬.

예시 URDF → `robot.yaml` 매핑:

```yaml
robot:
  source: urdf
  source_rel: models/myarm.urdf
  base_position: [0.0, 0.0, 0.0]
sim:
  physics_hz: 400
  substeps: 1
  solver: xpbd              # URDF 니까 xpbd 권장
  ground_plane: true
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  # ... (grep 결과 그대로)
home_pose:
  shoulder_pan_joint:  0.0
  shoulder_lift_joint: -1.5708
drive:
  mode: position
  stiffness: 10000.0
  damping:    100.0
ros:
  joint_states_topic:  /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz:     100
```

### 5) `fetch_assets.sh` 에 한 블록 추가

[scripts/host/fetch_assets.sh](../scripts/host/fetch_assets.sh) 의 UR5e 블록(§4) 패턴을 그대로 따라가면 됩니다.

```bash
# -- 5) myarm ------------------------------------------------------------
log "populating robots/myarm/models"
mkdir -p robots/myarm/models

# 케이스 1: 다른 repo 의 파일을 rsync
MYARM_SRC="${REPO_ROOT}/../other-repo/robots/myarm"
if [[ -d "${MYARM_SRC}/urdf" ]]; then
    rsync -a --delete "${MYARM_SRC}/urdf/"   robots/myarm/models/
    rsync -a --delete "${MYARM_SRC}/meshes/" robots/myarm/models/meshes/ 2>/dev/null || true
# 케이스 2: apt 의 *_description 에서 xacro 변환
elif dpkg -s ros-jazzy-myarm-description >/dev/null 2>&1; then
    SHARE="$(ros2 pkg prefix myarm_description)/share/myarm_description"
    ros2 run xacro xacro "${SHARE}/urdf/myarm.urdf.xacro" \
        > robots/myarm/models/myarm.urdf
    rsync -a "${SHARE}/meshes/" robots/myarm/models/meshes/ 2>/dev/null || true
    sed -i 's|package://myarm_description/meshes|meshes|g' \
        robots/myarm/models/myarm.urdf
else
    die "no myarm source: set up ../other-repo or apt install ros-jazzy-myarm-description"
fi
```

재실행 안전(idempotent)하게 쓰는 것이 규칙 — `rm -rf` 후 `rsync -a --delete` 로 교체.

### 6) `.gitignore` 확인

기본 `.gitignore` 에 `robots/*/models/` 가 이미 포함되어 있어서 새 pack 도 자동으로 무시됩니다. 추가 조치 불필요.

---

## 경로 B: MJCF 기반 새 로봇 (mujoco_menagerie 등)

### 1) 소스 확보

[scripts/host/fetch_assets.sh](../scripts/host/fetch_assets.sh) 가 `mujoco_menagerie` 를 `assets/_cache/` 에 이미 clone 해 두므로 그걸 재활용.

```bash
# fetch_assets.sh 에 블록 추가
log "populating robots/yourmjcf/models"
mkdir -p robots/yourmjcf/models
rm -rf robots/yourmjcf/models/*
cp -r "${MENAGERIE}/<dir_in_menagerie>/"*.xml robots/yourmjcf/models/
cp -r "${MENAGERIE}/<dir_in_menagerie>/assets" robots/yourmjcf/models/ 2>/dev/null || true
```

Menagerie 외의 곳에서 오는 MJCF 도 동일하게 `robots/<name>/models/<name>.xml` + 인접 `assets/` 레이아웃을 지키면 됩니다.

### 2) `robot.yaml` 작성

```yaml
robot:
  source: mjcf
  source_rel: models/yourmjcf.xml
  base_position: [0.0, 0.0, 0.0]
sim:
  physics_hz: 400
  substeps: 1
  solver: mujoco            # MJCF 에 <actuator> 있으면 mujoco 권장
  ground_plane: true
joint_names:
  # MJCF 의 <joint name="..."> 에서 추출
  - joint1
  - joint2
# ...
```

joint 이름 추출:

```bash
grep -oP '<joint[^>]*name="\K[^"]+' robots/yourmjcf/models/yourmjcf.xml
```

- MJCF 에는 `<joint type="free">` 가 섞이는 경우가 있음. base link 의 free joint 는 `floating=False` 로 parse 하면 `joint_names` 에 안 들어가지만, 일부 씬에서는 강제로 노출될 수 있으니 `scripts/container/verify.sh` 로 확인.
- Gripper finger 는 의도적으로 제외하거나, 쓰고 싶다면 추가 — [franka pack 주석](../robots/franka/robot.yaml) 참고.

---

## 경로 C: xacro 를 컨테이너 안에서 변환하고 싶을 때

기본 전략은 **호스트 xacro → 결과 URDF commit-ignore** 지만, CI 등에서 reproducibility 가 필요하면 컨테이너 안에 xacro 를 설치할 수 있습니다.

```dockerfile
# docker/Dockerfile 조각
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
```

그리고 `fetch_assets.sh` 의 xacro 호출을 `docker compose -f docker/compose.yml run --rm newton-bridge ros2 run xacro xacro ...` 로 감싸면 됩니다. 비용: 이미지 크기 ~수십 MB 증가. 현재 repo 는 이 경로를 쓰지 않음 (호스트가 이미 ROS 2 Jazzy 보유 전제).

---

## 검증 루프 (모든 경로 공통)

```bash
# 1) 에셋이 제대로 배치됐는지
./scripts/host/fetch_assets.sh
ls robots/myarm/models/

# 2) pack 이 파싱 + finalize + 5-step 되는지 (컨테이너 내부)
./scripts/host/run.sh verify
# ↑ 섹션 6 에서 "pack myarm parses + finalizes ... ok: dof=N" 이 떠야 PASS

# 3) sim 실제 기동 + ROS 2 토픽 확인
ROBOT=myarm ./scripts/host/run.sh sim
# 다른 터미널(호스트):
source /opt/ros/jazzy/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

`/joint_states.name` 순서가 `robot.yaml: joint_names` 와 정확히 일치해야 합니다.

---

## 자주 만나는 실패 모드

| 증상 | 원인 | 조치 |
|---|---|---|
| `joints in robot.yaml not found in Newton model: [...]` | 이름이 URDF/MJCF 의 실제 joint 와 안 맞음 (철자/접두사) | `verify.sh` 가 출력하는 "available joints: [...]" 를 복사해서 `joint_names` 갱신 |
| mesh 로드 실패, SDF/STL not found | URDF 가 `package://...` 를 그대로 참조 | §A.3 의 `sed` 로 상대경로/절대경로로 치환 |
| `/joint_states` 는 뜨는데 로봇이 꿈틀대기만 함 (MJCF) | `solver: mujoco` 인데 MJCF 에 `<actuator>` 가 없음 | `solver: xpbd` 로 바꾸고 `drive.stiffness/damping` 부여 |
| `/joint_states` 가 뜨는데 값이 NaN 또는 폭주 | PD gain 너무 높음, 또는 `physics_hz` 대비 `substeps` 부족 | `stiffness` 를 10× 줄이거나 `substeps: 2~4`, `physics_hz: 1000` 시도 |
| 다중-DoF joint (spherical, free) 의 home_pose 가 틀림 | `_apply_home_pose` 는 첫 component 만 세팅 ([src/newton_bridge/world.py](../src/newton_bridge/world.py)) | multi-DoF 를 쓰려면 `world.py` 의 home_pose 로직을 per-component 로 확장 |
| handshake 에서 state 가 업데이트 안 됨 | `/sim/step` 콜 없이 `/joint_states` 기다림 | `ros2 service call /sim/step std_srvs/srv/Trigger {}` 로 한 step |

더 깊은 디버깅은 [ARCHITECTURE.md §알려진 이슈](ARCHITECTURE.md#알려진-이슈) 참조.

---

## Checklist — 새 로봇 PR 전

- [ ] `robots/<name>/robot.yaml` 만 Git 에 추가 (에셋은 .gitignore 기본 규칙으로 자동 제외)
- [ ] `scripts/host/fetch_assets.sh` 에 재실행 안전한 블록 추가
- [ ] `./scripts/host/fetch_assets.sh` 재실행 clean 통과
- [ ] `./scripts/host/run.sh verify` 섹션 6 에서 PASS
- [ ] `ROBOT=<name> ./scripts/host/run.sh sim` + 호스트에서 `ros2 topic hz /joint_states` 정상 rate
- [ ] `README.md` 의 "Supported robots" 표에 한 줄 추가
