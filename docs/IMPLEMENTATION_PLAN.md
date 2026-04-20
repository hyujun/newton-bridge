# Newton 기능 확장 구현 계획

본 문서는 newton-bridge가 Newton 1.1.0의 주요 기능 대부분을 호환·노출하도록
확장하는 **일회성** 마이그레이션 가이드입니다. 모든 phase가 완료되면 이 파일은
삭제됩니다 (README / docs 본문으로 흡수).

**생성**: 2026-04-20
**대상 Newton 버전**: 1.1.0 (warp 1.12.1, CUDA 12.9)
**시작 커밋**: a140779 (master)

---

## Phase 0 — API snapshot (완료)

**목표**: Newton 1.1.0의 expose된 API를 파악해 후속 phase의 근거로 삼는다.

### 산출물 요약

- Top-level: `newton.{ModelBuilder, Model, State, Control, Contacts, JointTargetMode, JointType, Axis, BodyFlags, GeoType, ParticleFlags, ShapeFlags, EqType, Mesh, TetMesh, Heightfield, SDF, Gaussian, CollisionPipeline}` + `eval_fk/ik/jacobian/mass_matrix`
- **Solvers**: XPBD, MuJoCo, Featherstone, SemiImplicit, Style3D, VBD, ImplicitMPM, Kamino (8종)
- **Sensors**: SensorContact, SensorFrameTransform, SensorIMU, SensorRaycast, SensorTiledCamera (모두 `update(model, state)` 인터페이스)
- **IK**: IKSolver + IKObjective{Position,Rotation,JointLimit} + IKOptimizer{LM,LBFGS} + IKSampler (batch n_problems 지원)
- **Viewers**: ViewerGL / ViewerUSD / ViewerFile (recording+playback) / ViewerNull / **ViewerRerun (web:9090)** / **ViewerViser (web:8080)**
- **Selection**: `selection.ArticulationView(model, pattern, include_joints=..., ...)` — name/pattern 기반 DOF/link 조작 + eval_fk/jacobian
- **Control 채널**: `joint_act` / `joint_f` / `joint_target_pos` / `joint_target_vel`
- **State**: joint_q, joint_qd, body_q, body_qd, body_f + EXTENDED (`mujoco:qfrc_actuator`, `body_parent_f`, `body_qdd`)
- **Model**: per-joint `armature`, `effort_limit`, `velocity_limit`, `friction`, `limit_ke/kd`; `equality_constraint_*`, `constraint_mimic_*`; `world_count`, `articulation_world`; `set_gravity()`
- **Builder multi-world**: `begin_world/end_world`, `add_world(builder, xform, label_prefix)`, `replicate(builder, N, spacing)`
- **Not available in 1.1.0**: `newton.rl`, `newton.sim` (internal only), `SolverImplicitPD`

### 결정 기록

| 결정 | 선택 | 메모 |
|---|---|---|
| A | pack yaml에 `articulation_pattern` 노출 | Phase 1 |
| B | `scene.yaml`로 통합 (단일 `robot.yaml`은 자동 shim) | Phase 2 |
| C | 단일 `/joint_command` + `target_mode` 해석 | Phase 3 |
| D | `/tf` 디폴트 ON | Phase 4 |
| E | 표준 ROS msg만 (커스텀 msg 패키지 없음) | Phase 5 |
| F | rigid 확장(6a)만, deformable(6b)은 **deferred** | Phase 6 |
| G | 기본 뷰어 `ViewerRerun`으로 변경 | Phase 7 |
| H | IK 서비스는 **deferred** (별도 이터레이션) | Phase 8 (out of scope) |
| I | RL은 별도 서브프로젝트 — 본 repo에 미포함 | Phase 9 (out of scope) |

---

## In-scope Phases (v2 순서)

머지 순서: **1 → 3 → 4 → 2 → 5 → 7 → 6a**

각 phase 완료 시 다음을 수행:
1. `docs/IMPLEMENTATION_PLAN.md` 진행상황 업데이트
2. `README.md` / 해당 `docs/*.md` 갱신
3. `memory/` 업데이트 (신규 insight)
4. `git commit` (테스트 통과한 상태로)

---

### Phase 1 — ArticulationView 기반 리팩터 (foundation) — ✅ COMPLETED

**구현 노트**:
- `articulation_pattern` 은 **fnmatch glob** (regex 아님). 기본값 `"*"` (모든 articulation).
- 팩의 `joint_names` 는 **ROS 노출 서브셋** — 시뮬은 전체 DOF, ROS 는 선언된 것만.
  (franka finger_joint1/2 같은 미노출 DOF 시뮬 지속을 위함)
- `view.get_dof_positions(state)` 는 `wp.array` (shape `(n_worlds, n_arts, dof)`) →
  `.numpy().reshape(-1)` 로 평탄화.
- `view.set_dof_positions(state, arr)` 는 numpy `(1, 1, dof)` 도 그대로 수용.


**목적**: `world.py`의 수동 joint_layout 인덱싱을 제거하고 `newton.selection.ArticulationView`로 교체. 모든 후속 phase의 기반이 됨.

**결정 A 반영**: pack yaml에 `articulation_pattern` 선택 필드 노출. 미지정 시 `".*"`(전체 매칭).

**변경 파일**:
- `src/newton_bridge/world.py` — joint_layout 수동 계산 → ArticulationView 호출로 교체
- `src/newton_bridge/robot_pack.py` — `articulation_pattern` 필드 허용 (default `.*`)
- `robots/*/robot.yaml` — 필요 시 pattern 명시

**핵심 API 매핑**:
- `view.joint_names` / `view.dof_count` — 스키마 정보
- `view.get_dof_positions(state)` → numpy/torch tensor
- `view.set_dof_positions(state, values)`
- `view.get_link_transforms(state)` — body pose
- `view.eval_fk(state)`, `view.eval_jacobian(state)`

**유지해야 할 기존 동작**:
- home_pose 반영 (builder 사전 설정 + state 초기화)
- 드라이브 ke/kd/mode 설정 (builder 단계)
- freerun/handshake 모두 작동
- verify.sh (in-container) §6 통과

#### Unit test 계획

**tests/test_articulation_view.py** (host-only, Newton 필요 없음):
- 해당 없음 (ArticulationView는 Newton 의존 → container 테스트로)

**scripts/container/verify.sh §6 확장** (integration):
- 기존: `Δq > 0.05` 체크
- 추가: `world.view.joint_names == pack['joint_names']` (순서 일치)
- 추가: `world.view.get_dof_positions(state).shape[-1] == dof_count`
- 추가: 홈 포즈 복귀 (`world.reset()` 후 q가 home과 일치)

**tests/test_robot_pack.py 확장** (host pytest):
- `articulation_pattern` 필드 누락 허용 (default `.*`)
- `articulation_pattern` 명시 시 값 그대로 보존

---

### Phase 2 — Scene.yaml multi-world / multi-articulation — ✅ COMPLETED

**구현 노트**:
- `scene.yaml` 이 canonical, `robot.yaml` 은 자동 shim (loader가 `_promote_robot_yaml`
  로 scene 형태로 승격).
- 내부 표현은 scene dict + primary-articulation 탑레벨 alias (`robot`, `joint_names`,
  `home_pose`, `drive`, `joints`, `articulation_pattern`). 기존 NewtonWorld/
  SimBridgeNode 코드는 변경 없음.
- **현 phase 스코프**: 1 world × 1 articulation. 다중 world / 다중 articulation
  은 `NotImplementedError` 로 거부 (별도 phase 대상).
- `ros.primary_articulation` 필드 추가 (미지정 시 첫 articulation).


**목적**: 단일 로봇 `robot.yaml`을 `scene.yaml`로 자연스럽게 확장. Newton의 `begin_world/end_world` + `add_world` primitive에 1:1 매핑.

**결정 B 반영**: `robot.yaml` 유지. 로더가 없으면 `scene.yaml`을 찾고 있으면 1-world / 1-articulation로 자동 래핑.

**새 스키마 (`robots/<pack>/scene.yaml`)**:
```yaml
sim:
  physics_hz: 400
  substeps: 1
  solver: mujoco
  ground_plane: true
  gravity: [0, 0, -9.81]

worlds:
  - label: env0
    gravity: [0, 0, -9.81]      # optional, overrides sim.gravity
    articulations:
      - label: ur5e_a
        source: urdf
        source_rel: models/ur5e.urdf
        xform:
          pos: [0.0, 0.0, 0.0]
          rot: [0, 0, 0, 1]   # quaternion xyzw
        articulation_pattern: ".*"
        joint_names: [shoulder_pan_joint, ...]
        home_pose: {...}
        drive: {...}

ros:
  joint_states_topic: /joint_states
  joint_command_topic: /joint_command
  publish_rate_hz: 100
  primary_articulation: ur5e_a    # ROS 토픽이 바라볼 articulation label
```

**변경 파일**:
- `src/newton_bridge/robot_pack.py` — scene.yaml / robot.yaml 둘 다 지원, 내부 통일된 `SceneSpec` dataclass
- `src/newton_bridge/world.py` — SceneSpec 기반 빌드, 여러 articulation view 관리
- `src/newton_bridge/node.py` — primary_articulation 기준 토픽 pub/sub, `/envs/<label>/joint_states` opt-in
- `robots/*/robot.yaml` 유지, 필요 시 `robots/*/scene.yaml` 추가 (초기엔 자동 shim만)

**Multi-env 기본값**: 1 world, 1 articulation. Multi-robot 예시는 별도 `robots/dual_ur5e/scene.yaml`로 (참고용).

#### Unit test 계획

**tests/test_scene_spec.py** (host-only):
- `robot.yaml` → SceneSpec 변환 정확성 (shim)
- `scene.yaml` → SceneSpec 직접 파싱
- 누락 필드 defaults 적용

**verify.sh §6 확장**:
- 기존 pack들 (ur5e, franka, kuka_iiwa_14) 모두 shim 경로로 여전히 통과
- `dual_ur5e` scene 로드 → 2개 articulation 모두 finalize 성공

---

### Phase 3 — 전체 액추에이션 (Control 4채널) — ✅ COMPLETED

**구현 노트**:
- `parse_drive_mode` 퍼블릭 헬퍼: `position|velocity|effort|position_velocity|none` (대소문자 무관)
- per-joint 오버라이드: `joints.<name>.drive.{mode,stiffness,damping}` + non-drive
  scalar (`armature`, `effort_limit`, `velocity_limit`, `friction`, `limit_ke`, `limit_kd`)
- DOF 이름은 빌더의 `joint_label` + `joint_dof_dim` 로부터 파생 (finalize 전).
  1-DOF 조인트는 short label 그대로, multi-DOF 은 `<label>_<i>`.
- `/joint_command` 세 필드 (`position`/`velocity`/`effort`) 모두 길이 == `name`
  이면 반영, 빈 배열은 "건드리지 않음".
- **솔버 호환성** (verify §7로는 미검증): XPBD는 target 채널 무시, MuJoCo는
  `add_body`의 auto-joint 패턴 거부, Featherstone은 mass 없는 바디에서 NaN.
  verify §7은 API wiring만 검증 (실제 motion은 verify §6의 MuJoCo/UR5e가 담당).


**목적**: position 전용 PD에서 Newton의 4채널(pos/vel/effort + act feedforward) + per-joint drive param + URDF `<mimic>` 지원으로 확장.

**결정 C 반영**: `/joint_command` 하나 유지. 메시지의 position / velocity / effort 필드 존재 여부 + pack의 drive.mode가 합쳐져 해석됨. 여러 채널이 동시에 들어오면 target_mode=POSITION_VELOCITY로 작동.

**변경 파일**:
- `src/newton_bridge/world.py`:
  - per-joint `drive:` 오버라이드 (yaml `joints.<name>.drive.*`)
  - `JointTargetMode`: POSITION / VELOCITY / EFFORT / POSITION_VELOCITY / NONE
  - `set_joint_targets(names, positions=None, velocities=None, efforts=None)`
  - URDF `<mimic>` 자동 연결: add_urdf 후 model의 `constraint_mimic_*` 검사 (파서가 만들어 주면 그대로, 아니면 skip)
- `src/newton_bridge/node.py`:
  - `_on_cmd`가 msg.position/velocity/effort 모두 읽고 world.set_joint_targets에 각각 전달

**per-joint 파라미터 (yaml)**:
```yaml
joints:
  shoulder_pan_joint:
    drive:
      mode: position      # overrides top-level drive.mode
      stiffness: 10000.0
      damping: 100.0
    armature: 0.01
    effort_limit: 150.0
    velocity_limit: 3.14
    friction: 0.1
    limit_ke: 1e5
    limit_kd: 1e3
```
미지정 시 `drive:` top-level 값이 fallback.

#### Unit test 계획

**tests/test_drive_schema.py** (host-only):
- top-level `drive:` 만 있을 때 모든 joint가 그 값 상속
- per-joint `joints.X.drive.*` 오버라이드 검증
- 잘못된 mode 문자열 → ValueError

**verify.sh §6 확장**:
- mode=position → position target 주입 후 Δq>0.05 (현존 테스트)
- mode=velocity → velocity target 주입 후 |qd|>0.05
- mode=effort → effort 주입 후 |qd|>0 (gravity 보상 부호만 확인)
- mode=position_velocity → pos+vel 둘 다 주입 시 pos error 작게 유지

---

### Phase 4 — Rich joint_states + /tf — ✅ COMPLETED

**구현 노트**:
- `/joint_states` 의 `velocity`/`effort` 도 채움 (기존엔 position 만)
- `effort` 는 `control.joint_f` readback (solver-applied 토크는 State에 노출 안 됨)
  — EFFORT 모드에서만 의미 있는 값; POSITION/VELOCITY 모드는 0.
- `/tf` 는 `ros.publish_tf: true` (default) 에서 활성. 모든 body 를
  `tf_root_frame`(`world`)의 child 로 평탄 퍼블리시 — parent→child 체인 아님.
- `ros.publish_frames: []` (default) = root 제외 전체. 화이트리스트 지원.
- `view.get_link_transforms(state)` → `wp.array[transformf]` shape `(1,1,N,7)`
  → numpy reshape `(-1, 7)` = (px,py,pz,qx,qy,qz,qw).


**목적**: `/joint_states` 3필드(position+velocity+effort) 채우고, 링크 프레임을 `/tf` 로 publish.

**결정 D 반영**: `/tf` 디폴트 ON. yaml `ros.publish_frames: [...]` 으로 범위 축소 가능, 빈 리스트면 **모든 articulation body**.

**변경 파일**:
- `src/newton_bridge/node.py`:
  - `_publish_state`: `msg.velocity = view.get_dof_velocities(state)`, `msg.effort = view.get_dof_forces(state)` (state.joint_f)
  - 신규 `_publish_tf`: `tf2_msgs/TFMessage` 에 `geometry_msgs/TransformStamped` 채워 publish
- `src/newton_bridge/world.py`:
  - `read_body_transforms() -> dict[label, (xyz, quat)]` — ArticulationView 우선, 없으면 state.body_q 직접

**의존성**:
- `tf2_msgs`, `geometry_msgs` 이미 ROS 2 base에 포함 (Jazzy Dockerfile)

#### Unit test 계획

**verify.sh §7 신규**:
```bash
# run sim in background for 2s, rostopic hz 체크
timeout 3 python3 -m newton_bridge &
PID=$!
sleep 1
ros2 topic hz /joint_states --window 10   # >50hz
ros2 topic hz /tf --window 10              # >10hz
kill $PID 2>/dev/null
```
(실제로 verify.sh에서 subprocess + 타임아웃 합성, 실패 시 skip)

**tests/test_joint_state_fields.py** (host-only):
- JointState msg 생성 path가 position/velocity/effort 모두 non-empty인지 (mock world)

---

### Phase 5 — Contact / IMU 센서 — ✅ COMPLETED

**구현 노트**:
- 새 모듈 `src/newton_bridge/sensors.py`: `SensorContact` / `SensorIMU` 래핑,
  msg 변환 헬퍼.
- yaml 키는 `bodies`/`shapes` (contact) / `site`/`sites` (imu) — Newton 내부
  인자 `sensing_obj_bodies` 등은 노출하지 않음.
- contact force 는 vec3 (Newton 1.1.0 제약) → `WrenchStamped.torque` 는 0.
- IMU 는 site 필요 → MJCF pack 에서만 의미 있음 (URDF 는 수동 site 추가 필요).
- `world.last_contacts` 를 expose 해서 sensor.update 가 `m.collide()` 재호출
  없이 접근. `world.step()` 에서 세팅.
- body label 은 articulation prefix 를 포함하므로 fnmatch glob 권장
  (e.g. `"*wrist_3_link*"`).


**목적**: `SensorContact` → `/contact_wrenches`, `SensorIMU` → `/imu/<site>`. 모두 **표준 msg**만 사용 (결정 E).

**scene.yaml 스키마 확장**:
```yaml
sensors:
  contact:
    - label: ee_contact
      bodies: [wrist_3_link]     # 또는 shapes:
      measure_total: true
      topic: /contact_wrenches/ee
  imu:
    - label: base_imu
      site: base_link
      topic: /imu/base
```

**토픽 매핑**:
- Contact → `geometry_msgs/WrenchStamped` (force + torque at body origin)
- IMU → `sensor_msgs/Imu` (linear_accel, angular_velocity, orientation은 unknown → covariance -1)

**변경 파일**:
- `src/newton_bridge/sensors.py` (신규): 센서 빌드 + 메시지 변환
- `src/newton_bridge/node.py`: publisher 등록, 매 state publish 시 센서 update + publish
- `src/newton_bridge/world.py`: SensorContact 사용 시 model.request_contact_attributes 호출 필요

#### Unit test 계획

**tests/test_sensor_schema.py** (host-only):
- sensors 블록 누락 시 빈 리스트로 정상 처리
- contact/imu 엔트리 필수 필드 검증

**verify.sh §8 신규** (container):
- 빈 scene에 ee contact 센서만 붙여 sim 50스텝 후 `SensorContact.update` 호출 성공 + msg 변환 NaN 없음

---

### Phase 6a — 솔버 튜닝 표면 + runtime env — ✅ COMPLETED

**구현 노트**:
- `sim.solver_params:` 딕셔너리 → solver constructor 로 `**kwargs` 전달.
- 솔버 매핑 확장: XPBD / MuJoCo / Featherstone / SemiImplicit / Style3D / VBD.
  ImplicitMPM, Kamino 는 `Config` 객체 필요해서 Phase 6b 이관.
- bad kwarg → clean `ValueError` (TypeError intercept).
- runtime gravity: `/sim/set_gravity` topic (`geometry_msgs/Vector3`, latest-wins).
  service 가 아닌 topic 이유: Trigger 가 payload 못 실어서.
- deformable (cloth/soft/particle) 은 out-of-scope — Phase 6b.


**목적**: 모든 Solver\*의 생성자 kwargs 를 pack yaml로 pass-through. `/sim/set_gravity` 서비스 추가.

**결정 F 반영**: deformable(cloth/soft/particle)은 Phase 6b로 분리 — 본 이터레이션 out of scope.

**scene.yaml 확장**:
```yaml
sim:
  solver: mujoco
  solver_params:
    iterations: 100
    tolerance: 1e-8
    cone: pyramidal
    integrator: implicit
```

**변경 파일**:
- `src/newton_bridge/world.py`:
  - `_build_solver`: `**pack['sim'].get('solver_params', {})` 로 kwargs 전달
  - 미지원 kwarg 는 명확한 ValueError (TypeError를 intercept)
- `src/newton_bridge/node.py`:
  - `/sim/set_gravity` 서비스 — `std_srvs/Trigger` 는 vec 파라미터 못받으므로 ROS param으로 접근 (`~gravity`) 또는 다중 Trigger (`/sim/gravity_default`, `/sim/gravity_zero`) — **선택지 작은 것으로**: `/sim/set_gravity` 는 `geometry_msgs/Vector3` 메시지 topic으로 (subscription)
- 솔버 가용 리스트 문서: `docs/ARCHITECTURE.md`

#### Unit test 계획

**tests/test_solver_params.py** (host-only):
- scene.yaml의 `solver_params` dict가 NewtonWorld `_build_solver` 로 pass-through (mock solver)

**verify.sh §9 신규**:
- 각 솔버에 대해 (XPBD/MuJoCo/Featherstone) 고유 kwarg 주입해 finalize + 10 step 성공

---

### Phase 7 — Viewer 확장 (Rerun 기본) — ✅ COMPLETED

**구현 노트**:
- `VIEWER` env 로 통합: `rerun|gl|usd|file|null|none` (default `rerun`).
- `ENABLE_VIEWER=1` 은 `SystemExit` 로 거부 + 마이그레이션 안내 (silent shim 없음).
- `rerun` 은 `network_mode: host` 덕분에 추가 포트 매핑 없이 `localhost:9090` 에 접근.
- `usd`/`file` 은 `workspace/runs/<ts>.{usd,nvpr}` 로 자동 저장 (`VIEWER_OUTPUT_PATH` 로 오버라이드).
- `resolve_mode()` 는 Newton import 없이 호스트 pytest 에서도 호출 가능.


**목적**: 뷰어 선택을 환경변수 하나(`VIEWER`) 로 단순화. Rerun 웹뷰어 기본 활성화.

**결정 G 반영**: `ENABLE_VIEWER=0/1` 는 deprecated (backwards compat shim 없이 제거). `VIEWER=rerun|gl|usd|file|null|none` (default **rerun**).

**동작**:
- `VIEWER=rerun` → ViewerRerun(app_id, serve_web_viewer=True, web_port=9090) — 호스트 브라우저로 접근
- `VIEWER=gl` → ViewerGL (X11 필요)
- `VIEWER=usd` → ViewerUSD(output_path=`workspace/runs/<ts>.usd`)
- `VIEWER=file` → ViewerFile(output_path=`workspace/runs/<ts>.nvpr`, auto_save=True)
- `VIEWER=null` → ViewerNull (profile)
- `VIEWER=none` → no viewer

**변경 파일**:
- `src/newton_bridge/viewer.py` — factory + dispatcher
- `src/newton_bridge/__main__.py` — ENABLE_VIEWER 제거, VIEWER 읽기
- `docker/compose.yml` — VIEWER=rerun 기본, 9090/9876 포트는 `network_mode: host` 로 이미 노출됨 (주석 추가)
- `scripts/host/run.sh` — VIEWER env 전달, README 업데이트

**Backwards compat**: `ENABLE_VIEWER=1` 을 감지하면 fatal error + "use VIEWER=gl" 메시지. (결정 G — rerun 기본으로 변경이니 silent shim 없음)

#### Unit test 계획

**tests/test_viewer_factory.py** (host-only, import-only):
- `VIEWER` 값별로 dispatcher가 올바른 클래스 import 요청하는지 (mock import)
- 알 수 없는 값은 ValueError

**verify.sh §10 신규**:
- `VIEWER=null` 로 sim 10step 실행 + viewer.end_frame 호출 성공
- `VIEWER=rerun` 은 GL/net 의존 → container에서 skip, 문서에 수동 확인 절차 명시

---

## Out-of-scope (deferred)

- **Phase 6b**: Deformable (cloth/soft/particle/cable/tet) pack 타입. 결정 F. 별도 이터레이션.
- **Phase 8**: `/ik/solve` 커스텀 srv + IKSolver 래퍼. 결정 H. 별도 이터레이션 — 커스텀 `newton_bridge_msgs` 패키지가 필요하므로 빌드 파이프라인 변경 동반.
- **Phase 9**: RL vec-env + SensorTiledCamera. 결정 I. **별도 서브프로젝트** (예: `newton-bridge-rl`) — gymnasium API + torch 의존, scope가 본 repo와 분리.

---

## 공통 테스트 실행

**host pytest** (Newton 미의존):
```bash
cd /home/junho/ros2_ws/newton-bridge
pip install -e .[dev]
pytest -q
```

**container integration** (verify.sh):
```bash
./scripts/host/run.sh verify
```

**끝-끝 smoke**:
```bash
./scripts/host/run.sh sim    # freerun, ur5e, rerun viewer
python3 examples/controller_demo.py --robot ur5e --mode freerun
```

---

## 진행 상황

| Phase | 상태 | 커밋 | 비고 |
|---|---|---|---|
| 0 | ✅ completed | 8427788 | API dump 확보 |
| 1 | ✅ completed | 64858ed | ArticulationView refactor — pattern in pack yaml |
| 3 | ✅ completed | bdb4205 | 4-channel actuation + per-joint drive overrides |
| 4 | ✅ completed | 9c733ee | JointState vel+effort + /tf default ON |
| 2 | ✅ completed | a7b0a80 | scene.yaml unification (robot.yaml auto-shim) |
| 5 | ✅ completed | 8c9e110 | Contact + IMU sensors (standard msgs) |
| 7 | ✅ completed | b793d56 | Viewer factory + Rerun default |
| 6a | ✅ completed | c49900b | solver_params + /sim/set_gravity |

**완료 정의**: 해당 phase의 unit test + verify.sh 관련 섹션이 모두 통과.
