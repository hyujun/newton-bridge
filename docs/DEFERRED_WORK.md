# Deferred work

본 문서는 2026-04 Newton 기능 확장 이터레이션 (Phases 1–7) 에서 **의도적으로 out-of-scope** 로 남긴 항목들의 구현 계획입니다. 각 항목은 독립 이터레이션으로 집어들 수 있도록 self-contained 하게 기술합니다.

| Phase | 주제 | 이유 | 블록 여부 |
|---|---|---|---|
| 2b | Multi-world / multi-articulation | Phase 2 구현 중 scope 축소; 단일 세계/조인트만 실행 | 없음 (loader 는 해당 형태 scene.yaml 을 `NotImplementedError` 로 거부) |
| 6b | Deformables (cloth / soft / particle / tet) | 결정 F — rigid 확장(6a) 과 scope 분리 | 없음 |
| 8 | `/ik/solve` 커스텀 서비스 | 결정 H — 별도 이터레이션. 커스텀 `newton_bridge_msgs` 패키지 필요 | 빌드 파이프라인 변경 |
| 9 | RL vec-env + SensorTiledCamera | 결정 I — 본 repo 외 **별도 서브프로젝트** (`newton-bridge-rl`) | 서브프로젝트 생성 |

---

## Phase 2b — Multi-world / multi-articulation

### 목표
현재 scene.yaml 로더는 **1 world × 1 articulation** 만 허용 (로더가 `NotImplementedError` 로 나머지를 명시적 거부). Newton 1.1.0 의 `begin_world/end_world` + `add_world` + 다중 `add_urdf/add_mjcf` 를 활용해 **한 scene 에 여러 로봇 + 여러 env** 를 실행하도록 확장.

### 사용 케이스
- 이기종 멀티-로봇 시나리오 (UR5e + Franka 를 같은 world 에서 협업)
- RL rollout 용 `num_envs=N` 배치 (Phase 9 가 이 위에 얹힘)
- 카메라/관전자 관점에서 여러 env 를 동시 시각화

### 구현 계획

**변경 파일**:
- `src/newton_bridge/robot_pack.py`
  - `_validate_scene` 의 `len(worlds) > 1` / `len(articulations) > 1` 거부 로직 제거
  - 각 articulation 의 per-field validation (label 유일성, primary_articulation 존재 확인) 확장
- `src/newton_bridge/world.py`
  - `_build_model` 을 world 단위로 iterate: `builder.begin_world(label=w.label, gravity=w.gravity)` → `add_urdf/add_mjcf` for each articulation → `builder.end_world()`
  - `_configure_builder_drive` 를 articulation 별로 적용 (builder 전역 인덱스가 아니라 articulation-scoped view 로)
  - `_build_view`: articulation 별 `ArticulationView` 의 dict → `self.views: dict[label, ArticulationView]`
  - `read_joint_positions/velocities/efforts` 와 `set_joint_targets`: 기본은 primary articulation, `articulation_label=` kwarg 로 특정 articulation 선택 가능
- `src/newton_bridge/node.py`
  - primary articulation 은 기존 `/joint_states` `/joint_command` 에 계속 바인딩
  - 추가 articulation 은 `/<articulation_label>/joint_states` 와 `/<articulation_label>/joint_command` 로 자동 등록
  - 추가 envs 는 opt-in: `ros.expose_all_envs: true` 일 때만 `/envs/<env_label>/<art_label>/*` 네임스페이스 공개 (default false — ROS side 는 primary world 만)

**scene.yaml 스키마** (이미 loader 가 파싱 가능한 형태):
```yaml
worlds:
  - label: env0
    gravity: [0, 0, -9.81]
    articulations:
      - {label: ur5e_a, source: urdf, source_rel: ..., xform: {pos: [0,0,0]}}
      - {label: franka_a, source: mjcf, source_rel: ..., xform: {pos: [1,0,0]}}
  - label: env1
    articulations:
      - {label: ur5e_b, source: urdf, source_rel: ..., xform: {pos: [0,1,0]}}
ros:
  primary_articulation: ur5e_a
  expose_all_envs: false
```

### Unit test 계획

**tests/test_scene_multi.py** (host, Newton-free):
- 2 articulation in 1 world → 로더가 허용 (현 `NotImplementedError` 제거 후)
- 2 world 스키마 라운드-트립
- `primary_articulation` 이 실제로 존재하는 articulation label 을 가리키는지

**verify.sh §12 신규** (container):
- `robots/dual_ur5e/scene.yaml` 픽스처 (UR5e 2 개, xform 분리) 로드 → 2×6 = 12 DOF 모델 빌드
- Primary articulation 의 `/joint_states` 만 기본 퍼블리시
- `expose_all_envs: true` 시 `/ur5e_b/joint_states` 도 등장

### 의존성
없음 (Newton 1.1.0 은 이미 begin_world/end_world/add_world/ArticulationView 를 지원).

### 예상 난이도
중간. ROS 토픽 네임스페이싱 설계 + 기존 single-articulation 코드 경로 보존이 주된 작업.

---

## Phase 6b — Deformables (cloth / soft body / particle / tet)

### 목표
Newton 의 deformable sim 능력 (SolverVBD, SolverStyle3D, SolverImplicitMPM, cloth/soft/particle/tet primitives) 을 pack 스키마로 노출.

### 사용 케이스
- 그래스핑 벤치마크 (천, 봉제물, 가변형 물체)
- 입자/유체 시뮬레이션 (곡물, 모래)
- Soft 이기종 객체와 rigid 로봇의 상호작용

### 구현 계획

**새 pack 타입**: `type: deformable` (또는 articulation 외의 object 섹션 추가)

scene.yaml 확장:
```yaml
worlds:
  - label: env0
    articulations: [...]    # 기존
    objects:                 # 신규 — rigid articulation 아닌 것들
      - type: cloth_grid
        label: cloth_a
        pos: [0, 0, 1.5]
        dim_x: 32
        dim_y: 32
        cell_x: 0.02
        cell_y: 0.02
        mass: 0.1
        tri_ke: 5000
        tri_ka: 5000
        tri_kd: 0.1
        particle_radius: 0.01
        fix_left: true
      - type: soft_mesh
        label: ball_a
        source: /workspace/assets/ball.obj
        pos: [0, 0.5, 1]
        density: 100
        k_mu: 1e5
        k_lambda: 1e5
        k_damp: 10
      - type: particle_grid
        label: sand_a
        dim_x: 16
        dim_y: 16
        dim_z: 8
        cell_x: 0.02
        ...
sim:
  solver: vbd              # rigid + cloth 하이브리드, 또는 solver_per_object 방식
  solver_params: {...}
```

**변경 파일**:
- `src/newton_bridge/scene_objects.py` (신규): object spec → builder call mapping
  - `add_cloth_grid`, `add_cloth_mesh`, `add_soft_grid`, `add_soft_mesh`, `add_particle_grid`
  - 각 object 는 particles / triangles / tetrahedra 로 시뮬되므로 **articulation 이 아님**
- `src/newton_bridge/world.py`
  - `_build_model` 이 articulations 외에 objects 도 iterate 하여 builder 에 추가
  - `_build_view` 는 articulations 에만 생성 (deformable 에는 ArticulationView 개념이 없음)
  - deformable 상태 readback 용 헬퍼: `read_particle_positions()`, `read_cloth_vertices()` 등
- `src/newton_bridge/node.py`
  - 센서 블록 확장: `sensors.particle_cloud` → `sensor_msgs/PointCloud2` 로 파티클 위치 퍼블리시 (opt-in)
  - `/cloth/<label>/mesh` 에 `visualization_msgs/Marker` (TRIANGLE_LIST) 퍼블리시
- `docs/ROBOTS.md` (또는 `docs/OBJECTS.md` 신규): object 스키마 레퍼런스

**솔버 매칭**:
- cloth 전용: `SolverStyle3D`
- cloth + rigid 혼합: `SolverVBD`
- particle/granular: `SolverImplicitMPM` (Config 객체 필요 — 추가 작업)
- tet soft body: `SolverVBD` 또는 `SolverXPBD`

### Unit test 계획

**tests/test_scene_objects.py** (host):
- yaml 의 `objects:` 블록이 object spec 리스트로 파싱되는지
- 각 object type 의 필수 필드 검증

**verify.sh §13 신규** (container):
- cloth_grid 하나 + ground_plane 만 있는 최소 씬 → SolverStyle3D 로 100 step → vertex 위치가 NaN 없이 갱신
- soft_mesh 하나 드롭 → SolverVBD 로 낙하 후 정지 상태 에너지 감소
- particle_grid 소형 (8×8×4) → SolverImplicitMPM init + 10 step (Config 객체는 기본값)

### 의존성
`newton.solvers.SolverImplicitMPM.Config` — 사용법이 문서화 적음. 필요 시 MPM은 Phase 6c 로 추가 분리.

### 예상 난이도
중상. deformable 은 상태 표현이 복잡 (particles / triangles / edges) 하고 ROS 메시지 매핑이 비자명. 실행 성능 튜닝 (solver iterations vs step rate) 도 케이스 바이 케이스.

---

## Phase 8 — `/ik/solve` 커스텀 서비스

### 목표
`newton.ik.IKSolver` 를 ROS 서비스로 노출. URDF/MJCF 조인트 chain 기반의 batch 가능한 역기구학.

### 사용 케이스
- 컨트롤러가 Cartesian 목표 (예: 엔드이펙터 pose) 를 ROS 콜로 joint angle 로 변환
- 플래너가 여러 candidate IK seed 로 batch 호출 (`n_problems > 1`, `n_seeds > 1`)
- 물리 기반 IK (joint limit objective, 관절 공간 프리페런스) 의 unified 인터페이스

### 커스텀 srv 패키지

기존 repo 는 표준 msg 만 썼지만, IK 는 `target_pose` (Pose) + `link` (string) + `joint_names` (string[]) 를 응답 `positions` (float64[]) + `success` (bool) + `message` (string) 로 매핑해야 하므로 **커스텀 srv** 필요.

**신규 ROS 2 패키지**: `newton_bridge_msgs/`
```
newton_bridge_msgs/
├── package.xml
├── CMakeLists.txt
└── srv/
    ├── SolveIK.srv
    └── SolveIKBatch.srv
```

`SolveIK.srv`:
```
# Request
geometry_msgs/Pose target_pose
string link                   # end-effector link name in the articulation
string[] joint_names          # empty = use all exposed joints (pack default)
float64[] seed                # optional initial q; empty = current state
int32 max_iter 50
float32 position_weight 1.0
float32 rotation_weight 1.0
bool respect_joint_limits true
---
# Response
bool success
string message
float64[] positions           # joint order = request.joint_names (or pack default)
float32 position_error
float32 rotation_error
int32 iterations
```

`SolveIKBatch.srv`: 동일하되 `geometry_msgs/Pose[] target_poses` + `float64[][] seeds` + `float64[][] positions` (row-major flatten + `batch_size` 필드) — batch IK 용.

### 구현 계획

**변경 파일**:
- **신규 패키지 `newton_bridge_msgs/`** (colcon/ament_cmake)
  - `docker/Dockerfile` 에 `colcon build --packages-select newton_bridge_msgs` 추가
  - host 에서도 `verify_ros.sh` 가 이 msg 를 import 할 수 있도록 설치 경로 업데이트
- `src/newton_bridge/ik.py` (신규): `IKService` 클래스
  - 생성자: `(world, node)` → IKSolver 를 pack 의 articulation 에 바인딩
  - `on_solve(request, response)`: IKObjectivePosition + IKObjectiveRotation + (optional) IKObjectiveJointLimit 구성, `IKOptimizerLM` 으로 풀기, 결과를 response 로 변환
- `src/newton_bridge/node.py`
  - pack yaml `ros.enable_ik: true` (default false) 시 IKService 등록
- `docs/TOPICS.md`: `/ik/solve` 서비스 계약 추가
- `docs/ROBOTS.md`: pack yaml `ros.ik: {enabled, default_link, ...}` 옵션 설명

### Unit test 계획

**tests/test_ik_schema.py** (host):
- `enable_ik: true` 일 때 pack 이 요구하는 필드 검증 (default_link 존재 등)

**verify.sh §14 신규** (container, rclpy):
- UR5e pack 로드 → 노드 띄움 (handshake mode)
- 별도 프로세스에서 `rclpy` 클라이언트로 `/ik/solve` 호출:
  - `target_pose` = 현재 forward-kinematics 결과 (자기 자신의 pose)
  - 기대: `success=true`, `positions` 이 현재 q 와 근사
- 도달 불가능한 타겟 → `success=false`

### 의존성
- colcon + ament_cmake (`newton_bridge_msgs` 빌드용) — Dockerfile 확장
- Phase 1 의 ArticulationView 상에서 IKSolver 인스턴스화 (`model` 을 넘기는 것이지 view 가 아님)
- `newton.ik.IKObjective*`, `IKOptimizerLM` — Newton 1.1.0 에서 바로 사용 가능 (Phase 0 probe 확인)

### 예상 난이도
중상. 핵심 리프트는 **커스텀 msg 패키지 빌드 파이프라인** 추가. IKSolver 자체는 직선적.

---

## Phase 9 — RL vec-env (별도 서브프로젝트)

### 목표
`replicate(builder, num_envs)` + ArticulationView batched access + torch-cu12 로 gym-style **vectorized RL 환경** 제공. SensorTiledCamera 를 관측에 추가.

### 범위 / 위치
**본 repo 외부** 에 별도 프로젝트 `newton-bridge-rl/` 를 생성 (결정 I). newton-bridge 를 editable 로 의존성으로 사용.

### 사용 케이스
- PPO/SAC 학습 (Isaac Gym / IsaacLab 대체)
- Sim-to-real 엔드투엔드 파이프라인의 sim 부분
- Diffsim (Newton 의 `requires_grad=True`) 연구

### 서브프로젝트 구조

```
newton-bridge-rl/
├── README.md
├── pyproject.toml              # depends: newton-bridge, gymnasium, torch, tensordict
├── src/newton_bridge_rl/
│   ├── vec_env.py              # gym-like batched env
│   ├── obs.py                  # joint_q/qd/contact/camera → torch tensor
│   ├── reward.py               # pluggable reward functions
│   ├── wrappers.py             # StepLimit, RewardScale, ObsNorm
│   └── camera.py               # SensorTiledCamera adapter
├── tasks/                       # task 정의 YAML (pack + reward + obs/action spec)
│   ├── ur5e_reach.yaml
│   └── franka_pick.yaml
├── examples/
│   ├── random_policy.py         # VecEnv smoke
│   ├── ppo_train.py             # clean-rl 기반 PPO
│   └── sac_train.py
├── tests/
└── docker/                      # newton-bridge 이미지 재사용, rl 레이어 추가
```

### 구현 계획

**핵심 API** (`newton_bridge_rl.vec_env.VecEnv`):
```python
class VecEnv(gymnasium.Env):
    def __init__(self, pack_dir, num_envs, task_cfg):
        # builder.replicate(base, num_envs, spacing) 으로 env 복제
        # 단일 model 에 num_envs 개 articulation
        # 하나의 ArticulationView 로 전체를 (N, dof) 텐서로 읽기/쓰기
        ...

    def reset(self, seed=None) -> tuple[torch.Tensor, dict]:
        # per-env 초기 q 를 pack.home_pose 또는 reset distribution 으로
        ...

    def step(self, actions: torch.Tensor) -> tuple[obs, reward, terminated, truncated, info]:
        # actions: (num_envs, action_dim) → control.joint_target_*
        # physics_dt 만큼 step (여러 substep 포함)
        # obs: (num_envs, obs_dim) 텐서 (zero-copy from wp.array 가능하면 선호)
        ...

    @property
    def observation_space(self): ...
    @property
    def action_space(self): ...
```

**관측**:
- 기본: `view.get_dof_positions` + `get_dof_velocities` (batched `(N, dof)`)
- contact sensor batched (SensorContact 도 articulation-aware)
- optional camera: `SensorTiledCamera` (Config 로 `num_cameras=num_envs` 설정 → `(N, H, W, C)` wp image array → torch tensor)

**액션**:
- `action_space`: Box(low=-1, high=1, shape=(dof,)) — 정규화, 실제 control 은 task yaml 의 scale 로 rescale

**Reward/done**:
- Task yaml 에 Python expression 또는 Warp kernel reference (pluggable)
- `terminated` = success/failure / joint limit violation
- `truncated` = step 한도 도달

**torch-cu12 zero-copy**:
- `wp.array(device='cuda:0', dtype=...)` → `torch.as_tensor(wp.array.ptr, ..., device='cuda')` (DLPack interop 이 Warp 에 있음)
- env reset/step 루프 전체를 CUDA 그래프로 캡처 가능하면 이상적

### Unit test 계획

**newton-bridge-rl/tests/** (host, container 내 실행):
- `test_vec_env_construction.py`: `VecEnv(num_envs=4)` 생성 → observation_space/action_space 정합성
- `test_step_produces_finite_obs.py`: random policy 로 50 step → obs/reward NaN 없음
- `test_reset_restores_home.py`: reset 후 q ≈ home_pose
- `test_parallel_envs_diverge.py`: 동일 seed 2 env 가 다른 action 받으면 state 가 분리되는지 (env 간 isolation 보증)

**Smoke**:
- `examples/random_policy.py` 5000 step × num_envs=64 로 throughput 측정 → 기준선: 50k steps/sec (3070 Ti 기준)
- `examples/ppo_train.py` 최소 1 iter 돌아가는지 (training curve 는 별도)

### 의존성
- **외부**: `gymnasium>=1.0`, `torch>=2.0` (이미 image 에 있음 — `torch-cu12`), `tensordict` (선택), `clean-rl` 또는 `rl_games` (training loop 레퍼런스)
- **Newton**: `ModelBuilder.replicate`, `ArticulationView` 배치 인터페이스, `SensorTiledCamera`. 전부 1.1.0 에서 이용 가능 (Phase 0 확인).

### 예상 난이도
상. torch interop + batched sensor + performance tuning 이 조합되어 엔지니어링 비중이 큼. 하지만 본 repo (`newton-bridge`) 의 API 가 잘 되어 있으면 깨끗한 layer 로 얹을 수 있음.

---

## 추천 우선순위

**즉시 가치 순** (실 사용자 피드백 기반으로 선택):
1. **Phase 8 (IK 서비스)** — 컨트롤러 개발자에게 즉각적인 가치, 작업량 중
2. **Phase 2b (multi-articulation)** — scene.yaml schema 는 이미 있음, 실 실행만 구현. 작업량 중
3. **Phase 9 (RL vec-env)** — 연구 타겟, 별도 프로젝트이므로 repo 간 coupling 낮음, 작업량 상
4. **Phase 6b (deformables)** — 가장 크고 case-by-case. 뚜렷한 사용 케이스 생기면 우선순위 상승

각 phase 는 독립적이므로 원하는 순서로 집어들 수 있습니다.
