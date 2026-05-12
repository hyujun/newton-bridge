# Passive Joints & Equality Constraints

Mimic (URDF) · equality (MJCF connect / weld / joint) — 그리퍼·평행핸드·폐사슬 구조에 필요한 passive joint 지원의 사용 가이드. 자세한 배경/검증 결과는 [ARCHITECTURE.md](ARCHITECTURE.md) 와 git log 의 Phase 1–3 커밋 참조.

## 한눈에 보는 호환성

| Solver | `<mimic>` (URDF) | `<equality>` (MJCF) | 권장 용도 |
|---|---|---|---|
| **mujoco** | ✅ | ✅ | passive joint 가 있는 모든 로봇 (그리퍼·핸드·평행 메커니즘) |
| xpbd | ❌ silent ignore | ❌ silent ignore | 단일 floating body / 강체 체인 (passive joint 없음) |
| featherstone | ❌ silent ignore | ❌ silent ignore | articulation 강체 (passive joint 없음) |
| semi_implicit / style3d / vbd | ❌ | ❌ | n/a (passive 미지원) |

`solver: mujoco` 가 아닌데 모델에 mimic/equality 가 있으면 `newton_bridge.world` 가 startup 시 WARNING 로그를 찍습니다. 무시하면 passive joint 가 _말 없이_ 안 동작합니다.

## 규칙 #1 — passive joint 는 drive 를 0 으로

Newton 의 position drive 는 **모든 DOF 에 인덱스 기반으로 적용**됩니다 (`joint_names` 는 ROS 토픽 노출 필터일 뿐, drive 게이팅이 아님). passive joint (mimic 의 driven 쪽, equality 로 끌려가는 쪽) 에도 default drive 가 걸려서 mimic/equality 와 싸우게 됩니다 — **drive 가 이깁니다**. 결과: mimic 비율이 ~10x 작게 보이거나, equality 가 home_pose 에서 벗어나지 못함.

`robot.yaml` 에서 명시적으로 zero out:

```yaml
joints:
  index_q2:                       # mimic driven joint (passive)
    drive: { stiffness: 0.0, damping: 0.0 }
  middle_q2:
    drive: { stiffness: 0.0, damping: 0.0 }
  # ... 모든 mimic-driven / equality-driven joint 에 동일
```

규칙: **`<mimic>` 의 `joint=` (driven 쪽) 와 `<equality><joint joint1=...>` 양쪽 모두 drive=0.**

## URDF `<mimic>`

```xml
<joint name="index_q2" type="revolute">
  <mimic joint="index_q1" multiplier="1.058" offset="0.723"/>
  ...
</joint>
```

- Newton 의 `builder.add_urdf()` 가 자동으로 `add_constraint_mimic(...)` 호출 — robot.yaml 추가 작업 없음 (drive zero 만 챙기면 됨).
- 검증: ability_hand 4-mimic (index/middle/ring/pinky q2 ← q1, mult=1.058, offset=0.723), q1 명령 후 q2 가 공식과 <0.001 rad 일치.

## MJCF `<equality>`

세 가지 타입 모두 `builder.add_mjcf()` 가 자동 등록 — robot.yaml 추가 작업 없음.

### `<equality><joint .../>` — 비율 결합

```xml
<equality>
  <joint joint1="finger_joint1" joint2="finger_joint2" polycoef="0 1 0 0 0"/>
</equality>
```

franka panda 그리퍼가 사용. `joint_names` 에 master (finger_joint1) 만 노출하면 slave (finger_joint2) 는 매 step equality 로 자동 추종.

### `<equality><connect .../>` — 두 body 의 점 고정

```xml
<equality>
  <connect body1="A" body2="B" anchor="0 0 0"/>
</equality>
```

평행 그리퍼·4절 링크 등 폐사슬 구조에 사용.

### `<equality><weld .../>` — 두 body 의 강체 고정 (위치+자세)

```xml
<equality>
  <weld body1="A" body2="B" relpose="0 0 0 1 0 0 0"/>
</equality>
```

검증: Newton 의 `tests/assets/constraints.xml` (CONNECT×2 + WELD×2 + JOINT×1) 을 SolverMuJoCo 로 2000 substep 돌렸을 때 `max|efc_pos|=3.68e-4` (≪0.01) — 세 타입 모두 end-to-end 작동.

## 알려진 quirk

### franka 그리퍼 open 비대칭 (해결됨)

이전 panda.xml 에는 `<tendon name="split">` (0.5·f1 + 0.5·f2) + `actuator8` 이 그리퍼 close 방향으로 항상 당기는 강한 spring 을 걸어, `finger_joint1=0.04` (open) 명령이 ~0.008 에서 멈추는 비대칭이 있었습니다 (`kp` 를 올려도 미해결). 현재 [robots/franka/models/panda.xml](../robots/franka/models/panda.xml) 에서 해당 tendon+actuator 를 제거 — equality 만으로 양 핑거가 대칭적으로 동작합니다.

다른 MJCF 로봇에서 `<tendon>` 기반 그리퍼 액추에이터를 만나면 동일 증상 가능. 진단: `model.actuators` 또는 MJCF `<actuator>` 섹션에서 `<general tendon=...>` 항목이 robot.yaml drive 와 싸우는지 확인.

### URDF mimic 의 multiplier=0 / offset 만

Newton 1.1.0 의 mimic 은 polycoef 1차항 (선형) 만 지원. URDF 의 mimic 은 원래 선형이므로 문제 없으나, 임의 다항식이 필요하면 MJCF `<equality><joint polycoef="...">` 로 표현하고 그 모델로 마이그레이션.

## 트러블슈팅 한 줄

| 증상 | 원인 |
|---|---|
| passive joint 가 안 따라옴 | solver 가 mujoco 가 아니거나 (startup WARNING 확인), passive joint 의 drive 가 0 이 아님 |
| mimic 비율이 잘못됨 (~10x 작음) | passive joint 의 drive stiffness 가 home_pose 로 끌어당김 → drive=0 |
| 그리퍼 한 방향만 동작 | MJCF `<tendon>`+`<actuator>` 가 robot.yaml drive 와 충돌 — tendon 제거 또는 actuator 비활성화 |
| `model.equality_constraint_count > 0` 인데 동작 안 함 | `solver: mujoco` 인지 확인 |
