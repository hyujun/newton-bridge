# docs/

`newton-bridge` 의 전체 문서 인덱스. 루트 [README.md](../README.md) 는 landing + quick start, 여기는 세부 문서 맵.

## 역할별 진입점

**새 유저 (아직 설치 안 한 상태)** →
[INSTALL.md](INSTALL.md) → [USAGE.md](USAGE.md) → [EXAMPLES.md](EXAMPLES.md)

**외부 컨트롤러 개발자** →
[TOPICS.md](TOPICS.md) → [ARCHITECTURE.md §Sync 모델](ARCHITECTURE.md#sync-모델) → [EXAMPLES.md](EXAMPLES.md)

**새 로봇 추가하려는 사용자** →
[ROBOTS.md](ROBOTS.md) → [CONFIGURATION.md §Pack YAML 스키마](CONFIGURATION.md#pack-yaml-스키마) → [EXAMPLES.md §새 pack 튜토리얼](EXAMPLES.md)

**sim 내부 / 확장 작업자** →
[ARCHITECTURE.md](ARCHITECTURE.md) → [DEFERRED_WORK.md](DEFERRED_WORK.md) → 코드 [src/newton_bridge/](../src/newton_bridge/)

**문제가 있는 경우** →
[TROUBLESHOOTING.md](TROUBLESHOOTING.md)

---

## 문서 목록

### 설치 / 빌드

| 파일 | 내용 |
|---|---|
| [INSTALL.md](INSTALL.md) | 호스트 prereq · `install.sh` · `fetch_assets.sh` · `build.sh` · `verify` |

### 사용

| 파일 | 내용 |
|---|---|
| [USAGE.md](USAGE.md) | `run.sh` 하위명령 · freerun/handshake 워크플로우 · `/joint_command` 패턴 |
| [CONFIGURATION.md](CONFIGURATION.md) | 전체 env var + `robot.yaml` / `scene.yaml` 스키마 레퍼런스 |
| [VIEWER.md](VIEWER.md) | `VIEWER` 모드 비교 · X11/Rerun/USD 녹화 |
| [EXAMPLES.md](EXAMPLES.md) | `controller_demo.py` · 센서 · Jupyter · 벤치마크 · 녹화 |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | 단계별 실패 모드 + 조치 |

### 계약 / 확장

| 파일 | 내용 |
|---|---|
| [TOPICS.md](TOPICS.md) | ROS 2 토픽/서비스 계약 (단위, QoS, 센서) |
| [ROBOTS.md](ROBOTS.md) | 새 robot pack 추가 (URDF/MJCF/xacro 3경로) |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 레이어 경계 · sync/time 모델 · pack 계약 · 알려진 이슈 |
| [DEFERRED_WORK.md](DEFERRED_WORK.md) | Phase 2b / 6b / 8 / 9 미구현 범위 + 계획 |

---

## 외부 자료

- Newton 공식 문서: <https://newton-physics.github.io/newton/latest/>
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- Rerun (웹 뷰어): <https://rerun.io/docs>
- mujoco_menagerie (MJCF 소스): <https://github.com/google-deepmind/mujoco_menagerie>
