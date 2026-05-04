# docs/

newton-bridge 문서 인덱스. 처음 사용자는 위에서부터 순서대로 읽으면 됩니다. 루트 [README.md](../README.md) 는 landing + quick start, 여기는 세부 문서.

## 처음 사용자를 위한 순서

1. **[INSTALL.md](INSTALL.md)** — 호스트 prereq · `install.sh` · `fetch_assets.sh` · `build.sh` · verify
2. **[USAGE.md](USAGE.md)** — `run.sh` 하위명령 + 일상 워크플로우 (freerun/sync, controller_demo, Jupyter, 벤치, 센서, /tf)
3. **[ROBOTS.md](ROBOTS.md)** — 내 로봇 추가. **호스트의 외부 robot_description 폴더 직접 사용 (Path D)** 이 가장 흔한 경로
4. **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** — 단계별 실패 모드 + 조치
5. **[CONFIGURATION.md](CONFIGURATION.md)** — 레퍼런스: 모든 env var · `robot.yaml`/`scene.yaml` 스키마 · ROS 토픽/서비스 계약 · viewer 모드

## 컨트리뷰터 / 내부 작업

- **[ARCHITECTURE.md](ARCHITECTURE.md)** — 레이어 경계 · sync/time 모델 · pack 계약 · 알려진 이슈

## 외부 자료

- Newton 공식 문서: <https://newton-physics.github.io/newton/latest/>
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- Rerun (웹 뷰어): <https://rerun.io/docs>
- mujoco_menagerie (MJCF 소스): <https://github.com/google-deepmind/mujoco_menagerie>
