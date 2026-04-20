# Viewer

Newton 자체 뷰어를 고르는 knob 은 `VIEWER` env var 하나. 디스패치 코드는 [src/newton_bridge/viewer.py](../src/newton_bridge/viewer.py), env var 레퍼런스는 [CONFIGURATION.md §Viewer](CONFIGURATION.md#viewer).

## 모드 비교

| `VIEWER=` | 출력 | X11 | GL 드라이버 | 비용 | 용도 |
|---|---|---|---|---|---|
| `rerun` (기본) | 웹 UI @ `http://localhost:9090` | 불필요 | 불필요 | 중 | 원격/Dev-container, 스크러빙 |
| `gl` | 호스트 X 창 | 필요 | 필요 | 높음 | 로컬 워크스테이션, 인터랙션 (카메라 drag) |
| `usd` | `workspace/runs/sim_<ts>.usd` | 불필요 | 불필요 | 저 (녹화) | Omniverse 로 옮겨서 분석 |
| `file` | `workspace/runs/sim_<ts>.nvpr` | 불필요 | 불필요 | 저 (녹화) | Newton 자체 replay |
| `null` | 무출력 (팩토리만 dispatch) | 불필요 | 불필요 | ~0 | 벤치마크 (pure sim cost 측정) |
| `none` | viewer 완전 비활성 | 불필요 | 불필요 | 0 | 프로덕션, CI |

`none` 과 `null` 의 차이: `null` 은 `ViewerNull.log_state()` 가 매 frame 호출됨 (overhead 가 있으나 작음). `none` 은 viewer 객체 자체를 만들지 않아 init/frame 비용 전부 0.

---

## `rerun` — 웹 UI (기본)

```bash
./scripts/host/run.sh sim
# 브라우저: http://localhost:9090
```

- `network_mode: host` 덕에 포트 매핑 불필요 — `9090` 은 호스트에 직접 바인드
- 원격 머신이면 SSH tunnel: `ssh -L 9090:localhost:9090 user@remote`
- `VIEWER_WIDTH` / `VIEWER_HEIGHT` 는 무시 (웹은 클라이언트가 뷰포트 소유)

**추가 녹화**:

```bash
RERUN_RECORD_TO=/workspace/workspace/runs/session.rrd ./scripts/host/run.sh sim
```

웹 뷰어와 동시에 `.rrd` 파일로도 저장. 나중에 `rerun ~/rerun-recording.rrd` 로 재생.

**포트 충돌**:

```bash
RERUN_WEB_PORT=9091 RERUN_GRPC_PORT=9877 ./scripts/host/run.sh sim
```

---

## `gl` — 네이티브 X11 창

```bash
VIEWER=gl ./scripts/host/run.sh sim
```

**prereq**:
- 호스트에 X 서버 (Wayland 세션에서도 XWayland 로 대부분 OK)
- `DISPLAY` 세팅 (기본 `:0`)
- `run.sh` 가 `xhost +local:docker` 자동 실행 (공용 호스트에서 보안 이슈 주의)
- nvidia GL 드라이버 + `NVIDIA_DRIVER_CAPABILITIES=...graphics,display` (compose.yml 기본값)

**창을 닫으면 sim 이 종료됩니다** ([node.py:288](../src/newton_bridge/node.py#L288) 의 `viewer.is_running()` 체크).

**일시정지**: 창에서 `Space`. sim step 은 멈추지만 rclpy spin 은 계속 → `/joint_command` 는 받아서 pending. 재개 시 반영.

**창 크기**:

```bash
VIEWER=gl VIEWER_WIDTH=1920 VIEWER_HEIGHT=1080 ./scripts/host/run.sh sim
```

**실패 모드**: X socket 접근 불가 → `Xlib.error.DisplayConnectionError`. [TROUBLESHOOTING.md #GL viewer](TROUBLESHOOTING.md#gl-viewer-가-안-뜸) 참조.

---

## `usd` — USD 녹화

```bash
VIEWER=usd ./scripts/host/run.sh sim
# sim 종료 후 workspace/runs/sim_<UTC>.usd 생성
```

**옵션**:

| 변수 | 기본 | 설명 |
|---|---|---|
| `VIEWER_OUTPUT_PATH` | `workspace/runs/sim_<ts>.usd` | 고정 경로로 저장 |
| `VIEWER_OUTPUT_DIR` | `/workspace/workspace/runs` | 타임스탬프 파일 부모 |
| `VIEWER_FPS` | `60` | 녹화 frame rate |
| `VIEWER_UP_AXIS` | `Z` | USD up axis |

출력 파일은 호스트 `workspace/runs/` 에 `HOST_UID` 소유로 생성.

**Omniverse 재생**: USD 를 Isaac Sim / USD Composer 에 drag. 애니메이션 timeline 이 sim 시간 기준으로 깔림.

---

## `file` — Newton 네이티브 녹화

```bash
VIEWER=file ./scripts/host/run.sh sim
# workspace/runs/sim_<UTC>.nvpr
```

Newton `ViewerFile` 의 바이너리 포맷. `.usd` 보다 빠름/작음 but Omniverse 호환 안 됨. Newton 예제로 재생:

```bash
./scripts/host/run.sh example player --viewer gl --file /workspace/workspace/runs/sim_<ts>.nvpr
```

`VIEWER_OUTPUT_PATH` / `VIEWER_OUTPUT_DIR` 는 `usd` 와 동일 의미.

---

## `null` — Viewer dispatch 유지

```bash
VIEWER=null FREERUN_RATE=max ./scripts/host/run.sh sim
```

benchmark 용. viewer API 는 호출되되 render output 이 0 비용. `verify.sh §10` 이 API round-trip 을 여기서 검증.

---

## `none` — 완전 비활성

```bash
VIEWER=none ./scripts/host/run.sh sim
```

- `newton.viewer.*` 를 import 조차 하지 않음
- Rerun/GL 의존성이 없는 환경에서도 동작
- 프로덕션/headless CI 에서 쓸 것

---

## Handshake 모드에서의 viewer

handshake sync 모드에서는 **프레임이 `/sim/step` 또는 `/sim/reset` 호출 시점에만 갱신**됩니다:

```bash
SYNC_MODE=handshake VIEWER=gl ./scripts/host/run.sh sim
# 창이 뜨지만 frozen. 다른 터미널에서 호출하면 step-by-step 갱신.
ros2 service call /sim/step std_srvs/srv/Trigger "{}"
```

`__main__.py` 가 이 경고를 터미널에 띄웁니다:

```
[newton_bridge] note: handshake mode renders only on /sim/step or /sim/reset;
                     the viewer will appear frozen until a controller calls those services.
```

연속 프레임을 보고 싶으면 `controller_demo.py --mode handshake` 로 루프 콜.

---

## 녹화 파일 cleanup

`workspace/runs/` 은 `.gitignore` 에 걸려 있어도 디스크에는 쌓임:

```bash
ls -lh workspace/runs/
rm workspace/runs/sim_2025*.usd      # 구버전 파일 제거
```

`.rrd` 파일도 별도로 큼.

---

## 내부 디스패치 흐름

[viewer.py](../src/newton_bridge/viewer.py) `build_viewer()`:

```
resolve_mode()             # VIEWER env 읽음, ENABLE_VIEWER 감지 시 SystemExit
    │
    ├─ "none"              → return None (caller 가 viewer 없이 진행)
    ├─ "rerun"             → ViewerRerun(web_port, grpc_port, record_to_rrd)
    ├─ "gl"                → ViewerGL(width, height, vsync=False)
    ├─ "usd"               → ViewerUSD(output_path, fps, up_axis)
    ├─ "file"              → ViewerFile(output_path, auto_save=True)
    └─ "null"              → ViewerNull()

      각 viewer 에 world.model 바인딩 (set_model)
```

실패 시 (예: X socket 부재 → `gl` init 실패) caller 가 catch 해서 headless 로 fallback — sim 은 죽지 않고 경고만 stderr 출력 ([__main__.py:62](../src/newton_bridge/__main__.py#L62)).

---

## 관련 문서

- [CONFIGURATION.md §Viewer](CONFIGURATION.md#viewer) — 전체 env var
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — X11 / Rerun 문제
- [USAGE.md](USAGE.md) — run.sh 조합 예시
