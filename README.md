# arte_libi_perception

비전 기반 **사람 추종(person-following)** 시스템. **Libi(도서관 사서 로봇)** 가 등록된 방문자를 따라간다. map-free(순찰/Nav2 제외, 추종만).

```
[로봇 Pi] 카메라 ─UDP영상(640/JPEG)→ [AI서버] perception ─위치(Detection)→ 제어 ─/cmd_vel→ [로봇] 모터
          (YOLO11n + ByteTrack + ReID + HSV)                (PID [+ LiDAR])
```

- **perception = "보고 식별"** — frame → 주인 한 명의 `Detection`(위치/거리 원천 + 식별). 순수 파이썬, ROS 무관, AI서버(GPU).
- **control = "주행"** — `Detection`[+ LiDAR `/scan`] → `/cmd_vel`. ROS 2, 로봇 로컬.
- **AI 서버는 명령하지 않는다** — 제어(특히 LiDAR 20Hz 루프)는 로봇 쪽에 둔다.

---

## 파이프라인 (매 프레임)

```
YOLO11n(검출)  →  ByteTrack(단기 ID 유지)  →  ReID + HSV(장기 주인 식별, 지속 재검증)  →  BBoxSmoother(coasting)
                                                         │
                                            Detection(cx, cy, area, is_owner …)
```
- **검출** YOLO11n — 사람 bbox (커스텀 `best.pt`, class 0 = `people`)
- **식별** ByteTrack(프레임 간 ID) + ReID(OSNet→MobileNet→colour) + HSV 히스토그램 **AND 게이트** — 매 프레임 재검증(잠금 없음)이라 ID 바뀌어도/가려도 외형으로 재식별
- **대상유지** Track Coasting(α-β 예측 + `COAST_LIMIT`) + Online Gallery(각도별 뷰 축적, `gal=N`)
- **위치/거리** `Detection`이 `cx`(방위각 원천)·`area`(거리 원천) 출력 → 제어가 해석

---

## 실시간 데모 (perception_server + Qt 뷰어)

```
camera_sender ─UDP(640·JPEG·최신프레임만)→ perception_server ─TCP JPEG→ Qt viewer(qt_demo)
                                             등록→ReID 추종→박스·cmd_vel(preview) 오버레이
```
- 웹캠(또는 UDP 로봇영상)에서 **[등록] 버튼** → 그 사람만 계속 추종
- 화면 오버레이: 모든 사람(회색), 등록대상(노랑), 주인(초록 `OWNER`), 3등분 방향선, `reid=/  hsv=/  gal=N`, `cmd_vel(preview) lin.x/ang.z`
- `cmd_preview.py`가 "발행할 cmd_vel 값"을 계산(거리=`√area` vs `TARGET_SIZE`, 방향=화면 3등분) — 실제 발행 전 미리보기

**실행법은 [`run.md`](run.md) 참고.**

---

## 배포 아키텍처 (실 로봇)

역할 3개: **AI서버(PC/GPU)** = perception, **로봇(Pi)** = 카메라+모터+LiDAR, **뷰어(아무 PC)** = Qt.

로봇을 실제로 구동할 때 — **장애물 회피 여부**로 구조가 갈린다:

| | (A) 회피 없음 — 간단 | (B) 회피 있음 — 제대로 |
|---|---|---|
| cmd 계산 | AI서버가 계산 → **`/cmd_vel` 직접** | AI서버는 **Detection만** → Pi의 `follower_control`(LiDAR+PID) |
| Pi 역할 | 멍청이 (camera_sender + bringup) — Detection 모름 | control까지 (LiDAR·Detection 20Hz 융합) |
| 안전 | ⚠️ LiDAR 회피 없음, cmd 네트워크 → **끊기면 정지** 필요 | LiDAR 회피 O, 20Hz 루프 로봇 로컬 |

**AI서버→로봇 전달 = ROS or 소켓:**
- **ROS**: AI서버에 ROS 2 + Pi와 같은 DDS망(또는 `domain_bridge`) → AI서버가 `/cmd_vel`(또는 Detection) **직접 발행**, Pi는 구독만 (글루 최소)
- **소켓**: AI서버 **ROS-free 유지**(순수 python), Pi에 작은 republish 노드. `follower_control`엔 이미 TCP Detection 수신기(`TcpDetectionSource`, :6000) 있음

> **`/cmd_vel` 자체는 항상 ROS 2** (control→모터, 로봇 로컬). 네트워크로 넘기는 건 작은 **Detection** 또는 **cmd 값**뿐.

---

## 저장소 구조

```
arte_libi_perception/
├── follower_perception/                 # AI 서버 (순수 파이썬, ROS 무관)
│   ├── follower_perception/             #   detection, constants, color_hist, reid_engine,
│   │                                    #   detector(YOLO11n+ByteTrack), target_matcher,
│   │                                    #   bbox_smoother, pipeline, profile, ai_server, mocks
│   ├── scripts/                         #   perception_server(실시간), camera_sender(로봇 UDP송신),
│   │                                    #   udp_video, frame_proto, cmd_preview, register_and_track
│   ├── weights/best.pt                  #   커스텀 person(people) 모델
│   └── tests/                           #   pytest 67
├── qt_demo/                             # Qt5 QML+C++ 뷰어 (영상 표시 + 등록/리셋)
│   ├── CMakeLists.txt · src/ · qml/
├── follower_control/                    # 중앙 제어 (ROS 2): pid, lidar_avoidance,
│   └── ...                              #   search_planner, bt_searching, tracking_controller …  (pytest 36)
├── run.md                               # 실행 명령 모음
└── docs/superpowers/{specs,plans}/      # 설계·구현 문서
```

---

## 설치

```bash
# AI 서버 (perception) — torch 있는 venv 권장
pip install numpy opencv-python ultralytics torch     # (선택) torchreid → OSNet(같은 색 구분↑)
# 중앙 제어 (control)
pip install py_trees transitions numpy                # + ROS 2 (예: Jazzy)
# Qt 뷰어
sudo apt install qtbase5-dev qtdeclarative5-dev cmake g++   # Qt5.15
```

## 테스트 (하드웨어·모델 불필요)

```bash
cd follower_perception && python3 -m pytest -q        # 67 passed (colour 백엔드, MockDetector)
cd follower_control    && python3 -m pytest -q        # 36 passed
```

## 실행

→ **[`run.md`](run.md)** — localhost 데모 / 실 배포 / 오프라인 프로필 검증 명령.

---

## 현재 상태 (정직하게)

| | 상태 |
|---|---|
| perception 코어 + 지속 ReID + 온라인 갤러리 | ✅ 구현·테스트(67) |
| UDP 영상 송/수신 (640·JPEG·최신프레임만) | ✅ 구현·localhost 검증 |
| Qt5 뷰어 (등록/추종/cmd_vel preview) | ✅ 빌드·offscreen 스모크 |
| 실모델(best.pt)+ReID(MobileNet) 추론 | ✅ venv에서 확인 |
| control (PID·LiDAR·탐색) | ✅ 구현·테스트(36) — 데모엔 미연결 |
| **AI서버 → 로봇 `/cmd_vel`(또는 Detection) 발행** | ⏳ **미연결** — ROS 발행 노드 or 소켓 (위 배포 A/B) |
| 실기기 추종 | ⏳ [체크리스트](docs/hardware-verification-checklist.md)로 검증 |

## 남은 링크 / 옵션
- **로봇 구동 마지막 링크**: AI서버 → `/cmd_vel`(A안, ROS/소켓) **또는** Detection → follower_control(B안). 회피 필요 여부로 선택.
- (선택) H.264 HW 인코딩(Pi CPU↓, GStreamer 필요) · OSNet(같은 색 옷 구분) · 갤러리 디스크 영속화.

## 문서
- 설계: [`docs/superpowers/specs/`](docs/superpowers/specs/) — perception/control/가이드프로필/Qt뷰어
- 구현 계획(TDD): [`docs/superpowers/plans/`](docs/superpowers/plans/)
- 실기기 검증: [`docs/hardware-verification-checklist.md`](docs/hardware-verification-checklist.md)

**파라미터는 전부 튜닝 대상:** perception=`constants.py`(REID/HSV/gallery 임계값)·`cmd_preview.py`(속도·목표거리), control=`config.py`.
