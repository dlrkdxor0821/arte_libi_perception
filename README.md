# arte_libi_perception

비전 기반 **사람 추종(person-following)** 시스템. Libi 로봇이 등록된 주인을 따라간다. map-free(순찰/Nav2 제외, 추종만).

두 개의 독립 패키지로 구성되며 `Detection` 계약(JSON)으로만 연결된다.

| 패키지 | 실행 위치 | 역할 | 경계 |
|---|---|---|---|
| **`follower_perception`** | AI 서버 (PC/GPU) | 카메라 프레임 → 주인 `Detection` (검출·식별·스무딩) | `frame → Detection` |
| **`follower_control`** | 중앙 제어기 (ROS 2) | `Detection` + LiDAR `/scan` → `/cmd_vel` (PID·회피·복구BT) | `Detection → cmd_vel` |

```
[로봇] 카메라 ──UDP영상──► [AI서버] follower_perception ──TCP Detection──► [중앙제어] follower_control ──/cmd_vel(ROS2)──► [로봇] 모터
[로봇] LiDAR /scan ────────────────────────────────────────────────────► follower_control
```

- **AI 서버는 명령하지 않는다** — "보고 식별"만. 주행 명령은 제어기가 만든다.
- **추종 제어는 map-free** — 상대좌표 PID + LiDAR. Nav2/AMCL/맵 없음.

---

## 저장소 구조

```
arte_libi_perception/
├── follower_perception/            # AI 서버 (순수 파이썬, ROS 무관)
│   ├── follower_perception/        #   detection, detector(YOLO11n+ByteTrack), reid_engine,
│   │                               #   color_hist, target_matcher, bbox_smoother, pipeline,
│   │                               #   ai_server(어댑터), constants, mocks
│   ├── tests/                      #   pytest (30개)
│   └── bytetrack.yaml
├── follower_control/               # 중앙 제어 (ROS 2 ament_python)
│   ├── follower_control/           #   pid, lidar_avoidance, search_planner, state_machine,
│   │                               #   tracking_controller, bt_searching, control_loop,
│   │                               #   detection_receiver, + ROS 글루(scan_provider,
│   │                               #   cmd_publisher, tcp_detection_source, control_node)
│   ├── tests/                      #   pytest (36개)
│   └── package.xml / setup.py
└── docs/
    ├── superpowers/specs/          # 설계 문서 (Spec 1, Spec 2)
    ├── superpowers/plans/          # 구현 계획 (TDD)
    └── hardware-verification-checklist.md
```

---

## 설치

### AI 서버 (perception)
```bash
pip install numpy opencv-python ultralytics torch    # 필요 시 --break-system-packages
# (선택) 더 좋은 ReID: pip install torchreid  — 없으면 MobileNet/colour로 폴백
```
> `yolo11n.pt`는 최초 실행 시 ultralytics가 자동 다운로드(인터넷 필요). COCO 사전학습에 `person`(class 0)이 있어 **커스텀 학습 없이 사람 검출 가능**.

### 중앙 제어기 (control)
```bash
# ROS 2 (예: Jazzy) 설치·source 후
pip install py_trees transitions numpy               # 필요 시 --break-system-packages
```

---

## 테스트 실행 (하드웨어·모델 불필요)

단위 테스트는 ROS·GPU·실모델 없이 돈다 (ReID는 colour 백엔드, detector는 정적 파싱).

```bash
# perception (30개)
cd follower_perception && python3 -m pytest -v

# control 순수 로직 (36개)
cd follower_control && python3 -m pytest -v
```
> `python3 -m pytest`(모듈 형태)로 실행해야 `import` 경로가 잡힌다. 편집형 설치(`pip install -e .`)는 불필요.

---

## 실행 방법

### 1. control 노드 (ROS 2) — 하드웨어 없이 스모크 가능
```bash
# 워크스페이스에서 빌드
colcon build --packages-select follower_control
source install/setup.bash

# 노드 실행 (기본: TCP :6000 Detection 수신, /scan 구독, /cmd_vel 발행)
ros2 run follower_control control_node
```

**하드웨어 없는 스모크 테스트** (터미널 3개):
```bash
# T1) 가짜 Detection 송신 (중앙에 있는 먼 주인 → 전진 기대)
python3 -c "import socket,json,time; s=socket.socket(); s.connect(('127.0.0.1',6000)); \
d={'cx':320,'cy':240,'area':100,'bbox':[0,0,10,10],'track_id':1,'is_owner':True,'confidence':0.9,'is_predicted':False}; \
[ (s.sendall((json.dumps(d)+'\n').encode()), time.sleep(0.05)) for _ in range(200) ]"

# T2) 노드
ros2 run follower_control control_node

# T3) 결과 관찰
ros2 topic echo /cmd_vel        # linear.x > 0 (전진) → 송신 중단 시 회전(탐색)
```
설정값은 `follower_control/follower_control/config.py` (TCP 포트·토픽명·게인 등).

### 2. perception (AI 서버)
현재 코어는 라이브러리로 바로 쓸 수 있다:
```python
from follower_perception.pipeline import FollowerPerception
fp = FollowerPerception()               # 실모델(YOLO11n+ReID) 로드
fp.register(frame)                       # 화면 중앙 사람 등록 (3프레임 안정 시 True)
fp.run(frame)                            # 매 프레임
det = fp.get_latest()                    # 주인 Detection 또는 None
```
어댑터 `ai_server.AiServer`는 **주입형 전송**(frame_source/result_sink/command_source)을 받는다.

> ⚠️ **아직 연결 필요(deferred):** 실제 **UDP 영상 수신기**와 **TCP Detection 송신기**의 구체 소켓 구현은 로봇 Image Sender 규약 확정 후 작성 예정(Spec 1 계획의 "Deferred"). control 쪽 수신기(`TcpDetectionSource`, TCP 서버)는 구현되어 있으므로, perception 쪽에 "control로 연결하는 TCP 클라이언트 result_sink"만 붙이면 end-to-end가 된다.

### 3. End-to-End (실기기)
전체 흐름(로봇 카메라 → AI 서버 → 제어기 → 모터) 검증은 ROS·GPU·로봇이 필요하다.
→ **[실기기 검증 체크리스트](docs/hardware-verification-checklist.md)** 를 따라 단계별로 진행.

---

## 현재 상태 (정직하게)

| | 상태 |
|---|---|
| 설계·계획·구현 | ✅ 두 패키지 완료 |
| 단위 테스트 | ✅ perception 30 + control 36 = **66 passed** (3회 연속 확인) |
| ROS 노드 실행 | ⏳ 코드·구문 검증 완료, `ros2` 환경에서 빌드·스모크 필요 |
| 실모델 추론 | ⏳ 코드 완료, `torch`/`ultralytics` 환경에서 실행 필요 |
| UDP/TCP 전송 wire | ⏳ 어댑터 주입형까지 완료, 구체 소켓 wire 일부 미연결(위 2번) |
| 실기기 추종 | ⏳ [체크리스트](docs/hardware-verification-checklist.md)로 검증 예정 |

---

## 문서

- 설계: [`docs/superpowers/specs/`](docs/superpowers/specs/) — Spec 1(perception), Spec 2(control)
- 구현 계획(TDD): [`docs/superpowers/plans/`](docs/superpowers/plans/)
- 실기기 검증: [`docs/hardware-verification-checklist.md`](docs/hardware-verification-checklist.md)

**파라미터는 전부 참고용(튜닝 대상):** perception=`constants.py`, control=`config.py`.
