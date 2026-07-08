# Follower Perception 설계 (Spec 1)

- **날짜:** 2026-07-08
- **상태:** 설계 확정 대기 (사용자 리뷰 전)
- **범위:** `follower_perception` (AI Server) 만. 제어(PID/BT/ROS)는 Spec 2로 분리.
- **참조:** `/home/asd/shoppinkki` (동형 아키텍처의 선행 구현)

---

## 1. 컨텍스트 & 목표

비전 기반 **사람 추종** 시스템의 **perception 층**을 설계한다. 이 층의 유일한 책임은:

> **카메라 프레임 → 추종 대상(주인) 한 명의 `Detection`(위치/거리 원천 + 식별 결과) 출력.**

주행 명령(cmd_vel)은 만들지 않는다. 그것은 중앙 제어 서버(ABA Service)의 몫이며 별도 스펙(Spec 2)에서 다룬다.

### 왜 perception을 먼저, 분리해서 하나
- perception은 **ROS·네트워크에 안 묶인 순수 파이썬 코어**로 둘 수 있어 이식성/테스트성이 높다("안정된 코어").
- 다른 repo에 병합되더라도 `frame → Detection` 계약만 지키면 그대로 재사용된다.
- 모델 미학습 상태여도 개발 가능하다(아래 참조).

---

## 2. 시스템 아키텍처 (SW Architecture 다이어그램 매핑)

```
[Equipment / Libi 로봇]              [Server]
Image Sender ──UDP(영상)──► AI Service (= 이 repo, perception)
(Drive/Handy Board)              │ frame → Detection
                                 │
                    ABA Service ◄──TCP(Detection/명령)── AI Service
                    (= 중앙 제어, Spec 2)
                         │ ROS2 DDS + domain_bridge
                         ▼
                    Libi Drive/Handy Controller (로봇 모터/LiDAR)
```

| 다이어그램 요소 | 본 설계 매핑 |
|---|---|
| AI Server / **AI Service** | **perception (이 repo, Spec 1)** |
| ABA Server / **ABA Service** | 중앙 제어 서버 (Spec 2) |
| Libi Drive/Handy Board | 로봇 (Equipment) |
| Image Sender → AI Server (**UDP**) | perception 영상 입력 |
| AI Service ↔ ABA Service (**TCP**) | Detection 결과 + 등록/리셋 명령 |
| ABA Service ↔ 로봇 (**ROS2 DDS**) | 제어 채널, `domain_bridge` — Spec 2 |

### 핵심 결론
- **AI 서버는 명령하지 않는다** — 오직 "보고 식별". 이유: 제어 루프(`PID + LiDAR 보정`)의 LiDAR는 로봇에 로컬이고, 20Hz 제어를 네트워크에 묶으면 위험. 따라서 제어는 중앙서버/로봇 측(Spec 2).
- **무거운 영상은 중앙서버(ABA)를 거치지 않는다** — `로봇 → AI서버` 직행(UDP). 중앙서버엔 작은 토픽(Detection, /scan, /cmd_vel)만 흐르므로 `domain_bridge` 사용해도 CPU 과부하 없음. 유일한 주의점은 CPU가 아니라 **네트워크 지연/지터**(Spec 2 이슈).
- **perception엔 ROS 의존성이 없다** — rclpy 불포함. 순수 CV/torch. ROS/domain_bridge는 전부 Spec 2.

---

## 3. 결정 사항 & 가정

| 항목 | 결정 |
|---|---|
| 검출 모델 | ultralytics **YOLO11n**, **COCO 사전학습(person=class 0) 그대로 시작**. 커스텀 학습 불필요, 가중치 경로 교체식. |
| 트래커 | ultralytics **내장 ByteTrack**(`model.track(persist=True)`). 검출+트래킹 한 호출. `track_buffer`가 coasting 시간제한. |
| ReID | **OSNet**(torchreid) 우선, **torchvision MobileNetV3** 폴백. (shoppinkki 동형) |
| 색상 | **HSV 48-bin**(H16+S16+V16) 히스토그램, 상관계수 매칭. |
| 주인 매칭 | **이중게이트** `reid_sim ≥ τ_reid AND hsv_sim ≥ τ_hsv`, `VERIFY_FRAMES` 연속 통과 시 `safe_id` 락 → 이후 fast-path. |
| Coasting | **α-β 필터**(예측 출력) + **ByteTrack track_buffer**(ID 유지) 분업. |
| Online Gallery | 주기적으로 주인 embedding을 gallery에 축적(뒷모습·각도 대응), 상한 있음. |
| 등록 방식 | 외부 UI **등록 버튼** → `register()` → **화면 중앙 최대 사람** 선택, **N프레임 안정** 후 확정. |
| 연산 환경 | **AI 서버 = PC/GPU(x86)**. pytorch YOLO11n + OSNet 매 프레임 실행. NCNN/양자화 불필요. |
| 전송 | 어댑터: **UDP 수신**(영상) + **TCP 양방향**(Detection 송신 / 명령 수신). 와이어 포맷은 통합 시 확정(교체가능). |
| 다중 소스 | Image Sender 2개(Drive/Handy) → `source_id`로 스트림 분리, 각자 독립 `FollowerPerception`. |
| 파라미터 | **전부 참고용** — 실측 튜닝 대상. `constants.py`에 집약. |

---

## 4. 폴더 구조

```
arte_libi_perception/
└── follower_perception/
    ├── follower_perception/
    │   ├── detection.py         · Detection dataclass (유일한 공개 계약) + 내부 TrackedBox
    │   ├── detector.py          · YOLO11n + 내장 ByteTrack (한 호출)
    │   ├── reid_engine.py       · ReID 임베딩 (OSNet → MobileNet 폴백)
    │   ├── color_hist.py        · HSV 48-bin 히스토그램 + 유사도
    │   ├── target_matcher.py    · 주인 특정: 이중게이트 + VERIFY + online gallery
    │   ├── bbox_smoother.py     · α-β 필터 (coasting 외삽)
    │   ├── pipeline.py          · FollowerPerception facade (공개 API)
    │   ├── constants.py         · 임계값 (전부 참고용, 튜닝 대상)
    │   ├── mocks.py             · MockDetector (테스트용)
    │   └── ai_server.py         · 서버 어댑터 (UDP 수신 + TCP 송수신)
    ├── tests/                   · 모듈별 pytest + 파이프라인/어댑터 통합
    ├── models/                  · yolo11n.pt (COCO 사전학습)
    ├── bytetrack.yaml           · ByteTrack 설정 (track_buffer 등)
    ├── requirements.txt
    └── README.md
```

---

## 5. Detection 계약 (perception의 유일한 출력 타입)

```python
@dataclass
class Detection:
    cx: float          # bbox 중심 x  → 방위각 (control이 소비)
    cy: float
    area: float        # w*h          → 거리 (√area)
    bbox: tuple        # (x1, y1, x2, y2)
    track_id: int      # ByteTrack ID (단기 식별)
    is_owner: bool     # 주인 확정 여부 (ReID+HSV 게이트 통과)
    confidence: float
    is_predicted: bool # True = coasting 외삽값(실측 아님)
```

`get_latest()`가 주인 미확정/미검출이면 `None`을 반환한다. control은 `Detection`만 알면 되고, 내부 알고리즘(ByteTrack/ReID/Kalman)은 모른다.

---

## 6. 모듈 인터페이스

```python
# detection.py
@dataclass
class Detection: ...          # 위 5절
@dataclass
class TrackedBox:             # 내부 후보 (공개 아님)
    bbox: tuple; cx: float; cy: float; area: float; track_id: int; confidence: float

# detector.py
class Detector:
    def detect(self, frame) -> list[TrackedBox]:
        """YOLO11n + 내장 ByteTrack. person(cls=0)만, 각 박스에 track_id 부착."""

# reid_engine.py
class ReIDEngine:
    def extract(self, roi_bgr) -> np.ndarray      # L2정규화 임베딩
    def similarity(self, a, b) -> float            # 코사인

# color_hist.py
def hsv_hist(roi_bgr) -> np.ndarray                # 48-bin 정규화
def hist_similarity(a, b) -> float                 # 상관계수 → [0,1]

# target_matcher.py
class TargetMatcher:
    def register(self, roi_bgr) -> None            # 템플릿(reid+hsv) 저장, gallery 초기화, safe_id=None
    def match(self, cands: list[TrackedBox], frame) -> int | None   # owner track_id or None
    def calibrate(self, owner_roi) -> None         # online gallery 축적
    @property
    def is_registered(self) -> bool
    def reset(self) -> None

# bbox_smoother.py
class BBoxSmoother:
    def update(self, cx, cy, area) -> None         # α-β 갱신
    def predict(self, dt) -> tuple                 # 외삽 (cx, cy, area)
    def reset(self) -> None

# pipeline.py — facade (공개 API)
class FollowerPerception:
    def register(self, frame) -> bool              # 등록 시도, 확정 시 True
    def run(self, frame) -> None                   # 매 프레임 파이프라인 갱신
    def get_latest(self) -> Detection | None       # 최신 주인 Detection (coasting 포함)
    def reset(self) -> None                        # 등록 해제
```

---

## 7. 데이터 흐름

### ① 등록 (UI 버튼 → `register(frame)` 반복 호출)
```
detect → 화면 중앙 + 최대 person 선택
      → N프레임 안정(IoU) 확인
      → roi 캡처 → matcher.register(roi): template = reid + hsv, gallery = [reid], safe_id = None
      → is_registered = True 반환
```

### ② `run(frame)` (매 프레임)
```
cands = detector.detect(frame)              # persons + ByteTrack id
owner_id = matcher.match(cands, frame)
   ├ 각 후보 roi → reid.extract + hsv_hist
   ├ 이중게이트: reid_sim ≥ τ_reid AND hsv_sim ≥ τ_hsv
   ├ VERIFY_FRAMES 연속 통과 → safe_id 락
   └ safe_id 락 후: 해당 id는 reid 스킵(fast-path), calibrate 주기에만 재계산
if owner:
    smoother.update(cx, cy, area)
    (주기적) matcher.calibrate(owner_roi)   # 뒷모습/각도 embedding 축적
```

### ③ `get_latest()` (coasting 포함 출력)
```
owner 최근 관측  → Detection(smoothed, is_predicted=False)
owner 일시 소실  → smoother.predict() → Detection(is_predicted=True)   # coasting 중
소실 한도 초과   → None                                              # 진짜 놓침 → Spec 2 BT 복구 트리거
```

### Coasting 분업 (둘의 역할이 다름)
| 메커니즘 | 역할 | 위치 |
|---|---|---|
| ByteTrack `track_buffer` | **ID 유지** — 버퍼 내 재등장 = 같은 track_id (재등록 방지) | `bytetrack.yaml` (=시간제한) |
| α-β smoother | **위치 출력** — 공백 동안 예측 bbox를 계속 내보내 control이 servo 유지 | `bbox_smoother.py` |

perception의 책임은 **"진짜 놓치면 `None`"까지**. 그 뒤 복구는 Spec 2의 BT가 `None`을 신호로 시작.

---

## 8. 서버 어댑터 (`ai_server.py`)

```
ai_server.py
├── FrameReceiver (UDP)     · Image Sender → (source_id, frame) 디코드 스트림
├── AbaChannel (TCP, 양방향) · ABA Service와:
│      ├─ recv 명령: {"cmd": "register"|"reset", "source": ...}   ← UI 등록버튼 도달점
│      └─ send 결과: Detection(JSON)
└── per-source 루프:
       perceptions[source] = FollowerPerception()       # 소스별 독립 주인
       프레임 도착 → p.run(frame) → p.get_latest() → AbaChannel.send(source, det)
       'register' 명령 → p.register(최근 frame)
       'reset' 명령    → p.reset()
```

- **2소스(Drive/Handy)**: `source_id` 딕셔너리 분리, 각자 독립 인스턴스.
- **등록 경로**: UI → ABA Web → ABA Service → (TCP) → `AbaChannel` register 명령 → `p.register()`.
- **와이어 포맷**(UDP 프레임 인코딩, TCP 프레이밍)은 Image Sender 구현(Spec 2)에 맞춰 통합 시 확정. `FrameReceiver`/`AbaChannel`을 얇게·교체가능하게 유지. 코어는 영향 0.

---

## 9. 테스트 전략

### 단위 (순수 코어, 네트워크·GPU 없이) — `tests/`
| 대상 | 검증 |
|---|---|
| `reid_engine` | 같은 roi 유사도 ≈ 1, 벡터 L2정규화·차원 |
| `color_hist` | 합 = 1, 동일 roi 유사도 = 1 |
| `bbox_smoother` | 등속 입력 → 예측 외삽 정확, 수렴 |
| `target_matcher` | register → 같은 roi = owner / 다른 roi = None / VERIFY 카운트 / calibrate 시 gallery 증가 |
| `pipeline` | **MockDetector**로: 등록 → run → owner, 검출 끊으면 coasting(is_predicted), 한도 초과 → None |

- **`mocks.py` — `MockDetector`**: 스크립트된 `TrackedBox` 리스트 반환. YOLO 가중치/GPU 없이 파이프라인·매칭 로직 검증.

### 통합 (어댑터)
- 가짜 UDP 송신기가 녹화 영상 스트림 → `ai_server` → 가짜 TCP ABA가 Detection 수신 → end-to-end 검증. **실물 테스트 하니스의 토대.**

### 실물
- Pi Image Sender → AI 서버 → Detection 스트림 확인.

---

## 10. Spec 2 (제어)로 이관 — 요구사항으로만 기록

- **PID**: 거리(√area → 전진/**후진**), 방위(cx → Visual Servoing 중앙정렬)
- **LiDAR** 장애물 보정 (로봇 로컬)
- **BT 복구 시나리오**: perception `None` 신호 → ① 10초 후 좌우 ±30° 탐색 → ② 다시 10초 후 180° 회전 후 ±30° 탐색 → ③ 못 찾으면 **순찰(patrol) 모드**
- **상태머신**: IDLE / TRACKING / SEARCHING / PATROL
- **cmd_vel 발행 + `domain_bridge`** (ABA ↔ 로봇 ROS2)
- **UI 등록 버튼** 배선 (ABA Web → ABA Service → AI 서버 명령)
- **네트워크 지연/지터** 대응 (제어 루프가 브리지 위를 지남; `/scan`은 SensorDataQoS, 필요 시 로봇 로컬 안전정지)

---

## 11. 확인 필요 (Open Items)

- Image Sender 2개(Drive/Handy)의 역할 구분 — Drive=주행 카메라, Handy=핸디 카메라? (스트림 처리엔 무관하나 명확히 하면 좋음)
- UDP 프레임 인코딩(JPEG? 프레임당 1패킷 vs 분할)·TCP 프레이밍 규약 — Image Sender 구현과 맞물림(통합 시 확정)
- ReID 의존성: torchreid(OSNet) 설치 가능 여부 — 불가 시 torchvision 폴백으로 시작

---

## 부록 A. 파라미터 (전부 참고용 — 실측 튜닝 대상)

> shoppinkki 값 기반 출발점. 값 자체엔 얽매이지 않는다. `constants.py`에 집약.

| 파라미터 | 참고 기본값 | 의미 |
|---|---|---|
| `MIN_CONFIDENCE` | 0.42 | YOLO 최소 신뢰도 |
| `τ_reid` (REID_THRESHOLD) | 0.48 | ReID 코사인 임계 |
| `τ_hsv` (HSV_THRESHOLD) | 0.38 | HSV 상관 임계 |
| `VERIFY_FRAMES` | 5 | safe_id 락까지 연속 통과 프레임 |
| `CALIBRATION_INTERVAL` | 30 | gallery 축적 주기(프레임) |
| `CALIBRATION_ADD_THRESHOLD` | 0.94 | 이 미만 유사도일 때만 gallery 추가 |
| `MAX_GALLERY_SIZE` | 50 | gallery 상한 |
| α, β (BBoxSmoother) | 0.45, 0.15 | α-β 필터 게인 |
| coasting 외삽 한도 | ~10 frame | 예측 출력 최대 지속 |
| `track_buffer` (ByteTrack) | 튜닝 | ID 유지 프레임 수 |
| HSV bins | 16/16/16 (=48) | H/S/V 히스토그램 |
