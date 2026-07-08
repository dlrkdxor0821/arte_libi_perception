# 실기기 검증 체크리스트 (Hardware Verification Checklist)

> **대상:** `follower_perception`(AI 서버) + `follower_control`(중앙 제어) + 로봇(Libi/pinky_pro)
> **목적:** 단위 테스트(66개)가 대신 못 하는 **실행·통합·실제 추종 행동**을 실장비에서 검증.
> **원칙:** 각 항목은 "코드가 있다"가 아니라 **"실제로 그 행동을 눈으로 관찰했다"** 를 체크한다.
> **안전 우선:** teleop/E-stop을 항상 준비하고, 넓고 장애물 적은 공간에서 저속으로 시작.

관련 문서: [Spec 1](superpowers/specs/2026-07-08-follower-perception-design.md) · [Spec 2](superpowers/specs/2026-07-08-follower-control-design.md) · [control plan Task 10 스모크](superpowers/plans/2026-07-08-follower-control.md)

---

## 0. 사전 준비 — 환경·의존성

### 0A. AI 서버 (PC/GPU, x86) — perception
- [ ] Python 3.10+ 확인
- [ ] `pip install torch ultralytics opencv-python numpy` (필요 시 `--break-system-packages`)
- [ ] (선택) `pip install torchreid` — 없으면 MobileNet/colour 폴백으로 동작
- [ ] `yolo11n.pt` 준비 — 최초 `Detector()` 생성 시 ultralytics가 자동 다운로드 (인터넷 필요)
- [ ] GPU 인식 확인: `python3 -c "import torch; print(torch.cuda.is_available())"` → **True** 기대

### 0B. 중앙 제어기 (ABA 서버) — control
- [ ] ROS 2 설치·source (예: Jazzy) → `ros2 --version` 동작
- [ ] `pip install py_trees transitions numpy`
- [ ] 워크스페이스에서 빌드: `colcon build --packages-select follower_control && source install/setup.bash` → **빌드 성공**

### 0C. 로봇 (Pi / Libi)
- [ ] `pinky_pro` 하드웨어 스택 설치 (모터/오도메트리/TF/URDF)
- [ ] LiDAR 드라이버(`sllidar_ros2` 등) 동작
- [ ] 카메라 + Image Sender(UDP 송출) 준비
- [ ] AI 서버·제어기와 네트워크 연결 (같은 LAN 권장 — 무선이면 지터 주의)

### 0D. 네트워크 경로 (아키텍처 3채널)
- [ ] 로봇 카메라 → AI 서버 : **UDP** (영상) 도달
- [ ] AI 서버 → 제어기 : **TCP** (Detection JSON) 도달
- [ ] 제어기 ↔ 로봇 : **ROS 2 DDS + domain_bridge** (도메인 다르면 브리지 설정)

---

## Phase 1 — 컴포넌트 단독 검증 (bottom-up)

### 1A. perception 코어 (AI 서버, 오프라인)
- [ ] **실모델 person 검출**: 웹캠/샘플영상으로 `Detector().detect(frame)` → person bbox + `track_id` 나옴
- [ ] **ReID 실백엔드 로드**: `ReIDEngine()`의 `feat_dim`이 512(OSNet) 또는 576(MobileNet) — **6(colour)이면 폴백 상태**이니 확인
- [ ] **파이프라인 등록→추종**: 실영상에서 `register()` 3프레임 안정 후 True → `run()`+`get_latest()`가 `is_owner=True` Detection 반환
- [ ] **⚠️ `CALIBRATION_ADD_THRESHOLD` 재튜닝**: 현재 0.99는 colour 테스트용 → 실 OSNet에선 갤러리 과성장할 수 있음. 실영상에서 갤러리 증가 속도 관찰 후 `constants.py`에서 하향 조정 (예 0.9 부근)
- [ ] **식별 강건성(단독)**: 등록 후 다른 사람이 프레임에 들어와도 `is_owner`가 주인에게만 유지

### 1B. perception 어댑터 (UDP 수신 / TCP 송신)
- [ ] 로봇 Image Sender → AI 서버 **UDP 프레임 디코드** 성공 (프레임 수신 로그)
- [ ] AI 서버 → **TCP Detection 송신**: 수신측에서 JSON(`cx,area,is_owner,is_predicted...`) 관찰
- [ ] **2소스(Drive/Handy)** 분리 처리 확인 (해당 시): source별 독립 주인

### 1C. control (ROS 2) — 플랜 Task 10 스모크
- [ ] `ros2 run follower_control control_node` 구동 (에러 없이 spin)
- [ ] **가짜 Detection 송신** (플랜 Task 10의 3-터미널 스모크) → `ros2 topic echo /cmd_vel`에 **`linear.x > 0`(전진)** 관찰
- [ ] 송신 중단 → `/cmd_vel`이 `0,0` 후 **회전(탐색)** 으로 전환 관찰
- [ ] `/scan` 구독 동작 (LiDAR 없으면 빈 리스트 → 회피 무동작이 정상)

### 1D. 로봇 하드웨어 (pinky_pro)
- [ ] bringup: 모터·오도메트리·TF 발행
- [ ] `ros2 topic echo /scan` → LiDAR 값 나옴 (inf/nan 섞여도 정상)
- [ ] 카메라 Image Sender UDP 송출 확인
- [ ] **수동 주행**: `/cmd_vel`에 Twist 직접 publish 또는 teleop → 실제 바퀴 회전 (전진/후진/회전 각각)

---

## Phase 2 — 통합 (둘씩)

- [ ] **perception ↔ control**: 실 Detection이 TCP로 control에 도달 → `/cmd_vel`이 대상 위치에 반응 (중앙=직진, 좌우=회전)
- [ ] **control ↔ robot**: `domain_bridge`로 `/cmd_vel`이 로봇 모터에 전달되고, 로봇 `/scan`이 제어기로 역방향 도달
  - [ ] QoS 확인: `/scan`은 SensorDataQoS(BEST_EFFORT) — 브리지 QoS 불일치 시 데이터 안 옴
- [ ] **perception ↔ robot**: 로봇 카메라 UDP → AI 서버 검출까지 실시간 흐름

---

## Phase 3 — End-to-End 추종 행동 (실제로 관찰)

> 각 항목은 **로봇의 실제 움직임**을 눈으로 확인. `/cmd_vel` echo와 병행하면 원인 추적 쉬움.

- [ ] **등록**: 등록 버튼/명령 → 화면 중앙 사람 등록, 이후 그 사람만 `is_owner`
- [ ] **전진**: 대상이 멀어지면 로봇 전진 (`linear.x > 0`)
- [ ] **정지**: 목표 거리 도달(`√area ≈ TARGET_SIZE`) 시 정지 (`linear.x ≈ 0`)
- [ ] **후진**: 대상이 너무 가까우면 후진 (`linear.x < 0`), **속도가 전진보다 작게 제한**되는지 확인 ← 후방 사각 주의
- [ ] **Visual Servoing**: 대상이 좌우로 이동 → 로봇이 회전해 대상을 화면 **중앙 유지**
- [ ] **데드존**: 대상이 중앙 근처(±45px)에서 미세하게 흔들려도 로봇이 덜덜 안 떠는지
- [ ] **장애물 회피(전방)**: 전방에 장애물 → `linear.x` 비례 감속(접촉 시 0)
- [ ] **장애물 회피(측면)**: 측면 벽 → 반대쪽으로 조향
- [ ] **Coasting**: 대상을 짧게 가림 → 예측으로 몇 프레임 유지(`is_predicted=True`), 곧 복구
- [ ] **놓침 → 탐색**: 대상 완전 소실 → `N_MISS_FRAMES`(≈40) 후 SEARCHING 진입
  - [ ] 1단계: ~10초 정면 유지/대기
  - [ ] 2단계: 좌우 ±30° 스캔 회전 (LKD 방향부터)
  - [ ] 3단계: 180° 회전 후 재스캔
- [ ] **재발견**: 탐색 중 대상 재등장 → 즉시 TRACKING 복귀
- [ ] **탐색 실패**: 3단계 모두 실패 → 정지 + "추종 종료" 신호 (이 repo는 여기서 끝, 순찰은 외부)
- [ ] **식별 강건성(실전)**: 비슷한 옷/다른 사람이 끼어들어도 **주인만** 추종 (ReID+HSV 이중게이트)
- [ ] **safe_id 재활용 엣지**: 주인이 오래 사라졌다 다른 사람이 같은 track_id를 받는 상황에서 오추종 여부 관찰 (알려진 한계 — 필요 시 재등록)

---

## Phase 4 — 안전 & 튜닝

- [ ] **후방 안전**: 후진 시 후방 충돌 위험(LiDAR 사각) — 저속 제한 확인, 위험하면 `LINEAR_X_REVERSE_MAX=0` 또는 후진 비활성
- [ ] **네트워크 끊김**: TCP/브리지 끊기면 로봇이 폭주하지 않고 정지하는지 (Detection None → 정지 → 탐색)
- [ ] **지연/지터**: 무선일 때 제어 반응 지연 관찰 → 유선 권장, `/scan` QoS 점검
- [ ] **PID 튜닝**: `KP_ANGLE`(회전), `KP_DIST`(전후진), `TARGET_SIZE`(추종 거리) 실측 조정
- [ ] **LiDAR 튜닝**: `MIN_DIST`(감속), `AVOID_DIST`(회피), 아크 각도 실측 조정
- [ ] **탐색 타이밍**: `SEARCH_HOLD_SEC`(10s), `SEARCH_SCAN_SEC`, `ANGULAR_Z_SEARCH` 실측 조정
- [ ] **파라미터 위치**: perception=`follower_perception/constants.py`, control=`follower_control/config.py`

---

## 코드 강건화 (실배포 전 권장 — 현재 알려진 사항)

- [ ] **`tcp_detection_source._serve`**: malformed JSON에서 `json.loads` 예외로 리더 스레드 종료 → `try/except`로 감싸 견고화
- [ ] **`CALIBRATION_ADD_THRESHOLD`**: 실 ReID 기준 재튜닝 (위 1A)
- [ ] **재등록 경로**: 주인 놓쳐 종료된 뒤 다시 추종하려면 `reset()` + 재등록 흐름(UI) 연결 확인

---

## 롤백/안전장치 (모든 실주행 공통)

- [ ] teleop override 또는 물리 E-stop 상시 준비
- [ ] 첫 테스트는 넓고 장애물 적은 공간, 저속
- [ ] `/cmd_vel` echo를 항상 띄워두고 예상과 다르면 즉시 정지
- [ ] 배터리/발열 모니터링

---

### 완료 판정
- **Phase 1~2 전부 통과** = 각 컴포넌트·통신이 실기기에서 동작.
- **Phase 3 전부 통과** = 실제 추종 시스템으로 검증 완료.
- Phase 3 중 하나라도 실패 시 → 해당 항목의 `/cmd_vel`·Detection·`/scan` 로그로 원인 격리 (perception? control? 통신?).
