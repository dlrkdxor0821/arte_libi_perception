# 트러블슈팅 & 설계 노트

실기기(Pi + AI서버) 연동하며 겪은 이슈, 원인, 해결, 그리고 대상복구(recovery) 설계 논의를 정리한다.

포트: `6001` 영상 Pi→AI(UDP) · `6002` cmd AI→Pi(UDP) · `5007` 뷰어 TCP · `6000` Detection→control(예정)

---

## 1. 해결된 이슈

| 증상 | 원인 | 해결 |
|---|---|---|
| Pi 카메라가 **검은 화면** | CSI(Pi)카메라는 libcamera라 `cv2.VideoCapture(0)`가 검정만 줌 (video0은 ISP 노드) | `camera_sender --picamera` (picamera2, `RGB888` 직접) |
| 영상 안 뜸 (송신은 되는데) | 서버 재시작 시 sender frame_id가 0으로 리셋 → 수신기가 "옛 프레임"으로 전부 버림 | `FrameReassembler`에 재시작 감지(재동기화) 추가 |
| 카메라 **거꾸로** | Pi CSI 카메라가 상하 반전 장착 | `camera_sender --picamera`면 `--rotate 180` 자동 |
| 서버 **크래시** `complex < float` | coasting 중 α-β가 area를 음수 예측 → `음수**0.5`=복소수 | `cmd_preview`/`get_latest`에서 area를 0 이상 클램프 |
| `/cmd_vel` 계속 **0** | 서버에 `--drive-host` 없어서 cmd 전송 안 함 (preview만) | `--drive-host 192.168.0.15 --drive-port 6002` (콘솔 `DRIVE ON` 확인) |
| Pi에 스크립트 없음 (`No such file`) | 로컬 변경분을 커밋 안 함 → git pull로 안 옴 | `scp -r .../follower_perception/scripts pinky@<Pi>:...` |
| Qt 글씨 안 보임 | 흰 글씨가 배경에 묻힘 | `_hud_text`로 검은 외곽선 + 크게, 흰색→빨강 |

### 재확인: "서칭 중 찾았다 잃으면 이전 검색에 이어서 하는 듯" → **코드는 정상**
- 시뮬 결과: SEARCHING(elapsed 20s) → 찾음(reset, 0s) → 다시 잃음(SCAN1, 0.1s) = **처음부터 재시작 O** (`recovery.py` FOLLOWING 분기에서 `_search.reset()`).
- 그럼에도 "이어서 하는 듯" 보이면 → **재식별(re-ID) 실패**가 원인일 가능성. `reid_threshold=0.68`이 높아 탐색 중 애매한 각도에서 대상이 보여도 재식별 못 함 → FOLLOWING 안 됨 → 리셋 안 됨.
- 진단: 탐색 중 대상 보일 때 **STATE가 FOLLOWING(초록)으로 바뀌는지** 확인. 안 바뀌면 `reid_threshold`를 0.5로 낮추거나 OSNet 도입.

---

## 2. 대상유지 vs 대상복구 — 기술 스택 구분

문서에 "Track Coasting — Kalman(ByteTrack track_buffer)"로 적혀 있으나 **실제 구현과 다름**:

| 화면/기능 | 실제 기술 | 위치 |
|---|---|---|
| 프레임 간 **ID 유지** | ByteTrack `track_buffer` (내부 칼만) | ByteTrack |
| **`predicted`** (위치 예측 코스팅) | **α-β 필터** (`BBoxSmoother`) | `bbox_smoother.py`+`pipeline.py` |

- ByteTrack track_buffer = **ID** 유지(association), α-β = **위치** 예측(follow 출력) — **다른 계층/목적**.
- `predicted` = 검출 끊긴 프레임에 α-β가 `마지막위치+속도×dt`로 외삽. 최대 `COAST_LIMIT=10`프레임, 넘으면 SEARCHING.
- α-β는 칼만의 단순화(고정 게인) 버전 — **더 좋아서가 아니라** 짧은 코스팅엔 칼만이 과해서 단순·충분한 α-β 선택. 길게/센서융합하면 칼만으로 승격.

---

## 3. 대상복구(Recovery) 설계

### 현재 (follower_BT, 독립 패키지, ROS/py_trees/lidar 없음)
상태머신: `IDLE`(평소) → 등록 → `FOLLOWING` → 놓침 → `SEARCHING` → 찾음 → `FOLLOWING` / 탐색실패 → `IDLE`

탐색 타임라인(대칭, LKD 없음): `±45° 왔다갔다(yaw) → 180° 회전 → 왔다갔다 → 180° 복귀 → 없으면 IDLE`
- yaw = 제자리 회전(`angular.z`, `linear.x=0`). 속도 `ANGULAR_SEARCH`.
- `preview_search.py`로 로봇 없이 회전 프로파일 미리보기 가능.

> ※ 지금은 상태머신(순수 파이썬). 나중에 follower_control의 py_trees BT로 **DrivePolicy만 교체** 가능 (핵심 로직 `search_command`는 동일 구조라 재사용).

### 코너/벽 문제 — LKD가 필요한 지점
사람이 **직각으로 꺾어 벽 뒤로** 가면:
1. α-β 코스팅은 "등속 직진" 가정 → 90° 꺾으면 예측이 엉뚱한 방향 → 바로 놓침.
2. 완전히 벽 뒤면 **제자리 회전(yaw)만으론 벽 너머를 못 봄** → 코너까지 **이동 후** 둘러봐야 함.
3. 근데 벽 쪽 전진 = **벽에 박을 위험** → **LiDAR 필요** (벽 앞 정지 후 회전).

### 설계한 2단계 복구 (LKD → Search)
```
놓침 → [LKD 단계] 마지막 본 방향으로 회전 (~3초)
          ├─ 다시 보이면 → FOLLOWING
          └─ 3초 지나도 없음 → [서치 BT] ±45→180→...
```
LKD 단계 내부 (라이다 있을 때):
| 방향 상황 | 동작 | 필요 |
|---|---|---|
| 뻥 뚫림 | 바로 그쪽 보고 전진(쫓기) | 라이다로 "뚫림" 판단 |
| 벽 | 벽 따라 좀 더 전진 → 벽 끝(코너)에서 그쪽 봄 | 라이다 필수 |

**지금(라이다 X) 가능:** LKD 단계 = "마지막 방향으로 **회전(yaw)만** ~3초" → 서치 BT. (안전, 전진 없음)
**나중(라이다 O):** 그 회전 자리에 뚫림→전진 / 벽→벽따라가기 분기만 끼움 (구조 그대로, 독립성 유지).

---

## 4. 튜닝 파라미터 위치

| 값 | 위치 | 현재 |
|---|---|---|
| `REID_THRESHOLD` / `HSV_THRESHOLD` | `follower_perception/constants.py` | 0.68 / 0.30 |
| `CALIBRATION_ADD_THRESHOLD` (갤러리) | `constants.py` | 0.85 |
| `COAST_LIMIT` (코스팅 프레임) | `constants.py` | 10 |
| 전진/후진/추종회전 속도 | `scripts/cmd_preview.py` | 0.06 / 0.04 / 0.25 |
| 목표거리 `TARGET_SIZE` | `cmd_preview.py` | 220 (√area, 미보정) |
| 탐색 회전 `ANGULAR_SEARCH` | `follower_BT/follower_BT/recovery.py` | 0.25 rad/s |
| 안전: cmd 워치독/속도클램프 | `scripts/cmd_bridge.py` | timeout 0.5s, max 0.15/0.8 |

---

## 5. 남은 작업 (우선순위)

1. **비슷한 인형 여러 개 ReID 테스트** — 등록한 대상을 유사한 것들 사이에서 구별하는지 확인 → 실패 시 **OSNet(`torchreid`) 설치**. (코드 아님, 검증 + 필요시 설치)
2. **LiDAR 장애물 회피** — `cmd_bridge`(Pi)에 `/scan` 구독 추가 → `/cmd_vel` 발행 직전 장애물이면 감속·정지·조향. (로봇 로컬, 이미 `follower_control/lidar_avoidance.py`에 정석 구현 있음)
3. **LKD 2단계 복구** — 서치 BT 앞에 LKD 회전(3초) 단계 추가 (yaw만, 라이다 자리 비워둠).
4. **코너 복구** — LiDAR 회피 위에 "LKD 방향으로 벽 앞까지 전진 → 둘러보기". (2·3 완료 후)
5. **TARGET_SIZE 캘리브레이션** — 원하는 추종 거리에서 실제 √area(또는 height) 값 측정해 설정.
6. **cmd_preview 거리 지표 area→height 전환** — 옆으로 돌 때 area 급감 오작동 방지(선택).

---

## 6. 실행 요약

- 로컬 데모/보기 3종·배포 실행법: [`run.md`](run.md)
- 아키텍처·구성: [`README.md`](README.md)
- Pi 3프로세스: `bringup` + `camera_sender(--picamera)` + `cmd_bridge`. AI서버: `perception_server --udp --drive-host <Pi> --drive-port 6002`.
