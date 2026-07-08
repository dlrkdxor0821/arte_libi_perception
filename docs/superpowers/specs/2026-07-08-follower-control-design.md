# Follower Control 설계 (Spec 2)

- **날짜:** 2026-07-08
- **상태:** 설계 확정 대기 (사용자 리뷰 전)
- **범위:** `follower_control` — **추종 제어만** (map-free). 순찰/Nav2/맵은 이 repo 밖.
- **선행:** [Spec 1 — Follower Perception](2026-07-08-follower-perception-design.md). 본 스펙은 Spec 1의 `Detection` 계약을 소비한다.
- **참조:** `/home/asd/shoppinkki` (`shoppinkki_nav/bt_tracking.py`, `bt_searching.py`, `shoppinkki_core`)

---

## 1. 컨텍스트 & 목표

Spec 1(perception)이 내보내는 주인 `Detection`을 받아 **로봇을 추종시키는 제어층**을 설계한다. 책임:

> **`Detection` + LiDAR `/scan` → PID + 장애물 보정 → `/cmd_vel`.** 대상을 놓치면 복구 행동(BT)을 수행.

### 이 repo의 범위 결정 (중요)
- 이 repo(`arte_libi_perception`)는 **추종을 한 번 테스트**하는 것이 목표다.
- 따라서 본 스펙은 **추종 제어 전용, map-free**.
- **PATROL(순찰) / Nav2 / AMCL / 맵 / domain_bridge = 이 repo 밖.** 프로덕션에선 중앙 ABA 서버가 순찰(Nav2 waypoint)을 담당하지만, 여기서는 다루지 않는다.
- PATROL은 "추종 종료 후 복귀하는 기본 상태"로만 개념 존재하며, 그 구현은 외부/후속이다. 본 스펙에서 SEARCHING 실패 시엔 **정지 + "추종 종료" 신호**만 낸다.

---

## 2. 데이터 흐름 & 실행 위치

```
[AI 서버 / Spec 1]                    [control_node / 이 스펙]           [로봇]
perception ──Detection(TCP)──►  detection_receiver
                                        │
로봇 LiDAR ──/scan(ROS2)──────► scan_provider
                                        │
                                   BT tick(20Hz): PID + LiDAR
                                        │
                                   cmd_publisher ──/cmd_vel──────────► 로봇 모터
```

- control은 **ROS2 노드**로 실행: `/scan` 구독, `/cmd_vel` 발행, Detection은 TCP로 수신.
- Detection 전송은 Spec 1의 어댑터(`ai_server.py`)가 TCP로 보내는 것을 그대로 수신.
- `domain_bridge`(ABA↔로봇 도메인 브리지)는 **프로덕션 배포 관심사** — 본 스펙은 로컬 ROS 토픽 기준으로 설계하고, 브리지는 열린 항목으로만 명시.

---

## 3. 결정 사항 & 가정

| 항목 | 결정 |
|---|---|
| 범위 | 추종 제어만. 순찰/Nav2/맵/AMCL 제외. |
| 제어 방식 | 상대좌표 **PID + LiDAR 보정** (map-free). |
| 거리 제어 | `√area` vs `TARGET_SIZE` → `linear_x`, **후진 포함**(−MAX..+MAX). |
| 방위 제어 | `cx` vs `IMAGE_WIDTH/2` → `angular_z` (**Visual Servoing** 중앙정렬), deadzone + 저역통과. |
| PID 형태 | P 지배(KI 미세, KD=0) — shoppinkki 동형. 파라미터 전부 참고용. |
| LiDAR | 전방 ±15° 비례 감속(< MIN_DIST) + 측면 arc shy-away(< AVOID_DIST). |
| 복구 행동 | **py_trees** BT (shoppinkki 동일 라이브러리). 3단계 시나리오. |
| 상태머신 | **transitions** — TRACKING / SEARCHING (+ 종료). |
| 놓침 판정 | perception이 coasting 후 `None` 반환 → control이 `None`을 놓침 신호로 사용. |
| 발행 주기 | BT tick 20Hz → `/cmd_vel` @ 20Hz. |
| 후진 안전 | LiDAR가 후방 미탐지 → 후진 속도 별도 제한(작게). |
| 파라미터 | 전부 참고용, 실측 튜닝. `config.py`에 집약. |

---

## 4. 폴더 구조 (Spec 1 옆에 추가)

```
arte_libi_perception/
├── follower_perception/            (Spec 1)
└── follower_control/               (Spec 2)
    └── follower_control/
        ├── control_node.py         · ROS2 노드, 20Hz tick, 매니저 배선
        ├── detection_receiver.py   · Detection 수신(TCP) + 최신값 캐시
        ├── scan_provider.py        · /scan 구독 캐시 (get_forward_scan 등)
        ├── cmd_publisher.py        · /cmd_vel (Twist) 발행 래퍼
        ├── state_machine.py        · transitions: TRACKING / SEARCHING
        ├── config.py               · 파라미터 (전부 참고용)
        └── bt/
            ├── bt_runner.py        · py_trees 루트 트리 + StateGuard + tick
            ├── bt_tracking.py      · PID + LiDAR 회피 (추종)
            └── bt_searching.py     · 3단계 복구 (탐색)
```

---

## 5. 상태머신 (`state_machine.py`, transitions)

```
[owner Detection 수신]
        │
        ▼
     TRACKING ──[Detection None = 놓침]──►  SEARCHING
        ▲                                      │
        └──────────[재발견]────────────────────┘
                                               │
                                    [3단계 모두 실패]
                                               ▼
                                     정지 + "추종 종료" 신호
                                     (순찰 복귀는 외부/후속)
```

- **진입**: `Detection.is_owner`가 도착하기 시작하면 TRACKING.
- **TRACKING → SEARCHING**: perception이 `None`(진짜 놓침)을 일정 프레임 연속 반환.
- **SEARCHING → TRACKING**: 탐색 중 주인 재검출.
- **SEARCHING → 종료**: 3단계 복구 모두 실패 → `cmd_vel=0` + 종료 이벤트(로그/신호).

---

## 6. 추종 제어 (`bt_tracking.py`)

py_trees 트리:
```
Selector[
  Sequence[ CheckDetection → ComputeVelocity → ObstacleAvoidance ],   # 추종
  HandleMiss                                                          # 놓침 처리
]
```

### 6.1 ComputeVelocity — PID
| 축 | 입력 → 오차 | 출력 | 비고 |
|---|---|---|---|
| 거리 | `√area` vs `TARGET_SIZE` | `linear_x` ∈ **[−MAX, +MAX]** | 멀면 전진, 너무 가까우면 **후진** |
| 방위 | `IMAGE_WIDTH/2 − cx` | `angular_z` | **Visual Servoing**: base 회전으로 대상을 화면 중앙 유지. deadzone + 저역통과(EMA) |

- P 지배(KI 미세, KD=0), anti-windup clamp. (shoppinkki 동형)
- 마지막 회전 방향 부호를 blackboard `last_known_direction`에 기록 → SEARCHING이 소비(LKD).

### 6.2 ObstacleAvoidance — LiDAR 후처리 (PID 출력 보정)
- **전방 ±15°** 최소거리 < `MIN_DIST` → `linear_x` 비례 감속(접촉 시 0).
- **측면 arc** < `AVOID_DIST` → 반대쪽으로 `angular_z` steer offset.
- 보정 후 `cmd_publisher.publish(linear_x, angular_z)`.

### 6.3 HandleMiss
- `Detection`이 `None`이면 `cmd_vel=0` 발행, miss 카운트 증가.
- 임계 초과 → FAILURE → 상태머신 SEARCHING 전환.

### 후진 안전 (주의)
LiDAR는 전방/측면만 본다. 후진 시 후방 장애물 미탐지 → **후진 최대속도를 전진보다 작게 제한**하고, 필요 시 후진 자체를 옵션 플래그로. (구현 시 명시적 주석)

---

## 7. 복구 BT (`bt_searching.py`, py_trees) — 3단계 시나리오

```
Selector[
  CheckRedetected,                                  # 재발견 → SUCCESS → TRACKING
  Sequence[ Phase1:  10초 정면 유지/대기 → 좌우 ±30° 스캔 회전 ],
  Sequence[ Phase2:  (추가 10초 후) 180° 회전 → ±30° 스캔 ],
  Fail:  cmd_vel=0 + "추종 종료" 신호               # 순찰 복귀는 외부
]
```

- **Phase1**: 대상 소실 직후 10초 정면 유지(재등장 대기) 후, 좌우 ±30° 왕복 스캔.
- **Phase2**: 그래도 못 찾으면 180° 회전(반대 방향 가정) 후 ±30° 스캔.
- **실패**: 모든 단계 후에도 미발견 → 정지 + 종료 신호(로그/이벤트). 이 repo에선 여기서 끝(외부 순찰이 이어받음).
- 어느 단계든 **재검출 시 즉시 TRACKING**(CheckRedetected가 Selector 최상단).
- LKD: Phase1 초기 스캔 방향은 blackboard `last_known_direction`으로 결정.
- 타이밍(10초)·각도(±30°, 180°)는 **참고용, 튜닝 대상**.

---

## 8. 테스트 전략

### 단위 (`tests/`)
| 대상 | 검증 |
|---|---|
| `ComputeVelocity` | 주어진 Detection → 예상 `linear_x`(전진/후진 경계 포함)/`angular_z`, deadzone, clamp |
| `ObstacleAvoidance` | 주어진 `/scan` → 전방 비례 감속, 측면 steer offset |
| `state_machine` | mock Detection 스트림(owner/None/재검출) → TRACKING↔SEARCHING 전환 |
| `bt_searching` | 시간 경과 mock → Phase1→Phase2→실패 순서, 재검출 시 즉시 SUCCESS |

- **Mock**: 가짜 Detection 소스 + 가짜 scan provider. 로봇/네트워크 없이 로직 검증.

### 통합
- 가짜 Detection 스트림 + 가짜 `/scan` 주입 → `/cmd_vel` 시퀀스 검증 (전진→정지→후진→탐색 회전 등 시나리오).

### 실물
- Spec 1(perception) + Spec 2(control) + 로봇 → 사람 추종 주행 확인.

---

## 9. 열린 항목 (Open Items)

- SEARCHING 정확한 타이밍(10초 단계)·각도(±30°, 180°) — 실측 튜닝.
- 후진 속도 안전 한계 값, 후진 on/off 정책.
- `domain_bridge`/배포 토폴로지(프로덕션 ABA↔로봇) — 이 repo 범위 밖, 통합 시 별도.
- control 노드가 이 repo에선 테스트용 독립 노드 — 프로덕션(ABA 통합) 배선은 후속.

---

## 부록 A. 파라미터 (전부 참고용 — 실측 튜닝 대상)

> shoppinkki 값 기반 출발점. 값에 얽매이지 않는다. `config.py`에 집약.

| 파라미터 | 참고 기본값 | 의미 |
|---|---|---|
| `TARGET_SIZE` | 360 | `√area` 목표(거리 setpoint) |
| `IMAGE_WIDTH` | 640 | 프레임 폭(cx 기준) |
| `KP_ANGLE` | 0.0010 | 방위 P 게인 |
| `KP_DIST` | 0.0030 | 거리 P 게인 |
| `ANGLE_DEADZONE` | 45 px | 방위 데드존 |
| `LINEAR_X_MAX` | 0.12 m/s | 전진 최대 |
| `LINEAR_X_REVERSE_MAX` | (전진의 ~절반) | 후진 최대(안전) |
| `ANGULAR_Z_MAX` | 0.60 rad/s | 회전 최대 |
| `MIN_DIST` | 0.20 m | 전방 감속 임계 |
| `AVOID_DIST` | 0.40 m | 측면 회피 임계 |
| `N_MISS_FRAMES` | 40 | SEARCHING 전환 놓침 프레임 |
| `SEARCH_PHASE_TIMEOUT` | 10 s | 각 복구 단계 대기 |
| `SEARCH_SCAN_ANGLE` | ±30° | 스캔 회전 폭 |
| `ANGULAR_Z_SEARCH` | 0.35 rad/s | 탐색 회전 속도 |
