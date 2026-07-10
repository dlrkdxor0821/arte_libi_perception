# Qt Perception Viewer (register/track 데모) 설계 (Spec 4)

- **날짜:** 2026-07-10
- **상태:** 설계 확정 대기 (사용자 리뷰 전)
- **범위:** 로컬 테스트용 Qt5 QML+C++ 뷰어 + Python perception 소켓 서버. "사진 1장 등록 → 지속 ReID"를 **웹캠으로 눈으로** 확인.
- **참조:** [Spec 3 — 가이드 프로필 등록](2026-07-10-guide-profile-registration-design.md), `follower_perception/ai_server.py`(주입형 어댑터), `arte_libi_gui/libi_gui`(Qt5.15/QML 스타일 원본).
- **비고:** 커밋은 사용자가 직접(에이전트는 커밋 안 함).

---

## 1. 컨텍스트 & 목표

Spec 3에서 만든 register-once + 지속 ReID를 **웹캠 라이브**로 확인할 GUI가 필요하다. 스택은 `arte_libi_gui`와 동일한 **Qt5 / QML / C++**.

> **목표:** Qt 창에 웹캠 영상이 뜨고, **[등록] 버튼**을 누르면 그 순간 사람이 등록되며, 이후 그 사람에게만 박스가 유지(지속 ReID)되는 것을 눈으로 확인한다.

**경계 (B안 — 승인됨):** perception(YOLO+ReID)은 Python에 그대로 둔다. **Qt는 "표시 + 버튼"만** — CV 코드 없음. 검출·ReID·박스/상태 렌더링은 전부 Python이 하고, 결과 프레임(JPEG)만 Qt로 스트림한다.

---

## 2. 아키텍처

```
[Qt5 QML+C++ 앱  (qt_demo/)]        ←— localhost TCP —→        [Python 서버 (perception_server.py)]
 · QML Main.qml:                                                · cv2.VideoCapture 로 웹캠 소유
     Image(스트림 표시) + [등록]/[리셋] 버튼 + 상태텍스트         · FollowerPerception(best.pt + ReID) 실행
 · C++ PerceptionClient(QObject):                               · 프레임에 박스+상태 그려 JPEG 인코딩
     - 소켓 연결, 길이접두 JPEG 수신 → QImage → QML             · 각 프레임 전 명령 폴링(select) → register/reset
     - register()/reset() → 명령 1줄 전송                        · register = 현재 프레임으로 register_from_image
```

- **Qt는 얇다:** 이미지 1개 + 버튼 2개. CV·모델·상태판단 전부 Python.
- **실제 시스템과 동형:** "로봇 카메라 → AI 서버(perception) → 결과" 구조를 로컬 소켓으로 축소.
- **순수 코어 불변:** `follower_perception/` 패키지엔 GUI/소켓 코드 안 들어감. 소켓 서버는 `scripts/`에, Qt는 별도 `qt_demo/`에.

---

## 3. 와이어 프로토콜 (단순·디버그 가능)

**단일 TCP 연결, 양방향.** localhost, 기본 포트 `5007`(설정 가능).

- **서버 → 클라이언트 (영상):** 프레임 반복. 각 프레임 = `4바이트 big-endian 길이 N` + `N바이트 JPEG`. (상태는 Python이 프레임 위에 텍스트로 그려 넣으므로 Qt가 파싱할 것 없음.)
- **클라이언트 → 서버 (명령):** 개행 종료 텍스트 — `"register\n"`, `"reset\n"`. 서버는 프레임 송신 전 `select()`로 비블로킹 폴링 후 적용.
- 연결 종료/에러: 양쪽 모두 소켓 정리 후 재연결 시도(클라이언트) / 다음 연결 대기(서버).

> 단일 스레드 서버 루프: `프레임 read → run → (명령 폴링·적용) → 박스 그림 → JPEG 전송`. 스레드/락 없음.

---

## 4. 컴포넌트 설계

### 4.1 Python 소켓 서버 (`follower_perception/scripts/perception_server.py`)
- **주입형(테스트 가능):** `serve(conn, *, frames, perception, draw=..., poll_cmd=...)` — 프레임 소스와 perception을 주입받는 순수 루프. 실제 실행부는 `cv2.VideoCapture` + `FollowerPerception` + 소켓을 이 루프에 연결.
- 프레임 소스 추상화: `next() -> frame | None`. 실 웹캠(`VideoCapture`) 또는 `--test-pattern`(합성 이동박스, 웹캠·torch 없이 동작).
- 명령 처리: `register` → `perception.register_from_image(current_frame)`; `reset` → `perception.reset()`.
- 렌더: `draw_overlay(frame, detection) -> frame` — 주인 박스(초록/예측 주황) + 텍스트(`owner id, reid/hsv sim, predicted`). 순수 함수(단위 테스트 대상).
- CLI: `--camera 0 | --test-pattern`, `--port 5007`, `--weights`, `--device`, `--no-hsv/--hsv-threshold`.

### 4.2 프레임 프로토콜 헬퍼 (`follower_perception/scripts/frame_proto.py`, 신규)
- `send_frame(sock, jpeg_bytes)` / `recv_frame(sock) -> jpeg_bytes | None` (길이접두 read/write). 순수·단위 테스트 대상. 서버·테스트클라이언트가 공유.

### 4.3 Qt C++ 클라이언트 (`qt_demo/src/PerceptionClient.{h,cpp}`)
- `QObject`, QML에 `client` 컨텍스트 프로퍼티로 노출.
- `QTcpSocket`으로 연결, `readyRead`에서 바이트 누적 → 길이접두 파싱 → `QImage`(JPEG 디코드) → `QQuickImageProvider`에 저장 + `frameChanged(counter)` 시그널.
- `Q_INVOKABLE void register_()` / `reset()` → `"register\n"`/`"reset\n"` write.
- `Q_PROPERTY(bool connected ...)`.

### 4.4 Qt QML (`qt_demo/qml/Main.qml`) + `main.cpp` + `CMakeLists.txt`
- `Image { source: "image://perc/frame?c=" + client.frameCounter }` — 시그널마다 갱신. (또는 동등한 최신 QImage 표시 패턴.)
- `[등록]` → `client.register_()`, `[리셋]` → `client.reset()`. 연결상태/안내 텍스트.
- 스타일: `arte_libi_gui/libi_gui`의 색/폰트/버튼 컴포넌트 **복사**해 사용(사용자 허용).
- `main.cpp`: `QGuiApplication` + `QQmlApplicationEngine` + imageProvider 등록 + `client` 노출.
- `CMakeLists.txt`: **Qt5**(Core Quick Network) — libi_gui와 동일 버전.

---

## 5. 이 세션에서 검증 가능 / 불가능

- ✅ **검증 가능(웹캠·torch 없이):** `send_frame`/`recv_frame` 왕복, `draw_overlay` 순수함수, 서버 루프(`--test-pattern` 합성프레임 + `MockDetector` + colour ReID)에 파이썬 테스트 클라이언트가 붙어 프레임 수신·`register` 명령 왕복. Qt는 **Qt5로 컴파일**(툴체인 존재).
- ⏳ **사용자 환경 필요:** 실 웹캠 + torch/ultralytics로 `best.pt` 실추론, Qt 창 실제 표시(웹캠 없이도 `--test-pattern`이면 데모 가능).

---

## 6. 에러 처리

| 상황 | 처리 |
|---|---|
| 웹캠 열기 실패 | 서버가 명확한 에러 출력 후 종료(또는 `--test-pattern` 안내). |
| torch/ultralytics 미설치 | `FollowerPerception`/`Detector`의 `ImportError` 잡아 안내 후 종료(단위 테스트는 colour). |
| 클라이언트 연결 끊김 | 서버는 다음 연결 대기, Qt는 재연결 시도 + "연결 끊김" 표시. |
| 잘린/손상 프레임 | `recv_frame`이 부분수신 처리(길이만큼 read), EOF면 None. |
| 등록 시 사람 없음 | `register_from_image`가 None → 서버가 프레임에 "사람 없음" 텍스트. |

---

## 7. 테스트 전략

pytest(파이썬), 하드웨어·torch 없이:
- `test_frame_proto.py`: `send_frame`→`recv_frame` 왕복(소켓쌍 `socket.socketpair()`), 부분수신/EOF.
- `test_perception_server.py`: `draw_overlay` 순수함수(주인/예측/없음 케이스 텍스트·색), 서버 루프 1~2프레임을 `--test-pattern`+`MockDetector`+colour로 돌려 클라이언트가 JPEG 수신 + `register` 적용 확인.
- Qt: 컴파일 성공(빌드). GUI 실행 스모크는 옵션(사용자/DISPLAY 환경).

---

## 8. YAGNI / 범위 밖

- 다중 클라이언트/인증/암호화 — 로컬 데모, 단일 연결.
- UDP/재전송/QoS — TCP localhost로 충분.
- libi_gui 본체(RobotController/ROS2) 통합 — 이번은 **독립 데모**. 통합은 이후 별도.
- Qt 쪽 검출/오버레이 로직 — 전부 Python(B안).

---

## 9. 완료 판정

1. `perception_server.py --test-pattern` 이 합성 프레임을 스트림하고, 파이썬 테스트 클라이언트가 프레임 수신 + `register`/`reset` 명령 왕복 (단위 테스트 green).
2. `send_frame/recv_frame`, `draw_overlay` 단위 테스트 통과.
3. `qt_demo/` 가 **Qt5로 빌드 성공**.
4. (사용자 환경) 실 웹캠 + torch: Qt 창에 영상 + [등록] 후 그 사람에게 박스 유지가 눈으로 보임.
