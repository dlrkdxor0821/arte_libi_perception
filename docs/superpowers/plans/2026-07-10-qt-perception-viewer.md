# Qt Perception Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 웹캠으로 "사진 1장 등록 → 지속 ReID"를 눈으로 확인할 Qt5 QML+C++ 뷰어와, 검출·ReID·박스렌더까지 다 하는 Python 소켓 서버(B안)를 만든다.

**Architecture:** Python 서버가 웹캠+perception+박스렌더를 하고 JPEG를 localhost TCP로 스트림, Qt는 그 이미지를 표시만 + [등록]/[리셋] 버튼으로 명령 전송. Qt엔 CV 코드 없음.

**Tech Stack:** Python(numpy, opencv, 실행시 ultralytics/torch 지연) + pytest; C++/Qt5.15(Core Gui Qml Quick Network) + CMake.

**Design doc:** [Spec 4 — Qt Perception Viewer](../specs/2026-07-10-qt-perception-viewer-design.md).

## Global Constraints

- **커밋 금지** — 에이전트는 커밋하지 않는다. 각 태스크 끝은 체크포인트(사용자 커밋 제안).
- **순수 코어 불변** — 소켓/서버 코드는 `follower_perception/scripts/`, Qt는 `qt_demo/`. 패키지 코어(`follower_perception/follower_perception/`)엔 안 넣는다.
- **지연 임포트** — ultralytics/torch는 실행 경로에서만. 단위 테스트는 colour 백엔드 + MockDetector.
- **Qt5.15** — libi_gui와 동일 버전. Controls 2 사용.
- **테스트 실행** — `follower_perception/` 루트에서 `python3 -m pytest`. Qt 빌드는 `qt_demo/`에서 cmake.

---

### Task 1: 프레임 프로토콜 (`frame_proto.py`)

**Files:**
- Create: `follower_perception/scripts/__init__.py` (scripts를 import 가능한 패키지로)
- Create: `follower_perception/scripts/frame_proto.py`
- Test: `follower_perception/tests/test_frame_proto.py`

**Interfaces:**
- Produces: `send_frame(sock, payload: bytes) -> None` (4바이트 big-endian 길이 + payload); `recv_frame(sock) -> bytes | None` (EOF면 None).

- [ ] **Step 1: 실패 테스트**

`follower_perception/tests/test_frame_proto.py`:
```python
import socket
from scripts.frame_proto import send_frame, recv_frame


def test_round_trip():
    a, b = socket.socketpair()
    send_frame(a, b"hello-jpeg")
    assert recv_frame(b) == b"hello-jpeg"
    a.close(); b.close()


def test_multiple_frames_in_order():
    a, b = socket.socketpair()
    send_frame(a, b"one"); send_frame(a, b"two")
    assert recv_frame(b) == b"one"
    assert recv_frame(b) == b"two"
    a.close(); b.close()


def test_eof_returns_none():
    a, b = socket.socketpair()
    a.close()
    assert recv_frame(b) is None
    b.close()
```

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_frame_proto.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'scripts'` 또는 `frame_proto`

- [ ] **Step 3: 구현**

`follower_perception/scripts/__init__.py`: (빈 파일)
```python
```
`follower_perception/scripts/frame_proto.py`:
```python
"""Length-prefixed frame framing over a stream socket. Shared by the
perception server and its test client."""
import struct


def send_frame(sock, payload):
    sock.sendall(struct.pack(">I", len(payload)) + bytes(payload))


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def recv_frame(sock):
    header = _recv_exact(sock, 4)
    if header is None:
        return None
    (n,) = struct.unpack(">I", header)
    return _recv_exact(sock, n)
```

- [ ] **Step 4: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_frame_proto.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/scripts/__init__.py \
#   follower_perception/scripts/frame_proto.py follower_perception/tests/test_frame_proto.py
# (사용자) git commit -m "feat(perception): length-prefixed frame protocol"
```

---

### Task 2: Perception 소켓 서버 (`perception_server.py`)

**Files:**
- Create: `follower_perception/scripts/perception_server.py`
- Test: `follower_perception/tests/test_perception_server.py`

**Interfaces:**
- Consumes: `frame_proto.send_frame`, `FollowerPerception`, `MockDetector`, `ReIDEngine`.
- Produces: `draw_overlay(frame, det, *, status_extra="") -> np.ndarray`; `serve_loop(conn, frames, perception, *, poll_cmd=None, jpeg_quality=80) -> None` (프레임마다: 명령 폴링→register/reset, run, 오버레이, JPEG 전송); `test_pattern_frames(n=None)` generator; `main()`.

- [ ] **Step 1: 실패 테스트**

`follower_perception/tests/test_perception_server.py`:
```python
import socket
import numpy as np
import cv2
from follower_perception.detection import TrackedBox
from follower_perception.reid_engine import ReIDEngine
from follower_perception.mocks import MockDetector
from follower_perception.pipeline import FollowerPerception
from scripts.frame_proto import recv_frame
from scripts.perception_server import draw_overlay, serve_loop


def _frame(color=(0, 0, 255), w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def _owner_det():
    return FollowerPerception  # placeholder, not used


def test_draw_overlay_preserves_shape_owner_and_none():
    from follower_perception.detection import Detection
    f = _frame()
    det = Detection(cx=32, cy=32, area=4096, bbox=(0, 0, 64, 64), track_id=1,
                    is_owner=True, confidence=0.9, is_predicted=False)
    out_owner = draw_overlay(f, det)
    out_none = draw_overlay(f, None)
    assert out_owner.shape == f.shape and out_owner.dtype == np.uint8
    assert out_none.shape == f.shape
    # owner overlay must differ from the plain-none overlay
    assert not np.array_equal(out_owner, out_none)


def test_serve_loop_streams_and_registers():
    a, b = socket.socketpair()
    frames = [_frame((0, 0, 255)) for _ in range(4)]
    perc = FollowerPerception(detector=MockDetector([[_full_box(1)]] * 4),
                              reid=ReIDEngine(backend="colour"))
    cmds = iter(["register"])

    def poll(conn):
        return next(cmds, None)

    serve_loop(a, frames, perc, poll_cmd=poll)
    a.close()   # EOF for client

    got = []
    while True:
        fr = recv_frame(b)
        if fr is None:
            break
        got.append(fr)
    b.close()

    assert len(got) == 4
    img = cv2.imdecode(np.frombuffer(got[0], np.uint8), cv2.IMREAD_COLOR)
    assert img.shape == (64, 64, 3)
    assert perc.matcher.is_registered is True
```

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_perception_server.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'scripts.perception_server'`

- [ ] **Step 3: 구현**

`follower_perception/scripts/perception_server.py`:
```python
#!/usr/bin/env python3
"""Perception socket server (B안): owns the webcam, runs FollowerPerception,
draws boxes, and streams annotated JPEG frames to a Qt viewer over localhost
TCP. Receives newline commands: register / reset.

Run from the follower_perception/ package root:
    python scripts/perception_server.py --camera 0 --port 5007
    python scripts/perception_server.py --test-pattern           # no webcam/torch
"""
import argparse
import os
import select
import socket
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

from scripts.frame_proto import send_frame


def draw_overlay(frame, det, *, status_extra=""):
    vis = frame.copy()
    if det is not None and det.is_owner:
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        color = (0, 165, 255) if det.is_predicted else (0, 255, 0)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        label = f"owner#{det.track_id} {'PRED' if det.is_predicted else 'OK'}"
        cv2.putText(vis, label, (max(0, x1), max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    else:
        cv2.putText(vis, "no owner", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    if status_extra:
        cv2.putText(vis, status_extra, (10, vis.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return vis


def serve_loop(conn, frames, perception, *, poll_cmd=None, jpeg_quality=80):
    for frame in frames:
        cmd = poll_cmd(conn) if poll_cmd else None
        if cmd == "register":
            perception.register_from_image(frame)
        elif cmd == "reset":
            perception.reset()
        perception.run(frame)
        det = perception.get_latest()
        rs = getattr(perception.matcher, "last_reid_sim", None)
        extra = f"reid={rs:.2f}" if rs is not None else ""
        vis = draw_overlay(frame, det, status_extra=extra)
        ok, buf = cv2.imencode(".jpg", vis,
                               [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        if ok:
            try:
                send_frame(conn, buf.tobytes())
            except (BrokenPipeError, ConnectionResetError, OSError):
                return


def make_socket_poller():
    """Return a poll_cmd(conn) that non-blockingly reads newline commands."""
    state = {"buf": b""}

    def poll(conn):
        r, _, _ = select.select([conn], [], [], 0)
        if not r:
            return None
        try:
            data = conn.recv(4096)
        except OSError:
            return None
        if not data:
            return None
        state["buf"] += data
        cmd = None
        while b"\n" in state["buf"]:
            line, state["buf"] = state["buf"].split(b"\n", 1)
            line = line.strip().decode(errors="ignore")
            if line:
                cmd = line   # last command this tick wins
        return cmd

    return poll


class _AlwaysBox:
    """Test-pattern detector: always reports one full-frame track."""
    def detect(self, frame):
        from follower_perception.detection import TrackedBox
        h, w = frame.shape[:2]
        return [TrackedBox(bbox=(w * 0.25, h * 0.2, w * 0.75, h * 0.9),
                           cx=w / 2, cy=h / 2, area=w * h * 0.3,
                           track_id=1, confidence=0.9)]

    def reset(self):
        pass


def test_pattern_frames(n=None):
    """Synthetic BGR frames: a coloured person-ish block on a moving bg.
    Runs with no webcam and no torch."""
    i = 0
    w, h = 640, 480
    while n is None or i < n:
        img = np.full((h, w, 3), 30, dtype=np.uint8)
        cx = int(w * (0.5 + 0.25 * np.sin(i * 0.05)))
        cv2.rectangle(img, (cx - 60, 120), (cx + 60, 400), (60, 40, 200), -1)
        yield img
        i += 1


def _camera_frames(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"[error] cannot open camera {index}"); raise SystemExit(2)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


def _build_perception(args):
    from follower_perception.reid_engine import ReIDEngine
    from follower_perception.pipeline import FollowerPerception
    if args.test_pattern:
        reid = ReIDEngine(backend="colour")
        p = FollowerPerception(detector=_AlwaysBox(), reid=reid)
    else:
        try:
            from follower_perception.detector import Detector
        except ImportError as e:  # pragma: no cover
            print(f"[error] ultralytics/torch not installed ({e}). Use "
                  f"--test-pattern or run in your model venv."); raise SystemExit(2)
        reid = ReIDEngine(device=args.device)
        p = FollowerPerception(detector=Detector(device=args.device), reid=reid)
    if args.no_hsv:
        p.matcher.hsv_threshold = None
    elif args.hsv_threshold is not None:
        p.matcher.hsv_threshold = float(args.hsv_threshold)
    return p


def main():
    ap = argparse.ArgumentParser(description="Perception socket server (B안)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--camera", type=int, default=0)
    src.add_argument("--test-pattern", dest="test_pattern", action="store_true")
    ap.add_argument("--port", type=int, default=5007)
    ap.add_argument("--device", default=None)
    ap.add_argument("--hsv-threshold", dest="hsv_threshold", type=float, default=None)
    ap.add_argument("--no-hsv", dest="no_hsv", action="store_true")
    args = ap.parse_args()

    frames = test_pattern_frames() if args.test_pattern else _camera_frames(args.camera)
    perception = _build_perception(args)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    print(f"[ok] perception server on 127.0.0.1:{args.port} "
          f"({'test-pattern' if args.test_pattern else f'camera {args.camera}'}); "
          f"waiting for Qt viewer…")
    while True:
        conn, addr = srv.accept()
        print(f"[ok] viewer connected: {addr}")
        try:
            serve_loop(conn, frames, perception, poll_cmd=make_socket_poller())
        finally:
            conn.close()
            print("[..] viewer disconnected; waiting again")
        if not args.test_pattern:
            # real camera generator is exhausted once closed; rebuild for next viewer
            frames = _camera_frames(args.camera)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_perception_server.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: 전체 스위트**

Run: `cd follower_perception && python3 -m pytest -q`
Expected: PASS (기존 50 + 신규 5 = 55)

- [ ] **Step 6: (옵션) test-pattern 서버 스모크**

`python scripts/perception_server.py --test-pattern --port 5007` 실행 후, 파이썬 한 줄 클라이언트로 프레임 1장 수신 확인(웹캠·torch 불필요).

- [ ] **Step 7: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/scripts/perception_server.py \
#   follower_perception/tests/test_perception_server.py
# (사용자) git commit -m "feat(perception): socket server streaming annotated frames (B안)"
```

---

### Task 3: Qt5 QML+C++ 뷰어 (`qt_demo/`)

**Files:**
- Create: `qt_demo/CMakeLists.txt`
- Create: `qt_demo/src/main.cpp`
- Create: `qt_demo/src/FrameImageProvider.h`
- Create: `qt_demo/src/PerceptionClient.h`
- Create: `qt_demo/src/PerceptionClient.cpp`
- Create: `qt_demo/qml/Main.qml`
- Create: `qt_demo/qml.qrc`

**Interfaces:**
- Produces: 실행형 `viewer` — TCP 연결, 길이접두 JPEG 수신→표시, `[등록]/[리셋]` 버튼→`register\n`/`reset\n`.

- [ ] **Step 1: 파일 작성**

`qt_demo/src/FrameImageProvider.h`:
```cpp
#pragma once
#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

class FrameImageProvider : public QQuickImageProvider {
public:
    FrameImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}
    QImage requestImage(const QString &, QSize *size, const QSize &) override {
        QMutexLocker lock(&m_mutex);
        if (size) *size = m_image.size();
        return m_image;
    }
    void setImage(const QImage &img) { QMutexLocker lock(&m_mutex); m_image = img; }
private:
    QImage m_image;
    QMutex m_mutex;
};
```

`qt_demo/src/PerceptionClient.h`:
```cpp
#pragma once
#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include <QString>

class FrameImageProvider;

class PerceptionClient : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(int frameCounter READ frameCounter NOTIFY frameChanged)
public:
    explicit PerceptionClient(FrameImageProvider *provider, QObject *parent = nullptr);
    bool connected() const { return m_connected; }
    int frameCounter() const { return m_counter; }
    Q_INVOKABLE void connectTo(const QString &host, int port);
    Q_INVOKABLE void doRegister();
    Q_INVOKABLE void doReset();
signals:
    void connectedChanged();
    void frameChanged();
private slots:
    void onReadyRead();
    void onConnected();
    void onDisconnected();
private:
    void sendCommand(const QByteArray &cmd);
    QTcpSocket m_sock;
    FrameImageProvider *m_provider;
    QByteArray m_buf;
    bool m_connected = false;
    int m_counter = 0;
    QString m_host;
    int m_port = 5007;
};
```

`qt_demo/src/PerceptionClient.cpp`:
```cpp
#include "PerceptionClient.h"
#include "FrameImageProvider.h"
#include <QImage>
#include <QTimer>

PerceptionClient::PerceptionClient(FrameImageProvider *provider, QObject *parent)
    : QObject(parent), m_provider(provider) {
    connect(&m_sock, &QTcpSocket::readyRead, this, &PerceptionClient::onReadyRead);
    connect(&m_sock, &QTcpSocket::connected, this, &PerceptionClient::onConnected);
    connect(&m_sock, &QTcpSocket::disconnected, this, &PerceptionClient::onDisconnected);
}

void PerceptionClient::connectTo(const QString &host, int port) {
    m_host = host; m_port = port;
    m_sock.abort();
    m_sock.connectToHost(host, quint16(port));
}

void PerceptionClient::onConnected() {
    m_connected = true; emit connectedChanged();
}

void PerceptionClient::onDisconnected() {
    m_connected = false; emit connectedChanged();
    // retry after a short delay
    QTimer::singleShot(1000, this, [this]() { connectTo(m_host, m_port); });
}

void PerceptionClient::onReadyRead() {
    m_buf.append(m_sock.readAll());
    while (m_buf.size() >= 4) {
        const quint32 n = (quint8(m_buf[0]) << 24) | (quint8(m_buf[1]) << 16)
                        | (quint8(m_buf[2]) << 8) | quint8(m_buf[3]);
        if (quint32(m_buf.size()) < 4 + n) break;
        const QByteArray jpeg = m_buf.mid(4, int(n));
        m_buf.remove(0, int(4 + n));
        QImage img;
        if (img.loadFromData(jpeg, "JPEG") && !img.isNull()) {
            m_provider->setImage(img);
            m_counter++;
            emit frameChanged();
        }
    }
}

void PerceptionClient::sendCommand(const QByteArray &cmd) {
    if (m_sock.state() == QAbstractSocket::ConnectedState) {
        m_sock.write(cmd);
        m_sock.flush();
    }
}

void PerceptionClient::doRegister() { sendCommand("register\n"); }
void PerceptionClient::doReset()    { sendCommand("reset\n"); }
```

`qt_demo/src/main.cpp`:
```cpp
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "FrameImageProvider.h"
#include "PerceptionClient.h"

int main(int argc, char **argv) {
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    auto *provider = new FrameImageProvider();
    engine.addImageProvider("perc", provider);

    auto *client = new PerceptionClient(provider, &app);
    engine.rootContext()->setContextProperty("client", client);

    engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    QString host = "127.0.0.1";
    int port = 5007;
    if (argc >= 2) host = QString::fromUtf8(argv[1]);
    if (argc >= 3) port = QString::fromUtf8(argv[2]).toInt();
    client->connectTo(host, port);

    return app.exec();
}
```

`qt_demo/qml/Main.qml`:
```qml
import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 720; height: 600
    title: "Libi Perception Viewer"
    color: "#0f1420"

    Column {
        anchors.centerIn: parent
        spacing: 16

        Rectangle {
            width: 640; height: 480
            color: "#000"; border.color: "#2a3550"; border.width: 1
            Image {
                anchors.fill: parent
                fillMode: Image.PreserveAspectFit
                cache: false
                source: "image://perc/frame?c=" + client.frameCounter
            }
            Text {
                anchors.centerIn: parent
                visible: client.frameCounter === 0
                text: client.connected ? "연결됨 — 프레임 대기" : "서버 연결 대기…"
                color: "#88aacc"; font.pixelSize: 18
            }
        }

        Row {
            spacing: 16
            anchors.horizontalCenter: parent.horizontalCenter
            Button { text: "등록"; onClicked: client.doRegister() }
            Button { text: "리셋"; onClicked: client.doReset() }
            Label {
                anchors.verticalCenter: parent.verticalCenter
                text: client.connected ? "● 연결됨" : "○ 끊김"
                color: client.connected ? "#4caf50" : "#f44336"
                font.pixelSize: 16
            }
        }
    }
}
```

`qt_demo/qml.qrc`:
```xml
<RCC>
  <qresource prefix="/">
    <file>qml/Main.qml</file>
  </qresource>
</RCC>
```

`qt_demo/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
project(libi_perception_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Core Gui Qml Quick Network REQUIRED)

add_executable(viewer
    src/main.cpp
    src/PerceptionClient.cpp
    src/PerceptionClient.h
    src/FrameImageProvider.h
    qml.qrc
)
target_link_libraries(viewer PRIVATE
    Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Network)
```

- [ ] **Step 2: 빌드 확인**

Run:
```bash
cd qt_demo && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
```
Expected: `viewer` 실행파일 생성(빌드 성공). MOC/RCC 자동 처리.

- [ ] **Step 3: (옵션) 실행 스모크**

터미널 1: `cd follower_perception && python3 scripts/perception_server.py --test-pattern`
터미널 2: `cd qt_demo && ./build/viewer 127.0.0.1 5007`
기대: 창에 움직이는 합성 프레임 + 박스, [등록]/[리셋] 동작. (웹캠·torch 불필요; DISPLAY 필요.)

실 웹캠+모델: 서버를 `--camera 0`(torch venv)로 띄우고 동일.

- [ ] **Step 4: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add qt_demo/
# (사용자) git commit -m "feat(qt): QML+C++ perception viewer (display + register/reset)"
```

---

## Self-Review

- **Spec 커버리지:** §3 프로토콜→Task1(frame_proto). §4.1 서버(draw_overlay/serve_loop/test_pattern/main)→Task2. §4.2 frame_proto→Task1. §4.3/4.4 Qt(PerceptionClient/ImageProvider/main/qml/cmake)→Task3. §5 검증→Task1·2 pytest + Task2 Step6 서버 스모크 + Task3 Step2 빌드. §6 에러(카메라/미설치/끊김/등록없음)→Task2(_camera_frames/_build_perception/send_frame try) + Task3(onDisconnected 재연결). §7 테스트→Task1·2. 
- **Placeholder 스캔:** 없음. (테스트의 `_owner_det` 미사용 헬퍼는 구현 시 넣지 말 것 — self-review에서 제거.)
- **타입 일관성:** `send_frame/recv_frame`, `draw_overlay(frame,det,*,status_extra)`, `serve_loop(conn,frames,perception,*,poll_cmd,jpeg_quality)`, Qt `doRegister/doReset/frameCounter/connected` 일관. Python `register_from_image/reset/run/get_latest/matcher.last_reid_sim`은 Spec 3에서 구현됨.
- **정정:** Task2 테스트의 `_owner_det()` 함수는 미사용이므로 실제 작성 시 제외한다.
