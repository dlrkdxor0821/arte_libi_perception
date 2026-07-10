# 실행 방법 (run.md)

**포트:** `6001` 로봇→AI서버(UDP 영상) · `5007` AI서버→Qt뷰어(TCP) · `6000` AI서버→제어(TCP Detection, *연결 예정*)

perception 명령은 `follower_perception/` 폴더에서 실행. 실모델은 torch 있는 venv 사용:
`PY=/home/ane/personal_repo/labeling_sam3/.venv/bin/python`

---

## A. localhost 데모 (한 PC, 로봇 없이) — 지금 되는 것

`follower_perception/`에서 터미널 3개:
```bash
# 1) 영상 송신 (웹캠 대신 test-pattern; 실제 웹캠은 --camera 0)
python3 scripts/camera_sender.py --host 127.0.0.1 --port 6001 --test-pattern

# 2) perception (UDP 수신 + 검출/ReID)
$PY scripts/perception_server.py --udp --udp-port 6001

# 3) Qt 뷰어 (별도 PC 가능, venv 불필요)
../qt_demo/build/viewer 127.0.0.1 5007
```
뷰어에서 **[등록]** → 그 사람 추종 + `cmd_vel(preview)` 값 확인.

### 보는 방법 3가지 (같은 화면)
```bash
# 1) Qt 뷰어 (C++)
../qt_demo/build/viewer 127.0.0.1 5007
# 2) 파이썬 뷰어 (빌드 불필요, 키: r=등록 x=리셋 q=종료)
python3 scripts/viewer.py 127.0.0.1 5007
# 3) 서버가 직접 창 띄우기 (클라이언트·소켓 불필요, 제일 간단)
$PY scripts/perception_server.py --camera 0 --show      # 키: r=등록 x=리셋 q=종료
```
> 1,2번은 서버(`--udp` 또는 `--camera 0`)가 떠 있어야 함. 3번은 서버가 웹캠 직접 잡아 혼자 표시.
> (뷰어는 디스플레이 필요. `--show`/`viewer.py`는 GUI OpenCV 필요 — pip opencv-python엔 포함.)

---

## B. 실제 배포 (Pi 로봇 + AI서버 PC)

### AI서버 (PC/GPU)
```bash
cd follower_perception
$PY scripts/perception_server.py --udp --udp-port 6001
```

### Pi (로봇) — 3개
```bash
# 1) 로봇 bringup (모터·오도메트리·TF·LiDAR)   ※ 실제 pinky_pro 런치로 교체
ros2 launch <pinky_pro_bringup>

# 2) 카메라 영상 송신 → AI서버
python3 scripts/camera_sender.py --host <AI서버_IP> --port 6001 --camera 0 --fps 15

# 3) 주행 제어(PID)  ── perception→control Detection 송신기 연결 후 동작
ros2 run follower_control control_node
```

### 뷰어 (아무 PC)
```bash
qt_demo/build/viewer <AI서버_IP> 5007
```

---

## C. 오프라인 프로필 검증 (파일 기반, 참고)
```bash
cd follower_perception
$PY scripts/register_and_track.py register --image 사람.jpg --profile profiles/v1
$PY scripts/register_and_track.py track    --video 영상.mp4 --profile profiles/v1 --out out.mp4
```

---

## 테스트 (하드웨어·모델 불필요)
```bash
cd follower_perception && python3 -m pytest -q
```
