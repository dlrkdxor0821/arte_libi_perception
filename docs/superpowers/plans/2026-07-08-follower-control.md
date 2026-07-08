# Follower Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the `follower_control` ROS 2 package — consumes owner `Detection` (TCP) + LiDAR `/scan`, runs PID + LiDAR avoidance to follow (with reverse), recovers a lost target with a py_trees 3-phase search, and publishes `/cmd_vel`.

**Architecture:** Pure Python control logic (PID, LiDAR correction, search planner, FSM, tracking controller, search BT, control loop) with zero ROS imports — all unit-tested deterministically. A thin rclpy node (`control_node`) wires transports (`/scan` sub, `/cmd_vel` pub, TCP Detection receiver) into `ControlLoop.tick()` on a 20 Hz timer. map-free (no Nav2/AMCL).

**Tech Stack:** Python 3.10+, ROS 2 (rclpy, sensor_msgs, geometry_msgs), py_trees, transitions, numpy. pytest for pure-logic tests; colcon + manual smoke test for the ROS node.

**Design docs:** [Spec 2 — control](../specs/2026-07-08-follower-control-design.md). Consumes the `Detection` JSON contract from [Spec 1](../specs/2026-07-08-follower-perception-design.md).

## Global Constraints

- **Pure control core** — no ROS imports in `pid.py`, `lidar_avoidance.py`, `search_planner.py`, `state_machine.py`, `tracking_controller.py`, `bt_searching.py`, `control_loop.py`, `detection.py`, `detection_receiver.py`. ROS only in `scan_provider.py`, `cmd_publisher.py`, `control_node.py`.
- **map-free** — no Nav2, AMCL, map, or TF. Relative PID + LiDAR only. PATROL/순찰 is out of scope (external).
- **Reverse is bounded** — `linear_x` clamps to `[-LINEAR_X_REVERSE_MAX, +LINEAR_X_MAX]`; reverse max is smaller than forward (LiDAR is blind to the rear).
- **Parameters are reference-only** — all values in `config.py`, tuning starting points.
- **Detection contract** — control receives Detection as JSON dicts with keys: `cx, cy, area, bbox, track_id, is_owner, confidence, is_predicted` (from Spec 1's `detection_to_dict`).
- **Determinism in tests** — inject `now()` clocks and fake transports; never rely on wall clock or real sockets in unit tests. Pure tests run with `cd follower_control && python -m pytest`.

---

### Task 1: Package scaffolding + config + Detection

**Files:**
- Create: `follower_control/package.xml`
- Create: `follower_control/setup.py`
- Create: `follower_control/setup.cfg`
- Create: `follower_control/pytest.ini`
- Create: `follower_control/resource/follower_control`
- Create: `follower_control/follower_control/__init__.py`
- Create: `follower_control/follower_control/config.py`
- Create: `follower_control/follower_control/detection.py`
- Test: `follower_control/tests/__init__.py`, `follower_control/tests/test_detection.py`

**Interfaces:**
- Produces: `config` module (reference constants); `Detection` dataclass (same fields as Spec 1); `detection_from_dict(d) -> Detection | None`.

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_detection.py`:
```python
from follower_control.detection import Detection, detection_from_dict


def _dict(**over):
    base = dict(cx=320.0, cy=240.0, area=400.0, bbox=[0, 0, 20, 20],
                track_id=1, is_owner=True, confidence=0.9, is_predicted=False)
    base.update(over)
    return base


def test_from_dict_none():
    assert detection_from_dict(None) is None


def test_from_dict_maps_fields():
    d = detection_from_dict(_dict())
    assert isinstance(d, Detection)
    assert d.cx == 320.0
    assert d.area == 400.0
    assert d.bbox == (0, 0, 20, 20)
    assert d.is_owner is True
    assert d.is_predicted is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_detection.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.detection'`

- [ ] **Step 3: Write the scaffolding and implementation**

`follower_control/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>follower_control</name>
  <version>0.1.0</version>
  <description>Vision-based person follow control (PID + LiDAR + recovery BT).</description>
  <maintainer email="dlrkdxor0821@gmail.com">leekt</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>python3-py-trees</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`follower_control/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'follower_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'transitions', 'numpy'],
    zip_safe=True,
    maintainer='leekt',
    maintainer_email='dlrkdxor0821@gmail.com',
    description='Vision-based person follow control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = follower_control.control_node:main',
        ],
    },
)
```

`follower_control/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/follower_control
[install]
install_scripts=$base/lib/follower_control
```

`follower_control/pytest.ini`:
```ini
[pytest]
testpaths = tests
```

`follower_control/resource/follower_control`:
```
```

`follower_control/follower_control/__init__.py`:
```python
```

`follower_control/follower_control/config.py`:
```python
# All values are reference starting points for tuning — not hard requirements.

# ---- distance PID (linear_x) ----
TARGET_SIZE = 360.0            # sqrt(area) setpoint
KP_DIST = 0.0030
KI_DIST = 0.0001
KD_DIST = 0.0
INTEGRAL_DIST_CLAMP = 50.0
LINEAR_X_MAX = 0.12            # forward max (m/s)
LINEAR_X_REVERSE_MAX = 0.06   # reverse max (m/s) — smaller: LiDAR blind rear

# ---- bearing PID (angular_z) ----
IMAGE_WIDTH = 640
KP_ANGLE = 0.0010
KI_ANGLE = 0.0
KD_ANGLE = 0.0
INTEGRAL_ANGLE_CLAMP = 200.0
ANGLE_DEADZONE = 45.0         # px
ANGULAR_Z_MAX = 0.60          # rad/s
ANGULAR_SMOOTHING = 0.3       # low-pass: 0=frozen, 1=no smoothing

# ---- LiDAR avoidance ----
MIN_DIST = 0.20               # front-arc slowdown threshold (m)
AVOID_DIST = 0.40             # side-arc shy-away threshold (m)
AVOID_KP = 0.50
FRONT_ARC_DEG = 15            # +/- degrees around 0 (front)
SIDE_ARC = (20, 71)          # degrees range for a side arc (start, stop-exclusive)

# ---- miss / search ----
N_MISS_FRAMES = 40            # consecutive None before TRACKING -> SEARCHING
SEARCH_HOLD_SEC = 10.0        # phase 1: hold/wait before scanning
SEARCH_SCAN_SEC = 4.0         # duration of a +/-30 deg scan sweep
ANGULAR_Z_SEARCH = 0.35       # rad/s during search rotation
SEARCH_TURN_ANGLE = 3.14159   # phase 2: ~180 deg turn (radians)

# ---- loop ----
TICK_HZ = 20.0
FRAME_DT = 0.05               # nominal seconds per tick

# ---- transport ----
DETECTION_TCP_HOST = '0.0.0.0'
DETECTION_TCP_PORT = 6000
SCAN_TOPIC = '/scan'
CMD_VEL_TOPIC = '/cmd_vel'
```

`follower_control/follower_control/detection.py`:
```python
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Detection:
    cx: float
    cy: float
    area: float
    bbox: Tuple[float, float, float, float]
    track_id: int
    is_owner: bool
    confidence: float
    is_predicted: bool


def detection_from_dict(d):
    if d is None:
        return None
    return Detection(
        cx=d['cx'], cy=d['cy'], area=d['area'], bbox=tuple(d['bbox']),
        track_id=d['track_id'], is_owner=d['is_owner'],
        confidence=d['confidence'], is_predicted=d['is_predicted'],
    )
```

`follower_control/tests/__init__.py`:
```python
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_detection.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/package.xml follower_control/setup.py follower_control/setup.cfg \
        follower_control/pytest.ini follower_control/resource follower_control/follower_control/ \
        follower_control/tests/
git commit -m "feat(control): ROS2 package scaffolding + config + Detection"
```

---

### Task 2: Follow PID (distance/bearing, with reverse)

**Files:**
- Create: `follower_control/follower_control/pid.py`
- Test: `follower_control/tests/test_pid.py`

**Interfaces:**
- Consumes: a `cfg` object with the distance/bearing constants from `config`.
- Produces: `FollowPID(cfg)`, `.compute(cx, area, dt) -> (linear_x, angular_z)`, `.reset()`. `clamp(v, lo, hi)` helper (module-level).

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_pid.py`:
```python
from types import SimpleNamespace
from follower_control.pid import FollowPID, clamp


def _cfg(**over):
    base = dict(TARGET_SIZE=360.0, KP_DIST=0.0030, KI_DIST=0.0, KD_DIST=0.0,
                INTEGRAL_DIST_CLAMP=50.0, LINEAR_X_MAX=0.12, LINEAR_X_REVERSE_MAX=0.06,
                IMAGE_WIDTH=640, KP_ANGLE=0.0010, KI_ANGLE=0.0, KD_ANGLE=0.0,
                INTEGRAL_ANGLE_CLAMP=200.0, ANGLE_DEADZONE=45.0, ANGULAR_Z_MAX=0.60,
                ANGULAR_SMOOTHING=1.0)   # smoothing=1 -> deterministic single-step
    base.update(over)
    return SimpleNamespace(**base)


def test_clamp():
    assert clamp(5, 0, 1) == 1
    assert clamp(-5, 0, 1) == 0
    assert clamp(0.5, 0, 1) == 0.5


def test_far_target_drives_forward():
    pid = FollowPID(_cfg())
    lin, _ = pid.compute(cx=320.0, area=100.0, dt=0.05)   # sqrt=10 << 360
    assert lin > 0


def test_too_close_target_reverses_and_is_bounded():
    pid = FollowPID(_cfg())
    lin, _ = pid.compute(cx=320.0, area=1_000_000.0, dt=0.05)  # sqrt=1000 >> 360
    assert lin < 0
    assert lin >= -0.06        # bounded by LINEAR_X_REVERSE_MAX


def test_target_left_of_center_turns_left():
    pid = FollowPID(_cfg())
    _, ang = pid.compute(cx=0.0, area=100.0, dt=0.05)  # cx < width/2 -> err>0 -> +ang
    assert ang > 0


def test_deadzone_zeroes_small_bearing_error():
    pid = FollowPID(_cfg())
    _, ang = pid.compute(cx=320.0 - 10.0, area=100.0, dt=0.05)  # |err|=10 < 45
    assert ang == 0.0


def test_angular_clamped():
    pid = FollowPID(_cfg(KP_ANGLE=1.0))
    _, ang = pid.compute(cx=0.0, area=100.0, dt=0.05)
    assert ang <= 0.60
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_pid.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.pid'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/pid.py`:
```python
import math


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowPID:
    """Distance PID (sqrt-area) -> linear_x (with reverse); bearing PID (cx) -> angular_z."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self._i_size = 0.0
        self._prev_size = 0.0
        self._i_cx = 0.0
        self._prev_cx = 0.0
        self._ang = 0.0

    def compute(self, cx, area, dt):
        cfg = self.cfg
        dt = dt if dt > 0 else 1e-3

        # distance -> linear_x
        size = math.sqrt(max(0.0, area))
        e = cfg.TARGET_SIZE - size
        self._i_size = clamp(self._i_size + e * dt,
                             -cfg.INTEGRAL_DIST_CLAMP, cfg.INTEGRAL_DIST_CLAMP)
        d = (e - self._prev_size) / dt
        self._prev_size = e
        lin = cfg.KP_DIST * e + cfg.KI_DIST * self._i_size + cfg.KD_DIST * d
        lin = clamp(lin, -cfg.LINEAR_X_REVERSE_MAX, cfg.LINEAR_X_MAX)

        # bearing -> angular_z
        e_cx = (cfg.IMAGE_WIDTH / 2.0) - cx
        if abs(e_cx) < cfg.ANGLE_DEADZONE:
            e_cx = 0.0
        self._i_cx = clamp(self._i_cx + e_cx * dt,
                           -cfg.INTEGRAL_ANGLE_CLAMP, cfg.INTEGRAL_ANGLE_CLAMP)
        d_cx = (e_cx - self._prev_cx) / dt
        self._prev_cx = e_cx
        target_ang = (cfg.KP_ANGLE * e_cx + cfg.KI_ANGLE * self._i_cx
                      + cfg.KD_ANGLE * d_cx)
        s = cfg.ANGULAR_SMOOTHING
        self._ang = s * target_ang + (1.0 - s) * self._ang
        ang = clamp(self._ang, -cfg.ANGULAR_Z_MAX, cfg.ANGULAR_Z_MAX)

        return lin, ang
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_pid.py -v`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/pid.py follower_control/tests/test_pid.py
git commit -m "feat(control): distance/bearing PID with bounded reverse"
```

---

### Task 3: LiDAR avoidance

**Files:**
- Create: `follower_control/follower_control/lidar_avoidance.py`
- Test: `follower_control/tests/test_lidar_avoidance.py`

**Interfaces:**
- Consumes: `pid.clamp`; a `cfg` with `MIN_DIST, AVOID_DIST, AVOID_KP, FRONT_ARC_DEG, SIDE_ARC, ANGULAR_Z_MAX`.
- Produces: `apply_avoidance(linear_x, angular_z, scan, cfg) -> (linear_x, angular_z)`. `scan` is a list of ranges (any length; index 0 = front, degrees increase CCW).

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_lidar_avoidance.py`:
```python
from types import SimpleNamespace
from follower_control.lidar_avoidance import apply_avoidance


def _cfg(**over):
    base = dict(MIN_DIST=0.20, AVOID_DIST=0.40, AVOID_KP=0.50,
                FRONT_ARC_DEG=15, SIDE_ARC=(20, 71), ANGULAR_Z_MAX=0.60)
    base.update(over)
    return SimpleNamespace(**base)


def _clear_scan():
    return [10.0] * 360


def test_no_scan_unchanged():
    assert apply_avoidance(0.1, 0.2, [], _cfg()) == (0.1, 0.2)


def test_clear_path_unchanged():
    lin, ang = apply_avoidance(0.1, 0.2, _clear_scan(), _cfg())
    assert abs(lin - 0.1) < 1e-9
    assert abs(ang - 0.2) < 1e-9


def test_front_obstacle_slows_down():
    scan = _clear_scan()
    scan[0] = 0.10               # 0.10 < MIN_DIST 0.20 -> factor 0.5
    lin, _ = apply_avoidance(0.10, 0.0, scan, _cfg())
    assert abs(lin - 0.05) < 1e-6


def test_left_obstacle_steers_right():
    scan = _clear_scan()
    scan[45] = 0.20              # inside SIDE_ARC (20..71), < AVOID_DIST
    _, ang = apply_avoidance(0.10, 0.0, scan, _cfg())
    assert ang < 0              # steer right (negative) away from left wall


def test_right_obstacle_steers_left():
    scan = _clear_scan()
    scan[315] = 0.20            # mirror side (360-45), < AVOID_DIST
    _, ang = apply_avoidance(0.10, 0.0, scan, _cfg())
    assert ang > 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_lidar_avoidance.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.lidar_avoidance'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/lidar_avoidance.py`:
```python
from .pid import clamp


def apply_avoidance(linear_x, angular_z, scan, cfg):
    """Post-process PID output with LiDAR: front slowdown + side shy-away."""
    if not scan:
        return linear_x, angular_z
    n = len(scan)
    step = n / 360.0

    def arc_min(deg_iter):
        idx = [int(i * step) % n for i in deg_iter]
        vals = [scan[i] for i in idx if scan[i] and scan[i] > 0.05]
        return min(vals) if vals else 10.0

    # front arc: proportional slowdown
    front = arc_min(range(-cfg.FRONT_ARC_DEG, cfg.FRONT_ARC_DEG + 1))
    if front < cfg.MIN_DIST:
        linear_x *= max(0.0, front / cfg.MIN_DIST)

    # side arcs: shy away
    lo, hi = cfg.SIDE_ARC
    left = arc_min(range(lo, hi))
    right = arc_min(range(360 - hi + 1, 360 - lo + 1))
    steer = 0.0
    if left < cfg.AVOID_DIST:
        steer -= (cfg.AVOID_DIST - left) * cfg.AVOID_KP    # wall on left -> steer right
    if right < cfg.AVOID_DIST:
        steer += (cfg.AVOID_DIST - right) * cfg.AVOID_KP   # wall on right -> steer left
    angular_z = clamp(angular_z + steer, -cfg.ANGULAR_Z_MAX, cfg.ANGULAR_Z_MAX)

    return linear_x, angular_z
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_lidar_avoidance.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/lidar_avoidance.py follower_control/tests/test_lidar_avoidance.py
git commit -m "feat(control): LiDAR front-slowdown + side shy-away"
```

---

### Task 4: Search planner (3-phase recovery timing)

**Files:**
- Create: `follower_control/follower_control/search_planner.py`
- Test: `follower_control/tests/test_search_planner.py`

**Interfaces:**
- Consumes: a `cfg` with `SEARCH_HOLD_SEC, SEARCH_SCAN_SEC, ANGULAR_Z_SEARCH, SEARCH_TURN_ANGLE`.
- Produces: `search_command(elapsed, cfg, lkd=1.0) -> (angular_z, done)`. Encodes: hold → scan(±) → 180° turn → scan → done.

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_search_planner.py`:
```python
from types import SimpleNamespace
from follower_control.search_planner import search_command


def _cfg(**over):
    base = dict(SEARCH_HOLD_SEC=10.0, SEARCH_SCAN_SEC=4.0,
                ANGULAR_Z_SEARCH=0.35, SEARCH_TURN_ANGLE=3.14159)
    base.update(over)
    return SimpleNamespace(**base)


def test_phase1_hold_is_stationary():
    ang, done = search_command(5.0, _cfg())
    assert ang == 0.0
    assert done is False


def test_phase1_scan_rotates():
    ang, done = search_command(12.0, _cfg(), lkd=1.0)   # 10..14 scan
    assert ang != 0.0
    assert done is False


def test_turn_phase_rotates():
    ang, done = search_command(16.0, _cfg())            # 14..(14+~8.98) turn
    assert ang != 0.0
    assert done is False


def test_exhausted_is_done():
    ang, done = search_command(1000.0, _cfg())
    assert done is True
    assert ang == 0.0


def test_direction_follows_lkd():
    a_pos, _ = search_command(12.0, _cfg(), lkd=1.0)
    a_neg, _ = search_command(12.0, _cfg(), lkd=-1.0)
    assert a_pos == -a_neg
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_search_planner.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.search_planner'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/search_planner.py`:
```python
def search_command(elapsed, cfg, lkd=1.0):
    """3-phase open-loop recovery given elapsed seconds since search start.

    Timeline:
      [0, HOLD)                      -> hold (0)
      [HOLD, HOLD+SCAN)              -> scan sweep (ANGULAR_Z_SEARCH * lkd)
      [.., + TURN)                   -> ~180 deg turn (ANGULAR_Z_SEARCH)
      [.., + SCAN)                   -> scan sweep (ANGULAR_Z_SEARCH * -lkd)
      after                          -> done (0)
    """
    hold = cfg.SEARCH_HOLD_SEC
    scan = cfg.SEARCH_SCAN_SEC
    turn = cfg.SEARCH_TURN_ANGLE / cfg.ANGULAR_Z_SEARCH
    t_scan1_end = hold + scan
    t_turn_end = t_scan1_end + turn
    t_scan2_end = t_turn_end + scan

    if elapsed < hold:
        return 0.0, False
    if elapsed < t_scan1_end:
        return cfg.ANGULAR_Z_SEARCH * lkd, False
    if elapsed < t_turn_end:
        return cfg.ANGULAR_Z_SEARCH, False
    if elapsed < t_scan2_end:
        return cfg.ANGULAR_Z_SEARCH * -lkd, False
    return 0.0, True
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_search_planner.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/search_planner.py follower_control/tests/test_search_planner.py
git commit -m "feat(control): 3-phase recovery search planner"
```

---

### Task 5: State machine (transitions FSM)

**Files:**
- Create: `follower_control/follower_control/state_machine.py`
- Test: `follower_control/tests/test_state_machine.py`

**Interfaces:**
- Produces: `ControlFSM()` with `.state`, triggers `.lost()`, `.reacquired()`, `.search_failed()`, `.restart()`. States: `TRACKING` (initial), `SEARCHING`, `ENDED`.

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_state_machine.py`:
```python
import pytest
from follower_control.state_machine import ControlFSM


def test_initial_state_tracking():
    assert ControlFSM().state == 'TRACKING'


def test_lost_then_reacquired():
    fsm = ControlFSM()
    fsm.lost()
    assert fsm.state == 'SEARCHING'
    fsm.reacquired()
    assert fsm.state == 'TRACKING'


def test_search_failed_ends():
    fsm = ControlFSM()
    fsm.lost()
    fsm.search_failed()
    assert fsm.state == 'ENDED'


def test_restart_from_ended():
    fsm = ControlFSM()
    fsm.lost()
    fsm.search_failed()
    fsm.restart()
    assert fsm.state == 'TRACKING'


def test_invalid_transition_raises():
    from transitions import MachineError
    fsm = ControlFSM()
    with pytest.raises(MachineError):
        fsm.reacquired()          # not valid from TRACKING
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_state_machine.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.state_machine'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/state_machine.py`:
```python
from transitions import Machine


class ControlFSM:
    """Minimal follow-control FSM: TRACKING <-> SEARCHING -> ENDED."""

    states = ['TRACKING', 'SEARCHING', 'ENDED']

    def __init__(self):
        self.machine = Machine(model=self, states=self.states,
                               initial='TRACKING', auto_transitions=False,
                               ignore_invalid_triggers=False)
        self.machine.add_transition('lost', 'TRACKING', 'SEARCHING')
        self.machine.add_transition('reacquired', 'SEARCHING', 'TRACKING')
        self.machine.add_transition('search_failed', 'SEARCHING', 'ENDED')
        self.machine.add_transition('restart', 'ENDED', 'TRACKING')
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_state_machine.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/state_machine.py follower_control/tests/test_state_machine.py
git commit -m "feat(control): TRACKING/SEARCHING/ENDED state machine"
```

---

### Task 6: Tracking controller

**Files:**
- Create: `follower_control/follower_control/tracking_controller.py`
- Test: `follower_control/tests/test_tracking_controller.py`

**Interfaces:**
- Consumes: `FollowPID`, `apply_avoidance`, `detection.Detection`.
- Produces: `TrackingController(publish, cfg)`, `.step(detection, scan, dt) -> None` (publishes cmd_vel, records `last_direction`), `.last_direction` (float ±1 or None), `.reset()`. `publish` is a callable `publish(linear_x, angular_z)`.

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_tracking_controller.py`:
```python
from types import SimpleNamespace
from follower_control.detection import Detection
from follower_control.tracking_controller import TrackingController


def _cfg(**over):
    base = dict(TARGET_SIZE=360.0, KP_DIST=0.0030, KI_DIST=0.0, KD_DIST=0.0,
                INTEGRAL_DIST_CLAMP=50.0, LINEAR_X_MAX=0.12, LINEAR_X_REVERSE_MAX=0.06,
                IMAGE_WIDTH=640, KP_ANGLE=0.0010, KI_ANGLE=0.0, KD_ANGLE=0.0,
                INTEGRAL_ANGLE_CLAMP=200.0, ANGLE_DEADZONE=45.0, ANGULAR_Z_MAX=0.60,
                ANGULAR_SMOOTHING=1.0, MIN_DIST=0.20, AVOID_DIST=0.40, AVOID_KP=0.50,
                FRONT_ARC_DEG=15, SIDE_ARC=(20, 71))
    base.update(over)
    return SimpleNamespace(**base)


def _det(cx=320.0, area=100.0):
    return Detection(cx=cx, cy=240.0, area=area, bbox=(0, 0, 10, 10),
                     track_id=1, is_owner=True, confidence=0.9, is_predicted=False)


class _Pub:
    def __init__(self):
        self.calls = []

    def __call__(self, lin, ang):
        self.calls.append((lin, ang))


def test_step_publishes_forward_for_far_target():
    pub = _Pub()
    ctrl = TrackingController(pub, _cfg())
    ctrl.step(_det(area=100.0), scan=[], dt=0.05)
    assert pub.calls[-1][0] > 0            # forward


def test_step_records_turn_direction():
    pub = _Pub()
    ctrl = TrackingController(pub, _cfg())
    ctrl.step(_det(cx=0.0), scan=[], dt=0.05)   # target far left -> +ang
    assert ctrl.last_direction == 1.0


def test_front_obstacle_slows_publish():
    pub = _Pub()
    ctrl = TrackingController(pub, _cfg())
    scan = [10.0] * 360
    scan[0] = 0.10
    ctrl.step(_det(area=100.0), scan=scan, dt=0.05)
    lin_blocked = pub.calls[-1][0]
    pub2 = _Pub()
    TrackingController(pub2, _cfg()).step(_det(area=100.0), scan=[], dt=0.05)
    assert lin_blocked < pub2.calls[-1][0]     # slowed vs unobstructed
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_tracking_controller.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.tracking_controller'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/tracking_controller.py`:
```python
from .pid import FollowPID
from .lidar_avoidance import apply_avoidance


class TrackingController:
    """PID + LiDAR avoidance -> publish cmd_vel. Records last turn direction (LKD)."""

    def __init__(self, publish, cfg):
        self.publish = publish
        self.cfg = cfg
        self.pid = FollowPID(cfg)
        self.last_direction = None

    def step(self, detection, scan, dt):
        lin, ang = self.pid.compute(detection.cx, detection.area, dt)
        lin, ang = apply_avoidance(lin, ang, scan, self.cfg)
        if abs(ang) > 0.01:
            self.last_direction = 1.0 if ang > 0 else -1.0
        self.publish(lin, ang)

    def reset(self):
        self.pid.reset()
        self.last_direction = None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_tracking_controller.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/tracking_controller.py follower_control/tests/test_tracking_controller.py
git commit -m "feat(control): tracking controller (PID + LiDAR + LKD)"
```

---

### Task 7: Searching behavior tree (py_trees)

**Files:**
- Create: `follower_control/follower_control/bt_searching.py`
- Test: `follower_control/tests/test_bt_searching.py`

**Interfaces:**
- Consumes: `search_planner.search_command`, `py_trees`.
- Produces: `SearchContext(get_detection, publish, cfg, now, lkd=1.0)`; `create_searching_tree(ctx) -> py_trees.behaviour.Behaviour`; `tick_tree(root) -> py_trees.common.Status` helper. Tree returns SUCCESS on re-detect, FAILURE when the search is exhausted, RUNNING while searching.

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_bt_searching.py`:
```python
from types import SimpleNamespace
import py_trees
from follower_control.bt_searching import (
    SearchContext, create_searching_tree, tick_tree,
)


def _cfg(**over):
    base = dict(SEARCH_HOLD_SEC=10.0, SEARCH_SCAN_SEC=4.0,
                ANGULAR_Z_SEARCH=0.35, SEARCH_TURN_ANGLE=3.14159)
    base.update(over)
    return SimpleNamespace(**base)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class _Pub:
    def __init__(self):
        self.calls = []

    def __call__(self, lin, ang):
        self.calls.append((lin, ang))


def test_reacquire_returns_success():
    pub = _Pub()
    ctx = SearchContext(get_detection=lambda: object(), publish=pub,
                        cfg=_cfg(), now=_Clock())
    root = create_searching_tree(ctx)
    assert tick_tree(root) == py_trees.common.Status.SUCCESS


def test_scanning_publishes_rotation_and_runs():
    clock = _Clock()
    pub = _Pub()
    ctx = SearchContext(get_detection=lambda: None, publish=pub,
                        cfg=_cfg(), now=clock)
    root = create_searching_tree(ctx)
    clock.t = 0.0
    tick_tree(root)               # establishes start time
    clock.t = 12.0                # into scan phase
    status = tick_tree(root)
    assert status == py_trees.common.Status.RUNNING
    assert pub.calls[-1][1] != 0.0   # rotating


def test_exhausted_returns_failure():
    clock = _Clock()
    pub = _Pub()
    ctx = SearchContext(get_detection=lambda: None, publish=pub,
                        cfg=_cfg(), now=clock)
    root = create_searching_tree(ctx)
    clock.t = 0.0
    tick_tree(root)
    clock.t = 10_000.0
    assert tick_tree(root) == py_trees.common.Status.FAILURE
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_bt_searching.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.bt_searching'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/bt_searching.py`:
```python
import py_trees

from .search_planner import search_command


class SearchContext:
    """Injected dependencies for the searching tree (no ROS)."""

    def __init__(self, get_detection, publish, cfg, now, lkd=1.0):
        self.get_detection = get_detection
        self.publish = publish
        self.cfg = cfg
        self.now = now
        self.lkd = lkd
        self.start = None


class CheckReacquired(py_trees.behaviour.Behaviour):
    def __init__(self, ctx):
        super().__init__(name='CheckReacquired')
        self.ctx = ctx

    def update(self):
        if self.ctx.get_detection() is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class SearchMotion(py_trees.behaviour.Behaviour):
    def __init__(self, ctx):
        super().__init__(name='SearchMotion')
        self.ctx = ctx

    def initialise(self):
        if self.ctx.start is None:
            self.ctx.start = self.ctx.now()

    def update(self):
        elapsed = self.ctx.now() - self.ctx.start
        ang, done = search_command(elapsed, self.ctx.cfg, self.ctx.lkd)
        if done:
            self.ctx.publish(0.0, 0.0)
            return py_trees.common.Status.FAILURE
        self.ctx.publish(0.0, ang)
        return py_trees.common.Status.RUNNING


def create_searching_tree(ctx):
    root = py_trees.composites.Selector(name='BT_Searching', memory=False)
    root.add_children([CheckReacquired(ctx), SearchMotion(ctx)])
    return root


def tick_tree(root):
    root.tick_once()
    return root.status
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_bt_searching.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/bt_searching.py follower_control/tests/test_bt_searching.py
git commit -m "feat(control): py_trees 3-phase searching tree"
```

---

### Task 8: Control loop (FSM orchestration)

**Files:**
- Create: `follower_control/follower_control/control_loop.py`
- Test: `follower_control/tests/test_control_loop.py`

**Interfaces:**
- Consumes: `ControlFSM`, `TrackingController`, `SearchContext`/`create_searching_tree`/`tick_tree`, `detection.Detection`.
- Produces: `ControlLoop(get_detection, get_scan, publish, cfg, now)`, `.tick() -> None`, `.state` (property → FSM state). `get_detection() -> Detection | None`, `get_scan() -> list`, `publish(lin, ang)`.

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_control_loop.py`:
```python
from types import SimpleNamespace
from follower_control.detection import Detection
from follower_control.control_loop import ControlLoop


def _cfg(**over):
    base = dict(TARGET_SIZE=360.0, KP_DIST=0.0030, KI_DIST=0.0, KD_DIST=0.0,
                INTEGRAL_DIST_CLAMP=50.0, LINEAR_X_MAX=0.12, LINEAR_X_REVERSE_MAX=0.06,
                IMAGE_WIDTH=640, KP_ANGLE=0.0010, KI_ANGLE=0.0, KD_ANGLE=0.0,
                INTEGRAL_ANGLE_CLAMP=200.0, ANGLE_DEADZONE=45.0, ANGULAR_Z_MAX=0.60,
                ANGULAR_SMOOTHING=1.0, MIN_DIST=0.20, AVOID_DIST=0.40, AVOID_KP=0.50,
                FRONT_ARC_DEG=15, SIDE_ARC=(20, 71), N_MISS_FRAMES=3,
                SEARCH_HOLD_SEC=10.0, SEARCH_SCAN_SEC=4.0, ANGULAR_Z_SEARCH=0.35,
                SEARCH_TURN_ANGLE=3.14159, FRAME_DT=0.05)
    base.update(over)
    return SimpleNamespace(**base)


def _det():
    return Detection(cx=320.0, cy=240.0, area=100.0, bbox=(0, 0, 10, 10),
                     track_id=1, is_owner=True, confidence=0.9, is_predicted=False)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class _Pub:
    def __init__(self):
        self.calls = []

    def __call__(self, lin, ang):
        self.calls.append((lin, ang))


def test_tracks_when_detection_present():
    pub = _Pub()
    loop = ControlLoop(get_detection=lambda: _det(), get_scan=lambda: [],
                       publish=pub, cfg=_cfg(), now=_Clock())
    loop.tick()
    assert loop.state == 'TRACKING'
    assert pub.calls[-1][0] > 0


def test_transitions_to_searching_after_misses():
    det_box = {'v': _det()}
    pub = _Pub()
    loop = ControlLoop(get_detection=lambda: det_box['v'], get_scan=lambda: [],
                       publish=pub, cfg=_cfg(N_MISS_FRAMES=3), now=_Clock())
    loop.tick()                       # tracking
    det_box['v'] = None
    for _ in range(3):
        loop.tick()
    assert loop.state == 'SEARCHING'


def test_searching_reacquire_returns_to_tracking():
    det_box = {'v': _det()}
    pub = _Pub()
    loop = ControlLoop(get_detection=lambda: det_box['v'], get_scan=lambda: [],
                       publish=pub, cfg=_cfg(N_MISS_FRAMES=1), now=_Clock())
    loop.tick()
    det_box['v'] = None
    loop.tick()                       # -> SEARCHING
    assert loop.state == 'SEARCHING'
    det_box['v'] = _det()
    loop.tick()                       # reacquired
    assert loop.state == 'TRACKING'


def test_search_timeout_ends():
    clock = _Clock()
    pub = _Pub()
    loop = ControlLoop(get_detection=lambda: None, get_scan=lambda: [],
                       publish=pub, cfg=_cfg(N_MISS_FRAMES=1), now=clock)
    # need one detection first so tracking starts; use a togglable source
    calls = {'n': 0}

    def src():
        calls['n'] += 1
        return _det() if calls['n'] == 1 else None

    loop2 = ControlLoop(get_detection=src, get_scan=lambda: [], publish=pub,
                        cfg=_cfg(N_MISS_FRAMES=1), now=clock)
    clock.t = 0.0
    loop2.tick()          # tracking
    loop2.tick()          # miss -> SEARCHING, search start=0
    clock.t = 10_000.0
    loop2.tick()          # search exhausted -> ENDED
    assert loop2.state == 'ENDED'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_control_loop.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.control_loop'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/control_loop.py`:
```python
import time

import py_trees

from .state_machine import ControlFSM
from .tracking_controller import TrackingController
from .bt_searching import SearchContext, create_searching_tree, tick_tree


class ControlLoop:
    """Orchestrates TRACKING (PID) and SEARCHING (BT) via the FSM. No ROS."""

    def __init__(self, get_detection, get_scan, publish, cfg, now=time.monotonic):
        self.get_detection = get_detection
        self.get_scan = get_scan
        self.publish = publish
        self.cfg = cfg
        self.now = now
        self.fsm = ControlFSM()
        self.tracker = TrackingController(publish, cfg)
        self.miss = 0
        self._search_ctx = None
        self._search_tree = None

    @property
    def state(self):
        return self.fsm.state

    def _start_search(self):
        lkd = self.tracker.last_direction or 1.0
        self._search_ctx = SearchContext(self.get_detection, self.publish,
                                         self.cfg, self.now, lkd=lkd)
        self._search_tree = create_searching_tree(self._search_ctx)

    def tick(self):
        if self.fsm.state == 'TRACKING':
            det = self.get_detection()
            if det is not None:
                self.miss = 0
                self.tracker.step(det, self.get_scan(), self.cfg.FRAME_DT)
            else:
                self.miss += 1
                self.publish(0.0, 0.0)
                if self.miss >= self.cfg.N_MISS_FRAMES:
                    self.fsm.lost()
                    self._start_search()
        elif self.fsm.state == 'SEARCHING':
            status = tick_tree(self._search_tree)
            if status == py_trees.common.Status.SUCCESS:
                self.fsm.reacquired()
                self.miss = 0
                self.tracker.reset()
            elif status == py_trees.common.Status.FAILURE:
                self.publish(0.0, 0.0)
                self.fsm.search_failed()
        # ENDED: idle (external patrol takes over)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_control_loop.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/control_loop.py follower_control/tests/test_control_loop.py
git commit -m "feat(control): control loop orchestrating tracking + searching"
```

---

### Task 9: Detection receiver (injectable TCP source)

**Files:**
- Create: `follower_control/follower_control/detection_receiver.py`
- Test: `follower_control/tests/test_detection_receiver.py`

**Interfaces:**
- Consumes: `detection.detection_from_dict`.
- Produces: `DetectionReceiver(source)`, `.update() -> None`, `.latest() -> Detection | None`. `source.poll() -> list[dict]` (each dict is a Detection JSON, or `None` sentinel for "no owner").

- [ ] **Step 1: Write the failing test**

`follower_control/tests/test_detection_receiver.py`:
```python
from follower_control.detection_receiver import DetectionReceiver


def _dict(cx):
    return dict(cx=cx, cy=240.0, area=400.0, bbox=[0, 0, 20, 20],
                track_id=1, is_owner=True, confidence=0.9, is_predicted=False)


class _Source:
    def __init__(self, batches):
        self.batches = list(batches)
        self.i = 0

    def poll(self):
        out = self.batches[self.i] if self.i < len(self.batches) else []
        self.i += 1
        return out


def test_latest_none_before_update():
    r = DetectionReceiver(_Source([]))
    assert r.latest() is None


def test_update_keeps_most_recent():
    r = DetectionReceiver(_Source([[_dict(1.0), _dict(2.0)]]))
    r.update()
    assert r.latest().cx == 2.0


def test_none_payload_clears_owner():
    r = DetectionReceiver(_Source([[_dict(1.0)], [None]]))
    r.update()
    assert r.latest().cx == 1.0
    r.update()
    assert r.latest() is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_control && python -m pytest tests/test_detection_receiver.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_control.detection_receiver'`

- [ ] **Step 3: Write minimal implementation**

`follower_control/follower_control/detection_receiver.py`:
```python
from .detection import detection_from_dict


class DetectionReceiver:
    """Holds the latest owner Detection parsed from incoming JSON dicts.

    `source.poll()` returns a list of payloads received since last poll;
    each payload is a Detection dict, or None meaning 'no owner this frame'.
    Concrete TCP socket wraps this small interface (integration-tested)."""

    def __init__(self, source):
        self._source = source
        self._latest = None

    def update(self):
        for payload in self._source.poll():
            self._latest = detection_from_dict(payload)

    def latest(self):
        return self._latest
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_control && python -m pytest tests/test_detection_receiver.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_control/follower_control/detection_receiver.py follower_control/tests/test_detection_receiver.py
git commit -m "feat(control): injectable Detection receiver"
```

---

### Task 10: ROS boundary (scan provider, cmd publisher, control node)

**Files:**
- Create: `follower_control/follower_control/scan_provider.py`
- Create: `follower_control/follower_control/cmd_publisher.py`
- Create: `follower_control/follower_control/tcp_detection_source.py`
- Create: `follower_control/follower_control/control_node.py`

**Interfaces:**
- Consumes: `rclpy`, `sensor_msgs/LaserScan`, `geometry_msgs/Twist`, `ControlLoop`, `DetectionReceiver`, `config`.
- Produces: `ScanProvider(node, topic)` with `.get() -> list`; `CmdPublisher(node, topic)` with `.publish(lin, ang)`; `TcpDetectionSource(host, port)` with `.poll() -> list[dict]`; `control_node.main()`.

**Note:** This is the ROS/socket glue — it is verified by a **manual smoke test** (below), not a unit test, because it requires a running ROS graph and a live Detection sender. All decision logic it invokes (`ControlLoop`, PID, LiDAR, search) is already unit-tested in Tasks 2–9.

- [ ] **Step 1: Write the ROS glue**

`follower_control/follower_control/scan_provider.py`:
```python
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class ScanProvider:
    """Caches the latest /scan ranges as a plain list."""

    def __init__(self, node, topic):
        self._ranges = []
        node.create_subscription(LaserScan, topic, self._cb,
                                 qos_profile_sensor_data)

    def _cb(self, msg):
        self._ranges = list(msg.ranges)

    def get(self):
        return self._ranges
```

`follower_control/follower_control/cmd_publisher.py`:
```python
from geometry_msgs.msg import Twist


class CmdPublisher:
    """Publishes (linear_x, angular_z) as geometry_msgs/Twist."""

    def __init__(self, node, topic):
        self._pub = node.create_publisher(Twist, topic, 10)

    def publish(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._pub.publish(msg)
```

`follower_control/follower_control/tcp_detection_source.py`:
```python
import json
import socket
import threading


class TcpDetectionSource:
    """TCP server that receives newline-delimited Detection JSON from the AI
    server and buffers payloads for ControlLoop. Non-owner frames arrive as
    the JSON literal `null`. `.poll()` drains the buffer."""

    def __init__(self, host, port):
        self._buf = []
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(1)
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            conn, _ = self._sock.accept()
            with conn:
                buffer = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buffer += chunk
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        if not line.strip():
                            continue
                        payload = json.loads(line.decode('utf-8'))
                        with self._lock:
                            self._buf.append(payload)

    def poll(self):
        with self._lock:
            out, self._buf = self._buf, []
        return out
```

`follower_control/follower_control/control_node.py`:
```python
import rclpy
from rclpy.node import Node

from . import config
from .control_loop import ControlLoop
from .detection_receiver import DetectionReceiver
from .scan_provider import ScanProvider
from .cmd_publisher import CmdPublisher
from .tcp_detection_source import TcpDetectionSource


class ControlNode(Node):
    def __init__(self):
        super().__init__('follower_control')
        self._scan = ScanProvider(self, config.SCAN_TOPIC)
        self._cmd = CmdPublisher(self, config.CMD_VEL_TOPIC)
        self._receiver = DetectionReceiver(
            TcpDetectionSource(config.DETECTION_TCP_HOST, config.DETECTION_TCP_PORT))
        self._loop = ControlLoop(
            get_detection=self._get_detection,
            get_scan=self._scan.get,
            publish=self._cmd.publish,
            cfg=config,
            now=lambda: self.get_clock().now().nanoseconds / 1e9,
        )
        self.create_timer(1.0 / config.TICK_HZ, self._tick)

    def _get_detection(self):
        self._receiver.update()
        return self._receiver.latest()

    def _tick(self):
        self._loop.tick()


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Build the package**

Run: `cd <workspace_root> && colcon build --packages-select follower_control && source install/setup.bash`
Expected: build succeeds.

- [ ] **Step 3: Smoke test (manual, requires ROS graph)**

In three terminals:
```bash
# T1: fake a Detection sender (owner centered, far -> forward)
python3 -c "import socket,json,time; s=socket.socket(); s.connect(('127.0.0.1',6000)); \
d={'cx':320,'cy':240,'area':100,'bbox':[0,0,10,10],'track_id':1,'is_owner':True,'confidence':0.9,'is_predicted':False}; \
[ (s.sendall((json.dumps(d)+'\n').encode()), time.sleep(0.05)) for _ in range(200) ]"

# T2: run the node
ros2 run follower_control control_node

# T3: observe cmd_vel
ros2 topic echo /cmd_vel
```
Expected: `/cmd_vel` shows `linear.x > 0` (forward) while the fake owner streams; stops (`0,0`) and then rotation once the sender stops (search).

- [ ] **Step 4: Commit**

```bash
git add follower_control/follower_control/scan_provider.py follower_control/follower_control/cmd_publisher.py \
        follower_control/follower_control/tcp_detection_source.py follower_control/follower_control/control_node.py
git commit -m "feat(control): ROS node wiring (scan, cmd_vel, TCP detection, 20Hz tick)"
```

- [ ] **Step 5: Run the full pure-logic suite**

Run: `cd follower_control && python -m pytest -v`
Expected: PASS (all Tasks 1–9 tests green)

---

## Deferred (not in this plan)

- Concrete registration-command channel wiring (UI 등록 버튼 → ABA → AI server) — that path lives on the perception/ABA side, not control.
- `domain_bridge` config and production ABA↔robot deployment topology — out of scope (this repo tests following locally).
- SEARCHING geometry refinement: true ±30° oscillating sweep and closed-loop 180° turn (current planner is open-loop, timing-based) — tuning follow-up.
- Reverse enable/disable policy and rear-safety tuning.

## Self-Review

- **Spec coverage:** Detection receive (Task 9/10), PID distance+bearing+reverse (Task 2), Visual servoing = bearing PID centering (Task 2), LiDAR avoidance (Task 3), miss→SEARCHING (Task 8), 3-phase recovery BT (Tasks 4, 7), state machine TRACKING/SEARCHING/ENDED (Task 5), LKD via `last_direction`→search `lkd` (Tasks 6, 8), cmd_vel publish @20Hz (Task 10), tracking controller (Task 6). All Spec 2 sections covered. Nav2/patrol correctly excluded.
- **Placeholder scan:** none — every code step has runnable code; Task 10 uses an explicit manual smoke test with exact commands (justified: ROS/socket glue).
- **Type consistency:** `Detection` fields consistent with Spec 1's `detection_to_dict`; `cfg` attribute names match `config.py`; `FollowPID.compute(cx, area, dt)`, `apply_avoidance(lin, ang, scan, cfg)`, `search_command(elapsed, cfg, lkd)`, `TrackingController.step(detection, scan, dt)`, `create_searching_tree(ctx)`/`tick_tree`, `ControlLoop.tick()`, `DetectionReceiver.poll`-source contract all consistent across tasks.
```
