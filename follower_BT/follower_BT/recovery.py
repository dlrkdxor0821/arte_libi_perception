"""Drive state machine + open-loop recovery search (follower_BT).

Independent package — no ROS / py_trees / lidar / PID — so it can be swapped for
follower_control's real py_trees BT later (same behaviour). The follow controller
is INJECTED, so this stays decoupled from perception's cmd computation.

States:
    IDLE  --register (owner visible)-->  FOLLOWING
    FOLLOWING  --owner lost-->  SEARCHING
    SEARCHING  --owner reacquired-->  FOLLOWING
    SEARCHING  --search gives up-->  IDLE

Search timeline (NO LKD; symmetric):
    scan   : oscillate left/right ~45 deg
    turn   : ~180 deg
    scan   : oscillate again
    turn   : ~180 deg (back to start)
    after  : give up -> IDLE
"""
import math

# Reference tuning (canonical values -> follower_control/config.py).
SCAN_ANGLE = math.radians(45)      # +/- sweep amplitude
TURN_ANGLE = math.radians(180)
ANGULAR_SEARCH = 0.25              # rad/s (slower search rotation; was 0.5)

_Q = SCAN_ANGLE / ANGULAR_SEARCH             # time to sweep 45 deg
SCAN_SEC = 4 * _Q                             # one full 0->+45->-45->0 oscillation
TURN_SEC = TURN_ANGLE / ANGULAR_SEARCH

IDLE, FOLLOWING, SEARCHING = "IDLE", "FOLLOWING", "SEARCHING"


def _scan_angular(t):
    """Oscillate 0 -> +45 -> -45 -> 0 across one scan window (symmetric, no LKD)."""
    if t < _Q:
        return ANGULAR_SEARCH           # 0 -> +45 (left)
    if t < 3 * _Q:
        return -ANGULAR_SEARCH          # +45 -> -45 (right)
    return ANGULAR_SEARCH               # -45 -> 0 (left)


def search_command(elapsed):
    """(angular_z, done, phase) for the recovery timeline."""
    t1 = SCAN_SEC
    t2 = t1 + TURN_SEC
    t3 = t2 + SCAN_SEC
    t4 = t3 + TURN_SEC
    if elapsed < t1:
        return _scan_angular(elapsed), False, "SCAN1"
    if elapsed < t2:
        return ANGULAR_SEARCH, False, "TURN180"
    if elapsed < t3:
        return _scan_angular(elapsed - t2), False, "SCAN2"
    if elapsed < t4:
        return ANGULAR_SEARCH, False, "TURN180B"
    return 0.0, True, "GIVEUP"


class RecoveryBehavior:
    def __init__(self):
        self._elapsed = 0.0
        self.done = False

    def reset(self):
        self._elapsed = 0.0
        self.done = False

    def update(self, dt):
        self._elapsed += dt
        ang, done, phase = search_command(self._elapsed)
        self.done = done
        return {"linear_x": 0.0, "angular_z": ang, "drive": "SEARCH", "turn": phase}


class DrivePolicy:
    """IDLE / FOLLOWING / SEARCHING state machine.

    follow_fn (det, frame_w) -> cmd dict is injected, so the follow controller
    (cmd_preview now, follower_control later) stays swappable."""

    def __init__(self, follow_fn):
        self._follow = follow_fn
        self._search = RecoveryBehavior()
        self.state = IDLE

    def step(self, det, frame_w, dt, registered=True):
        if not registered:                                  # nothing registered
            self.state = IDLE
            self._search.reset()
            return self._tag({"linear_x": 0.0, "angular_z": 0.0,
                              "drive": "STOP", "turn": "-"})
        if det is not None and getattr(det, "is_owner", False):
            self.state = FOLLOWING
            self._search.reset()
            return self._tag(self._follow(det, frame_w))
        # registered but owner not visible -> search
        cmd = self._search.update(dt)
        if self._search.done:                               # gave up
            self.state = IDLE
            cmd = {"linear_x": 0.0, "angular_z": 0.0, "drive": "STOP", "turn": "GIVEUP"}
        else:
            self.state = SEARCHING
        return self._tag(cmd)

    def _tag(self, cmd):
        cmd["state"] = self.state
        return cmd
