"""LiDAR obstacle avoidance for the simple drive path (bang-bang, no PID).

Pure logic (no ROS) so it's testable and swappable for follower_control's
lidar_avoidance later. cmd_bridge feeds it the LaserScan and applies the result
right before publishing /cmd_vel (robot-local, so the safety loop stays fast).

Rules:
  BRAKE (block translation head-on in the travel direction):
    moving forward & front < STOP_DIST  -> linear_x = 0
    moving back    & back  < STOP_DIST  -> linear_x = 0
  DRIFT (while advancing, gently steer off the CLOSER side wall so it weaves
  through tight gaps instead of stalling):
    take whichever side is nearer; if it is within SIDE_NEAR, add a small angular
    push AWAY from it, proportional to how close it is. As the robot drifts the
    other wall becomes the nearer one -> it steers back -> it weaves through.
  angular_z is only ADDED to (never zeroed), so target-facing rotation survives.
  Drift acts only while advancing (linear_x > 0) -> no spinning when idle.

Note: the followed person is also "in front" — keep STOP_DIST well BELOW the
follow distance so the owner at follow distance doesn't trigger a stop.
"""
import math

STOP_DIST = 0.15          # m: block translation if blocked closer than this
SIDE_NEAR = 0.30          # m: start drifting away from a side wall within this
SIDE_DRIFT = 0.15         # rad/s: max gentle steer used to weave off the walls


def _norm(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def sector_min(ranges, angle_min, angle_inc, lo_deg, hi_deg):
    """Min valid range (m) over angles in [lo_deg, hi_deg]. inf if none."""
    best = math.inf
    for i, r in enumerate(ranges):
        if not (r and math.isfinite(r) and r > 0.0):
            continue
        a = _norm(math.degrees(angle_min + i * angle_inc))
        if lo_deg <= a <= hi_deg:
            best = min(best, r)
    return best


def sectors4(ranges, angle_min, angle_inc, flip_180=False):
    """(front, back, left, right) minimum ranges (m) — 8-way coverage.

    The circle is split into 8 sectors of 45 deg. To keep a diagonal obstacle
    from slipping through a gap, each side is the MIN over its three sub-sectors:
        left  = min(front-left, left, back-left)      # 좌상, 좌, 좌하
        right = min(front-right, right, back-right)    # 우상, 우, 우하
    front/back stay narrow (±22.5 deg) for head-on braking. flip_180=True swaps
    front<->back and left<->right (LiDAR mounted rotated 180 deg).
    """
    def m(lo, hi):
        return sector_min(ranges, angle_min, angle_inc, lo, hi)
    front = m(-22.5, 22.5)
    left = min(m(22.5, 67.5), m(67.5, 112.5), m(112.5, 157.5))          # 좌상 / 좌 / 좌하
    right = min(m(-67.5, -22.5), m(-112.5, -67.5), m(-157.5, -112.5))   # 우상 / 우 / 우하
    back = min(m(157.5, 180.0), m(-180.0, -157.5))
    if flip_180:
        front, back = back, front
        left, right = right, left
    return front, back, left, right


def _push(dist, near):
    """0 (far) .. 1 (touching): how strongly a wall at `dist` should push away."""
    return 0.0 if dist >= near else (near - dist) / near


def avoid_cmd(linear_x, angular_z, front, back, left, right):
    """Brake head-on + gently drift off side walls (weave through tight gaps).

    Returns (linear_x, angular_z, reason): "clear" | "front" | "back" | "drift".
    Rotation is only ADDED to (never zeroed), so target-facing is preserved.
    """
    reason = "clear"
    moving_fwd = linear_x > 0.0
    if moving_fwd and front < STOP_DIST:
        linear_x = 0.0
        reason = "front"
    elif linear_x < 0.0 and back < STOP_DIST:
        linear_x = 0.0
        reason = "back"
    if moving_fwd:                                   # weave only while advancing
        # steer away from whichever wall is CLOSER, proportional to how close
        if left < right:
            drift = -SIDE_DRIFT * _push(left, SIDE_NEAR)     # left nearer -> steer right
        else:
            drift = SIDE_DRIFT * _push(right, SIDE_NEAR)     # right nearer -> steer left
        if drift != 0.0:
            angular_z += drift
            if reason == "clear":
                reason = "drift"
    return linear_x, angular_z, reason
