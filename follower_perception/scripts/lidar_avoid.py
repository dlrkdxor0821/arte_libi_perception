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
FRONT_DEG = 30            # front arc half-width (deg)
BACK_DEG = 30             # back arc half-width around 180 deg
SIDE_LO, SIDE_HI = 30, 90  # side arc (deg) for left/right (drift + display)


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


def sectors(ranges, angle_min, angle_inc, swap_sides=False):
    """(front, left, right) minimum ranges from a LaserScan.

    swap_sides=True swaps left/right — use it when the LiDAR scans clockwise
    (RPLidar) or is mounted flipped, so "left" in code = physical left.
    """
    front = sector_min(ranges, angle_min, angle_inc, -FRONT_DEG, FRONT_DEG)
    left = sector_min(ranges, angle_min, angle_inc, SIDE_LO, SIDE_HI)
    right = sector_min(ranges, angle_min, angle_inc, -SIDE_HI, -SIDE_LO)
    if swap_sides:
        left, right = right, left
    return front, left, right


def sectors4(ranges, angle_min, angle_inc, flip_180=False):
    """(front, back, left, right) minimum ranges (m) — for display + avoidance.

    flip_180=True is for a LiDAR mounted rotated 180 deg (yaw): it swaps
    front<->back AND left<->right so the sectors match the robot's own frame.
    """
    front, left, right = sectors(ranges, angle_min, angle_inc)      # raw scan frame
    back = min(sector_min(ranges, angle_min, angle_inc, 180.0 - BACK_DEG, 180.0),
               sector_min(ranges, angle_min, angle_inc, -180.0, -(180.0 - BACK_DEG)))
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
