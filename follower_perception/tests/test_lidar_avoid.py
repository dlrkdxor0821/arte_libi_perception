import math
from scripts.lidar_avoid import (
    sector_min, sectors, sectors4, avoid_cmd, STOP_DIST, SIDE_NEAR,
)

FAR = 5.0
BLOCK = STOP_DIST * 0.5             # clearly inside the stop distance
SIDE_CLOSE = SIDE_NEAR * 0.5       # clearly inside the drift band


def test_sector_min_front_only():
    # rays at -90,-45,0,45,90 deg
    ranges = [3.0, 2.0, 0.5, 2.0, 3.0]
    amin, ainc = math.radians(-90), math.radians(45)
    assert sector_min(ranges, amin, ainc, -30, 30) == 0.5     # only the 0deg ray
    assert sector_min(ranges, amin, ainc, 30, 90) == 2.0      # +45,+90 -> min 2.0


def test_sector_min_ignores_bad_values():
    ranges = [float("inf"), 0.0, -1.0, 1.5]
    amin, ainc = math.radians(-10), math.radians(10)
    assert sector_min(ranges, amin, ainc, -30, 30) == 1.5     # only the valid one


def test_sectors_swap_sides():
    # obstacle at +45deg -> normally the LEFT sector; swap moves it to RIGHT
    ranges = [3.0, 3.0, 3.0, 0.4, 3.0]
    amin, ainc = math.radians(-90), math.radians(45)
    _, l, r = sectors(ranges, amin, ainc, swap_sides=False)
    assert l == 0.4 and r == 3.0
    _, l, r = sectors(ranges, amin, ainc, swap_sides=True)
    assert l == 3.0 and r == 0.4


def test_sectors4_back():
    ranges = [0.5, 3, 3, 3, 3, 3, 3, 3]   # obstacle at -180deg (raw back)
    amin, ainc = math.radians(-180), math.radians(45)
    f, b, l, r = sectors4(ranges, amin, ainc)
    assert b == 0.5 and f == 3.0


def test_sectors4_flip_front_back():
    ranges = [3, 3, 3, 3, 0.5, 3, 3, 3]   # obstacle at 0deg (raw front, index 4)
    amin, ainc = math.radians(-180), math.radians(45)
    f, b, _, _ = sectors4(ranges, amin, ainc, flip_180=False)
    assert f == 0.5
    f, b, _, _ = sectors4(ranges, amin, ainc, flip_180=True)
    assert b == 0.5                        # 180 flip -> now behind


def test_sectors4_flip_sides():
    ranges = [3, 3, 3, 3, 3, 0.4, 3, 3]   # obstacle at +45deg (raw left)
    amin, ainc = math.radians(-180), math.radians(45)
    _, _, l, r = sectors4(ranges, amin, ainc, flip_180=False)
    assert l == 0.4
    _, _, l, r = sectors4(ranges, amin, ainc, flip_180=True)
    assert r == 0.4                        # 180 flip -> now on the right


# --- avoid_cmd: brake head-on + gently drift off side walls ---

def test_clear_passes_through():
    lin, ang, r = avoid_cmd(0.06, 0.1, front=FAR, back=FAR, left=FAR, right=FAR)
    assert lin == 0.06 and ang == 0.1 and r == "clear"


def test_front_blocks_forward_but_keeps_rotation():
    lin, ang, r = avoid_cmd(0.06, 0.3, front=BLOCK, back=FAR, left=FAR, right=FAR)
    assert lin == 0.0 and ang == 0.3 and r == "front"   # rotation preserved


def test_forward_ignores_back():
    lin, ang, r = avoid_cmd(0.06, 0.0, front=FAR, back=BLOCK, left=FAR, right=FAR)
    assert lin == 0.06 and r == "clear"                 # back irrelevant going forward


def test_back_blocks_reverse():
    lin, ang, r = avoid_cmd(-0.04, 0.0, front=FAR, back=BLOCK, left=FAR, right=FAR)
    assert lin == 0.0 and r == "back"


def test_reverse_ignores_front():
    lin, ang, r = avoid_cmd(-0.04, 0.0, front=BLOCK, back=FAR, left=FAR, right=FAR)
    assert lin == -0.04 and r == "clear"                # front irrelevant reversing


def test_left_wall_drifts_right():
    lin, ang, r = avoid_cmd(0.06, 0.0, front=FAR, back=FAR, left=SIDE_CLOSE, right=FAR)
    assert ang < 0 and lin == 0.06 and r == "drift"     # left wall -> steer right


def test_right_wall_drifts_left():
    lin, ang, r = avoid_cmd(0.06, 0.0, front=FAR, back=FAR, left=FAR, right=SIDE_CLOSE)
    assert ang > 0 and r == "drift"                     # right wall -> steer left


def test_closer_wall_wins():
    # both within the band, LEFT closer -> steer right (away from the closer wall)
    lin, ang, r = avoid_cmd(0.06, 0.0, front=FAR, back=FAR,
                            left=SIDE_NEAR * 0.3, right=SIDE_NEAR * 0.7)
    assert ang < 0 and r == "drift"


def test_drift_scales_with_closeness():
    _, near_ang, _ = avoid_cmd(0.06, 0.0, front=FAR, back=FAR, left=SIDE_NEAR * 0.2, right=FAR)
    _, far_ang, _ = avoid_cmd(0.06, 0.0, front=FAR, back=FAR, left=SIDE_NEAR * 0.8, right=FAR)
    assert abs(near_ang) > abs(far_ang)                 # closer wall -> stronger drift


def test_no_drift_when_stopped():
    lin, ang, r = avoid_cmd(0.0, 0.3, front=FAR, back=FAR, left=SIDE_CLOSE, right=FAR)
    assert lin == 0.0 and ang == 0.3 and r == "clear"   # not advancing -> no weave
