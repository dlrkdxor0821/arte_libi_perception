import numpy as np
from follower_perception.detection import TrackedBox
from follower_perception.reid_engine import ReIDEngine
from follower_perception.mocks import MockDetector
from follower_perception.pipeline import FollowerPerception
from follower_perception import constants


def _frame(color_bgr, w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def _perception(script):
    return FollowerPerception(detector=MockDetector(script),
                              reid=ReIDEngine(backend='colour'))


def test_register_requires_stable_frames():
    script = [[_full_box(1)]] * constants.REGISTRATION_STABLE_FRAMES
    p = _perception(script)
    red = _frame((0, 0, 255))
    results = [p.register(red) for _ in range(constants.REGISTRATION_STABLE_FRAMES)]
    assert results[-1] is True            # confirmed on the last stable frame
    assert results[0] is False            # not yet stable on frame 1


def test_run_then_get_latest_returns_owner():
    n = constants.REGISTRATION_STABLE_FRAMES
    script = [[_full_box(1)]] * (n + 1)
    p = _perception(script)
    red = _frame((0, 0, 255))
    for _ in range(n):
        p.register(red)
    p.run(red)
    det = p.get_latest()
    assert det is not None
    assert det.is_owner is True
    assert det.is_predicted is False
    assert det.track_id == 1


def test_coasting_then_none():
    n = constants.REGISTRATION_STABLE_FRAMES
    # n stable frames to register, 1 run with owner, then misses (empty lists)
    script = [[_full_box(1)]] * (n + 1) + [[]] * (constants.COAST_LIMIT + 2)
    p = _perception(script)
    red = _frame((0, 0, 255))
    for _ in range(n):
        p.register(red)
    p.run(red)                            # owner seen
    assert p.get_latest().is_predicted is False
    p.run(red)                            # first miss
    assert p.get_latest().is_predicted is True   # coasting
    for _ in range(constants.COAST_LIMIT + 1):
        p.run(red)
    assert p.get_latest() is None         # beyond coast limit


def test_reset_clears_everything():
    n = constants.REGISTRATION_STABLE_FRAMES
    p = _perception([[_full_box(1)]] * (n + 1))
    red = _frame((0, 0, 255))
    for _ in range(n):
        p.register(red)
    p.reset()
    assert p.get_latest() is None
