import numpy as np
from follower_perception.detection import TrackedBox
from follower_perception.reid_engine import ReIDEngine
from follower_perception.target_matcher import TargetMatcher
from follower_perception import constants


def _frame(color_bgr, w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def _matcher():
    return TargetMatcher(ReIDEngine(backend='colour'))


def test_not_registered_returns_none():
    m = _matcher()
    assert m.is_registered is False
    assert m.match([_full_box(1)], _frame((0, 0, 255))) is None


def test_matches_registered_colour_owner():
    m = _matcher()
    m.register(_frame((0, 0, 255)))          # red owner
    assert m.is_registered is True
    assert m.match([_full_box(1)], _frame((0, 0, 255))) == 1


def test_rejects_different_colour():
    m = _matcher()
    m.register(_frame((0, 0, 255)))          # red owner
    assert m.match([_full_box(9)], _frame((255, 0, 0))) is None  # blue candidate


def test_locks_safe_id_after_verify_frames():
    m = _matcher()
    m.register(_frame((0, 0, 255)))
    red = _frame((0, 0, 255))
    for _ in range(constants.VERIFY_FRAMES):
        m.match([_full_box(1)], red)
    assert m.safe_id == 1


def test_calibrate_grows_gallery_for_novel_view():
    m = _matcher()
    m.register(_frame((0, 0, 255)))
    start = len(m.gallery)
    m.calibrate(_frame((0, 40, 200)))        # different enough -> append
    assert len(m.gallery) == start + 1


def test_reset_clears_registration():
    m = _matcher()
    m.register(_frame((0, 0, 255)))
    m.reset()
    assert m.is_registered is False
    assert m.safe_id is None
