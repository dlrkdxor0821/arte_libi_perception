from follower_perception.detection import Detection
from scripts.cmd_preview import compute_cmd_vel, TARGET_SIZE


def _det(cx, area, w=640):
    return Detection(cx=cx, cy=240, area=area, bbox=(0, 0, 10, 10), track_id=1,
                     is_owner=True, confidence=0.9, is_predicted=False)


# area giving sqrt(area) == TARGET_SIZE -> inside deadband -> linear stops,
# so direction tests isolate angular.z.
_HOLD_AREA = TARGET_SIZE ** 2


def test_no_owner_stops():
    cmd = compute_cmd_vel(None, 640)
    assert cmd["linear_x"] == 0.0 and cmd["angular_z"] == 0.0
    assert cmd["drive"] == "STOP"


def test_left_third_turns_left():
    cmd = compute_cmd_vel(_det(cx=50, area=_HOLD_AREA), 640)
    assert cmd["angular_z"] > 0 and cmd["turn"] == "LEFT"


def test_right_third_turns_right():
    cmd = compute_cmd_vel(_det(cx=600, area=_HOLD_AREA), 640)
    assert cmd["angular_z"] < 0 and cmd["turn"] == "RIGHT"


def test_center_no_turn():
    cmd = compute_cmd_vel(_det(cx=320, area=_HOLD_AREA), 640)
    assert cmd["angular_z"] == 0.0 and cmd["turn"] == "CENTER"


def test_far_goes_forward():
    cmd = compute_cmd_vel(_det(cx=320, area=10000), 640)   # sqrt=100 << target
    assert cmd["linear_x"] > 0 and cmd["drive"] == "FWD"


def test_close_goes_backward():
    cmd = compute_cmd_vel(_det(cx=320, area=90000), 640)   # sqrt=300 >> target
    assert cmd["linear_x"] < 0 and cmd["drive"] == "BACK"
