import numpy as np
from follower_perception.detector import Detector


class _Arr:
    def __init__(self, data):
        self._data = np.asarray(data)

    def cpu(self):
        return self

    def numpy(self):
        return self._data


class _Boxes:
    def __init__(self, xyxy, ids, confs):
        self.xyxy = _Arr(xyxy)
        self.id = _Arr(ids) if ids is not None else None
        self.conf = _Arr(confs)


class _Result:
    def __init__(self, boxes):
        self.boxes = boxes


def test_parse_returns_tracked_boxes():
    result = _Result(_Boxes(
        xyxy=[[10, 20, 30, 60]],   # w=20, h=40 -> area 800, cx 20, cy 40
        ids=[5],
        confs=[0.9],
    ))
    out = Detector._to_tracked_boxes(result)
    assert len(out) == 1
    tb = out[0]
    assert tb.track_id == 5
    assert tb.cx == 20.0
    assert tb.cy == 40.0
    assert tb.area == 800.0
    assert tb.bbox == (10.0, 20.0, 30.0, 60.0)


def test_parse_no_ids_returns_empty():
    result = _Result(_Boxes(xyxy=[[0, 0, 1, 1]], ids=None, confs=[0.5]))
    assert Detector._to_tracked_boxes(result) == []


def test_parse_none_boxes_returns_empty():
    assert Detector._to_tracked_boxes(_Result(None)) == []
