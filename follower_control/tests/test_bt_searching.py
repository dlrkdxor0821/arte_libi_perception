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
