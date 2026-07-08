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
        # Stamp the search start time now (when SEARCHING begins), not on the
        # search tree's first tick_tree() call — those can be ticks apart,
        # which would understate elapsed search time.
        self._search_ctx.start = self.now()
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
