import py_trees

from .search_planner import search_command


class SearchContext:
    """Injected dependencies for the searching tree (no ROS)."""

    def __init__(self, get_detection, publish, cfg, now, lkd=1.0):
        self.get_detection = get_detection
        self.publish = publish
        self.cfg = cfg
        self.now = now
        self.lkd = lkd
        self.start = None


class CheckReacquired(py_trees.behaviour.Behaviour):
    def __init__(self, ctx):
        super().__init__(name='CheckReacquired')
        self.ctx = ctx

    def update(self):
        if self.ctx.get_detection() is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class SearchMotion(py_trees.behaviour.Behaviour):
    def __init__(self, ctx):
        super().__init__(name='SearchMotion')
        self.ctx = ctx

    def initialise(self):
        if self.ctx.start is None:
            self.ctx.start = self.ctx.now()

    def update(self):
        elapsed = self.ctx.now() - self.ctx.start
        ang, done = search_command(elapsed, self.ctx.cfg, self.ctx.lkd)
        if done:
            self.ctx.publish(0.0, 0.0)
            return py_trees.common.Status.FAILURE
        self.ctx.publish(0.0, ang)
        return py_trees.common.Status.RUNNING


def create_searching_tree(ctx):
    root = py_trees.composites.Selector(name='BT_Searching', memory=False)
    root.add_children([CheckReacquired(ctx), SearchMotion(ctx)])
    return root


def tick_tree(root):
    root.tick_once()
    return root.status
