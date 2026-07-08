import rclpy
from rclpy.node import Node

from . import config
from .control_loop import ControlLoop
from .detection_receiver import DetectionReceiver
from .scan_provider import ScanProvider
from .cmd_publisher import CmdPublisher
from .tcp_detection_source import TcpDetectionSource


class ControlNode(Node):
    def __init__(self):
        super().__init__('follower_control')
        self._scan = ScanProvider(self, config.SCAN_TOPIC)
        self._cmd = CmdPublisher(self, config.CMD_VEL_TOPIC)
        self._receiver = DetectionReceiver(
            TcpDetectionSource(config.DETECTION_TCP_HOST, config.DETECTION_TCP_PORT))
        self._loop = ControlLoop(
            get_detection=self._get_detection,
            get_scan=self._scan.get,
            publish=self._cmd.publish,
            cfg=config,
            now=lambda: self.get_clock().now().nanoseconds / 1e9,
        )
        self.create_timer(1.0 / config.TICK_HZ, self._tick)

    def _get_detection(self):
        self._receiver.update()
        return self._receiver.latest()

    def _tick(self):
        self._loop.tick()


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
