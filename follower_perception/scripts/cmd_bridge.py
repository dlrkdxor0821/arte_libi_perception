#!/usr/bin/env python3
"""Robot (Pi) side: receive (linear_x, angular_z) over UDP from the AI server and
publish geometry_msgs/Twist to /cmd_vel at a fixed rate, with a SAFETY WATCHDOG
(publish zero if no fresh command) and speed clamps.

This is the ONLY ROS piece of the simple drive path. It is intentionally a thin,
standalone node so it can be SWAPPED for follower_control later (same /cmd_vel
output). Run on the Pi with ROS 2 sourced:

    python3 scripts/cmd_bridge.py --port 6002
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.cmd_channel import CmdReceiver


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    ap = argparse.ArgumentParser(description="UDP cmd -> /cmd_vel bridge (Pi side)")
    ap.add_argument("--port", type=int, default=6002, help="UDP cmd port from AI server")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--rate", type=float, default=20.0, help="publish Hz")
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="STOP if no cmd received within this many seconds (watchdog)")
    ap.add_argument("--max-linear", dest="max_linear", type=float, default=0.15,
                    help="m/s clamp (safety, start low)")
    ap.add_argument("--max-angular", dest="max_angular", type=float, default=0.8,
                    help="rad/s clamp (safety)")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node("cmd_bridge")
    pub = node.create_publisher(Twist, args.topic, 10)
    recv = CmdReceiver(args.port)
    node.get_logger().info(
        f"cmd_bridge: UDP :{args.port} -> {args.topic} "
        f"(watchdog {args.timeout}s, max_lin {args.max_linear}, max_ang {args.max_angular})")

    def tick():
        t = Twist()
        v = recv.latest()
        if v is not None and v[2] <= args.timeout:          # fresh command
            t.linear.x = _clamp(float(v[0]), -args.max_linear, args.max_linear)
            t.angular.z = _clamp(float(v[1]), -args.max_angular, args.max_angular)
        # else: stale / none -> zero Twist (STOP)
        pub.publish(t)

    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())        # final stop
        recv.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
