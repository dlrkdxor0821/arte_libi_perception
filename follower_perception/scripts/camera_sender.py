#!/usr/bin/env python3
"""Robot (Pi) side: capture camera -> UDP-send 640-wide JPEG to the AI server.

    python scripts/camera_sender.py --host <AI_SERVER_IP> --port 6001 --camera 0
    python scripts/camera_sender.py --host 127.0.0.1 --port 6001 --test-pattern

Independent of ROS — runs alongside the robot bringup, reads the camera directly.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.udp_video import UdpVideoSender
from scripts.perception_server import test_pattern_frames, _camera_frames


def main():
    ap = argparse.ArgumentParser(description="UDP camera sender (robot side)")
    ap.add_argument("--host", required=True, help="AI server IP")
    ap.add_argument("--port", type=int, default=6001)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--camera", type=int, default=0)
    src.add_argument("--test-pattern", dest="test_pattern", action="store_true")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--quality", type=int, default=70)
    ap.add_argument("--fps", type=float, default=15.0)   # lower fps = less Pi CPU/bandwidth
    args = ap.parse_args()

    frames = test_pattern_frames() if args.test_pattern else _camera_frames(args.camera)
    sender = UdpVideoSender(args.host, args.port, width=args.width, quality=args.quality)
    print(f"[ok] sending video -> {args.host}:{args.port} ({args.width}w, q{args.quality})")
    delay = 1.0 / args.fps if args.fps > 0 else 0.0
    n = 0
    try:
        for frame in frames:
            sender.send(frame)
            n += 1
            if n % 60 == 0:
                print(f"[..] sent {n} frames")
            if delay:
                time.sleep(delay)
    finally:
        sender.close()


if __name__ == "__main__":
    main()
