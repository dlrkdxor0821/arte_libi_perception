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


def _picamera_frames(width=640, height=480):
    """Raspberry Pi CSI camera via libcamera (picamera2). Yields BGR frames.
    Run with the SYSTEM python3 (picamera2 is a system package)."""
    import cv2
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("[error] picamera2 not found. Install: sudo apt install -y python3-picamera2\n"
              "        and run this with the system python3 (not a venv).")
        raise SystemExit(2)
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"size": (width, height), "format": "XRGB8888"}))
    picam2.start()
    print("[ok] picamera2 started (libcamera)")
    try:
        while True:
            yield cv2.cvtColor(picam2.capture_array(), cv2.COLOR_BGRA2BGR)
    finally:
        picam2.stop()


def main():
    ap = argparse.ArgumentParser(description="UDP camera sender (robot side)")
    ap.add_argument("--host", required=True, help="AI server IP")
    ap.add_argument("--port", type=int, default=6001)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--camera", type=int, default=0)
    src.add_argument("--picamera", action="store_true", help="Pi CSI camera (libcamera/picamera2)")
    src.add_argument("--test-pattern", dest="test_pattern", action="store_true")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--quality", type=int, default=70)
    ap.add_argument("--fps", type=float, default=15.0)   # lower fps = less Pi CPU/bandwidth
    args = ap.parse_args()

    if args.picamera:
        frames = _picamera_frames(args.width, int(args.width * 3 / 4))
    elif args.test_pattern:
        frames = test_pattern_frames()
    else:
        frames = _camera_frames(args.camera)
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
