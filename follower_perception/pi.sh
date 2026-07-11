#!/usr/bin/env bash
# Launch the 3 Pi (robot) processes in a tmux session (split panes):
#   pane0: bringup   pane1: camera_sender (video -> AI)   pane2: cmd_bridge (cmd -> /cmd_vel)
#
#   ./pi.sh <AI_SERVER_IP>
#
# Detach: Ctrl-b d   |   Kill all: tmux kill-session -t libi_pi
set -euo pipefail
AI_IP="${1:?usage: ./pi.sh <AI_SERVER_IP>}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"        # follower_perception/
SESSION="libi_pi"

# ---- edit for your robot (or override via env) ----
ROS_SETUP="${ROS_SETUP:-/home/pinky/pinky_pro/install/setup.bash}"
BRINGUP_CMD="${BRINGUP_CMD:-ros2 launch pinky_bringup bringup.launch.py}"   # <-- set your REAL bringup
VIDEO_PORT="${VIDEO_PORT:-6001}"
CMD_PORT="${CMD_PORT:-6002}"
CAM_ARGS="${CAM_ARGS:---picamera --fps 15}"
# ---------------------------------------------------

command -v tmux >/dev/null || { echo "tmux 없음: sudo apt install -y tmux"; exit 1; }
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session  -d -s "$SESSION" -c "$DIR" -n libi

# pane 0: bringup
tmux send-keys -t "$SESSION" "source '$ROS_SETUP'; $BRINGUP_CMD" C-m
# pane 1: camera sender -> AI server
tmux split-window -v -t "$SESSION" -c "$DIR"
tmux send-keys -t "$SESSION" "python3 scripts/camera_sender.py --host $AI_IP --port $VIDEO_PORT $CAM_ARGS" C-m
# pane 2: cmd bridge (needs ROS sourced)
tmux split-window -v -t "$SESSION" -c "$DIR"
tmux send-keys -t "$SESSION" "source '$ROS_SETUP'; python3 scripts/cmd_bridge.py --port $CMD_PORT" C-m

tmux select-layout -t "$SESSION" even-vertical
tmux attach -t "$SESSION"
