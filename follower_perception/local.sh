#!/usr/bin/env bash
# Launch the local (AI server + Qt viewer) processes in a tmux session (split panes):
#   pane0: perception_server (recv video, run perception, send cmd -> Pi)
#   pane1: Qt viewer
#
#   ./local.sh <PI_IP>
#
# Detach: Ctrl-b d   |   Kill all: tmux kill-session -t libi_local
set -euo pipefail
PI_IP="${1:?usage: ./local.sh <PI_IP>}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"        # follower_perception/
REPO="$(cd "$DIR/.." && pwd)"
SESSION="libi_local"

# ---- edit for your machine (or override via env) ----
VENV_PY="${VENV_PY:-/home/ane/personal_repo/labeling_sam3/.venv/bin/python}"
VIDEO_PORT="${VIDEO_PORT:-6001}"
CMD_PORT="${CMD_PORT:-6002}"
VIEWER_PORT="${VIEWER_PORT:-5007}"
VIEWER="${VIEWER:-$REPO/qt_demo/build/viewer}"
# -----------------------------------------------------

command -v tmux >/dev/null || { echo "tmux 없음: sudo apt install -y tmux"; exit 1; }
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session  -d -s "$SESSION" -c "$DIR" -n libi

# pane 0: perception server (+ drive to Pi)
tmux send-keys -t "$SESSION" \
  "$VENV_PY scripts/perception_server.py --udp --udp-port $VIDEO_PORT --drive-host $PI_IP --drive-port $CMD_PORT" C-m
# pane 1: Qt viewer (wait a moment for the server to bind 5007)
tmux split-window -h -t "$SESSION" -c "$DIR"
tmux send-keys -t "$SESSION" "sleep 2; '$VIEWER' 127.0.0.1 $VIEWER_PORT" C-m

tmux attach -t "$SESSION"
