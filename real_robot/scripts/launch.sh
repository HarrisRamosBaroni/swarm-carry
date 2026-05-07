#!/usr/bin/env bash
# Launch laptop-side processes for a swarm experiment.
#
# Run from the repo root:
#   ./real_robot/scripts/launch.sh --mode central --goal 2 0 0
#   ./real_robot/scripts/launch.sh --mode decentralised
#
# central:       tmux session "swarm-laptop" with two windows:
#                  mocap      — mocap_pub streaming poses from PhaseSpace
#                  controller — central_runner sending commands to all robots
# decentralised: tmux session "swarm-laptop" with one window:
#                  mocap      — mocap_pub only (robots self-drive)
#
# Options:
#   --mode central|decentralised   (required)
#   --goal X Y Z                   goal pose in metres/rad (central only, default: 5 0 0)
#   --n-robots N                   number of robots (default: from network.yaml)
#   --server IP                    PhaseSpace server IP (default: 192.168.1.25)
#   --config PATH                  network.yaml path (default: real_robot/config/network.yaml)
#   --gt-payload                   use live mocap payload pose (central only)
#   --relative-goal                treat --goal as offset from initial centroid (central only)
#   --viewer                       launch live_viewer alongside controller (central only)
#   --goal-setter                  open goal_setter in a tmux window (works in both modes)
#
# Attach: tmux attach -t swarm-laptop
# Kill:   tmux kill-session -t swarm-laptop

set -euo pipefail

CONFIG="${CONFIG:-real_robot/config/network.yaml}"
MOCAP_SERVER="${MOCAP_SERVER:-192.168.1.25}"
PYTHON="${PYTHON:-python3}"
TMUX_SESSION="swarm-laptop"

MODE=""
GOAL="5.0 0.0 0.0"
N_ROBOTS=""
GT_PAYLOAD=false
RELATIVE_GOAL=false
VIEWER=false
GOAL_SETTER=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --mode)          MODE="$2"; shift 2 ;;
    --goal)          GOAL="$2 $3 $4"; shift 4 ;;
    --n-robots)      N_ROBOTS="$2"; shift 2 ;;
    --server)        MOCAP_SERVER="$2"; shift 2 ;;
    --config)        CONFIG="$2"; shift 2 ;;
    --gt-payload)    GT_PAYLOAD=true; shift ;;
    --relative-goal) RELATIVE_GOAL=true; shift ;;
    --viewer)        VIEWER=true; shift ;;
    --goal-setter)   GOAL_SETTER=true; shift ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

if [[ -z "$MODE" ]]; then
  echo "Usage: $0 --mode [central|decentralised] [options]"
  echo "       $0 --mode central --goal 2 0 0 --n-robots 2"
  echo "       $0 --mode decentralised"
  exit 1
fi

if [[ "$MODE" != "central" && "$MODE" != "decentralised" ]]; then
  echo "error: --mode must be 'central' or 'decentralised'"
  exit 1
fi

# Derive n-robots from yaml if not specified.
if [[ -z "$N_ROBOTS" ]]; then
  N_ROBOTS=$(python3 -c "
import yaml
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
print(len(cfg['robots']))
")
fi

# Kill any existing session cleanly.
tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true

MOCAP_CMD="$PYTHON -m real_robot.scripts.mocap_pub --config $CONFIG --server $MOCAP_SERVER"

tmux new-session -d -s "$TMUX_SESSION" -n mocap
tmux send-keys -t "$TMUX_SESSION:mocap" "$MOCAP_CMD" Enter

if [[ "$MODE" == "central" ]]; then
  CTRL_CMD="$PYTHON real_robot/laptop/central_runner.py --config $CONFIG --n-robots $N_ROBOTS --goal $GOAL"
  if $GT_PAYLOAD;    then CTRL_CMD="$CTRL_CMD --gt-payload";    fi
  if $RELATIVE_GOAL; then CTRL_CMD="$CTRL_CMD --relative-goal"; fi
  if $VIEWER;        then CTRL_CMD="$CTRL_CMD --viewer";         fi

  tmux new-window -t "$TMUX_SESSION" -n controller
  tmux send-keys -t "$TMUX_SESSION:controller" "$CTRL_CMD" Enter
  tmux select-window -t "$TMUX_SESSION:mocap"
fi

if $GOAL_SETTER; then
  GS_CMD="$PYTHON -m real_robot.laptop.goal_setter --config $CONFIG --n-robots $N_ROBOTS"
  tmux new-window -t "$TMUX_SESSION" -n goal_setter
  tmux send-keys -t "$TMUX_SESSION:goal_setter" "$GS_CMD" Enter
  tmux select-window -t "$TMUX_SESSION:mocap"
fi

echo ""
echo "Laptop session '$TMUX_SESSION' started (mode: $MODE, n-robots: $N_ROBOTS)."
if [[ "$MODE" == "central" ]]; then
  echo "  mocap:      $MOCAP_CMD"
  echo "  controller: $CTRL_CMD"
else
  echo "  mocap: $MOCAP_CMD"
fi
echo ""
echo "Attach: tmux attach -t $TMUX_SESSION"
