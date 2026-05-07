#!/usr/bin/env bash
# Launch laptop-side processes for a swarm experiment.
#
# Run from the repo root:
#   ./real_robot/scripts/launch.sh --mode central
#   ./real_robot/scripts/launch.sh --mode central --goal 2 0 0
#   ./real_robot/scripts/launch.sh --mode decentralised
#
# central:       tmux session "swarm-laptop" with three windows:
#                  mocap          — mocap_pub streaming poses from PhaseSpace
#                  controller     — central_runner sending commands to all robots
#                  control_panel  — live map, goal placement, and E-stop
# decentralised: tmux session "swarm-laptop" with two windows:
#                  mocap          — mocap_pub only (robots self-drive)
#                  control_panel  — live map, goal placement, and E-stop
#
# Options:
#   --mode central|decentralised   (required)
#   --goal X Y theta               pre-set initial goal in metres/rad (central only);
#                                  omit to hold until control_panel sends the first goal
#   --n-robots N                   number of robots (default: from network.yaml)
#   --server IP                    PhaseSpace server IP (default: 192.168.1.25)
#   --config PATH                  network.yaml path (default: real_robot/config/network.yaml)
#   --gt-payload                   use live mocap payload pose (central only)
#   --relative-goal                treat --goal as offset from initial centroid (central only)
#   --no-control-panel             skip the control_panel window (e.g. if setting goal via CLI)
#
# Attach: tmux attach -t swarm-laptop
# Kill:   tmux kill-session -t swarm-laptop

set -euo pipefail

CONFIG="${CONFIG:-real_robot/config/network.yaml}"
MOCAP_SERVER="${MOCAP_SERVER:-192.168.1.25}"
PYTHON="${PYTHON:-python3}"
TMUX_SESSION="swarm-laptop"

MODE=""
GOAL=""
N_ROBOTS=""
GT_PAYLOAD=false
RELATIVE_GOAL=false
CONTROL_PANEL=true

while [[ $# -gt 0 ]]; do
  case $1 in
    --mode)              MODE="$2"; shift 2 ;;
    --goal)              GOAL="$2 $3 $4"; shift 4 ;;
    --n-robots)          N_ROBOTS="$2"; shift 2 ;;
    --server)            MOCAP_SERVER="$2"; shift 2 ;;
    --config)            CONFIG="$2"; shift 2 ;;
    --gt-payload)        GT_PAYLOAD=true; shift ;;
    --relative-goal)     RELATIVE_GOAL=true; shift ;;
    --no-control-panel)  CONTROL_PANEL=false; shift ;;
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
  CTRL_CMD="$PYTHON real_robot/laptop/central_runner.py --config $CONFIG --n-robots $N_ROBOTS"
  if [[ -n "$GOAL" ]];    then CTRL_CMD="$CTRL_CMD --goal $GOAL";     fi
  if $GT_PAYLOAD;         then CTRL_CMD="$CTRL_CMD --gt-payload";      fi
  if $RELATIVE_GOAL;      then CTRL_CMD="$CTRL_CMD --relative-goal";   fi

  tmux new-window -t "$TMUX_SESSION" -n controller
  tmux send-keys -t "$TMUX_SESSION:controller" "$CTRL_CMD" Enter
  tmux select-window -t "$TMUX_SESSION:mocap"
fi

if $CONTROL_PANEL; then
  CP_CMD="$PYTHON -m real_robot.laptop.control_panel --config $CONFIG --n-robots $N_ROBOTS"
  tmux new-window -t "$TMUX_SESSION" -n control_panel
  tmux send-keys -t "$TMUX_SESSION:control_panel" "$CP_CMD" Enter
  tmux select-window -t "$TMUX_SESSION:mocap"
fi

echo ""
echo "Laptop session '$TMUX_SESSION' started (mode: $MODE, n-robots: $N_ROBOTS)."
echo "  mocap: $MOCAP_CMD"
if [[ "$MODE" == "central" ]]; then
  echo "  controller: $CTRL_CMD"
fi
if $CONTROL_PANEL; then
  echo "  control_panel: $CP_CMD"
fi
echo ""
echo "Attach: tmux attach -t $TMUX_SESSION"
