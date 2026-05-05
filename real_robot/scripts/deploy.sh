#!/usr/bin/env bash
# Deploy and launch agent_runner on all robots defined in network.yaml.
#
# Run from the repo root on the laptop:
#   ./real_robot/scripts/deploy.sh [--yaml] [--pull] [--launch] [--all]
#
# Flags (combine freely):
#   --yaml    scp network.yaml to each robot
#   --pull    git pull on each robot
#   --launch  start (or restart) the tmux session on each robot
#   --all     shorthand for --yaml --pull --launch
#
# Each robot gets two tmux windows:
#   ros   — roslaunch myagv_ros myagv_active.launch
#   agent — agent_runner with the correct --id for that robot
#
# To attach after launch:
#   ssh ubuntu@<ip> -t tmux attach -t swarm

set -euo pipefail

CONFIG="${CONFIG:-real_robot/config/network.yaml}"
REMOTE_CONFIG="/home/ubuntu/network.yaml"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_REPO="${REMOTE_REPO:-/home/ubuntu/swarm-carry}"
TMUX_SESSION="swarm"
AGENT_EXTRA_ARGS="${AGENT_EXTRA_ARGS:---passive}"   # override: AGENT_EXTRA_ARGS="--horizon 10" ./deploy.sh

DO_YAML=false
DO_PULL=false
DO_LAUNCH=false

for arg in "$@"; do
  case $arg in
    --yaml)   DO_YAML=true ;;
    --pull)   DO_PULL=true ;;
    --launch) DO_LAUNCH=true ;;
    --all)    DO_YAML=true; DO_PULL=true; DO_LAUNCH=true ;;
    *) echo "unknown arg: $arg"; exit 1 ;;
  esac
done

if ! $DO_YAML && ! $DO_PULL && ! $DO_LAUNCH; then
  echo "Usage: $0 [--yaml] [--pull] [--launch] [--all]"
  exit 1
fi

# Parse robot ids and IPs from yaml
mapfile -t ROBOT_IDS < <(python3 -c "
import yaml
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
for r in cfg['robots']:
    print(r['id'])
")
mapfile -t ROBOT_IPS < <(python3 -c "
import yaml
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
for r in cfg['robots']:
    print(r['ip'])
")

for i in "${!ROBOT_IDS[@]}"; do
  ID="${ROBOT_IDS[$i]}"
  IP="${ROBOT_IPS[$i]}"
  echo ""
  echo "==> robot $ID @ $IP"

  if $DO_YAML; then
    echo "    [yaml] syncing $CONFIG -> $REMOTE_USER@$IP:$REMOTE_CONFIG"
    scp "$CONFIG" "$REMOTE_USER@$IP:$REMOTE_CONFIG"
  fi

  if $DO_PULL; then
    echo "    [pull] git pull in $REMOTE_REPO"
    ssh "$REMOTE_USER@$IP" "cd $REMOTE_REPO && git pull"
  fi

  if $DO_LAUNCH; then
    echo "    [launch] starting tmux session '$TMUX_SESSION'"
    ssh "$REMOTE_USER@$IP" bash << EOF
      tmux kill-session -t $TMUX_SESSION 2>/dev/null || true
      tmux new-session -d -s $TMUX_SESSION -n ros
      tmux send-keys -t $TMUX_SESSION:ros \
        'source /opt/ros/noetic/setup.bash && roslaunch myagv_ros myagv_active.launch' Enter
      tmux new-window -t $TMUX_SESSION -n agent
      tmux send-keys -t $TMUX_SESSION:agent \
        'cd $REMOTE_REPO && sudo PYTHONPATH=\$PYTHONPATH python3 -m real_robot.robot.agent_runner --config $REMOTE_CONFIG --id $ID $AGENT_EXTRA_ARGS' Enter
EOF
    echo "    attach: ssh $REMOTE_USER@$IP -t tmux attach -t $TMUX_SESSION"
  fi
done

echo ""
echo "done."
