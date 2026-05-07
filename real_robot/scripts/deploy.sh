#!/usr/bin/env bash
# Deploy and launch agent_runner on all robots defined in network.yaml.
#
# Run from the repo root on the laptop:
#   ./real_robot/scripts/deploy.sh --mode [central|decentralised] [--yaml] [--pull] [--launch] [--all]
#
# Flags (combine freely):
#   --mode central        robots run --passive, waiting for central_runner (default)
#   --mode decentralised  robots run self-driven; --neighbors computed automatically
#   --yaml                scp network.yaml to each robot
#   --pull                git pull on each robot
#   --launch              start (or restart) the tmux session on each robot
#   --reload              shorthand for --yaml --launch (yaml changed, no pull needed)
#   --all                 shorthand for --yaml --pull --launch
#   --stop                kill the tmux session on each robot (ros + agent)
#
# Extra agent args appended after mode-derived args:
#   AGENT_EXTRA_ARGS="--gbp-async" ./deploy.sh --mode decentralised --launch
#
# Each robot gets two tmux windows:
#   ros   — roslaunch myagv_ros myagv_active.launch
#   agent — agent_runner with the correct --id and mode args
#
# To attach after launch:
#   ssh ubuntu@<ip> -t tmux attach -t swarm

set -euo pipefail

CONFIG="${CONFIG:-real_robot/config/network.yaml}"
REMOTE_CONFIG="/home/ubuntu/network.yaml"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_REPO="${REMOTE_REPO:-/home/ubuntu/swarm-carry}"
TMUX_SESSION="swarm"
AGENT_EXTRA_ARGS="${AGENT_EXTRA_ARGS:-}"   # appended after mode-derived args

MODE="central"
DO_YAML=false
DO_PULL=false
DO_LAUNCH=false
DO_STOP=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --mode)   MODE="$2"; shift 2 ;;
    --yaml)   DO_YAML=true; shift ;;
    --pull)   DO_PULL=true; shift ;;
    --launch) DO_LAUNCH=true; shift ;;
    --reload) DO_YAML=true; DO_LAUNCH=true; shift ;;
    --all)    DO_YAML=true; DO_PULL=true; DO_LAUNCH=true; shift ;;
    --stop)   DO_STOP=true; shift ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

if [[ "$MODE" != "central" && "$MODE" != "decentralised" ]]; then
  echo "error: --mode must be 'central' or 'decentralised'"
  exit 1
fi

if ! $DO_YAML && ! $DO_PULL && ! $DO_LAUNCH && ! $DO_STOP; then
  echo "Usage: $0 --mode [central|decentralised] [--yaml] [--pull] [--launch] [--all]"
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

echo "mode: $MODE"

for i in "${!ROBOT_IDS[@]}"; do
  ID="${ROBOT_IDS[$i]}"
  IP="${ROBOT_IPS[$i]}"
  echo ""
  echo "==> robot $ID @ $IP"

  # Compute per-robot agent args based on mode.
  if [[ "$MODE" == "decentralised" ]]; then
    NEIGHBOR_IDS=()
    for j in "${!ROBOT_IDS[@]}"; do
      [[ "$j" -ne "$i" ]] && NEIGHBOR_IDS+=("${ROBOT_IDS[$j]}")
    done
    BASE_ARGS="--neighbors ${NEIGHBOR_IDS[*]}"
  else
    BASE_ARGS="--passive"
  fi
  ROBOT_ARGS="${BASE_ARGS}${AGENT_EXTRA_ARGS:+ $AGENT_EXTRA_ARGS}"

  if $DO_STOP; then
    echo "    [stop] killing tmux session '$TMUX_SESSION'"
    ssh "$REMOTE_USER@$IP" "tmux kill-session -t $TMUX_SESSION 2>/dev/null && echo stopped || echo not running"
  fi

  if $DO_YAML; then
    echo "    [yaml] syncing $CONFIG -> $REMOTE_USER@$IP:$REMOTE_CONFIG"
    scp "$CONFIG" "$REMOTE_USER@$IP:$REMOTE_CONFIG"
  fi

  if $DO_PULL; then
    echo "    [pull] git pull in $REMOTE_REPO"
    ssh "$REMOTE_USER@$IP" "cd $REMOTE_REPO && git pull"
  fi

  if $DO_LAUNCH; then
    echo "    [launch] starting tmux session '$TMUX_SESSION' (agent args: $ROBOT_ARGS)"
    ssh "$REMOTE_USER@$IP" bash << EOF
      tmux kill-session -t $TMUX_SESSION 2>/dev/null || true
      tmux new-session -d -s $TMUX_SESSION -n ros
      tmux send-keys -t $TMUX_SESSION:ros \
        'source /home/ubuntu/myagv_ros/devel/setup.bash && roslaunch myagv_odometry myagv_active.launch' Enter
      tmux new-window -t $TMUX_SESSION -n agent
      tmux send-keys -t $TMUX_SESSION:agent \
        'cd $REMOTE_REPO && sudo PYTHONPATH=\$PYTHONPATH python3 -m real_robot.robot.agent_runner --config $REMOTE_CONFIG --id $ID $ROBOT_ARGS' Enter
EOF
    echo "    attach: ssh $REMOTE_USER@$IP -t tmux attach -t $TMUX_SESSION"
  fi
done

echo ""
echo "done."
