#!/usr/bin/env bash
# Power off all robots defined in network.yaml.
#
# Run from the repo root on the laptop:
#   ./real_robot/scripts/poweroff.sh

set -euo pipefail

CONFIG="${CONFIG:-real_robot/config/network.yaml}"
REMOTE_USER="${REMOTE_USER:-ubuntu}"

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

echo "Powering off ${#ROBOT_IDS[@]} robot(s). Press Ctrl-C within 3 seconds to cancel."
sleep 3

for i in "${!ROBOT_IDS[@]}"; do
  ID="${ROBOT_IDS[$i]}"
  IP="${ROBOT_IPS[$i]}"
  echo "==> robot $ID @ $IP — shutting down"
  ssh "$REMOTE_USER@$IP" "sudo poweroff" || true
done

echo ""
echo "Shutdown commands sent."
