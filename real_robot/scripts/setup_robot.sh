#!/usr/bin/env bash
# One-time setup for a new robot.
#
# Run from the repo root on the laptop:
#   ./real_robot/scripts/setup_robot.sh <ip>
#
# What it does:
#   1. Copies your laptop SSH key to the robot (passwordless SSH from laptop)
#   2. Fixes ~/.ssh and home directory permissions on the robot
#   3. Copies the shared deploy key (~/.ssh/id_ed25519 from robot 0) for GitHub
#   4. Configures SSH on the robot to use port 443 for GitHub (port 22 is blocked)
#   5. Adds GitHub's host key to the robot's known_hosts
#   6. Switches the repo remote to SSH and does a test pull
#
# Prerequisites:
#   - Your laptop's ~/.ssh/id_ed25519.pub exists
#   - The deploy key has already been added to GitHub:
#       github.com/HarrisRamosBaroni/swarm-carry -> Settings -> Deploy keys
#     The key is: ubuntu@dotmyagv2's id_ed25519.pub (same key shared across all robots)
#   - Robot 0 (192.168.1.115) is already set up (it holds the canonical deploy key)

set -euo pipefail

REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_REPO="${REMOTE_REPO:-/home/ubuntu/swarm-carry}"
ROBOT_0="192.168.1.115"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot-ip>"
  exit 1
fi

IP="$1"
echo "==> Setting up $REMOTE_USER@$IP"

# 1. Copy laptop SSH key so future connections are passwordless
echo "  [1/6] copying laptop SSH key (you may need to enter the password once)"
ssh-copy-id "$REMOTE_USER@$IP"

# 2. Fix permissions (sshd ignores authorized_keys if home dir is world-writable)
echo "  [2/6] fixing ~/.ssh permissions"
ssh "$REMOTE_USER@$IP" "chmod 755 /home/ubuntu && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"

# 3. Copy shared deploy key from robot 0
echo "  [3/6] copying deploy key from robot 0 ($ROBOT_0)"
scp "$REMOTE_USER@$ROBOT_0:~/.ssh/id_ed25519" /tmp/robot_deploy_key
scp /tmp/robot_deploy_key "$REMOTE_USER@$IP:~/.ssh/id_ed25519"
ssh "$REMOTE_USER@$IP" "chmod 600 ~/.ssh/id_ed25519"
rm /tmp/robot_deploy_key

# 4. Configure SSH to reach GitHub over port 443 (port 22 blocked on robot network)
echo "  [4/6] configuring SSH for GitHub (port 443)"
ssh "$REMOTE_USER@$IP" "grep -q 'ssh.github.com' ~/.ssh/config 2>/dev/null || cat >> ~/.ssh/config << 'EOF'
Host github.com
  Hostname ssh.github.com
  Port 443
  User git
  IdentityFile ~/.ssh/id_ed25519
EOF"

# 5. Add GitHub host key
echo "  [5/6] adding GitHub host key"
ssh "$REMOTE_USER@$IP" "ssh-keyscan -p 443 ssh.github.com >> ~/.ssh/known_hosts 2>/dev/null"

# 6. Switch remote to SSH and test pull
echo "  [6/6] switching git remote to SSH and testing pull"
ssh "$REMOTE_USER@$IP" "cd $REMOTE_REPO && git remote set-url origin git@github.com:HarrisRamosBaroni/swarm-carry.git && git pull"

echo ""
echo "done. $IP is ready — deploy.sh --pull and passwordless SSH will work."
