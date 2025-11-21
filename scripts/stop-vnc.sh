#!/bin/bash
# Stop VNC server and noVNC

echo "=== Stopping VNC Services ==="

DISPLAY_NUM=${VNC_DISPLAY:-1}

# Stop noVNC
if pgrep -f websockify.*6080 > /dev/null; then
    echo "Stopping noVNC..."
    pkill -f 'websockify.*6080'
fi

# Stop VNC server
if [ -f /tmp/.X${DISPLAY_NUM}-lock ]; then
    echo "Stopping VNC server on :${DISPLAY_NUM}..."
    vncserver -kill :${DISPLAY_NUM}
fi

echo ""
echo "✓ VNC services stopped"
