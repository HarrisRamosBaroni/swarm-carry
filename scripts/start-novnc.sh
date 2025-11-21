#!/bin/bash
# Start noVNC web interface for browser-based VNC access

echo "=== Starting noVNC Web Interface ==="

# Check if VNC server is running
if ! pgrep -x Xtigervnc > /dev/null; then
    echo "Error: VNC server not running!"
    echo "Start it first: ./scripts/start-vnc.sh"
    exit 1
fi

# Check if noVNC is already running
if pgrep -f websockify.*6080 > /dev/null; then
    echo "noVNC already running on port 6080"
    echo "Access at: http://localhost:6080"
    exit 0
fi

DISPLAY_NUM=${VNC_DISPLAY:-1}
VNC_PORT=$((5900 + DISPLAY_NUM))

echo "Connecting to VNC display :${DISPLAY_NUM} (port ${VNC_PORT})"
echo ""

# Start websockify (noVNC)
websockify --web=/usr/share/novnc 6080 localhost:${VNC_PORT} &

echo ""
echo "✓ noVNC started!"
echo ""
echo "Access GUI in browser: http://localhost:6080/vnc.html"
echo "Password: vncpass"
echo ""
echo "To stop: pkill -f 'websockify.*6080'"
