#!/bin/bash
# Start VNC server for GUI access

echo "=== Starting VNC Server ==="

# Check if VNC is already running
if [ -f /tmp/.X1-lock ]; then
    echo "VNC server already running on :1"
    echo "To restart, run: vncserver -kill :1 && $0"
    exit 0
fi

# Set default resolution if not provided
RESOLUTION=${VNC_RESOLUTION:-1920x1080}
DISPLAY_NUM=${VNC_DISPLAY:-1}

echo "Starting VNC on display :${DISPLAY_NUM}"
echo "Resolution: ${RESOLUTION}"
echo "Password: vncpass (default)"
echo ""

# Start VNC server
vncserver :${DISPLAY_NUM} \
    -geometry ${RESOLUTION} \
    -depth 24 \
    -localhost no

echo ""
echo "✓ VNC Server started!"
echo ""
echo "Connection options:"
echo "  1. VNC Client: localhost:590${DISPLAY_NUM}"
echo "     - Password: vncpass"
echo "     - Clients: TigerVNC, RealVNC, Remmina"
echo ""
echo "  2. noVNC (browser): http://localhost:6080"
echo "     - Run: ./scripts/start-novnc.sh"
echo ""
echo "To stop VNC: vncserver -kill :${DISPLAY_NUM}"
