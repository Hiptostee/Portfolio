#!/bin/bash
# ----------------------------
# start.sh: launch VNC with working xterm startup
# ----------------------------

# Kill any old VNC sessions
vncserver -kill :1 >/dev/null 2>&1 || true

# Start VNC server with xterm directly as suggested in logs
tigervncserver :1 -geometry 1280x800 -depth 24 -localhost no -xstartup /usr/bin/xterm

# Keep container alive
tail -f /dev/null