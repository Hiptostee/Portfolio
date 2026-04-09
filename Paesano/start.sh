#!/bin/bash
# ----------------------------
# start.sh: launch VNC in the foreground
# ----------------------------

# Kill any old VNC sessions just in case
vncserver -kill :1 >/dev/null 2>&1 || true

# Start VNC server in the foreground.
exec /usr/bin/vncserver :1 -geometry 1280x800 -depth 24 -localhost no -fg