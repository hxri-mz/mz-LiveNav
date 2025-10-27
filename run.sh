#!/bin/bash
set -e

python3 gnss_bridge.py >/dev/null 2>&1 &
GNSS_PID=$!

python3 live_nav.py >/dev/null 2>&1 &
NAV_PID=$!

nohup xdg-open "http://127.0.0.1:5000/static/index.html" >/dev/null 2>&1 &

echo "   Servers started!"
echo "   GNSS PID: $GNSS_PID"
echo "   LiveNav PID: $NAV_PID"
echo "-------------------------------------"
echo " Press [Ctrl + X] to stop both servers"
echo " Press [Ctrl + C] to quit watcher"
echo "-------------------------------------"

trap 'echo -e "\nExiting watcher (servers still running)..."; exit 0' INT

while true; do
  read -rsn1 key < /dev/tty 2>/dev/null || continue

  if [[ "$key" == $'\x18' ]]; then
    echo ""
    echo "Stopping servers..."
    kill "$GNSS_PID" "$NAV_PID" 2>/dev/null || true
    pkill -f gnss_bridge.py 2>/dev/null || true
    pkill -f live_nav.py 2>/dev/null || true
    echo "All stopped!"
    break
  fi
done
