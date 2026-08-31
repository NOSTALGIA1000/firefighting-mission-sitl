#!/bin/bash
# Stop every simulation process and do not return until the machine is clean.
PATTERN="gzserver|gzclient|bin/px4|px4-simulator|roslaunch|rosmaster|mavros_node|rosout"
for attempt in 1 2 3 4 5 6; do
  remaining=$(pgrep -f "$PATTERN" | grep -v "^$$\$" | wc -l)
  if [ "$remaining" -eq 0 ]; then
    echo "clean after $((attempt-1)) rounds"
    exit 0
  fi
  if [ "$attempt" -le 2 ]; then
    pkill -f "$PATTERN"
  else
    pkill -9 -f "$PATTERN"
  fi
  sleep 4
done
echo "STILL RUNNING:"
pgrep -a -f "$PATTERN" | cut -c1-70
exit 1
