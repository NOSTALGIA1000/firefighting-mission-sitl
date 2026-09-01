#!/bin/bash
# Stop every simulation process and do not return until the machine is clean.
PATTERN="gzserver|gzclient|bin/px4|px4-simulator|roslaunch|rosmaster|mavros_node|rosout"
# PX4 refuses to start while a lock from a killed instance is left behind:
# "PX4 daemon already running for instance 0", and roslaunch then reports the
# required sitl node dead with exit 255 before Gazebo ever comes up.
clean_px4_locks() {
  rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null
}

# roslaunch scans ~/.ros/log on every start, and a campaign leaves a session
# directory per run.  Once that reaches a couple of gigabytes gzserver starts
# aborting at launch (exit 134) before Gazebo ever comes up, which shows up
# as runs lost to infrastructure rather than to the thing being measured.
trim_ros_logs() {
  local keep=12
  local dir="$HOME/.ros/log"
  [ -d "$dir" ] || return 0
  ls -1dt "$dir"/*/ 2>/dev/null | tail -n +$((keep + 1)) | while read -r old; do
    rm -rf "$old"
  done
}

for attempt in 1 2 3 4 5 6; do
  remaining=$(pgrep -f "$PATTERN" | grep -v "^$$\$" | wc -l)
  if [ "$remaining" -eq 0 ]; then
    clean_px4_locks
    trim_ros_logs
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
clean_px4_locks
echo "STILL RUNNING:"
pgrep -a -f "$PATTERN" | cut -c1-70
exit 1
