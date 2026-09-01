#!/bin/bash
# Repeat the full avoidance chain N times and record one line per run.
# Results accumulate as they happen, so a partial campaign is still usable.
RUNS=${1:-10}
RESULTS=${RESULTS:-/home/ss/campaign_results.txt}
: > "$RESULTS"

for index in $(seq 1 "$RUNS"); do
  bash $(dirname "$0")/sim_teardown.sh > /dev/null 2>&1
  sleep 4
  log=/tmp/campaign_${index}.log
  out=/tmp/campaign_${index}.out
  rm -f "$log" "$out"
  setsid ${DEMO_LAUNCHER:-/home/ss/run_avoid_demo.sh} < /dev/null > "$log" 2>&1 &
  sleep 45

  source /opt/ros/melodic/setup.bash
  source /home/ss/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  export ROS_HOSTNAME=127.0.0.1
  # Worst case is the 120 s hover wait plus three 100 s legs, so 400 killed
  # the driver before it printed anything and the run scored driver_failed.
  # That ate two of ten runs in one campaign.
  timeout 600 python -u $(dirname "$0")/fly_avoidance_chain.py "$index" > "$out" 2>&1

  line=$(grep '^RESULT' "$out" | tail -1)
  [ -z "$line" ] && line="RESULT run=$index outcome=driver_failed"

  gz=$(ps -eo args --no-headers | grep -cE '[g]zserver')
  blind=$(grep -ac 'blind land' "$log")
  lock=$(grep -ac 'Critical failure detected: lockdown' "$log")
  nooff=$(grep -ac 'no RC and no offboard' "$log")
  resets=$(grep -ac 'reset position to ev' "$log")
  ulog=$(find /home/ss/.ros -name '*.ulg' 2>/dev/null | xargs ls -t 2>/dev/null | head -1)

  echo "$line gzserver_alive=$gz blind_land=$blind lockdown=$lock no_offboard=$nooff ev_resets=$resets ulog=$ulog" >> "$RESULTS"
  echo "[$(date +%H:%M:%S)] run $index/$RUNS done"
done

bash $(dirname "$0")/sim_teardown.sh > /dev/null 2>&1
echo "campaign finished"
