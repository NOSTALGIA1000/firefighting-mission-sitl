#!/usr/bin/env bash
set -euo pipefail

package_root="$(rospack find firefighting_mission)"
seeds=(1 4 10 2)
export DISPLAY="${DISPLAY:-:0}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

for seed in "${seeds[@]}"; do
  timeout 150s rostest firefighting_mission visual_avoidance_smoke.test \
    "seed:=$seed"
  python - "$package_root/artifacts/avoidance_matrix/$seed/smoke.json" <<'PY'
from __future__ import print_function
import json
import sys

with open(sys.argv[1], 'r') as handle:
    evidence = json.load(handle)
required = set(('BRAKE', 'OBSERVE', 'SELECT_SIDE', 'SIDESTEP', 'PASS', 'REJOIN'))
assert evidence['collision'] is False, evidence
assert evidence['reached_goal'] is True, evidence
assert evidence['minimum_transit_altitude'] >= 1.10, evidence
assert evidence['maximum_transit_altitude'] <= 1.30, evidence
assert set(evidence['states']).issuperset(required), evidence
print('PASS seed=%s states=%s altitude=%.3f..%.3f' % (
    evidence['seed'], ','.join(evidence['states']),
    evidence['minimum_transit_altitude'], evidence['maximum_transit_altitude']))
PY
done
