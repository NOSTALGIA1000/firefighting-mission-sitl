#!/usr/bin/env bash
set -euo pipefail

seed="${1:-4501}"
record="${RECORD:-true}"
roslaunch firefighting_mission firefighting_headless.launch "seed:=${seed}" "record:=${record}"
