#!/usr/bin/env bash
set -euo pipefail

seed="$1"
world="$2"
gui="$3"
package_root="$(rospack find firefighting_mission)"
sdf="${4:-$package_root/models/fire_iris/fire_iris.sdf}"
spawn_z="${5:-0.2}"
px4_root="${PX4_FIRMWARE_DIR:-/home/ss/PX4_Firmware}"
export ROS_PACKAGE_PATH="$px4_root:$px4_root/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
gazebo_system_plugin_path="/usr/lib/x86_64-linux-gnu/gazebo-9/plugins"
export GAZEBO_PLUGIN_PATH="/opt/ros/melodic/lib:$gazebo_system_plugin_path:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$package_root/models:${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="$gazebo_system_plugin_path:${LD_LIBRARY_PATH:-}"
source "$px4_root/Tools/setup_gazebo.bash" "$px4_root" \
  "$px4_root/build/px4_sitl_default"

python "$package_root/scripts/generate_world.py" --seed "$seed" --output "$world"
exec roslaunch "$px4_root/launch/mavros_posix_sitl.launch" vehicle:=iris "world:=$world" \
  "sdf:=$sdf" "gui:=$gui" "z:=$spawn_z" interactive:=false \
  "verbose:=${GAZEBO_VERBOSE:-false}"
