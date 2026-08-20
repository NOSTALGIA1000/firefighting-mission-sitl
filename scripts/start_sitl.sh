#!/usr/bin/env bash
set -euo pipefail

seed="$1"
world="$2"
gui="$3"
package_root="$(rospack find firefighting_mission)"
px4_root="${PX4_FIRMWARE_DIR:-/home/ss/PX4_Firmware}"
export ROS_PACKAGE_PATH="$px4_root:$px4_root/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "$px4_root/Tools/setup_gazebo.bash" "$px4_root" \
  "$px4_root/build/px4_sitl_default"

python "$package_root/scripts/generate_world.py" --seed "$seed" --output "$world"
exec roslaunch "$px4_root/launch/mavros_posix_sitl.launch" vehicle:=iris "world:=$world" \
  "sdf:=$package_root/models/fire_iris/fire_iris.sdf" "gui:=$gui" interactive:=false
