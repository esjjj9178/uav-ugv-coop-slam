#!/usr/bin/env bash
# Watch a running run_live.sh: Gazebo GUI attached to the running server + RViz with both live maps.
#   ros2 run couv_map_merge watch_live.sh
# The drone map is shown at T0 relative to the UGV map (both maps start with the robots on the pad).
SHARE_R="$(ros2 pkg prefix couv_sim_robots)/share"
SHARE_M="$(ros2 pkg prefix couv_map_merge)/share/couv_map_merge"
read -r X Y Z YAW < <(python3 -c "import yaml; t=yaml.safe_load(open('$SHARE_R/couv_sim_robots/config/frames.yaml'))['landing_pad_offset']; print(t['x'], t['y'], t['z'], t['yaw'])")
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${SHARE_R}"
gz sim -g > /dev/null 2>&1 &
GUI=$!
ros2 run tf2_ros static_transform_publisher --x "$X" --y "$Y" --z "$Z" --yaw "$YAW" \
  --frame-id ugv/map --child-frame-id drone/map --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
TF=$!
rviz2 -d "$SHARE_M/rviz/live.rviz" --ros-args -p use_sim_time:=true > /dev/null 2>&1
kill $GUI $TF 2>/dev/null
pkill -TERM -f "static_transform_publisher.*drone/map" 2>/dev/null
