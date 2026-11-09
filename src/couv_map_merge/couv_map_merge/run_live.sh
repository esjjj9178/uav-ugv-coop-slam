#!/usr/bin/env bash
# Run the serpentine ArUco mission with live per-robot SLAM (no image recording), then export.
#   ros2 run couv_map_merge run_live.sh [out_dir]      (default ~/ros2_ws/maps/serpentine_<date>)
# Output: <robot>.db, <robot>_cloud.ply, <robot>_poses.txt (base), <robot>_camera_poses.txt,
#         <robot>_rgb/ <robot>_depth/ <robot>_calib/ (per-node images), aux_bag/, images/, *.log
set -uo pipefail
# job control: background jobs keep the default SIGINT handling (otherwise they inherit SIG_IGN and
# launch cannot stop the bag recorder / python nodes cleanly)
set -m
OUT="${1:-$HOME/ros2_ws/maps/serpentine_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
CFG="$(ros2 pkg prefix couv_map_merge)/share/couv_map_merge/config/slam.yaml"
read -r VOXEL DECIM MAXR < <(python3 -c "import yaml; e=yaml.safe_load(open('$CFG'))['export']; print(e['voxel'], e['decimation'], e['max_range'])")
echo "[run_live] output: $OUT"

ros2 launch couv_sim_robots sim.launch.py gui:=false rviz:=false gt_tf:=false wheel_odom_tf:=false \
  pointcloud:=false compress:=false drone_cam_pitch_deg:=45 ugv_spawn_x:=-8 ugv_spawn_y:=-6.5 ugv_spawn_yaw:=0 \
  > "$OUT/sim.log" 2>&1 &
SIM=$!
sleep 25
ros2 launch couv_map_merge slam_live.launch.py out_dir:="$OUT" > "$OUT/slam.log" 2>&1 &
SLAM=$!
sleep 10
ros2 run couv_sim_robots mission_aruco.py --ros-args -p use_sim_time:=true > "$OUT/mission.log" 2>&1 &
MIS=$!
echo "[run_live] mission running (log: $OUT/mission.log)"
while kill -0 $MIS 2>/dev/null; do
  grep -qE "MISSION COMPLETE|aborted" "$OUT/mission.log" && break
  sleep 5
done
grep -E "state ->|MISSION" "$OUT/mission.log" | sed 's/.*\]: //'
sleep 3
pkill -TERM -f "lib/couv_sim_robots/mission_aruco.py" 2>/dev/null; kill -TERM $MIS 2>/dev/null
# on SIGTERM ros2 launch sends SIGINT to its nodes (rtabmap stores the databases, the bag is closed)
kill -TERM $SLAM; wait $SLAM 2>/dev/null
kill -TERM $SIM; for _ in $(seq 20); do kill -0 $SIM 2>/dev/null || break; sleep 1; done
kill -9 $SIM 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -TERM -f "lib/(ros_gz_bridge/parameter_bridge|robot_state_publisher/robot_state_publisher)" 2>/dev/null
pkill -TERM -f "lib/couv_map_merge/aruco_detector.py" 2>/dev/null
[[ -f "$OUT/aux_bag/metadata.yaml" ]] || ros2 bag reindex "$OUT/aux_bag" sqlite3 > /dev/null 2>&1
grep -E "aruco_(start|end):" "$OUT/slam.log" | sed 's/.*\]: //'

for ns in ugv drone; do
  [[ -f "$OUT/$ns.db" ]] || { echo "[run_live] missing $OUT/$ns.db" >&2; exit 1; }
  rtabmap-export --cloud --voxel "$VOXEL" --decimation "$DECIM" --max_range "$MAXR" --output_dir "$OUT" --output "$ns" "$OUT/$ns.db" > "$OUT/export_$ns.log" 2>&1
  rtabmap-export --poses --poses_format 11 --output_dir "$OUT" --output "$ns" "$OUT/$ns.db" >> "$OUT/export_$ns.log" 2>&1
  rtabmap-export --images_id --poses_camera --poses_format 11 --output_dir "$OUT" --output "$ns" "$OUT/$ns.db" >> "$OUT/export_$ns.log" 2>&1
done
du -sh "$OUT"; ls "$OUT"
