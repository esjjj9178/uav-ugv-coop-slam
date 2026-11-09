#!/usr/bin/env bash
# Build per-robot RTAB-Map databases from a bag and export clouds + poses.
#   ros2 run couv_map_merge run_slam.sh <bag_dir> [out_dir]
# out_dir default: ~/ros2_ws/maps/<bag name>/  ->  ugv.db, drone.db, ugv_cloud.ply, drone_cloud.ply,
#                                                 ugv_poses.txt, drone_poses.txt (stamp x y z qx qy qz qw id)
set -euo pipefail
BAG="$(realpath "${1:?usage: run_slam.sh <bag_dir> [out_dir]}")"
OUT="${2:-$HOME/ros2_ws/maps/$(basename "$BAG")}"
PITCH="$(basename "$BAG" | sed -n 's/.*_pitch\([0-9]\+\).*/\1/p')"
CFG="$(ros2 pkg prefix couv_map_merge)/share/couv_map_merge/config/slam.yaml"
read -r VOXEL DECIM MAXR < <(python3 -c "import yaml; e=yaml.safe_load(open('$CFG'))['export']; print(e['voxel'], e['decimation'], e['max_range'])")
mkdir -p "$OUT"
echo "[run_slam] bag=$BAG out=$OUT drone_cam_pitch_deg=${PITCH:-default}"
ros2 launch couv_map_merge slam_from_bag.launch.py bag:="$BAG" out_dir:="$OUT" drone_cam_pitch_deg:="$PITCH" \
  > "$OUT/slam.log" 2>&1 || true
echo "[run_slam] SLAM log: $OUT/slam.log"
for ns in ugv drone; do
  [[ -f "$OUT/$ns.db" ]] || { echo "[run_slam] missing $OUT/$ns.db" >&2; exit 1; }
  rtabmap-export --cloud --voxel "$VOXEL" --decimation "$DECIM" --max_range "$MAXR" \
    --output_dir "$OUT" --output "$ns" "$OUT/$ns.db" > "$OUT/export_$ns.log" 2>&1
  rtabmap-export --poses --poses_format 11 --output_dir "$OUT" --output "$ns" "$OUT/$ns.db" >> "$OUT/export_$ns.log" 2>&1
done
ls -la "$OUT"
