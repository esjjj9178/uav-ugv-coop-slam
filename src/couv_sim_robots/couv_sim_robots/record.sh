#!/usr/bin/env bash
# Record the couv_sim_robots topics to rosbag2 (mcap) for the RTAB-Map / map-merging stage.
#
#   ros2 run couv_sim_robots record.sh [options]
#     --raw              record raw images instead of the compressed streams (~130 MB per sim second!)
#     --points           also record .../camera/depth/color/points (very large)
#     --pitch DEG        drone camera pitch for the file name (default: read from /tf_static)
#     --out DIR          output directory (default: recording.yaml output_dir)
#     --until-complete   stop automatically when /mission/state becomes "complete" or "aborted"
#
# Topic list, storage and output dir come from config/recording.yaml.
# Bag name: couv_<YYYYmmdd_HHMMSS>_pitch<DEG>[_raw][_points]
set -euo pipefail

RAW=0; POINTS=0; PITCH=""; OUT=""; UNTIL=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --raw) RAW=1 ;;
    --points) POINTS=1 ;;
    --pitch) PITCH="$2"; shift ;;
    --out) OUT="$2"; shift ;;
    --until-complete) UNTIL=1 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 1 ;;
  esac
  shift
done

SHARE="$(ros2 pkg prefix couv_sim_robots)/share/couv_sim_robots"
CFG="$SHARE/config/recording.yaml"

# --- topics / storage from recording.yaml
mapfile -t TOPICS < <(python3 - "$CFG" "$RAW" "$POINTS" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])); raw, points = sys.argv[2] == "1", sys.argv[3] == "1"
rt = cfg["robot_topics"]
topics = list(cfg["common_topics"])
for ns, extra in cfg["robots"].items():
    groups = rt["always"] + (rt["raw"] if raw else rt["compressed"]) + (rt["points"] if points else [])
    topics += [t.format(ns=ns) for t in groups] + extra
print("\n".join(dict.fromkeys(topics)))
PY
)
read -r OUTCFG STORAGE PRESET < <(python3 -c "import yaml,sys; c=yaml.safe_load(open('$CFG')); print(c['output_dir'], c['storage'], c['storage_preset'])")
OUT="${OUT:-$OUTCFG}"; OUT="${OUT/#\~/$HOME}"

if [[ "$STORAGE" == "mcap" ]] && ! ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  echo "[record.sh] WARNING: rosbag2_storage_mcap not installed -> using sqlite3." >&2
  echo "[record.sh]          install: sudo apt install ros-humble-rosbag2-storage-mcap" >&2
  STORAGE=sqlite3
fi
STORAGE_ARGS=(-s "$STORAGE")
[[ "$STORAGE" == "mcap" ]] && STORAGE_ARGS+=(--storage-preset-profile "$PRESET")

# --- drone camera pitch (from the static TF published by robot_state_publisher)
if [[ -z "$PITCH" ]]; then
  PITCH=$(timeout 15 python3 - <<'PY' || true
import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = Node("record_pitch_probe"); found = []
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "drone/base_link" and t.child_frame_id == "drone/camera_link":
            q = t.transform.rotation
            found.append(math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))))
n.create_subscription(TFMessage, "/tf_static", cb, QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while rclpy.ok() and not found:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"{found[0]:.0f}")
PY
)
  PITCH="${PITCH:-unknown}"
fi

NAME="couv_$(date +%Y%m%d_%H%M%S)_pitch${PITCH}"
[[ $RAW == 1 ]] && NAME+="_raw"
[[ $POINTS == 1 ]] && NAME+="_points"
mkdir -p "$OUT"
echo "[record.sh] bag: $OUT/$NAME  storage: ${STORAGE_ARGS[*]}  topics: ${#TOPICS[@]}"
printf '  %s\n' "${TOPICS[@]}"

ros2 bag record "${STORAGE_ARGS[@]}" -o "$OUT/$NAME" --use-sim-time "${TOPICS[@]}" &
REC=$!
trap 'kill -INT $REC 2>/dev/null || true; wait $REC 2>/dev/null || true' INT TERM

if [[ $UNTIL == 1 ]]; then
  python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = Node("record_until_complete"); done = []
n.create_subscription(String, "/mission/state", lambda m: m.data in ("complete", "aborted") and done.append(m.data),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while rclpy.ok() and not done:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"[record.sh] mission {done[0]} -> stopping recorder")
PY
  sleep 1
  kill -INT $REC
fi
wait $REC || true
echo "[record.sh] done: $OUT/$NAME"
du -sh "$OUT/$NAME" || true
