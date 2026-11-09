"""Per-robot RTAB-Map running live next to the simulation (no image recording), plus the
ArUco detector and a small log bag (ground truth, rangefinder, mission state, marker poses).

    ros2 launch couv_map_merge slam_live.launch.py out_dir:=<dir>

Start couv_sim_robots sim.launch.py with wheel_odom_tf:=false first (rgbd_odometry owns
<robot>/vo_odom -> <robot>/base_link). Stop with Ctrl-C: rtabmap stores <out_dir>/<robot>.db.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOG_TOPICS = ["/ugv/ground_truth/odom", "/drone/ground_truth/odom", "/drone/rangefinder", "/mission/state",
              "/drone/aruco/marker_pose", "/ugv/vo_odom", "/drone/vo_odom", "/clock"]


def setup(context):
    cfg = yaml.safe_load(open(os.path.join(get_package_share_directory("couv_map_merge"), "config", "slam.yaml")))
    out_dir = os.path.abspath(os.path.expanduser(LaunchConfiguration("out_dir").perform(context)))
    os.makedirs(out_dir, exist_ok=True)
    sim_time = {"use_sim_time": True}
    actions = []
    for ns in cfg["robots"]:
        cam = f"/{ns}/camera"
        # visual odometry on /<robot>/vo_odom: /ugv/odom is already the wheel odometry from Gazebo
        remaps = [("rgb/image", f"{cam}/color/image_raw"), ("depth/image", f"{cam}/depth/image_rect_raw"),
                  ("rgb/camera_info", f"{cam}/color/camera_info"), ("odom", f"/{ns}/vo_odom")]
        common = {"frame_id": f"{ns}/base_link", "approx_sync": False, "wait_for_transform": 0.5,
                  "topic_queue_size": 30, "sync_queue_size": 30, **sim_time}
        actions += [
            Node(package="rtabmap_odom", executable="rgbd_odometry", namespace=ns, name="rgbd_odometry",
                 output="screen", remappings=remaps,
                 parameters=[{**common, "odom_frame_id": f"{ns}/vo_odom", "publish_tf": True, **cfg["odometry"]}]),
            Node(package="rtabmap_slam", executable="rtabmap", namespace=ns, name="rtabmap", output="screen",
                 arguments=["-d"], remappings=remaps,
                 parameters=[{**common, "map_frame_id": f"{ns}/map", "subscribe_depth": True,
                              "database_path": os.path.join(out_dir, f"{ns}.db"), **cfg["slam"]}]),
        ]
    actions += [
        Node(package="couv_map_merge", executable="aruco_detector.py", name="aruco_detector", output="screen",
             parameters=[{"image_dir": os.path.join(out_dir, "images"), **sim_time}]),
        ExecuteProcess(cmd=["ros2", "bag", "record", "-s", "sqlite3", "--use-sim-time",
                            "-o", os.path.join(out_dir, "aux_bag"), *LOG_TOPICS], output="log"),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("out_dir", description="output directory (databases, log bag, images)"),
        OpaqueFunction(function=setup),
    ])
