"""Build one RTAB-Map database per robot from a couv_sim_robots bag.

    ros2 launch couv_map_merge slam_from_bag.launch.py bag:=<bag dir> out_dir:=<dir> [drone_cam_pitch_deg:=90]

Writes <out_dir>/ugv.db and <out_dir>/drone.db, then shuts down when the bag has been played.
Use scripts run_slam.sh to also export the clouds and poses.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    cfg = yaml.safe_load(open(os.path.join(get_package_share_directory("couv_map_merge"), "config", "slam.yaml")))
    robots_share = get_package_share_directory("couv_sim_robots")
    bag, out_dir = os.path.abspath(os.path.expanduser(arg("bag"))), os.path.abspath(os.path.expanduser(arg("out_dir")))
    os.makedirs(out_dir, exist_ok=True)
    sim_time = {"use_sim_time": True}
    pitch = arg("drone_cam_pitch_deg")
    urdf = {
        "ugv": xacro.process_file(os.path.join(robots_share, "urdf", "ugv.urdf.xacro")).toxml(),
        "drone": xacro.process_file(os.path.join(robots_share, "urdf", "drone.urdf.xacro"),
                                    mappings={"cam_pitch_deg": pitch} if pitch else {}).toxml(),
    }
    actions = [LogInfo(msg=f"[couv_map_merge] bag {bag} -> {out_dir} (rate {cfg['playback_rate']})")]
    topics = []
    for ns in cfg["robots"]:
        topics += [t.format(ns=ns) for t in cfg["play_topics"]]
        cam = f"/{ns}/camera"
        rgb, depth, info = f"{cam}/color/image_raw", f"{cam}/depth/image_rect_raw", f"{cam}/color/camera_info"
        remaps = [("rgb/image", rgb), ("depth/image", depth), ("rgb/camera_info", info)]
        actions += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns,
                 parameters=[{"robot_description": urdf[ns], "frame_prefix": f"{ns}/", **sim_time}]),
            Node(package="image_transport", executable="republish", namespace=ns, name="color_decompress",
                 arguments=["compressed", "raw"], parameters=[sim_time],
                 remappings=[("in/compressed", f"{rgb}/compressed"), ("out", rgb)]),
            Node(package="image_transport", executable="republish", namespace=ns, name="depth_decompress",
                 arguments=["compressedDepth", "raw"], parameters=[sim_time],
                 remappings=[("in/compressedDepth", f"{depth}/compressedDepth"), ("out", depth)]),
            Node(package="rtabmap_odom", executable="rgbd_odometry", namespace=ns, name="rgbd_odometry",
                 output="screen", remappings=remaps,
                 parameters=[{"frame_id": f"{ns}/base_link", "odom_frame_id": f"{ns}/vo_odom", "publish_tf": True,
                              "approx_sync": False, "wait_for_transform": 0.5, "topic_queue_size": 30,
                              "sync_queue_size": 30, **sim_time, **cfg["odometry"]}]),
            Node(package="rtabmap_slam", executable="rtabmap", namespace=ns, name="rtabmap", output="screen",
                 arguments=["-d"], remappings=remaps + [("odom", "odom")],
                 parameters=[{"frame_id": f"{ns}/base_link", "map_frame_id": f"{ns}/map", "subscribe_depth": True,
                              "approx_sync": False, "wait_for_transform": 0.5, "topic_queue_size": 30,
                              "sync_queue_size": 30, "database_path": os.path.join(out_dir, f"{ns}.db"),
                              **sim_time, **cfg["slam"]}]),
        ]
    play = ExecuteProcess(cmd=["ros2", "bag", "play", bag, "--clock", "200", "-r", str(cfg["playback_rate"]),
                               "--topics", *topics], output="screen")
    actions += [
        TimerAction(period=4.0, actions=[play]),  # let the nodes come up first
        RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[
            LogInfo(msg="[couv_map_merge] bag finished, storing databases"),
            TimerAction(period=float(cfg["shutdown_delay"]), actions=[EmitEvent(event=Shutdown(reason="bag done"))])])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="bag directory recorded by couv_sim_robots record.sh"),
        DeclareLaunchArgument("out_dir", description="output directory for ugv.db / drone.db"),
        DeclareLaunchArgument("drone_cam_pitch_deg", default_value="",
                              description="drone camera pitch used for this bag (default: frames.yaml)"),
        OpaqueFunction(function=setup),
    ])
