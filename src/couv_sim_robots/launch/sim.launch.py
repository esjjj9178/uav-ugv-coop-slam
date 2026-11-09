"""canopy_world + UGV + drone + ros_gz bridge + robot_state_publishers + RViz.

    ros2 launch couv_sim_robots sim.launch.py [drone_cam_pitch_deg:=45] [gui:=false] [rviz:=false] [mission:=true]

All geometry / spawn numbers come from config/frames.yaml. The drone is spawned on the
UGV landing pad at T0 = frames.yaml `landing_pad_offset` (pose of drone/base_link in
ugv/base_link).
"""
import math
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PKG = "couv_sim_robots"


def yaw_compose(base, off):
    """base = (x, y, z, yaw) in map, off = (x, y, z, yaw) in base frame (roll = pitch = 0)."""
    c, s = math.cos(base[3]), math.sin(base[3])
    return (base[0] + c * off[0] - s * off[1], base[1] + s * off[0] + c * off[1], base[2] + off[2], base[3] + off[3])


def truthy(v):
    return str(v).lower() in ("1", "true", "yes", "on")


def setup(context):
    share = get_package_share_directory(PKG)
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    frames_yaml = arg("frames_yaml")
    cfg = yaml.safe_load(open(frames_yaml))
    u, d, t0 = cfg["ugv"], cfg["drone"], cfg["landing_pad_offset"]
    world = cfg["world"]["name"]
    pitch = arg("drone_cam_pitch_deg") or str(d["camera"]["pitch_deg"])
    noise = arg("depth_noise_stddev") or str(cfg["d435"]["depth_noise_stddev"])
    cam_h = arg("ugv_cam_height") or str(u["camera"]["height"])
    sim_time = {"use_sim_time": True}

    ugv_urdf = xacro.process_file(os.path.join(share, "urdf", "ugv.urdf.xacro"), mappings={
        "frames_yaml": frames_yaml, "cam_height": cam_h, "depth_noise_stddev": noise}).toxml()
    drone_urdf = xacro.process_file(os.path.join(share, "urdf", "drone.urdf.xacro"), mappings={
        "frames_yaml": frames_yaml, "cam_pitch_deg": pitch, "depth_noise_stddev": noise}).toxml()

    # spawn poses: ugv/base_link sits wheel_radius above ugv/base_footprint
    sp = dict(u["spawn"])
    for k in ("x", "y", "yaw"):  # launch overrides (e.g. another start point for a different mission)
        if arg(f"ugv_spawn_{k}"):
            sp[k] = float(arg(f"ugv_spawn_{k}"))
    ugv_pose = (sp["x"], sp["y"], sp["z"] + u["wheel_radius"] + u["spawn_clearance"], sp["yaw"])
    drone_pose = yaw_compose(ugv_pose, (t0["x"], t0["y"], t0["z"], t0["yaw"]))
    pad_top = u["landing_pad"]["top_height"] - u["wheel_radius"]
    seat = pad_top + d["body"]["size_z"] / 2 + d["leg_height"]
    msgs = [LogInfo(msg=f"[couv_sim_robots] ugv spawn {ugv_pose}, drone spawn {tuple(round(v, 4) for v in drone_pose)} "
                        f"(T0 = {t0}), drone_cam_pitch_deg = {pitch}")]
    if not 0.0 <= t0["z"] - seat <= 0.02:
        msgs.append(LogInfo(msg=f"[couv_sim_robots] WARNING: T0.z={t0['z']} but skids rest at {seat:.4f} "
                                f"(pad top {pad_top:.4f} + drone {seat - pad_top:.4f})"))

    def spawn(name, pose):
        return Node(package="ros_gz_sim", executable="create", output="screen",
                    arguments=["-world", world, "-name", name, "-topic", f"/{name}/robot_description",
                               "-x", str(pose[0]), "-y", str(pose[1]), "-z", str(pose[2]), "-Y", str(pose[3])])

    def rsp(ns, urdf, prefix):
        return Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns,
                    name="robot_state_publisher" if not prefix.startswith("gt/") else "gt_robot_state_publisher",
                    output="screen",
                    parameters=[{"robot_description": urdf, "frame_prefix": prefix, **sim_time}],
                    remappings=[("robot_description", f"/{ns}/robot_description" if not prefix.startswith("gt/")
                                 else f"/{ns}/gt_robot_description"),
                                ("joint_states", f"/{ns}/joint_states")])

    spawn_ugv = spawn(u["name"], ugv_pose)
    spawn_drone = spawn(d["name"], drone_pose)
    actions = msgs + [
        rsp(u["name"], ugv_urdf, f"{u['name']}/"),
        rsp(d["name"], drone_urdf, f"{d['name']}/"),
        spawn_ugv,
        # the drone is spawned after the UGV exists so it lands on the pad
        RegisterEventHandler(OnProcessExit(target_action=spawn_ugv, on_exit=[spawn_drone])),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="ros_gz_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml"), **sim_time}]),
    ]
    if truthy(arg("pointcloud")):
        # realsense-style .../depth/color/points (optical frame) from the registered depth + colour
        for ns in (u["name"], d["name"]):
            actions.append(ComposableNodeContainer(
                name="pointcloud_container", namespace=ns, package="rclcpp_components",
                executable="component_container", output="screen", parameters=[sim_time],
                composable_node_descriptions=[ComposableNode(
                    package="depth_image_proc", plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="point_cloud_xyzrgb", namespace=ns, parameters=[sim_time],
                    remappings=[("rgb/camera_info", f"/{ns}/camera/color/camera_info"),
                                ("rgb/image_rect_color", f"/{ns}/camera/color/image_raw"),
                                ("depth_registered/image_rect", f"/{ns}/camera/depth/image_rect_raw"),
                                ("points", f"/{ns}/camera/depth/color/points")])]))
    if truthy(arg("compress")):
        # image_transport compressed streams for recording (see config/recording.yaml)
        comp = yaml.safe_load(open(os.path.join(share, "config", "recording.yaml")))["compression"]
        for ns in (u["name"], d["name"]):
            for kind, base, transport, params in (
                    ("color", f"/{ns}/camera/color/image_raw", "compressed",
                     {"out.jpeg_quality": comp["jpeg_quality"], "out.format": "jpeg"}),
                    ("depth", f"/{ns}/camera/depth/image_rect_raw", "compressedDepth",
                     {"out.png_level": comp["png_level"], "out.depth_max": comp["depth_max"],
                      "out.depth_quantization": comp["depth_quantization"]})):
                actions.append(Node(package="image_transport", executable="republish", namespace=ns,
                                    name=f"{kind}_compress", arguments=["raw", transport],
                                    parameters=[{**params, **sim_time}],
                                    remappings=[("in", base), (f"out/{transport}", f"{base}/{transport}")]))
    if truthy(arg("wheel_odom_tf")):
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="wheel_odom_tf_bridge",
                            arguments=[f"/{u['name']}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
                            remappings=[(f"/{u['name']}/tf", "/tf")], parameters=[sim_time]))
    if truthy(arg("gt_tf")):
        actions += [
            rsp(u["name"], ugv_urdf, f"gt/{u['name']}/"),
            rsp(d["name"], drone_urdf, f"gt/{d['name']}/"),
            Node(package=PKG, executable="gt_viz.py", name="gt_viz", output="screen",
                 parameters=[{"robots": [u["name"], d["name"]], "map_frame": cfg["world"]["map_frame"],
                              "cloud_rate": 2.0, **sim_time}]),
        ]
    if truthy(arg("rviz")):
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "sim.rviz")], parameters=[sim_time]))
    if truthy(arg("mission")):
        actions.append(Node(package=PKG, executable="mission_runner.py", name="mission_runner", output="screen",
                            parameters=[{"mission_yaml": arg("mission_yaml"), **sim_time}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    world_launch = os.path.join(get_package_share_directory("couv_sim_world"), "launch", "world.launch.py")
    args = [
        ("frames_yaml", os.path.join(share, "config", "frames.yaml"), "geometry / spawn configuration"),
        ("mission_yaml", os.path.join(share, "config", "mission.yaml"), "mission waypoints"),
        ("drone_cam_pitch_deg", "", "drone D435 pitch, 90 = straight down, 45 = forward-down (default: frames.yaml)"),
        ("ugv_cam_height", "", "UGV D435 optical centre height above ground [m] (default: frames.yaml)"),
        ("depth_noise_stddev", "", "gaussian depth noise [m] for both D435 (default: frames.yaml)"),
        ("ugv_spawn_x", "", "UGV start x in map [m] (default: frames.yaml)"),
        ("ugv_spawn_y", "", "UGV start y in map [m] (default: frames.yaml)"),
        ("ugv_spawn_yaw", "", "UGV start heading [rad] (default: frames.yaml); the drone follows via T0"),
        ("gui", "true", "start the Gazebo GUI"),
        ("rviz", "true", "start RViz"),
        ("gt_tf", "true", "publish ground truth as map -> gt/<robot>/... TF (separate tree, for RViz only)"),
        ("pointcloud", "true", "publish .../camera/depth/color/points (depth_image_proc)"),
        ("compress", "true", "publish JPEG / compressedDepth image streams (used by record.sh)"),
        ("wheel_odom_tf", "true", "bridge the UGV wheel odometry TF ugv/odom -> ugv/base_link"),
        ("mission", "false", "start mission_runner immediately"),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v, description=desc) for n, v, desc in args] + [
            # package:// mesh URIs resolve through <install>/share
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(share)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch), launch_arguments={
                "headless": PythonExpression(["'false' if '", LaunchConfiguration("gui"),
                                              "'.lower() in ('1', 'true') else 'true'"])}.items()),
            OpaqueFunction(function=setup),
        ])
