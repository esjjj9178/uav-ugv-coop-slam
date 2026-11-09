"""Show the registration methods of a serpentine run in RViz.

    ros2 launch couv_map_merge aruco_viz.launch.py run:=~/ros2_ws/maps/serpentine_aruco             # 1 | 2 | 3 side by side
    ros2 launch couv_map_merge aruco_viz.launch.py run:=~/ros2_ws/maps/serpentine_aruco mode:=cycle  # A-D + GT in turn

Needs the outputs of run_live.sh in `run`: <robot>_poses.txt, <robot>_camera_poses.txt, <robot>_cloud.ply,
<robot>_rgb/ <robot>_depth/ <robot>_calib/ and aux_bag/.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("couv_map_merge")
    run = os.path.abspath(os.path.expanduser(LaunchConfiguration("run").perform(context)))
    mode = LaunchConfiguration("mode").perform(context)
    actions = [Node(package="couv_map_merge", executable="aruco_viz.py", name="aruco_viz", output="screen",
                    parameters=[{"run": run, "mode": mode,
                                 "step_period": float(LaunchConfiguration("step_period").perform(context))}])]
    if LaunchConfiguration("rviz").perform(context).lower() in ("1", "true"):
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "compare.rviz" if mode == "compare" else "aruco.rviz")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run", default_value=os.path.expanduser("~/ros2_ws/maps/serpentine_aruco"),
                              description="output directory of run_live.sh"),
        DeclareLaunchArgument("mode", default_value="compare", description="compare (1 | 2 | 3 side by side) or cycle"),
        DeclareLaunchArgument("step_period", default_value="4.0", description="[s] per method (mode cycle)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup),
    ])
