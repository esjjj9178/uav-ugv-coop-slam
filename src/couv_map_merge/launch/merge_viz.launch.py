"""Register the drone map to the UGV map and replay the steps in RViz.

    ros2 launch couv_map_merge merge_viz.launch.py map_dir:=~/ros2_ws/maps/<bag name> bag:=~/ros2_ws/bags/<bag name>

`bag` defaults to ~/ros2_ws/bags/<basename of map_dir> (ground truth + rangefinder are read from it).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("couv_map_merge")
    map_dir = os.path.abspath(os.path.expanduser(LaunchConfiguration("map_dir").perform(context)))
    bag = LaunchConfiguration("bag").perform(context) or os.path.join(
        os.path.expanduser("~/ros2_ws/bags"), os.path.basename(map_dir))
    actions = [Node(package="couv_map_merge", executable="merge_viz.py", name="merge_viz", output="screen",
                    parameters=[{"map_dir": map_dir, "bag": os.path.abspath(os.path.expanduser(bag)),
                                 "merge_yaml": LaunchConfiguration("merge_yaml").perform(context)}])]
    if LaunchConfiguration("rviz").perform(context).lower() in ("1", "true"):
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "merge.rviz")]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("couv_map_merge")
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", description="output directory of run_slam.sh"),
        DeclareLaunchArgument("bag", default_value="", description="bag the maps were built from"),
        DeclareLaunchArgument("merge_yaml", default_value=os.path.join(share, "config", "merge.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup),
    ])
