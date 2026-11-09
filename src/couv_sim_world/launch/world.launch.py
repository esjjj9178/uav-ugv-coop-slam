"""Launch canopy_world in Gazebo Harmonic.

    ros2 launch couv_sim_world world.launch.py                 # server + GUI
    ros2 launch couv_sim_world world.launch.py headless:=true  # server only (-s -r)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("couv_sim_world")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    def gz(args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": args, "on_exit_shutdown": "true"}.items(),
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "canopy_world.sdf"),
                              description="World SDF file"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true: run the server only (gz sim -s -r)"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg, "models")),
        gz(["-s -r ", world], IfCondition(headless)),
        gz(["-r ", world], UnlessCondition(headless)),
    ])
