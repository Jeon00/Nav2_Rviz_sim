import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_name = "rHomMover_gazebo_sim"

    pkg_path = os.path.join(get_package_share_directory("rHomMover_gazebo_sim"))
    xacro_file = os.path.join(pkg_path, "urdf", "rHomMover_4.xacro")
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": use_sim_time}

    controller_yaml = os.path.join(FindPackageShare(pkg_name).find(pkg_name), 'config', 'diff_drive_controller.yaml')


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[params,  controller_yaml ],
                output='screen'
            ),
            # diff_drive_controller 스폰
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont'],
                output='screen'
            ),


        ]
    )