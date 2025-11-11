import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'rHomMover_controller'
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time = LaunchConfiguration("use_sim_time")

    package_path = os.path.join(get_package_share_directory("rHomMover_controller"))

    # test_controller = Node(
    #     package=pkg_name,
    #     executable='test_controller',
    #     name='test_controller_node',
    #     output='screen'
    # )

    box_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='box_cmd_vel_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ]
    )

    return LaunchDescription(
        [box_cmd_vel_bridge]
    )