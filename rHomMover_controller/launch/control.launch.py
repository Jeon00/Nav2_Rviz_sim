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

    rhom_controller = Node(
        package=pkg_name,
        executable='rhom_mover_controller',
        name='controller_node',
        output='screen'
    )

    # ros_gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='right_wheel_bridge',
    #     output='screen',
    #     arguments=[
    #         '/right_wheel_vel@std_msgs/msg/Float64@ignition.msgs.Double'
    #     ]
    # )
    right_wheel_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_wheel_bridge',
        output='screen',
        arguments=[
            '/right_wheel_vel@std_msgs/msg/Float64@ignition.msgs.Double'
        ]
    )
    left_wheel_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_wheel_bridge',
        output='screen',
        arguments=[
            '/left_wheel_vel@std_msgs/msg/Float64@ignition.msgs.Double'
        ]
    )

    right_arm_ang = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_arm_bridge',
        output='screen',
        arguments=[
            '/right_arm_angle@std_msgs/msg/Float64@ignition.msgs.Double'
        ]
    )

    left_arm_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_arm_bridge',
        output='screen',
        arguments=[
            '/left_arm_angle@std_msgs/msg/Float64@ignition.msgs.Double'
        ]
    )

    # box_cmd_vel_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='box_cmd_vel_bridge',
    #     output='screen',
    #     arguments=[
    #         '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
    #     ]
    # )
    box_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='box_cmd_vel_bridge',
        output='screen',
        arguments=[
            '/body_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ]
    )



    return LaunchDescription(
        [rhom_controller,
         box_cmd_vel_bridge,
         right_wheel_vel,
         left_wheel_vel,
         right_arm_ang,
         left_arm_vel]

    )


# 여기 더 작성해서 컨트롤 노드랑 ros gz bridge 같이 실행될 수 있도록 하면 됨. 