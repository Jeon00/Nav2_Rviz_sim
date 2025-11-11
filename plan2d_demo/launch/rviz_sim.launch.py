from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'plan2d_demo'
    pkg_share = get_package_share_directory(pkg)
    default_map    = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_params = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'blank.rviz')

    declare_map = DeclareLaunchArgument('map',default_value=default_map)
    declare_params = DeclareLaunchArgument('params',default_value=default_params)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params')]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server', 'planner_server', 'controller_server']
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        output='screen'
    )
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
    #     output='screen'
    # )

    path_client = Node(
        package=pkg,
        executable='path_client',
        name='path_client',
        output='screen'
    )
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0','0','0','0','0','0','map','odom'],
        output='screen'
    )
    kinematic = Node(
        package='plan2d_demo',
        executable='kinematic_sim',
        name='kinematic_sim',
        output='screen',
        parameters=[{'rate_hz': 50.0}]
    )
    robot_marker = Node(
        package='plan2d_demo',
        executable='robot_marker',
        name='robot_marker',
        output='screen'
    )

    return LaunchDescription([declare_map, declare_params, map_server, planner_server, controller_server,
                              static_map_to_odom,kinematic, lifecycle_manager, rviz, path_client,
                              robot_marker])
