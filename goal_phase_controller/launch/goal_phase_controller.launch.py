from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('goal_phase_controller')
    params = os.path.join(share, 'params', 'goal_phase_controller_params.yaml')
    return LaunchDescription([
        Node(
            package='goal_phase_controller',
            executable='goal_phase_controller',
            name='goal_phase_controller_client',
            output='screen',
            parameters=[params, {'use_sim_time': True}]
        )
    ])