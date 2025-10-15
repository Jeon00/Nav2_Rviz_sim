import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "rHomMover_gazebo_sim"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "rHomMover_5.launch.py")] # 여기에 몇번꺼 킬건지 결정
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"], # 여기서 로봇의 이름을 결정, 현재는 with_robot
        output="screen",
    ) # robot description을 위쪽에서 퍼블리시 한 뒤 엔티티를 스폰해야 한다는 점 주의. 


    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
