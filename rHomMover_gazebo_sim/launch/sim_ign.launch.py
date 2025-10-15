import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node



def generate_launch_description():
    # 경로의 경우 절대 하드코딩하지 말고 패키지 share 폴더를 찾아서 사용할 것. 
    package_name = "rHomMover_gazebo_sim"
    pkg_share =  get_package_share_directory(package_name)
    
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation (Ignition) clock"
    )

    package_path = os.path.join(get_package_share_directory("rHomMover_gazebo_sim"))
    
    # xacro 뭐 쓸지 설정하는 부분!
    xacro_file = os.path.join(package_path, "urdf", "rHomMover_5.xacro")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_file,
    ])


    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content,
                     "use_sim_time": use_sim_time}],
        output="screen",
    )

    world_file = os.path.join(
        get_package_share_directory("rHomMover_gazebo_sim"), "worlds", "demo_world.sdf",
    )

    ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-name", "with_robot",
    #         "-string", robot_description_content,
    #         "-z", "0.1"
    #     ],
    #     output="screen",
    # )
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "with_robot",
            "-string", robot_description_content,
            "-urdf",
            "-z", "0.1",
        ],
        output="screen",
    )


    # Launch them all!
    return LaunchDescription(
        [
            declare_use_sim_time,
            ign,
            rsp,
            spawn_entity,
        ]
    )
