# 개요
- 이 프로젝트는 Nav2 스택을 활용하여 특정 상황에서 자율주행 및 도킹을 구현하는 것을 목표로 합니다. 
- 개발환경
    - Ubuntu 22.04 LTS & ROS2 humble
    - Nav2
- 참여자
    - 장동욱
    - 전영진

# 사용법
1. ROS2 humble 및 Nav2 설치
아래 링크들을 통해 설치하시면 됩니다. 
- ROS2 humble : [https://docs.ros.org/en/humble/Installation.html#](https://docs.ros.org/en/humble/Installation.html#)
- Nav2 : [https://docs.nav2.org/getting_started/index.html#installation](https://docs.nav2.org/getting_started/index.html#installation)

2. 워크스페이스 생성
- 아래 링크의 방법에 따라 워크스페이스를 생성하시고, src 디렉토리 내에 본 레포지토리를 clone 하시면 됩니다. 
- ROS2 워크스페이스 생성 : [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)


3. 실행 명령어

- `cd ..`을 통해 이동한 워크스페이스 디렉토리에서 `colcon build`를 통해 빌드한 이후, `source install/setup.bash`를 통해 설정을 가져오시면 됩니다. 

- 이후 아래 명령어를 실행하면 Rviz가 실행되고, Rviz의 메뉴를 통해 시작 지점(2D Pose Estimate), 목표 지점(2D Goal Pose)를 지정하시면 됩니다. 
```
ros2 launch plan2d_demo rviz_plan_only.launch.py map:=[워크스페이스까지의 절대경로]/src/Nav2_Rviz_sim/plan2d_demo/maps/map.yaml
```

