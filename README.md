# 개요
- 이 프로젝트는 Nav2 스택을 활용하여 특정 상황에서 자율주행 및 도킹을 구현하는 것을 목표로 합니다. 
- 개발환경
    - Ubuntu 22.04 LTS & ROS2 humble
    - Nav2
    - python 3.10.12
- 참여자
    - 장동욱
    - 전영진

# 사용법
1. ROS2 humble 및 Nav2 설치

    : 아래 링크들을 통해 설치하시면 됩니다. 
    - ROS2 humble : [https://docs.ros.org/en/humble/Installation.html#](https://docs.ros.org/en/humble/Installation.html#)
    - Nav2 : [https://docs.nav2.org/getting_started/index.html#installation](https://docs.nav2.org/getting_started/index.html#installation)

2. 워크스페이스 생성
    - 아래 링크의 방법에 따라 워크스페이스를 생성하시고, src 디렉토리 내에 본 레포지토리를 clone 하시면 됩니다. 
    - ROS2 워크스페이스 생성 : [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)


3. 실행 명령어

    - `cd ..`을 통해 이동한 워크스페이스 디렉토리에서 `colcon build`를 통해 빌드한 이후, `source install/setup.bash`를 통해 설정을 가져오시면 됩니다. 

    - 먼저 아래 명령어를 실행하여 Rviz 상에서 map을 불러오는지 확인하세요.

        ```
        ros2 launch plan2d_demo rviz_plan_only.launch.py map:=[워크스페이스까지의 절대경로]/src/Nav2_Rviz_sim/plan2d_demo/maps/map.yaml
        ```
    - 이후 Rviz의 메뉴를 통해 시작 지점(2D Pose Estimate)을 지정하고, 로봇이 잘 움직인 뒤, 새 터미널 창에서 아래 명령어를 실행하면 로봇이 도킹을 위한 2개의 목표 지점을 설정하고 주행하게 됩니다. 
        ```
        ros2 run plan2d_demo two_phase_client_humble \--ros-args --params-file $(ros2 pkg prefix plan2d_demo)/share/plan2d_demo/params/two_phase_params.yaml
        ```

# 파라미터 수정

 본 프로젝트에서는 주행 알고리즘으로 자동 도킹 결과물을 구현하기 위해, 여러 [파라미터](plan2d_demo/params/nav2_params.yaml) 수정이 있었습니다. 아래 표는 보통의 주행 프로젝트에서 사용하는 것과 다르게 적용한 내용들에 대한 요약입니다. 

### 1) Planner - Smac Hybrid

| 파라미터 | 역할 | 관례적 범위 | 현재값 |  설명 |
| :--- | :----- | :---: | :---: | :--- |
| `motion_model_for_search` | 탐색 운동모델 | - | `REEDS_SHEPP` | 후진을 허용하고 좁은 곳에서 유리 |
| `angle_quantization_bins` | 헤딩 양자화(각 해상도) | 16–72 | **72** | 고해상도 |
| `analytic_expansion_max_cost` | 해석 확장 허용 코스트 상한 | 50–150 | **150** | 필연적으로 inflated 된 장애물 사이를 지나가야 하므로, 최대로 설정  |
| `analytic_expansion_max_cost_override` | 목표 근접 시 상한 무시 | 보통 `true` | **false** |  장애물에 대한 충돌을 보수적으로 방지 / 복잡한 도킹 구역에서 실패 가능 |
| `minimum_turning_radius` | 최소 회전반경 | 0.2–1.0+ | **0.05** | tank turn을 표현하기 위한 장치 |
| `reverse_penalty` | 후진 가중(≥1) | 1.0–3.0 | **1.5** | 후진을 약하게만 억제 |
| `change_penalty` | 좌/우 전환 가중(≥0) | 1.0± | **0.0** | tank turn을 표현하기 위해, 좌우 전환에 대한 패널티를 주지 않음 |
| `non_straight_penalty` | 곡선 가중(≥1) | 1.0–1.5 | **1.2** | 직선 선호 약간↑ |
| `cost_penalty` | 코스트맵 회피 가중 | 1.3–3.5 | **1.2** | 필연적으로 costmap 내부로 진입해야 하므로, 최소한으로 설정 |
| `use_quadratic_cost_penalty` | 코스트 제곱 가중 | 보통 `true`(보수) | **False** | 고코스트 억제가 약함(장애물에 가까이 지나감 허용) |
| `tolerance` | 목표 근방 허용거리 | 0.05–0.25 m | **0.05** | 매우 타이트(경로 계획 실패 가능성은 높으나, 프로젝트 요구 조건) |
| `max_iterations` | 탐색 반복 상한 | 1e4–1e5 | **1,000,000** | 상한 매우 큼(실제론 `max_planning_time=5s`가 한계) |
| `max_on_approach_iterations` | 목표근접 추가 반복 | 200–1000 | **1000** | 목표 부근 tolance를 맞추기 위함 |
    
### 2) Global / local Costmap
| 파라미터 | 역할 | 관례적 범위 | 현재값 |  설명 |
| :--- | :----- | :---: | :---: | :--- |
| `global_costmap.resolution` | 전역 해상도 | 0.03–0.10 m/px | **0.03** | 고해상도 |
| `global_costmap.inflation_layer.inflation_radius` | 전역 인플레이트 | 0.4–0.7 m | **0.79** | 전역 경로는 보수적으로 설정 |
| `local_costmap.inflation_layer.inflation_radius` | 지역 인플레이트 | 0.3–0.6 m | **0.20** | 로봇이 장애물에 바짝 붙게 설정 |
| `track_unknown_space` | 미탐색 공간 취급 | 전역 `true` 권장 | **true** | 미지영역 회피(Planner `allow_unknown=true`와 조합 주의) |
| `footprint` | 로봇 외곽(다각형) | 실제 형상 | - | 넓은 폭 1m로 설정, 앞뒤 길이는 0.96m |

### 3) Controller - DWB
| 파라미터 | 역할 | 관례적 범위 | 현재값 |  설명 |
| :--- | :----- | :---: | :---: | :--- |
| `xy_goal_tolerance` | 위치 허용오차 | 0.05–0.25 m | **0.1** | 타이트함 |
| `yaw_goal_tolerance` | 자세 허용오차 | 0.10–0.26 rad | **0.045** | **매우 타이트(≈2.6°)**, 도킹 구현을 위해 필요 |
| `vx_samples / vtheta_samples` | 샘플 개수 | 10–20 | **20 / 20** | 연산량이 많아도 품질을 높이기 위해 높게 설정 |
| `PathAlign.scale / GoalAlign.scale / PathDist.scale / GoalDist.scale` | 정렬/거리 가중 | 5–30 | **32 / 24 / 32 / 24** | 경로·목표 정렬/접근을 강하게 선호 |
| `BaseObstacle.scale` | 장애물 회피 가중 | 0.5–3.0 | **0.08** | **이례적으로 낮음** → 회피보다 경로/목표 중시 |
| `RotateToGoal.lookahead_time` | 회전 예측 | 0.5–2.0 / 자동 | **-1.0** | 자동 규칙 사용(상황 의존) |
| `stateful` | 목표 근처 상태 유지 | true/false | **True** | 목표 부근 안정성↑ |

### 4) Progress / Goal Checker
| 파라미터 | 역할 | 관례적 범위 | 현재값 |  설명 |
| :--- | :----- | :---: | :---: | :--- |
| `required_movement_radius` | 정체 판단 거리 | 0.1–0.3 m | **0.5** | **크다**. 10초(`movement_time_allowance`) 동안 0.5 m 미진전 시 실패 |
| `movement_time_allowance` | 정체 시간 | 5–15 s | **10.0** | 보통 |
| `stateful` (goal_checker) | 목표근처 상태 유지 | true/false | **True** | 목표 부근 판정 안정화 |
