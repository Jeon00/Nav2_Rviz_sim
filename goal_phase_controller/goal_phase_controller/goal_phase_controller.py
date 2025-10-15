#!/usr/bin/env python3
import math, time, ast
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType


# TODO : 변수 등 변경 시 ***로 표시된 부분 확인. 

goal_dict_len = 4
goal_dict_inspect = True

def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

def make_pose(x, y, yaw, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    _, _, qz, qw = yaw_to_quat(float(yaw))
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p

class GoalPhaseController(Node):
    def __init__(self):
        super().__init__('goal_phase_controller_client',
                         automatically_declare_parameters_from_overrides=True)
        
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._phase_timer = None

        # ---------- 입력 파라미터 ----------
        # goal_a / goal_b: [x, y, yaw(rad)]
        # self.declare_parameter('frame_id', 'map')
        # self.declare_parameter('goals.001', [10.0, 0.5 , 100.0]) # ***
        # self.declare_parameter('goals.002', [5.67, 9.6, 1.57]) # ***

        if self.has_parameter('frame_id'):
            self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        else:
            self.frame_id = 'map'
        
        goals_dict = self.get_parameters_by_prefix('goals')
        self.goals = [list(map(float, goals_dict[k].value)) for k in sorted(goals_dict.keys())]
        if goal_dict_inspect:
            num_goals = len(self.goals)
            if num_goals != goal_dict_len:
                self.get_logger().warn(f"Declared goal count ({goal_dict_len}) != Retrieved goal count ({num_goals})")
        for i in range(len(self.goals)):
            g = self.goals[i]
            if len(g) != 3:
                self.get_logger().error(f"Goal {i+1:03} malformed: expected 3 elements (x,y,yaw), got {len(g)}")
                continue
            self.get_logger().info(f"Loaded goal_{i+1:03}: {g}")
        self.get_logger().info(f"All {len(self.goals)} Gaols loaded.")
        self.goals_idx:int =0

        

        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.phase_pub = self.create_publisher(String, 'control_phase', 10)

        self.declare_parameter('planner_action', 'compute_path_to_pose')
        self.declare_parameter('controller_action', 'follow_path')
        planner_action = self.get_parameter('planner_action').get_parameter_value().string_value
        controller_action = self.get_parameter('controller_action').get_parameter_value().string_value


        # ---------- 액션 클라이언트 ----------
        self.plan_ac   = ActionClient(self, ComputePathToPose, planner_action)
        self.follow_ac = ActionClient(self, FollowPath,       controller_action)


        # ---------- 파라미터 서비스 클라이언트 ----------
        # self.ctrl_param_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        # self.lcm_param_cli  = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')

        self._wait_and_start()

    def _wait_and_start(self):
        self.get_logger().info('Waiting action servers...')
        self.plan_ac.wait_for_server()
        self.follow_ac.wait_for_server()
        self.get_logger().info('Waiting parameter services...')
        # self.ctrl_param_cli.wait_for_service()
        # self.lcm_param_cli.wait_for_service(timeout_sec=2.0)
        self.get_logger().info('Ready. Start Phase-1.')
        self._run_phase(self.goals[0], 'PHASE-1', after=self._phase_done)

    # --------------- plan → follow 공통 루틴 ----------------
    def _run_phase(self, g_xyz, tag, after=None, controller_id:str=''):
        x, y, yaw = g_xyz
        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = make_pose(x, y, yaw, self.frame_id)
        plan_goal.use_start = False

        self.get_logger().info(f"{tag}: planning to ({x:.2f}, {y:.2f})")
        self.plan_ac.wait_for_server()
        pf = self.plan_ac.send_goal_async(plan_goal)

        def on_plan_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().error(f"{tag}: planner goal REJECTED")
                return
            self.get_logger().info(f"{tag}: planner ACCEPTED; waiting result...")
            rf = gh.get_result_async()

            def on_plan_result(rfut):
                res = rfut.result()
                if res.status != 4:
                    self.get_logger().error(f"{tag}: planning failed (status={res.status})")
                    return
                path = res.result.path
                self.get_logger().info(f"{tag}: planning OK. Following path with {len(path.poses)} poses")

                try:
                    # frame_id가 비어있으면 설정 보정(보통 plan에 들어있지만 방어적으로)
                    if not path.header.frame_id:
                        path.header.frame_id = self.frame_id
                    self.path_pub.publish(path)
                except Exception as e:
                    self.get_logger().warn(f"Failed to publish planned_path: {e}")

                follow_goal = FollowPath.Goal()
                follow_goal.path = path
                if controller_id:
                    # 필요 시 다른 컨트롤러 플러그인으로 전환
                    follow_goal.controller_id = controller_id
                ff = self.follow_ac.send_goal_async(follow_goal)

                def on_follow_sent(ffut):
                    gh2 = ffut.result()
                    if not gh2.accepted:
                        self.get_logger().error(f"{tag}: follow goal REJECTED")
                        return
                    self.get_logger().info(f"{tag}: follow ACCEPTED; waiting result...")
                    rf2 = gh2.get_result_async()

                    def on_follow_result(r2fut):
                        res2 = r2fut.result()
                        if res2.status != 4:
                            self.get_logger().error(f"{tag}: follow failed (status={res2.status})")
                            return
                        self.get_logger().info(f"{tag}: follow SUCCEEDED.")
                        if after:
                            after()

                    rf2.add_done_callback(on_follow_result)

                ff.add_done_callback(on_follow_sent)

            rf.add_done_callback(on_plan_result)

        pf.add_done_callback(on_plan_sent)

    # --------------- Phase-(i) → 파라미터 변경 → Phase-(i+1) ---------------
    def _phase_done(self):
        self.get_logger().info(f"PHASE done. Applying parameter updates...")
        # 여기다 다른 파라미터 변경점 넣으면 될 듯.
        if self.goals_idx+1 < len(self.goals):
            if self.goals[self.goals_idx][0] ==self.goals[self.goals_idx+1][0] and self.goals[self.goals_idx][1] ==self.goals[self.goals_idx+1][1]:
                msg = String()
                msg.data ="TANK_TURN"
                self.phase_pub.publish(msg)
            else:
                msg = String()
                msg.data ="FORWARD"
                self.phase_pub.publish(msg)
        else:
            msg = String()
            msg.data ="STOP"
            self.phase_pub.publish(msg)
        

        self._phase_timer = self.create_timer(2.0, self._on_phase_timer, clock=self._wall_clock)

    # --------------- 다음 Phase 시작 ---------------
    
    def _on_phase_timer(self):
        try:
            if self._phase_timer is not None:
                self._phase_timer.cancel()
        finally:
            self._phase_timer = None

        self._start_next()

    def _start_next(self):
        self.goals_idx += 1
        if self.goals_idx < len(self.goals):
            self._run_phase(self.goals[self.goals_idx], f'PHASE-{self.goals_idx+1}', after=self._phase_done)
        else:
            self.get_logger().info("All phases done.")
    



def main():
    rclpy.init()
    node = GoalPhaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
