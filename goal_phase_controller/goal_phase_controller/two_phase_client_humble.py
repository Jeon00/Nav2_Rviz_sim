#!/usr/bin/env python3
import math, time, ast
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType


# TODO : 변수 등 변경 시 ***로 표시된 부분 확인. 

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

# def parse_literal(raw: str):
#     raw = raw.strip()
#     try:
#         return ast.literal_eval(raw)
#     except Exception:
#         if raw.lower() == 'true':  return True
#         if raw.lower() == 'false': return False
#         try:
#             return float(raw) if '.' in raw else int(raw)
#         except Exception:
#             return raw

# def to_param_msg(name: str, value) -> ParamMsg:
#     v = ParameterValue()
#     if isinstance(value, bool):
#         v.type = ParameterType.PARAMETER_BOOL
#         v.bool_value = value
#     elif isinstance(value, float):
#         v.type = ParameterType.PARAMETER_DOUBLE
#         v.double_value = value
#     elif isinstance(value, int):
#         v.type = ParameterType.PARAMETER_INTEGER
#         v.integer_value = value
#     elif isinstance(value, str):
#         v.type = ParameterType.PARAMETER_STRING
#         v.string_value = value
#     elif isinstance(value, list):
#         # 필요하면 배열형으로 세분화해서 구현(예: float list → double_array_value)
#         # 여기선 문자열 배열로 안전 처리
#         v.type = ParameterType.PARAMETER_STRING_ARRAY
#         v.string_array_value = [str(x) for x in value]
#     else:
#         v.type = ParameterType.PARAMETER_STRING
#         v.string_value = str(value)
#     return ParamMsg(name=name, value=v)


class TwoPhaseClientHumble(Node):
    def __init__(self):
        super().__init__('two_phase_client_humble')

        # ---------- 입력 파라미터 ----------
        # goal_a / goal_b: [x, y, yaw(rad)]
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_a', [5.7, 8.75 , 1.57]) # ***
        self.declare_parameter('goal_b', [5.67, 9.6, 1.57]) # ***

        self.path_pub = self.create_publisher(Path, 'planned_path', 10)

        # 일단 파라미터 업데이트는 주석 처리하고, 필요한 경우 수정. 

        # # 컨트롤러 서버에 적용할 파라미터 업데이트 목록 ("name:=value" 문자열)
        # self.declare_parameter('updates_controller', [
        #     "FollowPath.sim_time:=2.5",
        #     "FollowPath.PathAlign.scale:=16.0",
        #     # "RPP.lookahead_dist:=0.8",
        # ])

        # # (선택) 로컬 코스트맵에 적용할 파라미터 (인플레이션 등)
        # self.declare_parameter('updates_local_costmap', [
        #     "inflation_layer.inflation_radius:=0.35"
        # ])

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.goal_a = list(self.get_parameter('goal_a').get_parameter_value().double_array_value)
        self.goal_b = list(self.get_parameter('goal_b').get_parameter_value().double_array_value)
        # self.updates_ctrl = list(self.get_parameter('updates_controller').get_parameter_value().string_array_value)
        # self.updates_lcm  = list(self.get_parameter('updates_local_costmap').get_parameter_value().string_array_value)

        self.get_logger().info(f"Phase-1 goal: {self.goal_a}")
        self.get_logger().info(f"Phase-2 goal: {self.goal_b}")
        # self.get_logger().info(f"Controller updates: {self.updates_ctrl}")
        # if self.updates_lcm:
        #     self.get_logger().info(f"Local costmap updates: {self.updates_lcm}")

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
        self._run_phase(self.goal_a, 'PHASE-1', after=self._phase1_done)

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

    # --------------- Phase-1 → 파라미터 변경 → Phase-2 ---------------
    def _phase1_done(self):
        # self.get_logger().info("PHASE-1 done. Applying parameter updates...")
        self.get_logger().info("PHASE-1 done. Dwell 0.5s before PHASE-2...")
        timer = self.create_timer(1.5, lambda: (self._cancel_timer_and_start_phase2(timer)))


        # # 1) controller_server 업데이트
        # if self.updates_ctrl:
        #     req = SetParameters.Request()
        #     req.parameters = []
        #     for s in self.updates_ctrl:
        #         if ':=' not in s:
        #             self.get_logger().warn(f"Skip malformed '{s}' (expected name:=value)")
        #             continue
        #         name, raw = s.split(':=', 1)
        #         val = parse_literal(raw)
        #         req.parameters.append(to_param_msg(name.strip(), val))

        #     fut = self.ctrl_param_cli.call_async(req)

        #     def after_ctrl(_):
        #         self.get_logger().info("controller_server parameters updated.")
        #         # 2) (선택) local_costmap 업데이트
        #         if self.updates_lcm and self.lcm_param_cli.service_is_ready():
        #             req2 = SetParameters.Request()
        #             req2.parameters = []
        #             for s in self.updates_lcm:
        #                 if ':=' not in s:
        #                     self.get_logger().warn(f"Skip malformed '{s}'")
        #                     continue
        #                 name, raw = s.split(':=', 1)
        #                 val = parse_literal(raw)
        #                 req2.parameters.append(to_param_msg(name.strip(), val))
        #             fut2 = self.lcm_param_cli.call_async(req2)
        #             fut2.add_done_callback(lambda __: self._start_phase2())
        #         else:
        #             self._start_phase2()

        #     fut.add_done_callback(after_ctrl)
        # else:
        #     # 업데이트 없으면 바로 Phase-2
        #     self._start_phase2()
    def _cancel_timer_and_start_phase2(self, timer):
        timer.cancel()
        self._start_phase2()
    def _start_phase2(self):
        # 필요하면 여기서 controller_id='RPP'처럼 다른 플러그인으로 전환도 가능
        self._run_phase(self.goal_b, 'PHASE-2', controller_id='')

def main():
    rclpy.init()
    node = TwoPhaseClientHumble()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
