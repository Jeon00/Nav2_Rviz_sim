import rclpy
import math, rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# teleop key에서 cmd_vel을 받아서 gz와 rviz로 보내주는 노드
# TODO : *** 표시된 물리 변수 수정

class rHomMoverController(Node):
    def __init__(self):
        super().__init__('rhom_mover_controller')
        self.get_logger().info('rhom_mover_controller node has been started.')

        # model ***
        self.width  = 0.91 # 폭
        self.length = 0.96 # 길이
        self.wheel_radius  = 0.23/2

        # rviz
        self.declare_parameter('twist_field', 'linear.x')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('init_x', 4.0)
        self.declare_parameter('init_y', 8.24)
        self.declare_parameter('init_yaw', 0.0)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.x = float(self.get_parameter('init_x').value)
        self.y = float(self.get_parameter('init_y').value)
        self.yaw = float(self.get_parameter('init_yaw').value)
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/self.rate, self._odom_step)
        self.get_logger().info('KinematicSim ready: listening to /cmd_vel')

        # gazebo
        self.last_joint_command = [0.0, 0.0, 0.0, 0.0]
        self.joint_command = [0.0, 0.0, 0.0, 0.0] # 오휠 / 왼휠 / 오팔 / 왼팔
        self.joint_state = [0.0, 0.0, 0.0, 0.0]
        
        self.inner_limit = 1.5 # 오른팔 기준 ***
        self.outer_limit = -1.0

        self.cmd_vel_sub = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,50)
        self.drv_mode_sub = self.create_subscription(String,'/control_phase',self.drv_mode_change,10)
        self.last_driving_mode = "BACKWARD"
        self.driving_mode = "BACKWARD"
        self.wait = False
        
        self.right_wheel_pub = self.create_publisher(Float64, '/right_wheel_vel', 20)
        self.left_wheel_pub = self.create_publisher(Float64, '/left_wheel_vel', 20)

        self.right_arm_pub = self.create_publisher(Float64, '/right_arm_angle', 20)
        self.left_arm_pub = self.create_publisher(Float64, '/left_arm_angle', 20)
        self.body_pub = self.create_publisher(Twist, '/body_vel', 20)

        self.initpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._initpose_cb, 50
        )

        # callback은 self.joint_state를 업데이트 하는걸로
        # 이후 calc_joint_angle에서 self.joint_state를 활용해서 계산. 
        
    def cmd_vel_callback(self, msg:Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        if self.wait:
            if self.last_driving_mode == "BACKWARD" and self.driving_mode == "TRANSITION":
                # 탱크턴 준비 모드

                self.calc_transition_joint(self.last_driving_mode)              

                self.right_wheel_pub.publish(Float64(data=self.joint_command[0]))
                self.left_wheel_pub.publish(Float64(data=self.joint_command[1]))

                self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
                self.left_arm_pub.publish(Float64(data=self.joint_command[3]))

                body_twist = msg
                body_twist.angular.z = self.w * 1.0

                self.body_pub.publish(body_twist)

                time.sleep(2.0) # 여기서 기다리면 얘가 움직이지 않을까

                self.last_driving_mode = "TRANSITION"
                self.wait = False
                self.driving_mode = "TANK_TURN"
            elif self.last_driving_mode == "TANK_TURN" and self.driving_mode == "TRANSITION":
                # 후진 준비 모드

                self.calc_transition_joint(self.last_driving_mode)

                self.right_wheel_pub.publish(Float64(data=self.joint_command[0]))
                self.left_wheel_pub.publish(Float64(data=self.joint_command[1]))

                self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
                self.left_arm_pub.publish(Float64(data=self.joint_command[3]))

                body_twist = msg
                body_twist.angular.z = self.w * 1.0

                self.body_pub.publish(body_twist)

                time.sleep(1.5)

                self.last_driving_mode = "TRANSITION"
                self.wait = False
                self.driving_mode = "BACKWARD"
        else:
            self.calc_joint_angle(msg)

            self.right_wheel_pub.publish(Float64(data=self.joint_command[0]))
            self.left_wheel_pub.publish(Float64(data=self.joint_command[1]))

            self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
            self.left_arm_pub.publish(Float64(data=self.joint_command[3]))

            body_twist = msg
            body_twist.angular.z = self.w * 0.95

            self.body_pub.publish(body_twist)


            self.get_logger().info(f'currnet mode : {self.driving_mode}, right wheel speed  : {self.joint_command[0]}, right arm angle : {self.joint_command[2]}')

    def drv_mode_change(self, msg:String):
        self.driving_mode = msg.data
        if self.driving_mode == "TRANSITION":
            self.wait = True
        if self.driving_mode == "TANK_TURN":
            self.joint_command[2] = -math.atan(2*self.length / self.width) # 오른팔
            self.joint_command[3] = math.atan(2*self.length / self.width)
            self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
            self.left_arm_pub.publish(Float64(data=self.joint_command[3]))
        if self.driving_mode == "BACKWARD":
            self.joint_command[2] = 0.0
            self.joint_command[3] = 0.0
            self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
            self.left_arm_pub.publish(Float64(data=self.joint_command[3]))
        if self.driving_mode == "STOP":
            self.joint_command[2] = 0.0
            self.joint_command[3] = 0.0
            self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
            self.left_arm_pub.publish(Float64(data=self.joint_command[3]))
        self.get_logger().info(f'Driving mode changed to : {self.driving_mode}')

    def calc_joint_angle(self, cmd_vel:Twist):
        v = float(cmd_vel.linear.x)
        w = float(cmd_vel.angular.z)

        self.joint_command[0] = v / self.wheel_radius   # right wheel
        self.joint_command[1] = v / self.wheel_radius


        now = time.time()
        dt = now - self.last
        self.last = now
        theta = cmd_vel.angular.z * dt # 이번 틱에서 회전한 각도
        eps = 1e-3
        if abs(theta) < eps:
            theta = 0.0
        
        if self.driving_mode == "BACKWARD":

            v = float(cmd_vel.linear.x)
            w = float(cmd_vel.angular.z)

            self.joint_command[0] = v / self.wheel_radius   # right wheel
            self.joint_command[1] = v / self.wheel_radius

            w_eps  = 1e-3
            v_eps  = 1e-3
            if abs(w) < w_eps or abs(v) < v_eps:
                # 거의 직진: 각도 0
                right_arm = 0.0
                left_arm  = 0.0
            else:
                R = v / w  # 회전반경
                # 발산 방지: R이 ±W/2에 가까우면 각도 90°로 튐 → 한계 각으로 제한
                denom_r = (R - self.width/2.0)
                denom_l = (R + self.width/2.0)
                # 안전막
                min_den = 1e-6
                if abs(denom_r) < min_den:
                    denom_r = math.copysign(min_den, denom_r)
                if abs(denom_l) < min_den:
                    denom_l = math.copysign(min_den, denom_l)

                right_arm = math.atan(self.length/denom_r)
                left_arm  = math.atan(self.length/ denom_l)
            if abs(self.last_joint_command[2] + right_arm) > 0.4 or abs(self.last_joint_command[3] + left_arm) > 0.4:
                self.joint_command[2] = -right_arm * 0.7
                self.joint_command[3] = -left_arm * 0.7
            
            else:
                self.joint_command[2] = -right_arm
                self.joint_command[3] = -left_arm

        elif self.driving_mode == "TANK_TURN":
            # 팔 각도
            self.joint_command[2] = -math.atan(2*self.length / self.width) # 오른팔
            self.joint_command[3] = math.atan(2*self.length / self.width)
            # 휠 속도
            self.joint_command[0] = cmd_vel.angular.z*math.sqrt((self.length**2)+(self.width/2.0)**2) /self.wheel_radius
            self.joint_command[1] = -cmd_vel.angular.z*math.sqrt((self.length**2)+(self.width/2.0)**2) /self.wheel_radius
        elif self.driving_mode == "STOP":
            # 휠 속도
            self.joint_command[0] = 0.0
            self.joint_command[1] = 0.0
            # 팔 각도
            self.joint_command[2] = 0.0
            self.joint_command[3] = 0.0

            

    def calc_transition_joint(self, last_mode):
        if last_mode == "BACKWARD":
            # 탱크턴으로 변환
            self.joint_command[0] = 0.0
            self.joint_command[1] = 0.0
            # 팔 각도 TODO
            self.joint_command[2] = math.atan(2*self.length / self.width) # 오른팔
            self.joint_command[3] = -math.atan(2*self.length / self.width)
        elif last_mode == "TANK_TURN":
            # 후진으로 변환
            self.joint_command[0] = 0.0
            self.joint_command[1] = 0.0
            # 팔 각도
            self.joint_command[2] = 0.0
            self.joint_command[3] = 0.0

    def _odom_step(self):
        now = time.time()
        dt = now-self.last
        self.last = now

        if not self.wait:
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.w * dt
        else: # transition 모드일 때는 위치 변화 없음
            pass

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self.yaw/2.0); qw = math.cos(self.yaw/2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)
        

    def _initpose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw 추출
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0*(q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.x =  float(p.x)
        self.y =  float(p.y)
        self.yaw = float(yaw)
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.get_logger().info(f'Reset pose from /initialpose -> x:{self.x:.2f}, y:{self.y:.2f}, yaw:{self.yaw:.2f}')
    def _set_initpose(self, x:float, y:float, yaw:float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.get_logger().info(f'Set initial pose -> x:{self.x:.2f}, y:{self.y:.2f}, yaw:{self.yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = rHomMoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()