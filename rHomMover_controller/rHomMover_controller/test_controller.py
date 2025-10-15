import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# teleop key에서 cmd_vel을 받아서 gz로 보내주는 노드
# 일단 멀티어레이 없이 개별 실수값을 보내주는걸로 만들자. 

class rHomMoverController(Node):
    def __init__(self):
        super().__init__('rhom_mover_controller')
        self.get_logger().info('rhom_mover_controller node has been started.')

        self.declare_parameter('twist_field', 'linear.x')

        self.wheel_radius = 0.25
        self.joint_command = [0.0, 0.0, 0.0, 0.0] # 오휠 / 왼휠 / 오팔 / 왼팔
        self.joint_state = [0.0, 0.0, 0.0, 0.0]

        # 오른팔을 기준으로 한 팔 각도 제한
        self.inner_limit = 1.5
        self.outer_limit = -1.0

        self.right_wheel_pub = self.create_publisher(Float64, '/right_wheel_vel', 10)
        self.left_wheel_pub = self.create_publisher(Float64, '/left_wheel_vel', 10)

        self.right_arm_pub = self.create_publisher(Float64, '/right_arm_angle', 10)
        self.left_arm_pub = self.create_publisher(Float64, '/left_arm_angle', 10)

        self.rossub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        

        # 조인트스테이트나 링크각도 등 서브스크립션
        # callback은 self.joint_state를 업데이트 하는걸로
        # 이후 calc_joint_angle에서 self.joint_state를 활용해서 계산. 
        
    def cmd_vel_callback(self, msg:Twist):
        self.calc_joint_angle(msg)

        self.right_wheel_pub.publish(Float64(data=self.joint_command[0]))
        self.left_wheel_pub.publish(Float64(data=self.joint_command[1]))

        self.right_arm_pub.publish(Float64(data=self.joint_command[2]))
        self.left_arm_pub.publish(Float64(data=self.joint_command[3]))
        self.get_logger().info(f'received data : {msg.linear.x}, output_right : {self.joint_command[0]}')

    def calc_joint_angle(self, cmd_vel:Twist):
        # 휠 속도
        self.joint_command[0] = cmd_vel.linear.x /self.wheel_radius
        self.joint_command[1] = cmd_vel.linear.x /self.wheel_radius

        # 팔 각도 : 일단 z가 0~1까지라고 가정하고 식 썼음. 
        if cmd_vel.angular.z >=0 : # 좌회전
            self.joint_command[2] = cmd_vel.angular.z * self.inner_limit
            self.joint_command[3] = -cmd_vel.angular.z * self.outer_limit
        else:
            self.joint_command[2] = cmd_vel.angular.z * self.outer_limit
            self.joint_command[3] = -cmd_vel.angular.z * self.inner_limit
    # 여기다가 각 상황에 맞춰 어케 계산을 할지 
    # sdf 딴에서 joint state 발행하는걸 받는 subscriber도 넣어야 함. 


def main(args=None):
    rclpy.init(args=args)
    node = rHomMoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()