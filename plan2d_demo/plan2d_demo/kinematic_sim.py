#!/usr/bin/env python3
import math, rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from rclpy.time import Time

class KinematicSim(Node):
    def __init__(self):
        super().__init__('kinematic_sim')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)

        self.rate = float(self.get_parameter('rate_hz').value)
        self.x = float(self.get_parameter('init_x').value)
        self.y = float(self.get_parameter('init_y').value)
        self.yaw = float(self.get_parameter('init_yaw').value)

        self.v = 0.0
        self.w = 0.0
        self.last = time.time()

        self.sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 50)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/self.rate, self._step)
        self.get_logger().info('KinematicSim ready: listening to /cmd_vel')

        self.initpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._initpose_cb, 50
        )

    def _cmd_cb(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def _step(self):
        now = time.time()
        dt = now - self.last
        self.last = now

        # simple unicycle model
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # publish odom
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

        # publish TF: odom -> base_link
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

        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = float(yaw)
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.get_logger().info(f'Reset pose from /initialpose -> x:{self.x:.2f}, y:{self.y:.2f}, yaw:{self.yaw:.2f}')


def main():
    rclpy.init()
    node = KinematicSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
