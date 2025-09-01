#!/usr/bin/env python3

# 실제 충돌 등과 아무 관련 없는 마커를 시각적으로 볼 수 있게 하는 코드. 
# 웬만하면 마커 대신에 footprint를 쓰자. 

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class RobotMarker(Node):
    def __init__(self):
        super().__init__('robot_marker')
        self.pub = self.create_publisher(Marker, 'robot_marker', 1)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 1.0, 1.0, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 0.8
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(sec=0)
        self.pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(RobotMarker())
    rclpy.shutdown()
