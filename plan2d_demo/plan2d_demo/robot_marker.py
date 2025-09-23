#!/usr/bin/env python3

# 실제 충돌 등과 아무 관련 없는 마커를 시각적으로 볼 수 있게 하는 코드. 
# 웬만하면 마커 대신에 footprint를 쓰자. 

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# class RobotMarker(Node):
#     def __init__(self):
#         super().__init__('robot_marker')
#         self.pub = self.create_publisher(MarkerArray, 'marker', 1)
#         self.timer = self.create_timer(0.1, self.tick)

#     def tick(self):
#         markers = MarkerArray()

#         m = Marker()
#         m.header.frame_id = 'base_link'
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.type = Marker.CUBE
#         m.action = Marker.ADD
#         m.scale.x, m.scale.y, m.scale.z = 0.96, 0.33, 0.05
#         m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 1.0
#         m.pose.position.x = - 0.96/2
#         m.lifetime = Duration(sec=0)
#         markers.markers.append(m)

#         m2 = Marker()
#         m2.header.frame_id = 'base_link'
#         m2.header.stamp = self.get_clock().now().to_msg()
#         m2.type = Marker.CUBE
#         m2.action = Marker.ADD
#         m2.scale.x, m.scale.y, m.scale.z = 0.23, 1.0, 0.05
#         m2.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 1.0
#         m2.pose.position.x = -0.845
#         m2.lifetime = Duration(sec=0)
#         markers.markers.append(m2)

#         self.pub.publish(markers)
#         self.did_publish = True
#         self.get_logger().info(f'Published {mid} rectangle tiles.')
#         # 한 번만 내보내면 끝내자
#         self.timer.cancel()

# def main():
#     rclpy.init()
#     rclpy.spin(RobotMarker())
#     rclpy.shutdown()



class RobotMarker(Node):
    def __init__(self):
        super().__init__('robot_marker')
        self.pub1 = self.create_publisher(Marker, 'robot_marker1', 1)
        self.pub2 = self.create_publisher(Marker, 'robot_marker2', 1)
        self.timer1 = self.create_timer(0.1, self.tick1)
        self.timer2 = self.create_timer(0.1, self.tick2)

    def tick1(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.96, 0.33, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 1.0
        m.pose.orientation.w = 1.0
        m.pose.position.x = -0.48
        m.lifetime = Duration(sec=0)
        self.pub1.publish(m)
    def tick2(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.23, 1.0, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 1.0
        m.pose.orientation.w = 1.0
        m.pose.position.x = -0.845
        m.lifetime = Duration(sec=0)
        self.pub2.publish(m)

def main():
    rclpy.init()
    rclpy.spin(RobotMarker())
    rclpy.shutdown()