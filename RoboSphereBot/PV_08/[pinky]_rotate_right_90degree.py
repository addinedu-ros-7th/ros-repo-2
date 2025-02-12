#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class Rotate90Degrees(Node):
    def __init__(self):
        super().__init__('rotate_90_degrees')
        time.sleep(10)
        # 퍼블리셔 설정
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 회전 속도 및 지속 시간 설정
        self.angular_speed = -1.0  # rad/s (반시계 방향)
        self.rotation_time = 0.9  # 90도 회전 시간 (1.5초)

        self.get_logger().info("Rotating 90 degrees...")

        # 로봇 회전 시작
        self.rotate()

    def rotate(self):
        """로봇을 90도 회전시키고, 일정 시간 후 종료"""
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_speed

        # 회전 시작
        self.velocity_publisher.publish(twist_msg)
        time.sleep(self.rotation_time)  # 일정 시간 동안 유지

        # 회전 멈춤
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)

        self.get_logger().info("Rotation complete.")

        # 1초 대기 후 노드 종료
        self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        """노드 종료"""
        self.get_logger().info("Shutting down Rotate90Degrees node.")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Rotate90Degrees()
    rclpy.spin(node)  # 실행 후 종료 (이제 다시 실행해도 정상 작동)

if __name__ == '__main__':
    main()
