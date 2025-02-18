#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Quaternion


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.waypoints = [
            {'name': 'point1', 'position': [0.43, 0.06]},
            {'name': 'point2', 'position': [1.99, 0.09]},
        ]
        self.current_waypoint_index = 0
        self.linear_pid = PIDController(1.0, 0.1, 0.05)
        self.angular_pid = PIDController(1.2, 0.1, 0.05)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Action server ready. Starting navigation...")
        self.send_goal()
        self.create_timer(0.1, self.control_loop)

    def send_goal(self):
        waypoint = self.waypoints[self.current_waypoint_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['position'][0]
        goal_msg.pose.pose.position.y = waypoint['position'][1]
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(0.0)
        self.get_logger().info(f"Navigating to {waypoint['name']} at {waypoint['position']}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_done_callback)

    def feedback_callback(self, feedback_msg):
        self.current_pose = feedback_msg.feedback.current_pose.pose

    def control_loop(self):
        if not hasattr(self, 'current_pose'):
            return

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        )
        yaw = self.quaternion_to_yaw(quaternion)

        waypoint = self.waypoints[self.current_waypoint_index]
        target_x = waypoint['position'][0]
        target_y = waypoint['position'][1]

        dx = target_x - current_x
        dy = target_y - current_y
        distance_error = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - yaw

        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        linear_speed = self.linear_pid.compute(distance_error, 0.1)
        angular_speed = self.angular_pid.compute(yaw_error, 0.1)

        twist_msg = Twist()
        twist_msg.linear.x = max(0.0, min(0.15, linear_speed))
        twist_msg.angular.z = max(-0.5, min(0.5, angular_speed))
        self.cmd_vel_publisher.publish(twist_msg)

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def quaternion_to_yaw(self, quaternion):
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected. Stopping.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:
            self.get_logger().info(f"Successfully reached {self.waypoints[self.current_waypoint_index]['name']}")
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            self.send_goal()
        else:
            self.get_logger().error("Failed to reach waypoint. Retrying...")
            self.send_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
