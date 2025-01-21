#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Define waypoints (x, y coordinates)
        self.waypoints = [
            {'name': 'point1', 'position': [0.43, 0.06]},
            {'name': 'point2', 'position': [1.99, 0.09]},
        ]
        self.current_waypoint_index = 0  # Current waypoint index

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Action server ready. Starting navigation...")
        self.send_goal()

    def send_goal(self):
        # Get current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['position'][0]
        goal_msg.pose.pose.position.y = waypoint['position'][1]
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        # Send goal
        self.get_logger().info(f"Navigating to {waypoint['name']} at {waypoint['position']}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_done_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback}")

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
        if result.status == 4:  # STATUS_SUCCEEDED
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
