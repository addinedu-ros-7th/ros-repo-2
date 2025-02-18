#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from math import sqrt


class LinearWaypointNavigator(Node):
    def __init__(self):
        super().__init__('linear_waypoint_navigator')

        # Define linear waypoints (x, y coordinates)
        self.waypoints = [{"x": i * 0.1, "y": 0.0} for i in range(16)]  # (0.0, 0.0) to (1.5, 0.0) in 0.1 increments
        self.current_waypoint_index = 0  # Start at the first waypoint

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Action server ready. Starting navigation...")

        # Start navigating to the first waypoint
        self.send_goal()

    def send_goal(self):
        """Send a navigation goal to the robot."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Reached the final waypoint. Staying idle.")
            return

        waypoint = self.waypoints[self.current_waypoint_index]

        # Prepare goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint["x"]
        goal_msg.pose.pose.position.y = waypoint["y"]
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        # Send goal
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
                               f"x={waypoint['x']}, y={waypoint['y']}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_done_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback}")

    def goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected. Stopping navigation.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}.")
            self.current_waypoint_index += 1  # Proceed to the next waypoint
            self.send_goal()  # Continue to the next waypoint
        else:
            self.get_logger().error("Failed to reach waypoint. Retrying...")
            self.send_goal()  # Retry the same waypoint


def main(args=None):
    rclpy.init(args=args)
    node = LinearWaypointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
