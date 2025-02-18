#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from math import sqrt


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Define waypoints (x, y coordinates)
        self.waypoints = [
            {"name": "point1", "position": [0.72, 0.1, 0]},
            {"name": "point2", "position": [0.72, 0.5, 0]},
            {"name": "point3", "position": [1.3, 0.5, 0]},
            {"name": "point4", "position": [2.2, 0.5, 0]},
            {"name": "point5", "position": [2.26, -0.2, 0]},
            {"name": "point6", "position": [2.17, -0.8, 0]},
            {"name": "point7", "position": [1.0, -0.1, 0]}
        ]

        self.current_waypoint_index = 0  # Current waypoint index
        self.dynamic_goal = None  # Dynamic goal received via topic

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Action server ready. Starting navigation...")

        # Create subscription for dynamic goals
        self.subscription = self.create_subscription(
            Point,
            '/dynamic_goal',
            self.dynamic_goal_callback,
            10
        )

        # Start navigating to the first waypoint
        self.send_goal()

    def dynamic_goal_callback(self, msg):
        """Handle dynamic goal updates from the topic."""
        self.get_logger().info(f"Received dynamic goal: x={msg.x}, y={msg.y}")
        self.dynamic_goal = {"name": "dynamic_goal", "position": [msg.x, msg.y, 0]}

    def send_goal(self):
        """Send a navigation goal to the robot."""
        # Determine which goal to send (dynamic or waypoint)
        if self.dynamic_goal:
            # Check if the dynamic goal is closer than the next waypoint
            current_pose = self.get_current_pose()
            distance_to_dynamic_goal = self.calculate_distance(
                current_pose['x'], current_pose['y'],
                self.dynamic_goal['position'][0], self.dynamic_goal['position'][1]
            )
            waypoint = self.waypoints[self.current_waypoint_index]
            distance_to_next_waypoint = self.calculate_distance(
                current_pose['x'], current_pose['y'],
                waypoint['position'][0], waypoint['position'][1]
            )

            if distance_to_dynamic_goal < distance_to_next_waypoint:
                self.get_logger().info(f"Dynamic goal is closer. Navigating to dynamic goal at {self.dynamic_goal['position']}")
                waypoint = self.dynamic_goal
                self.dynamic_goal = None  # Reset dynamic goal after navigating
            else:
                self.get_logger().info(f"Dynamic goal is farther. Continuing to {waypoint['name']} at {waypoint['position']}")
        else:
            waypoint = self.waypoints[self.current_waypoint_index]

        # Prepare goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['position'][0]
        goal_msg.pose.pose.position.y = waypoint['position'][1]
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        # Send goal
        self.get_logger().info(f"Sending goal to {waypoint['name']} at {waypoint['position']}")
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
            self.get_logger().info(f"Successfully reached waypoint {self.waypoints[self.current_waypoint_index]['name']}")
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        else:
            self.get_logger().error("Failed to reach waypoint. Retrying...")

        # Continue to the next waypoint
        self.send_goal()

    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_current_pose(self):
        """Retrieve the current robot pose (mock implementation)."""
        # Replace this with actual data from /amcl_pose or /odom
        # For now, return a fixed position for testing
        return {"x": 0.0, "y": 0.0}


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
