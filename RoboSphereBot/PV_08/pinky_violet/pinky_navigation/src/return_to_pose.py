#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class ReturnToSpecifiedPose(Node):
    def __init__(self):
        super().__init__('return_to_specified_pose')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("NavigateToPose action server ready.")

        # 원하는 위치로 이동 시작
        self.move_to_pose()

    def move_to_pose(self):
        # 목표 위치 및 방향 설정
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"  # 좌표계: 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정 (스크린샷 기반)
        goal_msg.pose.pose.position.x = 0.006567347204793159
        goal_msg.pose.pose.position.y = 0.009906382339869968
        goal_msg.pose.pose.position.z = 0.0

        # 방향 설정 (쿼터니언)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0012288850929873909
        goal_msg.pose.pose.orientation.w = 0.9999992449225066

        # 액션 서버로 목표 전송
        self.get_logger().info(f"Moving to position: {goal_msg.pose.pose.position}")
        self.get_logger().info(f"With orientation: {goal_msg.pose.pose.orientation}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Successfully reached the specified pose!")
        else:
            self.get_logger().error(f"Failed to reach the specified pose with status: {result.status}")

def main(args=None):
    rclpy.init(args=args)
    node = ReturnToSpecifiedPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
