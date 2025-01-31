#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import time


class DynamicWaypointNavigator(Node):
    def __init__(self):
        super().__init__('dynamic_waypoint_navigator')

        # 기본 웨이포인트 리스트
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "theta": 1.57},
            {"x": 0.0, "y": 0.5, "theta": 1.57},
            {"x": -0.5, "y": 0.5, "theta": -1.57},
        ]
        self.current_waypoint_index = 0  # 현재 목표 웨이포인트 인덱스
        self.robot_stopped = False  # 로봇 정지 여부
        self.cooldown_start_time = None
        self.cooldown_time = 5.0  # 새로운 웨이포인트 추가 후 쿨다운 시간

        # 탁구공 감지 관련 변수
        self.previous_ball_coords = None
        self.stable_coords_time = None
        self.start_wait_time = None
        self.stability_threshold = 5.0  # 안정성 검사 시간 (초)
        self.max_coord_deviation = 0.1  # 좌표 변동 허용 범위

        # Publisher
        self.reached_goal_pub = self.create_publisher(Bool, '/reached_goal', 10)

        # Subscriber
        self.create_subscription(PointStamped, '/ping_pong_map_coords', self.ping_pong_callback, 10)

        # Navigation Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Action server ready. Starting navigation...")

        # 웨이포인트 주행 시작
        self.send_goal()

    def ping_pong_callback(self, msg):
        """탁구공 감지 및 안정적인 좌표 확인 후 웨이포인트 추가"""
        ball_x, ball_y = msg.point.x, msg.point.y

        # 쿨다운 중이면 무시
        if self.cooldown_start_time and time.time() - self.cooldown_start_time < self.cooldown_time:
            self.get_logger().info("Cooldown active. Ignoring new ping pong coordinates.")
            return

        # 로봇이 이동 중이면 즉시 정지
        if not self.robot_stopped:
            self.stop_robot()
            self.robot_stopped = True
            self.start_wait_time = time.time()
            self.stable_coords_time = time.time()
            self.previous_ball_coords = (ball_x, ball_y)
            self.get_logger().info("Ping pong ball detected. Stopping to check stability.")
            return

        # 좌표 안정성 확인
        prev_x, prev_y = self.previous_ball_coords
        deviation = math.sqrt((ball_x - prev_x) ** 2 + (ball_y - prev_y) ** 2)

        if deviation <= self.max_coord_deviation:
            if time.time() - self.stable_coords_time >= self.stability_threshold:
                self.get_logger().info(f"Ping pong ball is stable. Adding waypoint at x={ball_x}, y={ball_y}.")
                
                # 새로운 웨이포인트를 현재 목표 웨이포인트 이전에 삽입
                self.add_dynamic_waypoint(ball_x, ball_y)

                self.robot_stopped = False  # 로봇 정지 해제
                self.cooldown_start_time = time.time()  # 웨이포인트 추가 후 쿨다운 시작
                self.send_goal()  # 새로운 웨이포인트로 이동 시작
                return
        else:
            self.stable_coords_time = time.time()  # 변동 발생 시 안정성 체크 초기화
            self.previous_ball_coords = (ball_x, ball_y)
            self.get_logger().info("Ping pong ball coordinates are unstable. Resetting stability check.")

        # 안정성이 5초 동안 유지되지 않으면 원래 경로 유지
        if time.time() - self.start_wait_time >= self.stability_threshold:
            self.get_logger().warn("Stability check timed out. Resuming original navigation.")
            self.robot_stopped = False
            self.send_goal()

    def add_dynamic_waypoint(self, x, y):
        """현재 목표 웨이포인트 앞에 새로운 웨이포인트 삽입"""
        dynamic_waypoint = {"x": x, "y": y, "theta": 0.0}
        
        # 현재 이동 중인 웨이포인트 앞에 삽입
        self.waypoints.insert(self.current_waypoint_index, dynamic_waypoint)
        
        self.get_logger().info(f"Dynamic waypoint added at x={x}, y={y}. Moving to the ball first.")

    def stop_robot(self):
        """로봇을 정지시키고 현재 목표 취소"""
        self.get_logger().info("Stopping the robot to check ping pong ball stability.")
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.goal_cancel_callback)

    def goal_cancel_callback(self, future):
        """목표 취소 콜백"""
        try:
            cancel_response = future.result()
            if cancel_response:
                self.get_logger().info("Goal cancellation processed.")
            else:
                self.get_logger().warn("Goal cancellation failed or no response received.")
        except Exception as e:
            self.get_logger().error(f"Error in cancel callback: {e}")

    def send_goal(self):
        """웨이포인트 이동"""
        if self.robot_stopped:  # 로봇이 멈춘 상태라면 목표를 설정하지 않음
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Reached the final waypoint.")
            self.reached_goal_pub.publish(Bool(data=True))
            self.cooldown_start_time = None
            return

        waypoint = self.waypoints[self.current_waypoint_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint["x"]
        goal_msg.pose.pose.position.y = waypoint["y"]

        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
                               f"x={waypoint['x']}, y={waypoint['y']}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_sent_callback)

    def goal_sent_callback(self, future):
        """목표가 정상적으로 전송되었는지 확인하는 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the server.")
            return
        self.get_logger().info("Goal accepted by the server.")
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """목표 도달 후 다음 웨이포인트로 이동"""
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}.")
            self.current_waypoint_index += 1
            self.send_goal()
        else:
            self.get_logger().error("Failed to reach waypoint. Retrying current waypoint.")
            self.send_goal()


def main(args=None):
    """ROS 2 노드를 실행하는 메인 함수"""
    rclpy.init(args=args)
    node = DynamicWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
