#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from example_interfaces.srv import Trigger
from std_msgs.msg import Bool
import time
import numpy as np
from interface_package.srv import PathRequest
from interface_package.msg import PointAndStatus
from geometry_msgs.msg import Point  # 좌표를 다루기 위해 필요
from geometry_msgs.msg import PoseStamped
import json


class DynamicWaypointNavigator(Node):
    def __init__(self, namespace):
        super().__init__('dynamic_waypoint_navigator')

        self.srv = self.create_service(PathRequest, f'/{namespace}/task_and_path_listener', self.handle_task_request)
        self.status_publisher = self.create_publisher(PointAndStatus, f'/{namespace}/status_publisher', 3)
        self.subscription = self.create_subscription(
            PoseStamped,  # /tracked_pose가 PoseStamped 형식이라고 가정
            # f'/{namespace}/tracked_pose_transfer',
            "/pinky1/tracked_pose_transfer",
            self.tracked_pose_callback,
            3)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.status = "Idle"
        self.target = None

    def handle_task_request(self, request, response):
        # Log the received request data
        self.get_logger().info(f'Received path request: {request.request_data}')

        if not request.request_data or not request.request_data.strip():
            self.get_logger().error("Received empty request data.")
            response.success = False
            response.message = "Request data is empty."
            return response

        try:
            # JSON 문자열을 딕셔너리로 변환
            parsed_data = json.loads(request.request_data)

            robot_id = parsed_data.get("robot_id", "Unknown")
            command = int(parsed_data.get("command", 0))
            table_id = int(parsed_data.get("table_id", -1))
            target_list = parsed_data.get("target", [[0, 0]])  # 기본값으로 2D 리스트 설정

            # 리스트를 numpy 배열로 변환
            target_array = np.array(target_list)

            # self.get_logger().info(f"Received Target: \n{target}")
            self.get_logger().info(f"Parsed Data -> Robot ID: {robot_id}, Command: {command}, Table ID: {table_id}, Target: {target_array}")

            self.target = target_array

            # 웨이포인트 변환 및 theta 설정
            self.waypoints = self.generate_waypoints(self.target)

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

            self.status = "Walking"

            response.success = True
            response.message = "Task received successfully."
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def tracked_pose_callback(self, msg):
        """ /tracked_pose 메시지를 받아서 변환 후 /{namespace}/tracked_pose_transfer 로 퍼블리시 """
        self.reception = PoseStamped()
        self.reception.header = msg.header  # 기존 헤더 유지
        self.reception.pose = msg.pose      # 기존 위치 & 자세 유지

        # 변환된 데이터 퍼블리시
        # self.publisher.publish(self.reception)
        self.get_logger().info(f'📡 Published Converted Pose: {self.reception.pose.position.x}, {self.reception.pose.position.y}, {self.reception.pose.position.z}')
    
    def publish_message(self):
        msg = PointAndStatus()
        msg.status = self.status

        # self.reception

        if self.target != None:
            # 현재 좌표 설정
            msg.current_position = Point()
            msg.current_position.x = self.reception.pose.position.x
            msg.current_position.y = self.reception.pose.position.y
            msg.current_position.z = self.reception.pose.position.z

            # 시작 좌표 설정
            msg.start_position = Point()
            msg.start_position.x = self.target[0,0]
            msg.start_position.y = self.target[0,1]
            msg.start_position.z = np.arctan2(self.target[1,1] - self.target[0,1], self.target[1,0] - self.target[0,0]) 

            # 목표 좌표 설정
            msg.goal_position = Point()
            msg.goal_position.x = self.target[-1,0]
            msg.goal_position.y = self.target[-1,1]
            msg.goal_position.z = np.arctan2(self.target[-1,1] - self.target[-2,1], self.target[-1,0] - self.target[-2,0]) 
        else :
            # 현재 좌표 설정
            msg.current_position = Point()
            msg.current_position.x = 0.0
            msg.current_position.y = 0.0
            msg.current_position.z = 0.0

            # 시작 좌표 설정
            msg.start_position = Point()
            msg.start_position.x = 0.0
            msg.start_position.y = 0.0
            msg.start_position.z = 0.0

            # 목표 좌표 설정
            msg.goal_position = Point()
            msg.goal_position.x = 0.0
            msg.goal_position.y = 0.0
            msg.goal_position.z = 0.0


        self.status_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')
    
    def generate_waypoints(self, target):
        """각 웨이포인트의 theta 값을 다음 좌표를 향하도록 자동 설정"""
        waypoints = [
            {
                "x": float(x),
                "y": float(y),
                "theta": math.atan2(target[i + 1][1] - y, target[i + 1][0] - x) if i < len(target) - 1 else 0.0
            }
            for i, (x, y) in enumerate(target)
        ]
        return waypoints

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


    # node = RobotService()  
    node = DynamicWaypointNavigator(namespace='pinky1') # 네임스페이스 지정
    # task_planner_service = TaskListenerService()

    try:
        rclpy.spin(node)
        # rclpy.spin(task_planner_service)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        # task_planner_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()