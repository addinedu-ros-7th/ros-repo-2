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
import math


class DynamicWaypointNavigator(Node):
    def __init__(self, namespace):
        super().__init__('dynamic_waypoint_navigator')

        self.reception = PoseStamped()
        self.srv = self.create_service(PathRequest, f'/{namespace}/task_and_path_listener', self.handle_task_request)
        self.status_publisher = self.create_publisher(PointAndStatus, f'/{namespace}/status_publisher', 3)
        self.subscription = self.create_subscription(
            PoseStamped,  # /tracked_pose가 PoseStamped 형식이라고 가정
            "/pinky1/tracked_pose",
            self.tracked_pose_callback,
            3)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.status = "Idle"
        self.target = None

        # publisher 생성 추가
        self.publisher = self.create_publisher(
            PoseStamped,
            f'/{namespace}/tracked_pose_transfer',
            3
        )

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

            if command == 4:
                self.get_logger().info(f'Processing command {command}')
                # "table_id"와 "target"을 실제 리스트로 변환
                table_id = json.loads(parsed_data["table_id"])  # "[1, 2, 4]" 형식에서 실제 리스트로 변환
                target_list = json.loads(parsed_data["target"])      # "[[...], [...]]" 형식에서 실제 2D 리스트로 변환

                self.get_logger().debug(f'Data types - table_id: {type(table_id)}, target_list: {type(target_list)}')
            else :
                self.get_logger().info(f'Processing command {command}')
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

            self.last_ping_pong_time = time.time()  #  마지막으로 탁구공이 감지된 시간
            self.max_wait_time = 5.0  #  탁구공 감지가 없으면 5초 후 원래 웨이포인트로 이동
            self.excluded_zones = [
            #    {"x_min": 0.41, "x_max": 0.97, "y_min": -1.4, "y_max": -1.06},  #  영역 1
            #   {"x_min": 1.76, "x_max": 2.21, "y_min": -0.69, "y_max": -0.38}  #  영역 2
            ]

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

            # 주기적으로 탁구공 감지 여부를 확인하는 타이머 추가
            # self.create_timer(10.0, self.check_ping_pong_timeout)

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def tracked_pose_callback(self, msg):
        """ /tracked_pose 메시지를 받아서 변환 후 /{namespace}/tracked_pose_transfer 로 퍼블리시 """
        
        self.reception.header = msg.header  # 기존 헤더 유지
        self.reception.pose = msg.pose      # 기존 위치 & 자세 유지

        # 변환된 데이터 퍼블리시
        self.publisher.publish(self.reception)

        # 좌표 정보는 INFO 레벨로 로깅
        self.get_logger().info(
            f'📡 Tracked Pose - '
            f'Position: x={self.reception.pose.position.x}, '
            f'y={self.reception.pose.position.y}, '
            f'z={self.reception.pose.position.z}'
        )

    def publish_message(self):
        msg = PointAndStatus()
        msg.status = self.status

        # 현재 좌표 설정 (tracked_pose에서 받아온 현재 위치)
        msg.current_position = Point()
        msg.current_position.x = self.reception.pose.position.x
        msg.current_position.y = self.reception.pose.position.y
        msg.current_position.z = self.reception.pose.position.z

        if self.target is not None:
            # 목표가 설정된 경우
            # 시작 좌표는 목표가 설정된 시점의 현재 위치로 고정
            if not hasattr(self, 'start_position'):
                self.start_position = Point()
                self.start_position.x = self.reception.pose.position.x
                self.start_position.y = self.reception.pose.position.y
                self.start_position.z = self.reception.pose.position.z
            
            msg.start_position = self.start_position
            
            # 목표 좌표 설정 (경로의 마지막 지점)
            msg.goal_position = Point()
            msg.goal_position.x = self.target[-1,0]
            msg.goal_position.y = self.target[-1,1]
            msg.goal_position.z = np.arctan2(
                self.target[-1,1] - self.target[-2,1],
                self.target[-1,0] - self.target[-2,0]
            )
        else:
            # 목표가 없는 경우
            # 시작 좌표는 현재 위치로 계속 업데이트
            msg.start_position = Point()
            msg.start_position.x = self.reception.pose.position.x
            msg.start_position.y = self.reception.pose.position.y
            msg.start_position.z = self.reception.pose.position.z
            
            # 목표 좌표는 0으로 설정
            msg.goal_position = Point()
            msg.goal_position.x = 0.0
            msg.goal_position.y = 0.0
            msg.goal_position.z = 0.0

            # 저장된 시작 위치 초기화
            if hasattr(self, 'start_position'):
                delattr(self, 'start_position')

        # 메시지 발행
        self.status_publisher.publish(msg)
        
        # 좌표 정보는 INFO 레벨로 로깅
        self.get_logger().info(
            f'Status: {self.status}\n'
            f'Current: ({msg.current_position.x}, {msg.current_position.y})\n'
            f'Start: ({msg.start_position.x}, {msg.start_position.y})\n'
            f'Goal: ({msg.goal_position.x}, {msg.goal_position.y})'
        )
    
    def generate_waypoints(self, target):
        """각 웨이포인트의 theta 값을 다음 좌표를 향하도록 설정,
        theta 값이 변하는 지점 + 마지막 웨이포인트만 선택하여 반환
        모든 웨이포인트에 오프셋 (+0.05, +0.05) 적용"""

        waypoints = []
        last_theta = None  # 이전 theta 값 저장

        for i in range(len(target)):
            if i < len(target) - 1:  # 다음 좌표를 향하도록 설정
                theta = math.atan2(target[i+1][1] - target[i][1], target[i+1][0] - target[i][0])
            else:  # 마지막 웨이포인트는 반대 방향
                theta = math.atan2(target[i][1] - target[i-1][1], target[i][0] - target[i-1][0]) + math.pi

            theta = round(theta, 2)  # 소수점 2자리로 반올림

            #  모든 웨이포인트에 오프셋 적용
            new_x = round(float(target[i][0]) + 0.1, 3)
            new_y = round(float(target[i][1]) - 0.1, 3)

            #  첫 번째 웨이포인트는 항상 추가, 이후 theta 차이가 0.1 이상일 경우 추가
            if last_theta is None or abs(theta - last_theta) > 0.1:
                waypoints.append({
                    "x": new_x,  
                    "y": new_y,  
                    "theta": theta
                })
                last_theta = theta  # 마지막 theta 갱신

        # 마지막 웨이포인트가 리스트에 포함되지 않았다면 추가 (오프셋 포함)
        final_x = round(float(target[-1][0]) + 0.1, 3)
        final_y = round(float(target[-1][1]) - 0.1, 3)
        final_theta = math.atan2(target[-1][1] - target[-2][1], target[-1][0] - target[-2][0]) + math.pi

        if waypoints[-1]["x"] != final_x or waypoints[-1]["y"] != final_y:
            waypoints.append({
                "x": final_x,  
                "y": final_y,  
                "theta": round(final_theta, 2)
            })

        # 로그 출력 (디버깅)
        for wp in waypoints:
            self.get_logger().info(f"Filtered waypoints: {wp}")

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

                # 제외 영역 검사 시작 로그 추가
                self.get_logger().info("Checking if the ball is in an excluded zone...")

                if self.is_excluded_zone(ball_x, ball_y):
                    self.get_logger().warn("Ball is in an excluded zone. Ignoring. Exiting ping_pong_callback function.")
                    self.robot_stopped = False

                    # 웨이포인트 이동을 확실하게 실행
                    self.get_logger().info("Resuming navigation to original waypoint.")

                    # 다시 감지하지 않도록 쿨다운 시작 (5초 동안 감지 무시)
                    self.cooldown_start_time = time.time()
                    self.get_logger().info(f"Cooldown started at {self.cooldown_start_time}, ignoring new detections for {self.cooldown_time} seconds.")

                    self.send_goal()  # 기존 웨이포인트로 이동하도록 보장
                    return

                # 제외 영역이 아닐 경우 웨이포인트 추가
                self.get_logger().info("Ball is not in an excluded zone. Adding waypoint...")
                self.add_dynamic_waypoint(ball_x, ball_y)

                self.robot_stopped = False
                self.cooldown_start_time = time.time()
                self.send_goal()
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

    def is_excluded_zone(self, x, y):
        """ 좌표가 여러 개의 제외 영역 중 하나에 포함되는지 확인"""
        for zone in self.excluded_zones:
            if zone["x_min"] <= x <= zone["x_max"] and zone["y_min"] <= y <= zone["y_max"]:
                return True  # 제외 영역에 포함됨
        return False  # 제외 영역이 아님
    
    def check_ping_pong_timeout(self):
        """ 일정 시간 동안 탁구공 감지가 없으면 원래 웨이포인트로 이동"""
        if time.time() - self.last_ping_pong_time >= self.max_wait_time:
            self.get_logger().warn("No ping pong ball detected for a while. Resuming original navigation.")
            if self.robot_stopped:
                self.robot_stopped = False
                self.send_goal()  # 원래 웨이포인트로 복귀

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
        """웨이포인트 이동 (필터링된 웨이포인트 적용)"""
        if self.robot_stopped:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Reached the final waypoint.")
            self.reached_goal_pub.publish(Bool(data=True))
            self.cooldown_start_time = None
            # 상태를 Idle로 변경
            self.status = "Idle"
            self.get_logger().info("Robot status changed to Idle")
            return

        waypoint = self.waypoints[self.current_waypoint_index]  # 필터링된 웨이포인트 사용

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # X, Y 위치 설정
        goal_msg.pose.pose.position.x = waypoint["x"]
        goal_msg.pose.pose.position.y = waypoint["y"]

        # Theta (Yaw) → Quaternion 변환
        theta = waypoint["theta"]
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2)

        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
                            f"x={waypoint['x']}, y={waypoint['y']}, theta={waypoint['theta']}")

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
