#!/usr/bin/env python3
import socket
import json
from queue import PriorityQueue
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import Trigger
from interface_package.srv import PathRequest
from interface_package.msg import PointAndStatus
from geometry_msgs.msg import Point  # 좌표를 다루기 위해 필요

task_queue = PriorityQueue()
robot_states = {}  # 로봇의 상태와 위치를 저장하는 전역 딕셔너리
is_all_robots_busy = False

def receive_tcp_signal():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('localhost', 9151))  # 서버 주소와 포트
            s.listen()
            print('Listening for incoming connections...')
            
            while True:
                conn, addr = s.accept()
                with conn:
                    print('Connected by', addr)
                    data = conn.recv(1024)
                    if not data:
                        break
                    signal = json.loads(data.decode('utf-8'))
                    process_signal(signal)
                    
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")
        exit()

    finally:
        print("clean up completed")            

def process_signal(signal):

    command = signal.get('command')
    target = signal.get('target')
    table_id = signal.get('table_id')

    # add task to queue
    task_queue.put((command, target, table_id))
    print(f"Added to queue: command={command}, target={target}, table_id={table_id}")
    
    # print current queue state
    print("Current queue state:")
    for item in list(task_queue.queue):
        print(item)

class PathPlannerClient(Node):

    def __init__(self):
        super().__init__('path_planner_client')
        self.client = self.create_client(PathRequest, 'path_planner_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_path_request(self, robot_id, command, table_id, target):
        # Create a request
        request = PathRequest.Request()
        request.request_data = f"Robot ID: {robot_id}, Command: {command}, Table ID: {table_id}, Target: {target}"
        
        # Call the service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')

def make_path(robot_id, command, table_id, target):
    if robot_id is None:
        print("No suitable robot found, skipping task.")
        return

    # 기존의 `rclpy.init()` 제거
    path_planner_client = PathPlannerClient()
    
    # 서비스 요청 전송
    path_planner_client.send_path_request(robot_id, command, table_id, target)
    
    # 노드 정리
    path_planner_client.destroy_node()

class RobotStatusSubscriber(Node):
    def __init__(self, namespace):
        super().__init__('robot_status_subscriber')
        self.namespace = namespace
        # PointAndStatus 메시지 타입으로 변경
        self.subscription = self.create_subscription(
            PointAndStatus,
            f'/{self.namespace}/status_publisher',
            self.listener_callback,
            3)
        self.subscription
        self.current_status = None
        self.current_position = None
        
    def listener_callback(self, msg):
        self.current_status = msg.status
        self.current_position = msg.current_position
        # 전역 딕셔너리에 상태와 위치 저장
        robot_states[self.namespace] = {
            'status': self.current_status,
            'position': self.current_position
        }
        self.get_logger().info(
            f'[{self.namespace}] Status: {self.current_status}, '
            f'Position: {self.current_position}'
        )

class Scheduler(Node):

    def __init__(self, pinky1_subscriber, pinky2_subscriber):
        super().__init__('scheduler')

        # RobotPoseSubscriber 인스턴스 저장
        self.robot_subscribers = {
            'pinky1': pinky1_subscriber,
            'pinky2': pinky2_subscriber
        }

        # 주기적 작업 추가
        self.timer = self.create_timer(300.0, self.add_periodic_task)

        # Task 실행 쓰레드 시작
        self.task_executor = threading.Thread(target=self.execute_tasks)
        self.task_executor.start()

    def add_periodic_task(self):
        """주기적으로 실행될 작업을 큐에 추가하는 메서드"""
        task = (1, None, None)
        task_queue.put(task)
        self.get_logger().info(f"Periodic task added to queue: {task}")

    # 목적지에 맞는 로봇의 ID를 반환, 남은 로봇이 없을 시 None 반환
    def get_task_robot_id(self, target):
        if not robot_states:
            return None
        
        if isinstance(target, str):
            target = target.replace("(", "").replace(")", "").split(",")  # 문자열을 튜플처럼 변환
            target = tuple(map(float, target))  # float로 변환

        # target을 Point 객체로 변환
        target_point = Point()
        target_point.x = float(target[0])
        target_point.y = float(target[1])
        target_point.z = 0.0
    
        # Idle 상태인 로봇 중에서 가장 가까운 로봇을 찾음
        closest_robot = None
        min_distance = float('inf')

        for namespace, state in robot_states.items():
            if state['status'] == "Idle":  # Idle 상태인 로봇만 고려
                distance = self.calculate_distance(state['position'], target_point)
                if distance < min_distance:
                    min_distance = distance
                    closest_robot = namespace

        return closest_robot

    def calculate_distance(self, position, target):
        # Calculate Euclidean distance between two points
        return ((position.x - target.x) ** 2 + 
                (position.y - target.y) ** 2) ** 0.5

    def check_available_robots(self):
        """로봇의 현재 상태를 확인하고 사용 가능한 로봇이 있는지 확인하는 함수"""
        for robot_id, state in self.robot_states.items():
            # state가 0이면 로봇이 유휴 상태(사용 가능)
            if state == 0:
                self.get_logger().info(f"Available robot found: {robot_id}")
                return True
        self.get_logger().info("No available robots - all robots are busy")
        return False

    def execute_tasks(self):
        global is_all_robots_busy
        
        # 모든 로봇이 작업 중이고, 사용 가능한 로봇이 없다면 실행하지 않음
        if is_all_robots_busy:
            if self.check_available_robots():
                is_all_robots_busy = False  # 사용 가능한 로봇이 있으면 플래그 초기화
            else:
                return  # 아직 모든 로봇이 작업 중이면 실행하지 않음
        
        while rclpy.ok():
            if not task_queue.empty():
                task = task_queue.get()
                self.process_task(task)

    def process_task(self, task):
        global is_all_robots_busy
        command, target, table_id = task

        """
        command
        1: 주기적 작업
        2: 종료 후 수거 작업
        3: 사용자 요청 수거 작업
        4: 서빙 작업
        """
        
        self.get_logger().info(f"Processing task: command={command}, target={target}, table_id={table_id}")

        if command:
            robot_id = self.get_task_robot_id(target)
            
            self.get_logger().info(f"Selected robot_id: {robot_id}, Type: {type(robot_id)}")
            
            if robot_id:
                robot_id = str(robot_id)
                make_path(robot_id, command, table_id, target)
                self.get_logger().info(f"Make Path: command={command}, target={target}, table_id={table_id}")
            else:
                # 로봇을 할당할 수 없는 경우
                is_all_robots_busy = True  # 전역 변수를 True로 설정
                self.task_queue.put(task)  # 작업을 다시 큐에 넣음
                self.get_logger().info("All robots are busy. Task requeued.")
        else:
            self.get_logger().info(f"Unknown command: {command}")

def main(args=None):
    rclpy.init(args=args)

    # Start receiving TCP signal
    tcp_thread = threading.Thread(target=receive_tcp_signal)
    tcp_thread.start()

    # Create subscribers for each robot
    pinky1_subscriber = RobotStatusSubscriber('pinky1')
    pinky2_subscriber = RobotStatusSubscriber('pinky2')

    # Create scheduler (구독자를 전달)
    scheduler = Scheduler(pinky1_subscriber, pinky2_subscriber)

    # Spin all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pinky1_subscriber)
    executor.add_node(pinky2_subscriber)
    executor.add_node(scheduler)

    try:
        executor.spin()
    finally:
        pinky1_subscriber.destroy_node()
        pinky2_subscriber.destroy_node()
        scheduler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()