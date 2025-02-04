import socket
import json
from queue import PriorityQueue
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import Trigger

task_queue = PriorityQueue()

# 전역 변수로 모든 로봇의 위치를 저장
robot_positions = {}

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
        print("celanup completed")            

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

# # start receiving tcp signal
# tcp_thread = threading.Thread(target=receive_tcp_signal)
# tcp_thread.start()

class PathPlannerClient(Node):

    def __init__(self):
        super().__init__('path_planner_client')
        self.client = self.create_client(Trigger, 'path_planner_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_path_request(self, robot_id, command, table_id, target):
        # Create a request
        request = Trigger.Request()
        request.data = f"Robot ID: {robot_id}, Command: {command}, Table ID: {table_id}, Target: {target}"
        
        # Call the service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')

def make_path(robot_id, command, table_id, target):
    # Create a node for the service client
    rclpy.init()
    path_planner_client = PathPlannerClient()
    
    # Send the path request
    path_planner_client.send_path_request(robot_id, command, table_id, target)
    
    # Shutdown the node after sending the request
    path_planner_client.destroy_node()
    rclpy.shutdown()

class RobotPoseSubscriber(Node):

    def __init__(self, namespace):
        super().__init__('robot_pose_subscriber')
        self.namespace = namespace
        self.subscription = self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/tracked_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_pose = None

    def listener_callback(self, msg):
        self.current_pose = msg.pose
        robot_positions[self.namespace] = self.current_pose
        self.get_logger().info(f'[{self.namespace}] Current Pose: {self.current_pose}')

    def get_task_robot_id(self, target):
        if self.current_pose is None:
            return None

        # 모든 로봇의 위치를 비교하여 가장 가까운 로봇을 찾음
        closest_robot = None
        min_distance = float('inf')

        for namespace, pose in robot_positions.items():
            distance = self.calculate_distance(pose, target)
            if distance < min_distance:
                min_distance = distance
                closest_robot = namespace

        return closest_robot

    def calculate_distance(self, pose, target):
        # Calculate Euclidean distance between two points
        return ((pose.position.x - target.x) ** 2 + 
                (pose.position.y - target.y) ** 2) ** 0.5

class Scheduler(Node):

    def __init__(self):
        super().__init__('scheduler')

        # timer to add periodic task
        self.timer = self.create_timer(300.0, self.add_periodic_task)

        # task executor
        self.task_executor = threading.Thread(target=self.execute_tasks)
        self.task_executor.start()

    # add periodic task to queue
    def add_periodic_task(self):
        task = (1, None, None)
        task_queue.put(task)
        self.get_logger().info(f"Periodic task added to queue: {task}")

    def execute_tasks(self):
        while rclpy.ok():
            if not task_queue.empty():
                task = task_queue.get()
                self.process_task(task)

    def process_task(self, task):
        command, target, table_id = task

        if command == 1:
            # this command is for periodic task
            self.get_logger().info(f"Periodic task: command={command}, start periodic robot task")

        elif command == 2 or command == 3:
            # command 2 is all area, command 3 is user requests
            self.get_logger().info(f"Table all task: command={command}, table_id={table_id}, target locate={target}")

            make_path( get_task_robot_id(target), command, table_id, target)

        else:
            self.get_logger().info(f"Unknown command: {command}")

        self.get_logger().info(f"Processing task: command={command}, target={target}, table_id={table_id}")

def main(args=None):
    rclpy.init(args=args)

    # Start receiving TCP signal
    tcp_thread = threading.Thread(target=receive_tcp_signal)
    tcp_thread.start()

    # Create subscribers for each robot
    pinky1_subscriber = RobotPoseSubscriber('pinky1')
    pinky2_subscriber = RobotPoseSubscriber('pinky2')

    # Create scheduler
    scheduler = Scheduler()

    # Spin all nodes
    rclpy.spin(pinky1_subscriber)
    rclpy.spin(pinky2_subscriber)
    rclpy.spin(scheduler)

    # Destroy the nodes explicitly
    pinky1_subscriber.destroy_node()
    pinky2_subscriber.destroy_node()
    scheduler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()