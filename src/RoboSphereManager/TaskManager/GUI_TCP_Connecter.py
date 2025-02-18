import socket
import json
from rclpy.node import Node
from interface_package.msg import PointAndStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
import numpy as np

class BatterySubscriber(Node):
    def __init__(self, namespace):
        super().__init__('battery_subscriber')
        self.data = 0.0
        self.subscription = self.create_subscription(
            Float32,
            f'/{namespace}/pinky_battery_present',
            self.battery_callback,
            10  # 큐 크기
        )
        self.subscription  # prevent unused variable warning

    def battery_callback(self, msg):
        self.data = msg.data
        self.get_logger().info(f'현재 배터리 상태: {msg.data}')
        # self.send_tcp_signal(msg.data)
    
    def send_signal_over_tcp(self, signal):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if self.namespace == "pinky1":
                s.connect(('localhost', 9152))  # 서버 주소와 포트
            else :
                s.connect(('localhost', 9153))  # 서버 주소와 포트
            # signal to json
            s.sendall(json.dumps(signal).encode('utf-8'))

    # receive table_id and send signal over tcp
    def send_tcp_signal(self, data):
        # make signal
        signal = {
            'bettery': data
        }
        self.get_logger().info(f'data : {data}')

        # send signal over tcp
        self.send_signal_over_tcp(signal)
        self.get_logger().info(f'signal : {signal}')

class StatusSubscriber(Node):
    def __init__(self, namespace):
        super().__init__('status_subscriber')
        self.subscription = self.create_subscription(
            PointAndStatus,
            f'/{namespace}/status_publisher',
            self.status_callback,
            10  # 큐 크기
        )
        self.subscription  # prevent unused variable warning
        print(f'/{namespace}/status_publisher',)
        self.namespace = namespace

    def status_callback(self, msg):
        self.data = msg.status
        self.get_logger().info(f'status: {msg}')
        self.send_tcp_signal(msg)
    
    def send_signal_over_tcp(self, signal):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if self.namespace == "pinky1":
                s.connect(('localhost', 9156))  # 서버 주소와 포트
            else :
                s.connect(('localhost', 9155))  # 서버 주소와 포트
            # signal to json
            s.sendall(json.dumps(signal).encode('utf-8'))

    # receive table_id and send signal over tcp
    def send_tcp_signal(self, data):
        # make signal
        # signal = {
        #     'data': self.point_and_status_to_dict(data)
        # }
        signal = self.point_and_status_to_dict(data)
        self.get_logger().info(f'data : {data}')

        # send signal over tcp
        self.send_signal_over_tcp(signal)
        self.get_logger().info(f'signal : {signal}')

    def point_and_status_to_dict(self, obj):
        return {
            'status': obj.status,
            'current_position': {
                'x': obj.current_position.x,
                'y': obj.current_position.y,
                'z': obj.current_position.z
            },
            'start_position': {
                'x': obj.start_position.x,
                'y': obj.start_position.y,
                'z': obj.start_position.z
            },
            'goal_position': {
                'x': obj.goal_position.x,
                'y': obj.goal_position.y,
                'z': obj.goal_position.z
            }
        }
    

def main():
    print("Starting GUI TCP Test Client...")
    rclpy.init()
    node1 = BatterySubscriber(namespace="pinky1")
    node2 = StatusSubscriber(namespace="pinky1")
    node3 = BatterySubscriber(namespace="pinky2")
    node4 = StatusSubscriber(namespace="pinky2")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.add_node(node3)
    executor.add_node(node4)

    try:
        executor.spin()  # rclpy.spin(node) 대신 사용
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

    # send_tcp_signal("exit", 2, (0.0, 0.0))
    # 여기에 실행 로직 추가
    # send_tcp_signal("request", 1)

if __name__ == "__main__":
    main()
