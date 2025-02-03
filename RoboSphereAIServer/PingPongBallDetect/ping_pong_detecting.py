import socket
import numpy as np
import cv2
from ultralytics import YOLO
from tf.transformations import euler_from_quaternion
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CameraUDPClient(Node):
    def __init__(self):
        super().__init__('camera_udp_client')
        
        # UDP 설정
        udp_ip = "0.0.0.0"
        udp_port = 34343
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.get_logger().info(f"Listening for UDP packets on {udp_ip}:{udp_port}...")

        # 카메라 및 모델 설정
        self.K = np.array([[273.04999718, 0, 155.82542991], [0, 274.11381476, 121.70811418], [0, 0, 1]])
        self.c_x = self.K[0, 2]
        self.c_y = self.K[1, 2]
        self.f_x = self.K[0, 0]
        self.f_y = self.K[1, 1]
        self.REAL_DIAMETER = 40
        self.distance_offset = 20
        self.camera_resolution_x = 320
        self.camera_resolution_y = 240
        self.model = YOLO("./model_train/best.pt")
        self.ping_pong_coordinates = []

        # 로봇의 위치 및 방향 초기화
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.yaw = 0.0

        # ROS 2 구독 설정
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.robot_position_x = position.x
        self.robot_position_y = position.y
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.get_logger().info(f"Updated robot position: ({self.robot_position_x}, {self.robot_position_y}), yaw: {self.yaw}")

    def process_frame(self):
        try:
            data, addr = self.sock.recvfrom(65536)
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                frame = cv2.resize(frame, (self.camera_resolution_x, self.camera_resolution_y))
                result = self.model(frame, verbose=False)
                detected_frame = result[0].plot()

                for result in result[0].boxes:
                    box = result.xyxy[0]
                    x1, y1, x2, y2 = map(int, box[:4])
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    pixel_width = x2 - x1

                    if pixel_width > 0:
                        distance = ((self.f_x * self.REAL_DIAMETER) / pixel_width) + self.distance_offset
                        z_distance = distance
                    else:
                        distance = -1
                        z_distance = -1

                    camera_x = ((x_center - self.c_x) * distance) / self.f_x
                    camera_y = ((y_center - self.c_y) * distance) / self.f_y
                    camera_z = z_distance

                    robot_x = self.robot_position_x + (camera_x * np.cos(self.yaw) - camera_y * np.sin(self.yaw))
                    robot_y = self.robot_position_y + (camera_x * np.sin(self.yaw) + camera_y * np.cos(self.yaw))

                    self.get_logger().info(f"Robot relative coordinate: ({robot_x}, {robot_y}, {camera_z})")

                    # 객체 인식 결과 저장 및 비교
                    self.update_coordinates((robot_x, robot_y, camera_z))

                cv2.imshow("ping pong", detected_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def update_coordinates(self, new_coord):
        # 기존 좌표와 비교하여 업데이트 로직 추가
        self.ping_pong_coordinates.append(new_coord)
        self.get_logger().info(f"Updated coordinates: {self.ping_pong_coordinates}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraUDPClient()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.process_frame()
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()