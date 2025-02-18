import socket
import numpy as np
import cv2
from ultralytics import YOLO
from transforms3d.euler import quat2euler
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker

class CameraUDPClient(Node):
    def __init__(self):
        super().__init__('camera_udp_client')
        
        # udp setting
        udp_ip = "0.0.0.0"
        udp_port = 34343
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.get_logger().info(f"Listening for UDP packets on {udp_ip}:{udp_port}...")

        # camera intrinsic matrix
        self.K = np.array([[273.04999718, 0, 155.82542991], [0, 274.11381476, 121.70811418], [0, 0, 1]])

        # camera center
        self.c_x = self.K[0, 2]
        self.c_y = self.K[1, 2]

        # camera focal length
        self.f_x = self.K[0, 0]
        self.f_y = self.K[1, 1]

        # real diameter
        self.REAL_DIAMETER = 40

        # distance offset
        self.distance_offset = 20

        # camera resolution
        self.camera_resolution_x = 320
        self.camera_resolution_y = 240

        # 카메라 위치 (로봇 기준)
        self.camera_offset_x = 0.06  # 로봇 앞쪽 6cm
        self.camera_offset_y = 0.0   # 로봇 정중앙 (좌우 오프셋 없음)
        self.camera_offset_z = -0.06  # 바닥 아래로 6cm

        self.robot_position = np.zeros(3)  # x, y, z
        self.robot_orientation = [0, 0, 0, 1]  # 쿼터니언 (x, y, z, w)

        # model
        self.model = YOLO("./best.pt")

        # ping pong coordinates
        self.ping_pong_coordinates = []

        # robot position and direction
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.robot_position_z = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # robot 3d world position and orientation subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )

        # 퍼블리셔 설정
        self.point_publisher = self.create_publisher(PointStamped, 'detected_points', 10)
        self.marker_publisher = self.create_publisher(Marker, 'detected_markers', 10)

    def pose_callback(self, msg):
        # robot 3d world position and orientation
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.robot_position_x = position.x
        self.robot_position_y = position.y
        self.roll, self.pitch, self.yaw = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        self.get_logger().info(f"Updated robot position: ({self.robot_position_x}, {self.robot_position_y}), yaw: {self.yaw}")

    def publish_point(self, world_x, world_y, world_z):
        point_msg = PointStamped()
        point_msg.header.frame_id = "map"  # SLAM 맵의 프레임 ID
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = world_x
        point_msg.point.y = world_y
        point_msg.point.z = world_z
        self.point_publisher.publish(point_msg)

    def publish_marker(self, world_x, world_y, world_z, marker_id):
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.id = marker_id  # 고유한 ID 설정
        marker_msg.pose.position.x = world_x
        marker_msg.pose.position.y = world_y
        marker_msg.pose.position.z = world_z
        marker_msg.scale.x = 0.1  # 마커의 크기
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1 
        marker_msg.color.a = 1.0  # 투명도
        marker_msg.color.r = 1.0  # 빨간색
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        self.marker_publisher.publish(marker_msg)

    def process_frame(self):
        try:
            data, addr = self.sock.recvfrom(65536)
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                frame = cv2.resize(frame, (self.camera_resolution_x, self.camera_resolution_y))
                result = self.model(frame, verbose=False)
                detected_frame = result[0].plot()

                for idx, result in enumerate(result[0].boxes):
                    box = result.xyxy[0]
                    x1, y1, x2, y2 = map(int, box[:4])

                    x_center = int((x1 + x2) / 2)
                    pixel_width = x2 - x1

                    if pixel_width > 0:
                        distance = ((self.f_x * self.REAL_DIAMETER) / pixel_width) * 0.001  # mm to m
                        camera_x = ((x_center - self.c_x) * distance) / self.f_x
                        camera_y = -0.06  # y는 항상 로봇 기준 바닥에서 -6cm
                        camera_z = distance

                        # 카메라 -> 로봇 좌표 변환
                        robot_x = camera_z + self.camera_offset_x  # 카메라 z축 -> 로봇 x축
                        robot_y = -camera_x + self.camera_offset_y  # 카메라 x축 -> 로봇 y축

                        # 로봇 -> 지도 좌표 변환
                        R = self.quaternion_to_rotation_matrix(*self.robot_orientation)
                        map_coords = np.dot(R, [robot_x, robot_y, 0]) + self.robot_position

                        self.publish_point(map_coords[0], map_coords[1], 0.0)
                        self.publish_marker(map_coords[0], map_coords[1], 0.0, idx)

                        self.get_logger().info(f"Published to map: {map_coords}")

                cv2.imshow("ping pong", detected_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def update_coordinates(self, new_coord):
        is_duplicate = False
        for coord in self.ping_pong_coordinates:
            # 두 좌표 간의 거리 계산
            distance = np.linalg.norm(np.array(coord) - np.array(new_coord))
            if distance < self.REAL_DIAMETER: # 40 mm 이내면 중복으로 간주
                is_duplicate = True
                break

        if not is_duplicate:
            self.ping_pong_coordinates.append(new_coord)
            self.get_logger().info(f"Updated coordinates: {self.ping_pong_coordinates}")
        else:
            self.get_logger().info(f"Duplicate coordinate detected: {new_coord}")

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """쿼터니언을 회전 행렬로 변환"""
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2 * (qy**2 + qz**2)
        R[0, 1] = 2 * (qx * qy - qz * qw)
        R[0, 2] = 2 * (qx * qz + qy * qw)
        R[1, 0] = 2 * (qx * qy + qz * qw)
        R[1, 1] = 1 - 2 * (qx**2 + qz**2)
        R[1, 2] = 2 * (qy * qz - qx * qw)
        R[2, 0] = 2 * (qx * qz - qy * qw)
        R[2, 1] = 2 * (qy * qz + qx * qw)
        R[2, 2] = 1 - 2 * (qx**2 + qy**2)
        return R

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