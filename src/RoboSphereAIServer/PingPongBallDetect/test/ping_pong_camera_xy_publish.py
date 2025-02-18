#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker  # RViz Marker 메시지
import socket
import numpy as np
import cv2
from ultralytics import YOLO

class PingPongPublisher(Node):
    def __init__(self):
        super().__init__('ping_pong_publisher')

        # ROS 2 Publishers
        self.ping_pong_map_pub = self.create_publisher(PointStamped, '/ping_pong_map_coords', 10)
        self.marker_pub = self.create_publisher(Marker, '/ping_pong_marker', 10)

        # ROS 2 Subscriber for robot position
        self.create_subscription(PoseStamped, '/tracked_pose', self.robot_pose_callback, 10)

        # 로봇 위치 및 자세 초기화
        self.robot_position = np.zeros(3)  # x, y, z
        self.robot_orientation = [0, 0, 0, 1]  # 쿼터니언 (x, y, z, w)

        # UDP 설정
        self.udp_ip = "0.0.0.0"
        self.udp_port = 34344
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"Listening for UDP packets on {self.udp_ip}:{self.udp_port}...")

        # 카메라 매트릭스 및 왜곡 계수
        self.K = np.array(
            [
                [255.4956081, 0, 163.1243505],
                [0, 256.37554862, 132.57356488],
                [0, 0, 1]
            ]
        )
        self.dist = np.array([[0.03083389, -0.29139029, 0.01392937, -0.00118382, 0.26052502]])

        # 카메라 위치 (로봇 기준)
        self.camera_offset_x = 0.06  # 로봇 앞쪽 6cm
        self.camera_offset_y = 0.0   # 로봇 정중앙 (좌우 오프셋 없음)
        self.camera_offset_z = -0.06  # 바닥 아래로 6cm

        # YOLO 모델 로드
        self.model = YOLO("/home/uze/ws/robosphere/src/best.pt")

        # ROS 타이머로 주기적인 호출 설정 (30 FPS 기준)
        self.timer = self.create_timer(1 / 30.0, self.process_frame)

        # 상태 초기화
        self.marker_ids = []  # 활성화된 마커 ID들을 추적
        self.next_marker_id = 0  # 다음 사용할 마커 ID

    def robot_pose_callback(self, msg):
        """로봇의 현재 위치와 자세를 업데이트"""
        self.robot_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.robot_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

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

    def process_frame(self):
        try:
            # UDP 데이터 수신
            data, addr = self.sock.recvfrom(65536)
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("Received an empty frame from UDP.")
                return

            # 프레임 왜곡 보정
            frame_undistorted = cv2.undistort(frame, self.K, self.dist)
            frame_resized = cv2.resize(frame_undistorted, (320, 240))

            # YOLO로 탁구공 감지
            result = self.model(frame_resized, verbose=False)

            if len(result[0].boxes) == 0:
                self.get_logger().warn("No objects detected by YOLO.")
                # 모든 활성 마커를 제거
                for marker_id in self.marker_ids[:]:
                    self.clear_marker(marker_id)
                self.marker_ids.clear()  # 마커 ID 목록 초기화
                return

            # 결과 처리
            for box in result[0].boxes:
                confidence = box.conf[0]  # confidence score
                if confidence >= 0.8:  # confidence가 0.8 이상인 경우만 처리
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # 탁구공 중심 좌표 계산 (픽셀 기준)
                    x_center = int((x1 + x2) / 2)
                    pixel_width = x2 - x1
                    if pixel_width > 0:
                        # 거리 및 카메라 좌표 계산
                        distance = ((self.K[0, 0] * 40) / pixel_width) / 1000.0  # mm -> meters
                        camera_x = ((x_center - self.K[0, 2]) * distance) / self.K[0, 0]
                        camera_y = -0.06  # y는 항상 로봇 기준 바닥에서 -6cm
                        camera_z = distance

                        # 카메라 -> 로봇 좌표 변환
                        robot_x = camera_z + self.camera_offset_x  # 카메라 z축 -> 로봇 x축
                        robot_y = -camera_x + self.camera_offset_y  # 카메라 x축 -> 로봇 y축 (부호 반전 추가)

                        # 로봇 -> 지도 좌표 변환
                        R = self.quaternion_to_rotation_matrix(*self.robot_orientation)
                        map_coords = np.dot(R, [robot_x, robot_y, 0]) + self.robot_position

                        # ROS 메시지 생성 및 게시
                        point_msg = PointStamped()
                        point_msg.header.frame_id = "map"
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.point.x, point_msg.point.y = map_coords[0], map_coords[1]
                        point_msg.point.z = 0.0  # 높이는 필요 없으므로 0으로 설정
                        self.ping_pong_map_pub.publish(point_msg)
                        self.publish_marker(map_coords[0], map_coords[1])
                        self.get_logger().info(f"Published to map: {map_coords}")
                    else:
                        self.get_logger().warn("Invalid pixel width for detected object.")

        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {e}")

    def publish_marker(self, x, y):
        """Publish a marker to RViz to show the ping pong ball."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.next_marker_id  # 새로운 ID 사용
        self.marker_ids.append(self.next_marker_id)
        self.next_marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # 투명도
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        
        # 일정 개수(예: 10개) 이상의 마커가 쌓이면 오래된 것부터 제거
        if len(self.marker_ids) > 10:
            self.clear_marker(self.marker_ids.pop(0))

    def clear_marker(self, marker_id):
        """Clear the specific marker from RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PingPongPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
