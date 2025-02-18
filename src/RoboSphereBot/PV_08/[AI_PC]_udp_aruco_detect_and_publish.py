#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import socket
import numpy as np
import cv2
from utils import ARUCO_DICT

# UDP 설정
UDP_IP = "192.168.4.11"  # 모든 IP에서 수신
UDP_PORT = 34344    # UDP 송신 코드와 동일한 포트

class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')

        # UDP 소켓 생성
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((UDP_IP, UDP_PORT))

        # ArUco 마커 Pose 퍼블리셔
        self.pose_publisher = self.create_publisher(Pose, '/aruco_pose', 10)

        self.aruco_dict_type = "DICT_5X5_100"

        # 카메라 보정 행렬 및 왜곡 계수 로드 (왜곡 계수는 사용하지 않음)
        self.camera_matrix = np.load('/home/uze/ws/aruco/pose_ArUCo/calibration_matrix.npy')

        self.get_logger().info("Aruco Pose Estimator Node started with UDP!")

        # UDP 데이터 수신 타이머 (약 15 FPS)
        self.timer = self.create_timer(1 / 15, self.receive_frame)

    def receive_frame(self):
        try:
            # UDP 데이터 수신
            data, _ = self.udp_socket.recvfrom(65536)

            # 수신된 데이터가 있는 경우, OpenCV 이미지로 변환
            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("Received an empty frame from UDP.")
                return

            # ArUco 마커 감지 및 Pose 추정
            tvec, rvec = self.pose_estimation(frame)

            if tvec is not None and rvec is not None:
                # Pose 메시지 생성
                pose_msg = Pose()
                pose_msg.position.x = tvec[0][0][0]
                pose_msg.position.y = tvec[0][0][1]
                pose_msg.position.z = tvec[0][0][2]

                # 회전 벡터를 쿼터니언으로 변환
                rotation_matrix, _ = cv2.Rodrigues(rvec[0][0])
                pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = \
                    self.rotation_matrix_to_quaternion(rotation_matrix)

                # 퍼블리시
                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f"Published Pose: {pose_msg.position}, {pose_msg.orientation}")

            # OpenCV로 영상 출력 (디버깅용)
            cv2.imshow("Aruco Pose Estimation", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to receive or process frame: {e}")

    def pose_estimation(self, frame):
        """ArUco 마커 감지 및 Pose 추정"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_dict_type])
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], 0.05, self.camera_matrix, None  # 🔹 왜곡 계수 제거
                )

                # ArUco 마커를 OpenCV로 시각화
                cv2.aruco.drawDetectedMarkers(frame, corners)

                return tvec, rvec  # 변환된 좌표 반환
        return None, None

    @staticmethod
    def rotation_matrix_to_quaternion(matrix):
        """회전 행렬을 쿼터니언으로 변환"""
        q = np.empty(4)
        q[3] = np.sqrt(max(0, 1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2])) / 2
        q[0] = np.sqrt(max(0, 1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) / 2
        q[1] = np.sqrt(max(0, 1 - matrix[0, 0] + matrix[1, 1] - matrix[2, 2])) / 2
        q[2] = np.sqrt(max(0, 1 - matrix[0, 0] - matrix[1, 1] + matrix[2, 2])) / 2
        q[0] *= np.sign(q[0] * (matrix[2, 1] - matrix[1, 2]))
        q[1] *= np.sign(q[1] * (matrix[0, 2] - matrix[2, 0]))
        q[2] *= np.sign(q[2] * (matrix[1, 0] - matrix[0, 1]))
        return q

    def destroy_node(self):
        self.udp_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
