#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # ROS 2에서 '/image_raw' 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 퍼블리셔와 동일한 토픽 이름
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Camera Subscriber Node started!")

    def listener_callback(self, msg):
        try:
            # ROS 2 Image 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # OpenCV 창에 결과 표시
            cv2.imshow("Camera Stream", frame)

            # 키 입력 처리
            key = cv2.waitKey(1) & 0xFF
            
            # 's' 키를 누르면 현재 프레임을 저장
            if key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")  # 현재 시간을 기반으로 파일 이름 생성
                filename = f"./images/frame_{timestamp}.png"
                cv2.imwrite(filename, frame)  # 현재 프레임 저장
                self.get_logger().info(f"Saved frame as {filename}")

            # 'q' 키를 누르면 종료
            if key == ord('q'):
                self.get_logger().info("Shutting down Camera Subscriber Node...")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to process frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
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
