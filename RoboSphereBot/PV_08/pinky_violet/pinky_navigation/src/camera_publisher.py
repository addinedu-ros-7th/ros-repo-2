#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from cv_bridge import CvBridge
import cv2
from libcamera import Transform

class PicameraPublisher(Node):
    def __init__(self):
        super().__init__('picamera_publisher')

        # ROS 2 퍼블리셔 생성
        self.publisher = self.create_publisher(Image, '/image_raw', 10)

        # Picamera2 초기화 및 출력 포맷 설정
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
                                    main={"size":(320,240)},
                                    transform=Transform(hflip=True, vflip=True))
        self.picam2.configure(config)
        self.picam2.start()

        # CvBridge 초기화
        self.cv_bridge = CvBridge()

        # 타이머 생성 (30 FPS)
        self.timer = self.create_timer(1 / 30, self.timer_callback)

    def timer_callback(self):
        try:
            # 카메라에서 프레임 캡처
            frame = self.picam2.capture_array()

            # 4채널 RGBA 이미지를 3채널 BGR로 변환
            if frame.shape[2] == 4:  # RGBA일 경우
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            # ROS 2 메시지로 변환 및 퍼블리싱
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)
            self.get_logger().info("Published a frame")

        except Exception as e:
            self.get_logger().error(f"Failed to capture or publish frame: {e}")

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = PicameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

