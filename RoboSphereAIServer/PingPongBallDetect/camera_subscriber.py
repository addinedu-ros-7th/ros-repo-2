#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

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

        # YOLO 모델 로드 (가중치 파일 경로 수정 필요)
        self.model = YOLO('/home/uze/ws/deeplearning_model/class1_pingpong/best.pt')
        
        # 탁구공 실제 지름 (단위: mm)
        self.REAL_DIAMETER = 40  # 탁구공의 실제 지름
        self.FOCAL_LENGTH = 687.5  # 초점 거리 (실험적으로 계산된 값)
        
        self.get_logger().info("Camera Subscriber Node started and YOLO model loaded!")

    def listener_callback(self, msg):
        try:
            # ROS 2 Image 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLO 모델로 객체 감지
            results = self.model(frame)
            annotated_frame = results[0].plot()

            # 감지된 객체 정보 처리
            for result in results[0].boxes:
                box = result.xyxy[0]  # 경계 상자 좌표
                x1, y1, x2, y2 = map(int, box[:4])

                # 중심 좌표 계산
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                # 픽셀 너비 계산
                pixel_width = x2 - x1

                # 거리 계산
                if pixel_width > 0:
                    distance = (self.FOCAL_LENGTH * self.REAL_DIAMETER) / pixel_width
                else:
                    distance = -1  # 유효하지 않은 경우

                # 좌표, 픽셀 너비 및 거리 출력
                label1 = f"Coord: ({x_center}, {y_center})"
                label2 = f"Width: {pixel_width}px"
                label3 = f"Dist: {distance:.2f}mm"

                # 결과 시각화
                cv2.circle(annotated_frame, (x_center, y_center), 5, (0, 0, 255), -1)  # 중심점 표시
                cv2.putText(annotated_frame, label1, (x1+5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(annotated_frame, label2, (x1+5, y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(annotated_frame, label3, (x1+5, y1 + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 경계 상자 표시

            # OpenCV 창에 결과 표시
            cv2.imshow("Ping Pong Ball Detection", annotated_frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
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
