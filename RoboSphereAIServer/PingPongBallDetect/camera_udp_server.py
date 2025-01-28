import rclpy
from rclpy.node import Node
from picamera2 import Picamera2
import socket
import cv2
from libcamera import Transform

# udp_ip="192.168.5.3"
udp_ip="192.168.13.197"
udp_port=34343

class PicameraUDP(Node):
    def __init__(self):
        super().__init__('picamera_udp')

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
                                    main={"size": (320, 240)},
                                    transform=Transform(hflip=True, vflip=True))
        self.picam2.configure(config)
        self.picam2.start()

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_address = (udp_ip, udp_port)

        self.timer = self.create_timer(1 / 30, self.timer_callback)

    def timer_callback(self):
        try:
            frame = self.picam2.capture_array()

            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            _, buffer = cv2.imencode('.jpg', frame)
            self.udp_socket.sendto(buffer.tobytes(), self.udp_address)

            self.get_logger().info("Sent a frame over UDP")

        except Exception as e:
            self.get_logger().error(f"Failed to capture or send frame: {e}")

    def destroy_node(self):
        self.picam2.stop()
        self.udp_socket.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = PicameraUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
