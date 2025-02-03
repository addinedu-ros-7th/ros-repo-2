import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        qos_profile = QoSProfile(depth=10)  # QoS 설정
        self.subscription = self.create_subscription(
            PoseStamped,  # Pose에서 PoseStamped로 수정
            '/tracked_pose',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation

        self.get_logger().info(f'Position: x={position.x}, y={position.y}, z={position.z}')
        self.get_logger().info(f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
