import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TrackedPoseConverter(Node):
    def __init__(self, namespace):
        super().__init__('tracked_pose_converter')
        
        # 퍼블리셔: 변환된 PoseStamped 메시지를 발행
        self.publisher = self.create_publisher(PoseStamped, f'/{namespace}/tracked_pose_transfer', 3)
        
        # 서브스크라이버: /tracked_pose 구독
        self.subscription = self.create_subscription(
            PoseStamped,  # /tracked_pose가 PoseStamped 형식이라고 가정
            '/tracked_pose',
            self.tracked_pose_callback,
            3)
        
        self.get_logger().info('🚀 Tracked Pose Transfer Node Started!')

    def tracked_pose_callback(self, msg):
        """ /tracked_pose 메시지를 받아서 변환 후 /{namespace}/tracked_pose_transfer 로 퍼블리시 """
        converted_msg = PoseStamped()
        converted_msg.header = msg.header  # 기존 헤더 유지
        converted_msg.pose = msg.pose      # 기존 위치 & 자세 유지

        # 변환된 데이터 퍼블리시
        self.publisher.publish(converted_msg)
        self.get_logger().info(f'📡 Published Converted Pose: {converted_msg.pose.position.x}, {converted_msg.pose.position.y}, {converted_msg.pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TrackedPoseConverter(namespace="pinky1")
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# import os

# class TrackedPoseBridge(Node):
#     def __init__(self, input_domain, output_domain, namespace="pinky1"):
#         super().__init__('tracked_pose_bridge')

#         # 입력(수신) 도메인 설정
#         os.environ["ROS_DOMAIN_ID"] = str(input_domain)

#         # /tracked_pose를 구독
#         self.subscription = self.create_subscription(
#             PoseStamped,
#             '/tracked_pose',
#             self.tracked_pose_callback,
#             3)
        
#         self.get_logger().info(f'🔄 Subscribing to /tracked_pose on ROS_DOMAIN_ID={input_domain}')
        
#         # 퍼블리셔: 데이터를 다른 도메인으로 전송
#         os.environ["ROS_DOMAIN_ID"] = str(output_domain)
#         self.publisher = self.create_publisher(PoseStamped, f'/{namespace}/tracked_pose_transfer', 3)
        
#         self.get_logger().info(f'🚀 Publishing to /{namespace}/tracked_pose_transfer on ROS_DOMAIN_ID={output_domain}')

#     def tracked_pose_callback(self, msg):
#         """ /tracked_pose 메시지를 받아서 다른 도메인에 전송 """
#         converted_msg = PoseStamped()
#         converted_msg.header = msg.header
#         converted_msg.pose = msg.pose

#         # 변환된 데이터 퍼블리시
#         self.publisher.publish(converted_msg)
#         self.get_logger().info(f'📡 Bridged Pose: {converted_msg.pose.position.x}, {converted_msg.pose.position.y}, {converted_msg.pose.position.z}')

# def main(args=None):
#     rclpy.init(args=args)

#     # 도메인 ID 설정 (송신과 수신)
#     input_domain = 74   # tracked_pose가 있는 도메인
#     output_domain = 73  # 변환된 메시지를 보내는 도메인

#     node = TrackedPoseBridge(input_domain, output_domain)
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
