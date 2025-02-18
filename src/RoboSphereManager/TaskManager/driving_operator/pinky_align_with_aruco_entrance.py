#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import time
import math


class ArucoMarkerFollower(Node):
    def __init__(self):
        super().__init__('aruco_marker_follower')

        # 목표 거리 및 정렬 오차
        self.target_distance = 0.13  # 목표 거리 (단위: m)
        self.alignment_tolerance = 0.05  # 정렬 오차 허용 범위 (단위: m)
        self.angular_tolerance = 0.05  # 각도 오차 허용 범위 (단위: rad)

        # 속도 설정
        self.min_speed_align = 0.8  # 정렬 최소 속도
        self.min_speed_move = 0.3  # 전진 최소 속도
        self.max_speed_align = 1.0  # 정렬 최대 속도
        self.max_speed_move = 0.5  # 전진 최대 속도

        # 최근 Pose 메시지 저장 및 타이머
        self.latest_pose_msg = None
        self.last_pose_time = self.get_clock().now()

        # 마지막 사용된 X축 제어값 (초기값: 0)
        self.last_lateral_error = 0.0

        # 동작 상태
        self.state = "ALIGNING"  # 초기 상태는 정렬 상태

        # /aruco_pose 구독
        self.subscription = self.create_subscription(
            Pose,
            '/aruco_pose',
            self.pose_callback,
            10
        )

        # 로봇 속도 퍼블리셔
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 타이머 기반 제어 루프 (0.1초 간격으로 실행)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Aruco Marker Follower Node has started!")

        time.sleep(10)

    def pose_callback(self, msg):
        """최근 Pose 메시지를 저장"""
        self.latest_pose_msg = msg
        self.last_pose_time = self.get_clock().now()

        # 마지막 X축 오차 값 업데이트
        self.last_lateral_error = msg.position.x

    def control_loop(self):
        """타이머 기반 제어 루프"""
        # ArUco 마커를 일정 시간 동안 감지하지 못한 경우
        time_since_last_pose = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if time_since_last_pose > 5.0:  # 5초 동안 업데이트가 없으면
            self.get_logger().warning("No Pose data received for 5 seconds. Searching for marker...")
            self.search_for_marker()  # 탐색 상태로 전환
            return

        if self.latest_pose_msg is None:
            self.get_logger().info("No Pose data received yet.")
            return  # Pose 데이터가 없으면 대기

        # 상태에 따른 동작 수행
        if self.state == "ALIGNING":
            self.align_to_marker()
        elif self.state == "MOVING":
            self.move_towards_marker()

    def align_to_marker(self):
        """로봇을 ArUco 마커 방향으로 정렬"""
        lateral_error = self.latest_pose_msg.position.x  # X축 기준 오차
        orientation = self.latest_pose_msg.orientation  # Orientation 값
        yaw = self.calculate_yaw(orientation)  # 오리엔테이션에서 yaw 값 계산

        cmd_vel = Twist()

        # X축 정렬 (좌우 회전)
        if abs(lateral_error) > self.alignment_tolerance:
            angular_speed = 0.5 * -lateral_error
            angular_speed = max(min(angular_speed, self.max_speed_align), -self.max_speed_align)  # 속도 제한
            angular_speed = max(abs(angular_speed), self.min_speed_align) * (-1 if angular_speed < 0 else 1)  # 최소 속도 보장
            cmd_vel.angular.z = angular_speed
            self.velocity_publisher.publish(cmd_vel)
            self.get_logger().info(f"Aligning X: Lateral Error = {lateral_error:.3f}, Angular Speed = {angular_speed:.3f}")
            self.apply_brake_and_wait_align()
            return
        


        # Yaw 정렬 (정면 회전)
        yaw_error = yaw - 0.0  # 정면(yaw=0)을 기준으로 오차 계산
        if abs(yaw_error) > self.angular_tolerance:
            angular_speed = 0.5 * -yaw_error
            angular_speed = max(min(angular_speed, self.max_speed_align), -self.max_speed_align)  # 속도 제한
            angular_speed = max(abs(angular_speed), self.min_speed_align) * (-1 if angular_speed < 0 else 1)  # 최소 속도 보장
            cmd_vel.angular.z = angular_speed
            self.velocity_publisher.publish(cmd_vel)
            self.get_logger().info(f"Aligning Yaw: Yaw Error = {yaw_error:.3f}, Angular Speed = {angular_speed:.3f}")
            self.apply_brake_and_wait_align()
            return

        # 정렬 완료
        cmd_vel.angular.z = 0.0
        self.velocity_publisher.publish(cmd_vel)
        self.get_logger().info("Alignment complete. Switching to MOVING state.")
        self.state = "MOVING"

    def calculate_yaw(self, orientation):
        """Quaternion을 Yaw 값으로 변환"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # Yaw 계산
        return yaw


    def move_towards_marker(self):
        """ArUco 마커를 향해 이동"""
        distance_error = self.latest_pose_msg.position.z - self.target_distance  # Z축 기준 오차
        lateral_error = self.latest_pose_msg.position.x  # X축 기준 오차
        cmd_vel = Twist()

        # 정렬이 벗어난 경우 다시 정렬 상태로 전환
        if abs(lateral_error) > self.alignment_tolerance:
            self.get_logger().warning(f"Marker off-center: Lateral Error = {lateral_error:.3f}. Switching to ALIGNING state.")
            self.state = "ALIGNING"
            return

        if abs(distance_error) > self.alignment_tolerance:
            # 목표 거리까지 전진
            linear_speed = distance_error * 0.4  # 비례 제어
            linear_speed = max(min(linear_speed, self.max_speed_move), -self.max_speed_move)  # 속도 제한
            linear_speed = max(abs(linear_speed), self.min_speed_move) * (-1 if linear_speed < 0 else 1)  # 최소 속도 보장
            cmd_vel.linear.x = linear_speed
            self.velocity_publisher.publish(cmd_vel)
            self.get_logger().info(f"Moving: Distance Error = {distance_error:.3f}, Linear Speed = {linear_speed:.3f}")
        else:
            # 목표 도달
            cmd_vel.linear.x = 0.0
            self.velocity_publisher.publish(cmd_vel)
            self.get_logger().info("Target distance reached. Task completed. Shutting down node.")
            self.shutdown_node()  # 노드 종료

        self.apply_brake_and_wait_approch()

    def search_for_marker(self):
        """마커를 탐색하기 위해 마지막 X축 제어값의 반대 방향으로 움직임"""
        cmd_vel = Twist()
        search_speed = self.last_lateral_error * 0.5  # 마지막 제어값의 반대 방향
        search_speed = max(min(search_speed, self.max_speed_align), -self.max_speed_align)  # 속도 제한
        search_speed = max(abs(search_speed), self.min_speed_align) * (-1 if search_speed < 0 else 1)  # 최소 속도 보장
        cmd_vel.angular.z = search_speed
        self.velocity_publisher.publish(cmd_vel)
        self.get_logger().info(f"Searching for marker: Angular Speed = {search_speed:.3f}")

        self.apply_brake_and_wait_align()

    def apply_brake_and_wait_align(self):
        """로봇을 잠시 멈추고 대기"""
        time.sleep(0.1)
        self.stop_robot()
        self.get_logger().info("Waiting to detect ArUco marker...")
        time.sleep(2)  # 2초 대기

    def apply_brake_and_wait_approch(self):
        """로봇을 잠시 멈추고 대기"""
        time.sleep(0.2)
        self.stop_robot()
        self.get_logger().info("Waiting to detect ArUco marker...")
        time.sleep(2)  # 2초 대기

    def stop_robot(self):
        """로봇을 정지"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.velocity_publisher.publish(cmd_vel)
        self.get_logger().info("Robot stopped.")

    def shutdown_node(self):
        """노드 종료"""
        self.get_logger().info("Shutting down Aruco Marker Follower Node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
