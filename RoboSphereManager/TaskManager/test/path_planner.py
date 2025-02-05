from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node
import json
import numpy as np

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planner_service')
        self.srv = self.create_service(Trigger, 'path_planner_service', self.handle_path_request)

    def handle_path_request(self, request, response):
        # Log the received request data
        self.get_logger().info(f'Received path request: {request.data}')

        # 데이터 저장 장소
        path = "/home/kjj73/ros-repo-2-ws/data/"

        # 받은 데이터를 쪼개는 부분
        try:
            # 데이터 파싱
            data_parts = request.data.split(", ")
            parsed_data = {part.split(": ")[0]: part.split(": ")[1] for part in data_parts}

            # 개별 변수로 저장
            robot_id = parsed_data.get("Robot ID", "Unknown")
            command = int(parsed_data.get("Command", 0))  # 문자열을 정수로 변환
            table_id = int(parsed_data.get("Table ID", -1))  # 테이블 ID도 정수로 변환
            
            # Target 좌표 파싱
            target_raw = parsed_data.get("Target", "(0,0)").strip("()")  # 괄호 제거
            target_x, target_y = map(float, target_raw.split(","))  # 좌표를 float로 변환

            # 로깅
            self.get_logger().info(f"Parsed Data -> Robot ID: {robot_id}, Command: {command}, Table ID: {table_id}, Target: ({target_x}, {target_y})")

            # 여기서 로봇 경로 계획 로직 실행 가능

            response.success = True
            response.message = "Path planning completed successfully"

        except Exception as e:
            self.get_logger().error(f"Error parsing request data: {e}")
            response.success = False
            response.message = "Failed to process request"

        who = robot_id
        why = command
        where = table_id
        index = "0"
        target = [target_x, target_y]

        name = (why + "_" + where + "_" + index)

        arr = np.loadtxt(path+name+".txt")

        # robot에 서비스 요청보내기 (signal)
        make_path(who, why, where, arr)
    
        # Set the response message
        response.success = True
        response.message = 'Path request received and logged.'
        return response

class TaskProviderService(Node):
    def __init__(self, namespace):
        super().__init__('task_and_path_provider')
        self.srv = self.create_client(Trigger, f'/{namespace}/task_and_path_provider')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_task_request(self, robot_id, command, table_id, target):
        # Create a request
        # target이 numpy 배열이면 리스트로 변환
        target_list = target.tolist()
        request = Trigger.Request()
        
        # JSON 형식으로 변환
        request_data = json.dumps({
            "robot_id": robot_id,
            "command": command,
            "table_id": table_id,
            "target": target_list
        })

        # Create a request
        request = Trigger.Request()
        request.data = request_data

        # Call the service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')


def make_path(robot_id, command, table_id, target):
    # Create a node for the service client
    rclpy.init()
    task_provider_client = TaskProviderService()
    
    # Send the path request
    task_provider_client.send_task_request(robot_id, command, table_id, target)
    
    # Shutdown the node after sending the request
    task_provider_client.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    path_planner_service = PathPlannerService()

    try:
        rclpy.spin(path_planner_service)
    except KeyboardInterrupt:
        pass

    path_planner_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 