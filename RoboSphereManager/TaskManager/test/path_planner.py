from example_interfaces.srv import Trigger
from interface_package.srv import PathRequest
import rclpy
from rclpy.node import Node
import json
import numpy as np
from interface_package.srv import PathRequest

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planner_service')
        self.srv = self.create_service(PathRequest, 'path_planner_service', self.handle_path_request)

    def handle_path_request(self, request, response):
        self.get_logger().info(f'Received path request: {request.request_data}')
        path = "/home/kjj73/test_folder/data/"

        if not request.request_data or not request.request_data.strip():
            self.get_logger().error("Received empty request data.")
            response.success = False
            response.message = "Request data is empty."
            return response

        try:
             # 문자열 데이터를 직접 파싱하여 key-value 추출
            data_parts = request.request_data.split(", ")
            parsed_data = {part.split(": ")[0]: part.split(": ")[1] for part in data_parts}

            robot_id = parsed_data.get("Robot ID", "Unknown")
            command = int(parsed_data.get("Command", 0))
            table_id = int(parsed_data.get("Table ID", -1))
            target_raw = parsed_data.get("Target", "0,0").strip("()")
            
            # Target 값 처리 (값이 하나인 경우 예외 처리)
            target_values = target_raw.split(",")
            if len(target_values) == 2:
                target_x, target_y = map(float, target_values)
            else:
                self.get_logger().warn(f"Invalid target format: {target_raw}, using default (0,0)")
                target_x, target_y = 0.0, 0.0

            self.get_logger().info(f"Parsed Data -> Robot ID: {robot_id}, Command: {command}, Table ID: {table_id}, Target: ({target_x}, {target_y})")
            
            # name = (command + "_" + table_id + "_" + target)
            name = "request_1_0"
            arr = np.loadtxt(path+name+".txt")
            make_path(robot_id, command, table_id, arr)

            response.success = True
            response.message = "Path planning completed successfully"
        except Exception as e:
            self.get_logger().error(f"Error parsing request data: {e}. Raw data: {request.request_data}")
            response.success = False
            response.message = "Failed to process request."
        
        return response

class TaskProviderService(Node):
    def __init__(self, namespace):
        super().__init__('task_and_path_provider')
        self.srv = self.create_client(PathRequest, f'/{namespace}/task_and_path_listener')

        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_task_request(self, robot_id, command, table_id, target):
        # Create a request
        # target이 numpy 배열이면 리스트로 변환
        if isinstance(target, np.ndarray):
            target_list = target.tolist()  # NumPy 배열을 Python 리스트로 변환
        else:
            target_list = target  # 리스트가 아닌 경우 그대로 사용

        # JSON 형식으로 변환
        request_data = json.dumps({
            "robot_id": robot_id,
            "command": command,
            "table_id": table_id,
            "target": target_list
        })

        # # Create a request
        # request = Trigger.Request()
        # request.data = request_data

        request = PathRequest.Request()
        request.request_data = request_data

        # Call the service
        future = self.srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')


def make_path(robot_id, command, table_id, target):
    # Create a node for the service client
    task_provider_client = TaskProviderService("pinky1")
    
    # Send the path request
    task_provider_client.send_task_request(robot_id, command, table_id, target)
    
    # Shutdown the node after sending the request
    task_provider_client.destroy_node()

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