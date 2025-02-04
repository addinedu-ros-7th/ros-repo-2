from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planner_service')
        self.srv = self.create_service(Trigger, 'path_planner_service', self.handle_path_request)

    def handle_path_request(self, request, response):
        # Log the received request data
        self.get_logger().info(f'Received path request: {request.data}')
        
        # Set the response message
        response.success = True
        response.message = 'Path request received and logged.'
        return response

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