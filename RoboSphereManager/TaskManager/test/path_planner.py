from example_interfaces.srv import Trigger
from interface_package.srv import PathRequest
import rclpy
from rclpy.node import Node
import json
import numpy as np
import heapq
import time
from PIL import Image
import yaml

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planner_service')
        self.srv = self.create_service(PathRequest, 'path_planner_service', self.handle_path_request)
        self.client = self.create_client(PathRequest, 'table_status')

        self.command_list = ["request", "routine", "exit", "collision"]

        self.filepath = "/home/kjj73/test_folder/src/PathMaker/"
        self.pgmpath = '/home/kjj73/test_folder/src/PathMaker/fourtable.pgm'
        self.yamlpath = '/home/kjj73/test_folder/src/PathMaker/fourtable.yaml'

        self.pixelSize = 5                              # 원래 픽셀 마다의 크기 0.05 cm
        self.collisionArea = 2                          # 로봇의 사이즈를 고려한 장애물 영역 설정
        self.mapRatio = self.collisionArea * 8          # 맵의 크기를 8배 늘려서 비율을 맞춰줌
        self.realAreaSize = int(self.mapRatio)

        with Image.open(self.pgmpath) as pgm_image:
            self.mapWidth, self.mapHeight = pgm_image.size
            self.img_array = np.array(pgm_image).T
            #self.img_array = np.flip(self.img_array)
            print("img_array:", self.img_array.shape)

        with open(self.yamlpath, 'r') as file:
            self.yaml = yaml.safe_load(file)

        self.mapLimitX = (self.mapWidth * 5) / 100
        self.mapLimitY = (self.mapHeight * 5) / 100

        self.pixmapWidth = self.mapWidth * 8
        self.pixmapHeight = self.mapHeight * 8

        self.mapArray = np.zeros((int(self.pixmapWidth / self.realAreaSize) + 1, int(self.pixmapHeight / self.realAreaSize) + 1))
        self.collision_cell_list = np.zeros((int(self.pixmapWidth / self.mapRatio) + 1, int(self.pixmapHeight / self.mapRatio) + 1), dtype=np.int32)

        # PGM 데이터를 기반으로 흰색이 아닌 부분 찾기
        for x in range(self.mapWidth):
            for y in range(self.mapHeight):
                pixel_value = self.img_array[x, y]
                if pixel_value < 240:  # 흰색이 아닌 영역 감지 (보통 255가 완전한 흰색)
                    # mapArray 좌표 변환
                    map_x = int(x / self.collisionArea)
                    map_y = int(y / self.collisionArea)

                    # **배열 크기 초과 방지**
                    map_x = min(map_x, self.mapArray.shape[0] - 1)
                    map_y = min(map_y, self.mapArray.shape[1] - 1)

                    # 장애물 표시
                    self.mapArray[map_x, map_y] = 1
                    self.collision_cell_list[map_x, map_y] = 1

        self.startPoint = [None, None]
        self.endPoint = [None, None]

        print("map width, height : ", self.mapWidth, self.mapHeight)
        print(self.pixmapWidth, self.pixmapHeight, self.realAreaSize)
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')

    def handle_path_request(self, request, response):
        self.get_logger().info(f'Received path request: {request.request_data}')
        path = "/home/kjj73/test_folder/src/PathMaker/"

        if not request.request_data or not request.request_data.strip():
            self.get_logger().error("Received empty request data.")
            response.success = False
            response.message = "Request data is empty."
            return response

        try:
             # 문자열 데이터를 직접 파싱하여 key-value 추출
            # data_parts = request.request_data.split(", ")
            # parsed_data = {part.split(": ")[0]: part.split(": ")[1] for part in data_parts}

            # # robot_id = parsed_data.get("Robot ID", "Unknown")
            # # command = int(parsed_data.get("Command", 0))
            # # table_id_raw = parsed_data.get("Table ID", -1).strip("[]")
            # # table_id_values = table_id_raw.split()
            # # target_raw = parsed_data.get("Target", "0,0").strip("()")
            # 쉼표로 분리한 후 각 항목을 나누어 값을 추출
            parts = request.request_data.split(", ")

            # 각 항목에서 'Robot ID:', 'Command:', 'Table ID:', 'Target:'을 제외하고 값만 추출
            robot_id = parts[0].split(": ")[1]
            command = parts[1].split(": ")[1]
            data_str = parts[2].split(": ")[1]
            table_id = np.fromstring(data_str.strip('[]'), sep=' ', dtype=int)
            target_raw = parts[3].split(": ")[1]
                    
            # print(type(table_id))

            # Target 값 처리 (값이 하나인 경우 예외 처리)
            target_values = target_raw.split(",")
            if len(target_values) == 2:
                target_x, target_y = map(float, target_values)
            else:
                self.get_logger().warn(f"Invalid target format: {target_raw}, using default (0,0)")
                target_x, target_y = 0.0, 0.0

            self.get_logger().info(f"Parsed Data -> Robot ID: {robot_id}, Command: {command}, Table ID: {table_id}, Target: ({target_x}, {target_y})")
            # self.get_logger().info(f"Table ID Data Type : {type(table_id)}")
            
            command_name = self.command_list[int(command)-1]
            serving_path = []
            pathPoint = []
            npPathPoint = []

            if command == "4":
                # 경로 생성 후 보냄
                # name = "request_1_0"
                for i in table_id :
                    # 활성 영역 데이터를 가져옴
                    name = (command_name + "_" + str(i) + "_" + "0")
                    arr = np.loadtxt(path+name+".txt")

                    arr[:, 0] = (arr[:, 0] + (self.mapLimitX - (self.mapLimitX - abs(self.yaml['origin'][0]))))
                    arr[:, 1] = (abs(arr[:, 1]) + (self.mapLimitY - abs(self.yaml['origin'][1])))

                    arr[:, 0] = (arr[:,0] * 8 * 20)
                    arr[:, 1] = (arr[:,1] * 8 * 20)

                    arr = np.round(arr).astype(int)

                    for i in range(0, int(len(arr))):
                        x = int(arr[i][0] / self.realAreaSize) * self.mapRatio # Cell Size Complete
                        y = int(arr[i][1] / self.realAreaSize) * self.mapRatio

                        self.collision_cell_list[x // self.realAreaSize, y // self.realAreaSize] = 1

                ################################## 이 부분에 출발 위치와 도착 위치를 지정해줘야 함
                start = [12, 14] 
                end = [34, 12]
                to_FoodCase = self.astarStart(start, end)
                
                start = [34, 12]
                end = [12, 14]
                to_EndPoint = self.astarStart(start, end)

                serving_path = np.vstack((to_FoodCase, to_EndPoint))

                pathPoint = ((serving_path * self.mapRatio) + 1)
                to_FoodCase = np.array(to_FoodCase)
                pathPoint = ((to_FoodCase * self.mapRatio) + 1)

                npPathPoint = np.array(pathPoint)
                npPathPoint = ((npPathPoint / 8) / 20)

                if not np.any(npPathPoint):
                    print("Empty")
                else :
                    print("limit x , y : ", self.mapLimitX, self.mapLimitY)
                    npPathPoint[:, 0] = npPathPoint[:, 0] - (self.mapLimitX - (self.mapLimitX - abs(self.yaml['origin'][0])))
                    npPathPoint[:, 1] = (npPathPoint[:, 1] - (self.mapLimitY - abs(self.yaml['origin'][1]))) * -1

                # self.get_logger().info(f"Table Collision : {arr}, serving path : {serving_path}, path : {pathPoint}, npPath : {npPathPoint}")
                self.get_logger().info(f"npPath : {npPathPoint}")
                    
                make_path(robot_id, command, table_id, npPathPoint)

            else :
                name = (command_name + "_" + table_id + "_" + 0)
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
    
    def astarStart(self, StartPoint, EndPoint):
        self.startPoint[0] = int(StartPoint[0])
        self.startPoint[1] = int(StartPoint[1])

        self.endPoint[0] = int(EndPoint[0])
        self.endPoint[1] = int(EndPoint[1])

        start = (self.startPoint[0], self.startPoint[1])
        goal = (self.endPoint[0], self.endPoint[1])
        grid = self.collision_cell_list

        # self.collision_cell_list = np.zeros((int(self.pixmapWidth / self.mapRatio) + 1, int(self.pixmapHeight / self.mapRatio) + 1), dtype=np.int32)

        path = self.astar(grid, start, goal)
        return path
        

    # Manhattan Distance 휴리스틱 함수 (목표까지의 예상 거리)
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
    # A* 알고리즘 구현
    def astar(self, grid, start, goal):
        # 이동할 수 있는 방향 (상, 하, 좌, 우)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # 오픈 리스트: (f_score, (x, y)) 형태로 저장
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        # 각 노드에 대해 f, g, h 값 저장
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 경로를 추적할 부모 노드
        came_from = {}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            # 목표 지점에 도달하면 경로를 반환
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # 역순으로 경로 반환
            
            # 현재 노드에서 갈 수 있는 이웃 탐색
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                # 이웃이 경계 내에 있고, 장애물이 아니어야 함
                if (0 <= neighbor[0] < len(grid)) and (0 <= neighbor[1] < len(grid[0])) and grid[neighbor[0]][neighbor[1]] != 1:
                    tentative_g_score = g_score[current] + 1  # 이동 비용 (상, 하, 좌, 우는 동일하게 1로 설정)
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        return None  # 경로가 없으면 None 반환

class TaskProviderService(Node):
    def __init__(self, namespace):
        super().__init__('task_and_path_provider')
        self.srv = self.create_client(PathRequest, f'/{namespace}/task_and_path_listener')

        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_task_request(self, robot_id, command, table_id, target):
        # Create a request
        # target이 numpy 배열이면 리스트로 변환
        print("target :", type(target), target)
        if isinstance(target, np.ndarray):
            target = np.round(target, 6).tolist()  # 각 요소 소수점 6자리로 반올림 후 리스트로 변환
            # print("Converted target to list:", target)
            
        if isinstance(table_id, np.ndarray):
            table_id = table_id.tolist()  # 리스트로 변환
            # print("Converted table_id to list:", table_id)
        # else:
        #     target = target  # 리스트가 아닌 경우 그대로 사용
        
        # # table_id와 target을 문자열로 변환
        table_id_str = str(table_id)  # 대괄호 포함된 리스트 형식으로 변환
        target_str = str(target)      # 대괄호 포함된 리스트 형식으로 변환
        # table_id와 target을 문자열로 변환
        # table_id_str = json.dumps(table_id)  # 리스트를 JSON 형식으로 문자열로 변환
        # target_str = json.dumps(target)      # 리스트를 JSON 형식으로 문자열로 변환

        print("robot_id :", type(robot_id), robot_id)
        print("command :", type(command), command)
        print("table_id :", type(table_id_str), table_id_str)
        print("target :", type(target_str), target_str)
        print()

        # JSON 형식으로 변환
        request_data = json.dumps({
            "robot_id": robot_id,
            "command": command,
            "table_id": table_id_str,
            "target": target_str
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