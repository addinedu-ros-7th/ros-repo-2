import socket
import json
# from interface_package.srv import PathRequest
# from interface_package.msg import PointAndStatus
import rclpy
from rclpy.node import Node
import numpy as np

def send_signal_over_tcp(signal):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(('localhost', 9151))  # 서버 주소와 포트
        # signal to json
        s.sendall(json.dumps(signal).encode('utf-8'))

# receive table_id and send signal over tcp
def send_tcp_signal(command, table_id, target):

    command_list = ["request", "routine", "exit", "collision"] # 요청 목록
    # table_index = 0 # 테이블 번호
    # file_index = 0 # 파일 번호

    # make signal
    # signal = {
    #     'command': (command_list.index(command)+1),
    #     'target': "0.000123",
    #     'table_id': table_id
    # }
    target_x, target_y = target

    active_table = np.array([1, 2, 4]) # table
    signal = {
        'command': 4,
        'target': str(target),
        'table_id': str(active_table)
    }

    # send signal over tcp
    send_signal_over_tcp(signal)

def main():
    print("Starting GUI TCP Test Client...")
    send_tcp_signal("exit", 2, (0.0, 0.0))
    # 여기에 실행 로직 추가
    # send_tcp_signal("request", 1)

if __name__ == "__main__":
    main()
