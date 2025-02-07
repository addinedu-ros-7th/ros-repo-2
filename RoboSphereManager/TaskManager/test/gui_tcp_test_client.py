import socket
import json

def send_signal_over_tcp(signal):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(('localhost', 9151))  # 서버 주소와 포트
        # signal to json
        s.sendall(json.dumps(signal).encode('utf-8'))

# receive table_id and send signal over tcp
def send_tcp_signal(command, table_id, file):

    command_list = ["request", "routine", "exit"] # 요청 목록
    # table_index = 0 # 테이블 번호
    # file_index = 0 # 파일 번호

    # make signal
    signal = {
        'command': (command_list.index(command)+1),
        'target': "0.000123",
        'table_id': table_id
    }

    # send signal over tcp
    send_signal_over_tcp(signal)

# test code


def main():
    print("Starting GUI TCP Test Client...")
    send_tcp_signal("exit", 2, 0)
    # 여기에 실행 로직 추가
    # send_tcp_signal("request", 1)

if __name__ == "__main__":
    main()
