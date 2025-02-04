import socket
import json

def send_signal_over_tcp(signal):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(('localhost', 9151))  # 서버 주소와 포트
        # signal to json
        s.sendall(json.dumps(signal).encode('utf-8'))

# receive table_id and send signal over tcp
def send_tcp_signal(table_id):

    # make signal
    signal = {
        'command': 2,
        'target': '0.00000123',
        'table_id': table_id
    }

    # send signal over tcp
    send_signal_over_tcp(signal)

# test code
send_tcp_signal(2)
