# import socket
# import numpy as np
# import cv2

# udp_ip = "0.0.0.0" 
# udp_port = 34343

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.bind((udp_ip, udp_port))

# print(f"Listening for UDP packets on {udp_ip}:{udp_port}...")

# while True:
#     try:
#         data, addr = sock.recvfrom(65536) 

#         nparr = np.frombuffer(data, np.uint8)
#         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

#         if frame is not None:
#             cv2.imshow("Received Frame", frame)

#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#     except Exception as e:
#         print(f"Error: {e}")
#         break

# sock.close()
# cv2.destroyAllWindows()
import socket
import numpy as np
import cv2
import os

udp_ip = "0.0.0.0" 
udp_port = 34343

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((udp_ip, udp_port))

print(f"Listening for UDP packets on {udp_ip}:{udp_port}...")

# 이미지 저장 경로 설정 (파일 이름 포함)
save_path = "./calibration_checkerboard/image_{}.jpg"  # 확장자 포함

# 디렉토리 존재 여부 확인 및 생성
if not os.path.exists("./calibration_checkerboard"):
    os.makedirs("./calibration_checkerboard")

while True:
    try:
        data, addr = sock.recvfrom(65536) 

        nparr = np.frombuffer(data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow("Received Frame", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):  
                if save_path:
                    # 고유한 파일 이름 생성
                    filename = save_path.format(int(cv2.getTickCount()))
                    cv2.imwrite(filename, frame)
                    print(f"Image saved as: {filename}")
                else:
                    print("Save path is not set. Please set the save_path variable.")

            elif key == ord('q'):  # 'q' 키를 누르면 종료
                break

    except Exception as e:
        print(f"Error: {e}")
        break

sock.close()
cv2.destroyAllWindows()
