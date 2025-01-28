import socket
import numpy as np
import cv2
from ultralytics import YOLO

# variables

# udp ip and port
udp_ip = "0.0.0.0"  # all ip allow
udp_port = 34343

# socket base variables
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((udp_ip, udp_port))
print(f"Listening for UDP packets on {udp_ip}:{udp_port}...")

K = np.array(
    [
        [273.04999718, 0, 155.82542991],
        [0, 274.11381476, 121.70811418],
        [0, 0, 1]
    ]
)

# 왜곡 계수
dist = np.array([[-0.0208246, 1.06411238, 0.00400323, 0.00484108, -3.84164103]])             

# camera 주점 좌표
c_x = K[0, 2]
c_y = K[1, 2]

# camera 초점 좌표
f_x = K[0, 0]
f_y = K[1, 1]

# ping pong distance offset
distance_offset = 20 # 40 mm / 2 -> 20 mm 탁구공의 중점

# camera 해상도
camera_resolution_x = 320
camera_resolution_y = 240

# ping pong size ( mm )
REAL_DIAMETER= 40

# test ping pong list
ping_pong_coordinates = []

# ping pong detect yolov8 model load
model = YOLO("./model_train/best.pt")

while True:
    try:
        # get image data
        data, addr = sock.recvfrom(65536)

        # transtle image data 
        nparr = np.frombuffer(data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is not None:
            # cv2.imshow("Received Frame", frame)
            frame = cv2.resize(frame, (camera_resolution_x, camera_resolution_y))

            # detect ping pong
            result = model(frame, verbose=False )

            # show results
            detected_frame = result[0].plot()

            # if multiple ping pong balls are detected
            for result in result[0].boxes:

                # detecting box locate
                box = result.xyxy[0]
                x1, y1, x2, y2 = map(int, box[:4])

                # ping pong center locate
                x_center = int(( x1 + x2 ) / 2 )
                y_center = int(( y1 + y2 ) / 2 )

                print(f"인식된 탁구공 픽셀 좌표 : {x_center, y_center}")

                # pixel width
                pixel_width = x2 - x1

                print(f"인식된 탁구공 픽셀 너비 : {pixel_width}")   

                # distance
                if pixel_width > 0:
                    distance = (( f_x * REAL_DIAMETER ) / pixel_width ) + distance_offset
                    z_distance = distance
                else:
                    distance = -1 # error value
                    z_distance = -1
                
                print(f"인식된 탁구공 거리 : {distance} mm")

                # conversion to camera coordinate system , detect ping pong locate - camera center locate
                camera_x = (( x_center - c_x ) * distance) / f_x
                camera_y = (( y_center - c_y ) * distance) / f_y 
                camera_z = z_distance

                print(f"탁구공 좌표 - 카메라 주점 좌표 * 거리 / 초점 좌표 : {camera_x, camera_y, camera_z}")

                ping_pong_coordinates.append(( camera_x, camera_y , camera_z ))

                # detecting ping pong circle draw
                cv2.circle(detected_frame, (x_center, y_center), 5, (0,0,255), -1)

                label = f"Distance: {distance:.2f} mm"
                cv2.putText(detected_frame, label, (x1 + 5, y1 + 15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # imshow!!!
            cv2.imshow("ping pong", detected_frame)

            # relative coordinates of the ping-pong by camera
            camera_position_frame = np.zeros(( camera_resolution_y, camera_resolution_x, 3), dtype=np.uint8 )

            for coord in ping_pong_coordinates:
                X, Y = coord[:2]
                pos_x = int(X) + 160
                pos_y = int(-Y) + 120

                print(f"카메라 좌표 : {pos_x, pos_y}")

                if 0 <= pos_x < camera_resolution_x and 0 <= pos_y < camera_resolution_y:
                    cv2.circle(camera_position_frame, (pos_x, pos_y), 5, (255, 0, 0), -1)
                    cv2.circle(camera_position_frame, (160, 120), 5, (0, 255, 0), -1)

            # imshow!!!!!!!!!
            cv2.imshow("Camera Coordinate System", camera_position_frame)

            # q 누르면 종료 ~
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error: {e}")
        break

# 자원 해제
sock.close()
cv2.destroyAllWindows()
