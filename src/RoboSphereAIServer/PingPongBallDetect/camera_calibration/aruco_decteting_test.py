import socket
import numpy as np
import cv2
import cv2.aruco as aruco

# UDP 설정
udp_ip = "0.0.0.0"  # 모든 IP 허용
udp_port = 34343

# 소켓 기본 변수 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((udp_ip, udp_port))
print(f"Listening for UDP packets on {udp_ip}:{udp_port}...")

# 카메라 내부 파라미터 설정
# 카메라 회전 행렬
mtx = np.array([[267.24122712, 0, 157.6635982],
                 [0, 269.65655429, 131.48863535],
                 [0, 0, 1]])

# 왜곡 계수
dist = np.array([[ 3.90488473e-01, -2.41532741e+00,  1.06162539e-02, 
                   2.40770798e-03,  2.78313370e+00]])

# Aruco 마커의 실제 크기 (미터 단위)
marker_length = 0.0354  # 예: 35.4mm

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco_dict_type)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(f"ids : {ids}")

    # 마커가 감지된 경우
    if ids is not None and len(corners) > 0:
        for i in range(len(ids)):
            # 각 마커의 자세 추정
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_length, matrix_coefficients, distortion_coefficients)
            print(f"Rotation vector for marker {ids[i]}: {rvec}")
            print(f"Translation vector for marker {ids[i]}: {tvec}\n")

            # 거리 계산
            distance = np.linalg.norm(tvec[0])  # tvec의 크기 계산
            print(f"Distance from camera to marker {ids[i]}: {distance:.3f} meters")  # 소수점 3자리까지 출력

            # 마커 주위에 사각형 그리기
            aruco.drawDetectedMarkers(frame, corners)

            # 축 그리기
            # aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)  # 길이 10cm의 축

    return frame

while True:
    try:
        # 이미지 데이터 수신
        data, addr = sock.recvfrom(65536)
        nparr = np.frombuffer(data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is not None:
            # 이미지 왜곡 보정
            undistorted_frame = cv2.undistort(frame, mtx, dist)

            # Aruco 마커 자세 추정 및 거리 계산
            output_frame = pose_estimation(undistorted_frame, aruco.DICT_5X5_100, mtx, dist)

            cv2.imshow('Aruco Marker Detection', output_frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error: {e}")
        break

# 자원 해제
sock.close()
cv2.destroyAllWindows()
