import cv2
import cv2.aruco as aruco

# Aruco 딕셔너리 선택
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# 마커 ID와 크기 설정
marker_id = 0  # 원하는 마커 ID (0~249)
marker_size = 200  # 픽셀 단위

# Aruco 마커 생성
marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# 마커 이미지를 저장
cv2.imwrite(f"aruco_marker_{marker_id}.png", marker_image)

# 마커 이미지 표시
cv2.imshow("Aruco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
