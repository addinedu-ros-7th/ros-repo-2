# icm20948 IMU SETUP

## 1. imu python 라이브러리 설치
```
pip3 install sparkfun-qwiic-icm20948
```

## 2. imu ros2 package 설치
```
cd ~/pinky_violet/src
git clone https://github.com/norlab-ulaval/ros2_icm20948.git
cd ~/pinky_violet/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 3. 패키지 수정
#### ros2_icm20948/launch/icm20948_node_launch.py 다음 내용으로 수정 (14, 15 line)
```
{"i2c_address": 0x69} -> {"i2c_address": 0x68}
{"frame_id": "imu_icm20948"} -> {"frame_id": "imu_link"}
```
#### /ros2_icm20948/ros2_icm20948/icm20948_node.py 다음 내용으로 수정 (40 line)
```
self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10) 
-> self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu", 10)
```

## 4. 실행
```
cd ~/pinky_violet/
source install/local_setup.bash
```
```
ros2 launch ros2_icm20948 icm20948_node_launch.py
```
## 5. 동작 확인 
```
ros2 topic echo /imu
```
