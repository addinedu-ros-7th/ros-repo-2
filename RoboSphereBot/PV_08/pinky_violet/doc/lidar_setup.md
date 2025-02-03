# RPLIDAR C1 SETUP

##  1.UART 설정
#### /boot/firmware/config.txt 열기
```
sudo nano /boot/firmware/config.txt
```
#### 맨아래 추가
```
dtparam=uart0=on
```
#### 재부팅
```
sudo reboot
```
#### AMA0가 생겼는지 확인
```
ls /dev/ttyAMA*
```
## 2. lidar 설치- https://github.com/Slamtec/sllidar_ros2 참고
```
cd ~/pinky_violet/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/pinky_violet/
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```

## 3. lidar udev 설정
#### rplidar.rules 파일을 만들고
```
sudo nano /etc/udev/rules.d/rplidar.rules
```
#### 다음내용 추가
```
# set the udev rule , make the device_port be fixed by rplidar
#
KERNEL=="ttyAMA0", MODE:="0777", SYMLINK+="RPLIDAR"
```

## 4. rulse 적용
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
## 5. 설정 터미널 확인 
```
ls -al /dev/RPLIDAR
```
```
lrwxrwxrwx 1 root root 7 Dec 12 15:50 /dev/RPLIDAR -> ttyAMA0
```
위와 같이 출력이 안되면 재부팅후 다시확인
```
sudo reboot
```

## 6.launch 파일 수정(15, 17 line)
```
 sudo nano ~/pinky_violet/src/sllidar_ros2/launch/sllidar_c1_launch.py
 ```

### -수정 전 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/USB0')

frame_id = LaunchConfiguration('frame_id', default='laser')
```

### -수정 후 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/RPLIDAR')

frame_id = LaunchConfiguration('frame_id', default='laser_link')
```
## 7. Lidar실행 및 터미널 확인
```
ros2 topic echo /imu
```
