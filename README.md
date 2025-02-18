# ros-repo-2
# 파이널 프로젝트 2조 저장소. 탁구장 보조 로봇 시스템

##### [발표 자료](https://docs.google.com/presentation/d/1xOxRUXOxwHskzUP0Xg-ggv8QNKAEsM2Z1EtBP2nl45w/edit?usp=sharing)

## 🤖 프로젝트 소개
<details markdown="1">
<summary><h3>탁구공 수거 및 서빙 시스템을 갖춘 탁구장 보조 로봇 서비스</h3></summary>
<li>여러분들은 탁구를 쳐 보신 경험이 있으신가요 ?</li>
<li>탁구를 치다 보면 공을 자주 떨어뜨리게 된다면 계속 주워야한다는 귀찮고 불편한 경험이 있으실 겁니다.</li>
<li>이런 경험을 로봇으로 해소하고자 다음과 같은 서비스를 기획하게 되었습니다.</li>
</details>
<br/>

## 🧠 구성원 및 역할
|이름|업무|
|:---|:---|
| 임시온(팀장) | • 탁구공 객체 인식 모델 제작 및 좌표 변환<br> • Domain Bridge 활용한 로봇 통신 및 작업 관리 스케쥴러 제작<br> • 시스템 설계 및 시스템 통합<br> • 데이터베이스 설계 및 기능 리스트 제작| 
| 김진재 | • 로봇 경로 생성 및 모니터링 프로그램 제작<br> • 하드웨어 및 맵 설계 제작<br> • UML 설계 및 시스템 설계<br> • 로봇 통신 시스템 테스트 및 관리| 
| 유재현 | • ArUco Marker를 통한 로봇 위치 보정<br> • 로봇 상태 관리 기반 주행<br> • 로봇 주행 파라미터 관리 및 주행 테스트| 
| 조지은 | • Admin, User, Kiosk GUI 구현<br> • GUI와 데이터 베이스 연동 및 관리<br> • RFID를 이용한 회원 구분 기능 제작| 

---

## 🖥️ 개발 환경 및 활용 기술
|구분|상세|
|:---|:---|
| 개발환경 | Ubuntu 24.04 LTS|
| 언어 | Python| 
| 하드웨어 | Raspberry Pi 5, Arduino, Pinky, Raspberry Camera|
| 데이터베이스 | Mysql|
| UI | PyQT5 |
| 객체인식 | Ultralytics YOLOv8, Opencv|
| 주행 | ROS2 (JAZZY), SLAM, NAV2, 2D LIDAR, ArUco Marker|
| 통신 | UDP, TCP, ROS|

---
## 시스템 설계

### 기능 리스트
#### 주 기능
- ##### 회원관리
- ##### 탁구공 수거
- ##### 제품 서빙
- ##### 탁구대 관리

![image](https://github.com/user-attachments/assets/5e1b16b2-b188-49c9-9093-b6efe1c72b8b)
![image](https://github.com/user-attachments/assets/c9bcd42f-c70f-47e5-b33f-420d268c26f1)
![image](https://github.com/user-attachments/assets/c26972e8-3a8c-4789-b430-cc7e39800985)

### 시스템 구성도
![RoboSphere_아키텍쳐-페이지-2 drawio (1)](https://github.com/user-attachments/assets/3b8454a0-23fd-4bc7-8ad1-8aaf2ce2f6af)

### UML
![image](https://github.com/user-attachments/assets/87110099-998e-4c49-908d-08f2b35a4cde)

### 시나리오 구성도
![image](https://github.com/user-attachments/assets/bff776d9-0233-41f8-ba6a-69ec8797d4ad)
![image](https://github.com/user-attachments/assets/1496e658-ab76-4a2e-9651-2d4b475e9281)
![image](https://github.com/user-attachments/assets/7b594423-543c-4c07-b5a5-0e31fa7b9c3f)

### 네트워크 구성도
![image](https://github.com/user-attachments/assets/8a1aa39d-1fe9-4dee-bb02-128a9bbeba52)

### 데이터 베이스 설계
![image](https://github.com/user-attachments/assets/e4669fbb-2ace-4f9d-876d-6a88f0956600)

### 로봇 구동 시나리오
> #### 상태 기반 주행
> ##### 대기, 작업 중, 작업 완료 의 3가지 상태가 있습니다.
> 
> #### 1. 주행 경로 생성
> - ##### 작업 할당 시 waypoint로 좌표를 목적지까지의 좌표를 할당함.
> #### 2. 실시간 객체 인식
> - ##### 카메라 프레임에서 실시간으로 객체 인식 후 좌표를 3D 월드 좌표로 변환
> - ##### 경로 최적화를 위해 로봇 기준 0.2 ~ 0.5 M 내외 인식
> #### 3. 제외 영역 판단
> - ##### 로봇이 탁구공 수거 작업을 시행 시 각 작업마다 수거 영역이 있고, 해당 작업의 수거 영역에 탁구공이 있는지 판단
> - ##### 수거 영역에 있다면 waypoint 추가
> - ##### 수거 영역에 없다면(=제외 영역) waypoint에 추가하지 않음

### 맵 구성
> ![image](https://github.com/user-attachments/assets/eb9eed58-5c90-413a-8a22-3ab67c1408b2)
> ![image](https://github.com/user-attachments/assets/8acd8962-822e-4c0b-b81b-89d0b288cab0)
> ![image](https://github.com/user-attachments/assets/f7fba563-4640-4c14-aa07-bc78cb1a21e0)
> ![image](https://github.com/user-attachments/assets/25db795b-0c83-4acd-a6b8-859fee28b997)
> #### 1. 탁구대 테이블 (4대)
> #### 2. 음식물 진열대
> #### 3. 카운터 / 키오스크
> #### 4. 탁구공 저장소
> #### 5. 쓰레기통
> #### 6. 로봇 스테이션( 로봇이 대기하는 장소 )

---


