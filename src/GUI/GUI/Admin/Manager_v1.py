import pymysql  # MySQL 데이터베이스 연동 라이브러리
import sys  # 시스템 관련 기능 (예: 프로그램 종료)
from PyQt5.QtWidgets import * #PyQt5에서 GUI 관련 기능을 사용
from PyQt5.QtCore import *
from PyQt5.QtCore import QTimer, Qt, QTime # 타이머 및 시간 관련 기능
from PyQt5.QtGui import * # 그래픽 관련 기능
from PyQt5 import uic # UI 파일을 불러오는 기능
import pymysql # MySQL 데이터베이스와 연결하는 기능
import serial  # RFID 리더기와 시리얼 통신을 하기 위한 라이브러리
import struct  # 바이너리 데이터 패킹 및 언패킹 (RFID 데이터 송수신에 사용)
import time    # 딜레이 설정을 위한 라이브러리
from datetime import datetime, timedelta
import socket
import json
import threading
import queue
import heapq
from PIL import Image
import yaml
import numpy as np

dict_lock = threading.Lock()
pinky1_latest_status = {'pinkyStatus': None, 'pinkyBettery' : None}
pinky2_latest_status = {'pinkyStatus': None, 'pinkyBettery' : None}

# UI 파일 로드
from_class = uic.loadUiType("/home/kjj73/test_folder/src/GUI/GUI/Admin/Manager.ui")[0]

class DatabaseManager:
    def __init__(self):
        """ MySQL 데이터베이스 연결 """
        self.conn = pymysql.connect(
            host="localhost",         
            user="lento",             
            password="0819",          
            database="test2",         
            charset="utf8mb4",       
            cursorclass=pymysql.cursors.DictCursor  # 결과를 딕셔너리 형태로 반환
        )
        self.cursor = self.conn.cursor()  

    def fetch_table_status(self):
        """ 📌 테이블 상태 가져오기 (사용 중 / 비어 있음 등) """
        query = "SELECT table_id, table_status_id FROM pp_table"
        self.cursor.execute(query)
        return self.cursor.fetchall()  

    def fetch_table_usage(self):
        """ 📌 과거 탁구대 이용 내역 가져오기 (`past_reservations` 활용) """
        query = """
        SELECT pp_table.table_id, past_reservations.start_time, past_reservations.end_time, past_reservations.price 
        FROM past_reservations 
        JOIN pp_table ON past_reservations.table_id = pp_table.table_id
        ORDER BY past_reservations.start_time DESC
        """
        self.cursor.execute(query)
        return self.cursor.fetchall()

    def fetch_sales(self):
        """ 📌 매출 조회 (`past_reservations` 활용) """
        query = """
        SELECT pp_table.table_id, past_reservations.start_time, past_reservations.end_time, past_reservations.price 
        FROM past_reservations 
        JOIN pp_table ON past_reservations.table_id = pp_table.table_id
        WHERE DATE(past_reservations.start_time) = CURDATE()
        ORDER BY past_reservations.start_time DESC
        """
        self.cursor.execute(query)
        return self.cursor.fetchall()

    def close_connection(self):
        """ 데이터베이스 연결 종료 """
        self.cursor.close()
        self.conn.close()


class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  
        self.setWindowTitle("Manager")

        # ✅ MySQL 연결
        self.db = DatabaseManager()

        # ✅ 테이블 UI 설정
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_2.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_3.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_4.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # ✅ 버튼 이벤트 연결
        self.pushButton.clicked.connect(lambda: self.switch_page(0, "실시간 위치"))
        self.pushButton_2.clicked.connect(lambda: self.switch_page(1, "매장 운영 내역"))
        self.pushButton_3.clicked.connect(self.load_table_data)  # 과거 이용 내역 검색
        self.pushButton_4.clicked.connect(self.load_sales_data)  # 매출 조회

        self.dateEdit.setDisplayFormat("yy/MM/dd")

        # ✅ 초기 데이터 불러오기
        self.load_table_status()
        # self.setup_robot_data()

        ###
        self.filepath = '/home/kjj73/test_folder/data/'
        self.pgmpath = self.filepath+'fourtable.pgm'
        self.yamlpath = self.filepath+'fourtable.yaml'

        self.pixelSize = 5                              # 원래 픽셀 마다의 크기 0.05 cm
        self.collisionArea = 2                          # 로봇의 사이즈를 고려한 장애물 영역 설정
        self.mapRatio = self.collisionArea * 8          # 맵의 크기를 8배 늘려서 비율을 맞춰줌
        self.realAreaSize = int(self.mapRatio)

        with Image.open(self.pgmpath) as pgm_image:
            self.mapWidth, self.mapHeight = pgm_image.size
            self.img_array = np.array(pgm_image).T
            #self.img_array = np.flip(self.img_array)
            print("img_array:", self.img_array.shape)

        self.mapLimitX = (self.mapWidth * 5) / 100
        self.mapLimitY = (self.mapHeight * 5) / 100

        with open(self.yamlpath, 'r') as file:
            self.yaml = yaml.safe_load(file)

        self.label_8.setGeometry(0,0,self.mapWidth * 8, self.mapHeight * 8)
        self.pixmap = QPixmap()
        self.pixmap.load(self.pgmpath)
        self.pixmap = self.pixmap.scaled(self.label_7.width(), self.label_7.height())
        self.label_8.setPixmap(self.pixmap)

        self.pixmapWidth = self.label_8.width()
        self.pixmapHeight = self.label_8.height()

        table_width = 60
        table_hegith = 97
        self.label_12.setGeometry(270,255,table_width, table_hegith)
        self.label_9.setGeometry(270,400,table_hegith + 2,table_width - 2)
        self.label_10.setGeometry(415,400,table_hegith ,table_width - 2)
        self.label_11.setGeometry(390,255,table_width,table_hegith)

        # self.label_9.setAttribute(Qt.WA_TranslucentBackground)
        # self.label_10.setAttribute(Qt.WA_TranslucentBackground)
        # self.label_11.setAttribute(Qt.WA_TranslucentBackground)
        # self.label_12.setAttribute(Qt.WA_TranslucentBackground)

        self.pinky1_bettery_queue = queue.Queue(maxsize=2)
        pinky1_bettery_tcp_thread = threading.Thread(target=self.pinky1_receive_tcp_bettery)
        pinky1_bettery_tcp_thread.daemon = True  # 데몬 스레드로 설정해서 프로그램 종료 시 자동 종료
        pinky1_bettery_tcp_thread.start()

        self.pinky2_bettery_queue = queue.Queue(maxsize=2)
        pinky1_bettery_tcp_thread = threading.Thread(target=self.pinky2_receive_tcp_bettery)
        pinky1_bettery_tcp_thread.daemon = True  # 데몬 스레드로 설정해서 프로그램 종료 시 자동 종료
        pinky1_bettery_tcp_thread.start()

        bettery_thread = threading.Thread(target=self.show_bettery_1, daemon=True)
        bettery_thread.start()
        bettery_thread = threading.Thread(target=self.show_bettery_2, daemon=True)
        bettery_thread.start()

        self.status_queue_1 = queue.Queue(maxsize=2)
        pinky1_status_tcp_thread = threading.Thread(target=self.pinky1_receive_tcp_status)
        pinky1_status_tcp_thread.daemon = True  # 데몬 스레드로 설정해서 프로그램 종료 시 자동 종료
        pinky1_status_tcp_thread.start()

        self.status_queue_2 = queue.Queue(maxsize=2)
        pinky2_status_tcp_thread = threading.Thread(target=self.pinky2_receive_tcp_status)
        pinky2_status_tcp_thread.daemon = True  # 데몬 스레드로 설정해서 프로그램 종료 시 자동 종료
        pinky2_status_tcp_thread.start()

        status_thread_1 = threading.Thread(target=self.show_status_1, daemon=True)
        status_thread_1.start()
        status_thread_2 = threading.Thread(target=self.show_status_2, daemon=True)
        status_thread_2.start()

        self.db_manager = DatabaseManager()  # 데이터베이스 연결 객체 생성
        self.previous_table_status = {}  # 이전 테이블 상태 저장
        self.check_in_status = {} #입실 여부를 저장하는 딕셔너리 초기화

        self.prev_pinky1_bettery = None
        self.prev_pinky2_bettery = None
        self.prev_pinky1_status = None
        self.prev_pinky2_status = None
        
        latest_status = threading.Thread(target=self.status_latest, daemon=True)
        latest_status.start()

    def status_latest(self):
        while True:
            self.setup_robot_data()
            time.sleep(0.5)

    def pinky1_receive_tcp_bettery(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', 9152))  # 서버 주소와 포트
                s.listen()
                print('Listening for incoming connections...')
                
                while True:
                    conn, addr = s.accept()
                    with conn:
                        print('Connected by', addr)
                        data = conn.recv(1024)
                        if not data:
                            break
                        signal = json.loads(data.decode('utf-8'))
                        self.process_bettery_1(signal)
                        
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")
            exit()

        finally:
            print("bettery clean up completed")

    def pinky2_receive_tcp_bettery(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', 9153))  # 서버 주소와 포트
                s.listen()
                print('Listening for incoming connections...')
                
                while True:
                    conn, addr = s.accept()
                    with conn:
                        print('Connected by', addr)
                        data = conn.recv(1024)
                        if not data:
                            break
                        signal = json.loads(data.decode('utf-8'))
                        self.process_bettery_2(signal)
                        
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")
            exit()

        finally:
            print("bettery clean up completed")

    # 받은 시그널 처리
    def process_bettery_1(self, signal):
        print(f"Received signal: {signal['bettery']}")
        if self.pinky1_bettery_queue.full():
            self.pinky1_bettery_queue.get()  # 큐가 가득 차면 가장 오래된 데이터 제거
        self.pinky1_bettery_queue.put(signal)  # 새로운 데이터 큐에 삽입

    def show_bettery_1(self):
        # 큐에서 최신 배터리 상태를 가져와서 출력
        while True:
            if not self.pinky1_bettery_queue.empty():
                signal = self.pinky1_bettery_queue.get()
                print(f'Bettery1: {signal["bettery"]}')
                with dict_lock:  # 락을 걸고 데이터를 수정
                    pinky1_latest_status['pinkyBettery'] = str(signal["bettery"])
            else:
                print("pinky1 Bettery No signal available or empty.")
            time.sleep(1)

    # 받은 시그널 처리
    def process_bettery_2(self, signal):
        print(f"Received signal: {signal['bettery']}")
        if self.pinky2_bettery_queue.full():
            self.pinky2_bettery_queue.get()  # 큐가 가득 차면 가장 오래된 데이터 제거
        self.pinky2_bettery_queue.put(signal)  # 새로운 데이터 큐에 삽입

    def show_bettery_2(self):
        # 큐에서 최신 배터리 상태를 가져와서 출력
        while True:
            if not self.pinky2_bettery_queue.empty():
                signal = self.pinky2_bettery_queue.get()
                print(f'Bettery2: {signal["bettery"]}')
                with dict_lock:  # 락을 걸고 데이터를 수정
                    pinky2_latest_status['pinkyBettery'] = str(signal["bettery"])
            else:
                print("pinky2 Bettery No signal available or empty.")
            time.sleep(1)

    def pinky1_receive_tcp_status(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 기존 바인딩된 포트 재사용 허용
                s.bind(('localhost', 9155))
                s.listen(5)
                s.listen()
                print('Listening for incoming connections...')
                
                while True:
                    conn, addr = s.accept()
                    with conn:
                        # print('Connected by', addr)
                        data = conn.recv(1024)
                        if not data:
                            break
                        signal = json.loads(data.decode('utf-8'))
                        self.process_status_1(signal)
                        
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")
            exit()

        finally:
            print("status clean up completed")

    def pinky2_receive_tcp_status(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', 9156))  # 서버 주소와 포트
                s.listen()
                print('Listening for incoming connections...')
                
                while True:
                    conn, addr = s.accept()
                    with conn:
                        # print('Connected by', addr)
                        data = conn.recv(1024)
                        if not data:
                            break
                        signal = json.loads(data.decode('utf-8'))
                        self.process_status_2(signal)
                        
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")
            exit()

        finally:
            print("status clean up completed")

    # 받은 시그널 처리
    def process_status_1(self, signal):
        # print(f"Received signal: {signal}")
        if self.status_queue_1.full():
            self.status_queue_1.get()  # 큐가 가득 차면 가장 오래된 데이터 제거
        self.status_queue_1.put(signal)  # 새로운 데이터 큐에 삽입

    def process_status_2(self, signal):
        # print(f"Received signal: {signal}")
        if self.status_queue_2.full():
            self.status_queue_2.get()  # 큐가 가득 차면 가장 오래된 데이터 제거
        self.status_queue_2.put(signal)  # 새로운 데이터 큐에 삽입

    def show_status_1(self):
        # 큐에서 최신 배터리 상태를 가져와서 출력
        self.pinky_status_1.setGeometry(0,0, self.label_7.width(), self.label_7.height())
        self.pixmap2 = QPixmap(self.label_7.width(), self.label_7.height())
        self.pixmap2.fill(Qt.transparent)  # 🎯 fill()을 먼저 실행
        self.pinky_status_1.setPixmap(self.pixmap2)
        # self.pinky_status.fill(Qt.transparent)
        # self.pinky_status.pixmap().fill(Qt.transparent)
        # positionPainter = QPainter(self.pinky_status.pixmap())
        # positionPainter = QPainter(self.pixmap2)

        pinky1_size = 50
        path = '/home/kjj73/test_folder/data/'
        pinky_emotion = path+'pinky_emoticon.png'

        # self.pinky1.setGeometry(0, 0, pinky1_size, pinky1_size)
        # self.pixmap3 = QPixmap()
        # self.pixmap3.load(pinky_emotion)
        # self.pixmap3 = self.pixmap3.scaled(pinky1_size, pinky1_size)
        # self.pinky1.setPixmap(self.pixmap3)

        while True:
            positionPainter = QPainter(self.pixmap2)
            if not self.status_queue_1.empty():
                signal = self.status_queue_1.get()
                # print(f'show status : {signal['status']}, c : {signal['current_position']}, s : {signal['start_position']}, g : {signal['goal_position']}, current_xyz : {signal["current_position"]["x"]}, {signal["current_position"]["y"]}, {signal["current_position"]["z"]}')
                # x y z에 접근
                # 예) signal["current_position"]["x"]
                # 위치 정보를 담고 있는 키 목록
                task_status = signal['status']
                with dict_lock:  # 락을 걸고 데이터를 수정
                    pinky1_latest_status['pinkyStatus'] = str(task_status)
                # 각 위치의 x, y, z 값을 실수형(float) NumPy 배열로 변환
                positions = ['current_position', 'start_position', 'goal_position']
                grouped_xyz = np.array([
                    np.array([signal[pos]['x'], signal[pos]['y'], signal[pos]['z']], dtype=np.float32) 
                    for pos in positions
                ], dtype=np.float32)

                print(f'show status original : {grouped_xyz}')
                print(f'show status [:,0] : {grouped_xyz[:,0]}')

                if not np.any(grouped_xyz):
                    print("Empty")
                else :
                    print(f'값 : {self.mapLimitX}, {self.mapLimitY}, {self.yaml['origin'][0]}, {self.yaml['origin'][1]}')
                    grouped_xyz[:, 0] = (grouped_xyz[:, 0] + (self.mapLimitX - (self.mapLimitX - abs(self.yaml['origin'][0]))))
                    grouped_xyz[:, 1] = (abs(grouped_xyz[:, 1]) + (self.mapLimitY - abs(self.yaml['origin'][1])))

                    # arr = arr * -1
                    grouped_xyz[:, 0] = (grouped_xyz[:, 0] * 8 * 20)
                    grouped_xyz[:, 1] = (grouped_xyz[:, 1] * 8 * 20)

                    grouped_xyz = np.round(grouped_xyz).astype(int)

                print(f'show status over : {grouped_xyz}')

                self.pixmap2.fill(Qt.transparent)  # 🎯 fill()을 먼저 실행
                # 🔴 빨간색 굵은 "X" 표시 (goal_position)
                goal_x, goal_y = int(grouped_xyz[2,0]), int(grouped_xyz[2,1])
                positionPainter.setPen(QPen(Qt.red, 5, Qt.SolidLine))  # 빨간색, 두께 5
                positionPainter.drawLine(goal_x - 10, goal_y - 10, goal_x + 10, goal_y + 10)  # X의 대각선 1
                positionPainter.drawLine(goal_x + 10, goal_y - 10, goal_x - 10, goal_y + 10)  # X의 대각선 2

                # cur_x, cur_y = int(grouped_xyz[0,0] - (pinky1_size/2)), int(grouped_xyz[0,1] - (pinky1_size/2))
                # print("cur : ", cur_x, cur_y)
                # try:
                #     self.pinky1.setGeometry(cur_x, cur_y, pinky1_size, pinky1_size)
                # except Exception as e:
                #     print(f"Error setting geometry: {e}")
                # # self.pinky1.show()
                
                # 🔵 초록색 동그라미 (start_position)
                cur_x, cur_y = int(grouped_xyz[0,0]), int(grouped_xyz[0,1])
                print("cur : ", cur_x, cur_y)
                positionPainter.setPen(QPen(Qt.green, 5, Qt.SolidLine))  # 파란색, 두께 3
                positionPainter.drawEllipse(cur_x - 10, cur_y - 10, 30, 30)  # (x-10, y-10)에서 20x20 크기의 원

                # 🔵 파란색 동그라미 (start_position)
                start_x, start_y = int(grouped_xyz[1,0]), int(grouped_xyz[1,1])
                positionPainter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))  # 파란색, 두께 3
                positionPainter.drawEllipse(start_x - 5, start_y - 5, 20, 20)  # (x-10, y-10)에서 20x20 크기의 원

                positionPainter.end()

                # 변경된 QPixmap을 위젯에 다시 설정하여 업데이트 반영
                self.pinky_status_1.setPixmap(self.pixmap2)
            else:
                print("Status No signal available or empty.")
                positionPainter.end()
            time.sleep(1)

    def show_status_2(self):
        # 큐에서 최신 배터리 상태를 가져와서 출력
        self.pinky_status_2.setGeometry(0,0, self.label_7.width(), self.label_7.height())
        self.pixmap4 = QPixmap(self.label_7.width(), self.label_7.height())
        self.pixmap4.fill(Qt.transparent)  # 🎯 fill()을 먼저 실행
        self.pinky_status_2.setPixmap(self.pixmap4)
        # self.pinky_status.fill(Qt.transparent)
        # self.pinky_status.pixmap().fill(Qt.transparent)
        # positionPainter = QPainter(self.pinky_status.pixmap())
        # positionPainter = QPainter(self.pixmap2)

        pinky1_size = 50
        path = '/home/kjj73/test_folder/data/'
        pinky_emotion = path+'pinky_emoticon.png'

        # self.pinky2.setGeometry(0, 0, pinky1_size, pinky1_size)
        # self.pixmap4 = QPixmap()
        # self.pixmap4.load(pinky_emotion)
        # self.pixmap4 = self.pixmap4.scaled(pinky1_size, pinky1_size)
        # self.pinky2.setPixmap(self.pixmap4)

        while True:
            positionPainter = QPainter(self.pixmap4)
            if not self.status_queue_2.empty():
                signal = self.status_queue_2.get()
                # print(f'show status : {signal['status']}, c : {signal['current_position']}, s : {signal['start_position']}, g : {signal['goal_position']}, current_xyz : {signal["current_position"]["x"]}, {signal["current_position"]["y"]}, {signal["current_position"]["z"]}')
                # x y z에 접근
                # 예) signal["current_position"]["x"]
                # 위치 정보를 담고 있는 키 목록
                task_status = signal['status']
                with dict_lock:  # 락을 걸고 데이터를 수정
                    pinky2_latest_status['pinkyStatus'] = str(task_status)
                # 각 위치의 x, y, z 값을 실수형(float) NumPy 배열로 변환
                positions = ['current_position', 'start_position', 'goal_position']
                grouped_xyz = np.array([
                    np.array([signal[pos]['x'], signal[pos]['y'], signal[pos]['z']], dtype=np.float32) 
                    for pos in positions
                ], dtype=np.float32)

                print(f'show status original : {grouped_xyz}')
                print(f'show status [:,0] : {grouped_xyz[:,0]}')

                if not np.any(grouped_xyz):
                    print("Empty")
                else :
                    print(f'값 : {self.mapLimitX}, {self.mapLimitY}, {self.yaml['origin'][0]}, {self.yaml['origin'][1]}')
                    grouped_xyz[:, 0] = (grouped_xyz[:, 0] + (self.mapLimitX - (self.mapLimitX - abs(self.yaml['origin'][0]))))
                    grouped_xyz[:, 1] = (abs(grouped_xyz[:, 1]) + (self.mapLimitY - abs(self.yaml['origin'][1])))

                    # arr = arr * -1
                    grouped_xyz[:, 0] = (grouped_xyz[:, 0] * 8 * 20)
                    grouped_xyz[:, 1] = (grouped_xyz[:, 1] * 8 * 20)

                    grouped_xyz = np.round(grouped_xyz).astype(int)

                print(f'show status over : {grouped_xyz}')

                self.pixmap4.fill(Qt.transparent)  # 🎯 fill()을 먼저 실행
                # 🔴 빨간색 굵은 "X" 표시 (goal_position)
                goal_x, goal_y = int(grouped_xyz[2,0]), int(grouped_xyz[2,1])
                positionPainter.setPen(QPen(Qt.red, 5, Qt.SolidLine))  # 빨간색, 두께 5
                positionPainter.drawLine(goal_x - 10, goal_y - 10, goal_x + 10, goal_y + 10)  # X의 대각선 1
                positionPainter.drawLine(goal_x + 10, goal_y - 10, goal_x - 10, goal_y + 10)  # X의 대각선 2

                # cur_x, cur_y = int(grouped_xyz[0,0] - (pinky1_size/2)), int(grouped_xyz[0,1] - (pinky1_size/2))
                # print("cur : ", cur_x, cur_y)
                # try:
                #     self.pinky1.setGeometry(cur_x, cur_y, pinky1_size, pinky1_size)
                # except Exception as e:
                #     print(f"Error setting geometry: {e}")
                # # self.pinky1.show()
                
                # 🔵 초록색 동그라미 (start_position)
                cur_x, cur_y = int(grouped_xyz[0,0]), int(grouped_xyz[0,1])
                print("cur : ", cur_x, cur_y)
                positionPainter.setPen(QPen(Qt.green, 5, Qt.SolidLine))  # 파란색, 두께 3
                positionPainter.drawEllipse(cur_x - 15, cur_y - 95, 30, 30)  # (x-10, y-10)에서 20x20 크기의 원

                # 🔵 파란색 동그라미 (start_position)
                start_x, start_y = int(grouped_xyz[1,0]), int(grouped_xyz[1,1])
                positionPainter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))  # 파란색, 두께 3
                positionPainter.drawEllipse(start_x - 10, start_y - 90, 20, 20)  # (x-10, y-10)에서 20x20 크기의 원

                positionPainter.end()

                # 변경된 QPixmap을 위젯에 다시 설정하여 업데이트 반영
                self.pinky_status_2.setPixmap(self.pixmap4)
            else:
                print("Status No signal available or empty.")
                positionPainter.end()
            time.sleep(1)

    def load_table_status(self):
        """ 📌 MySQL에서 테이블 상태 불러오기 """
        data = self.db.fetch_table_status()
        table_data = []
        status_map = {1: "비어 있음", 2: "사용 중", 3: "청소 중"}

        for row in data:
            table_id = f"테이블 {row['table_id']}"
            status = status_map.get(row['table_status_id'], "알 수 없음")
            remaining_time = "" if status != "청소 중" else "5분 남음"
            table_data.append([table_id, status, remaining_time])

        self.populate_table(self.tableWidget, table_data)

    def load_table_data(self):
        """ 📌 MySQL에서 과거 탁구대 이용 내역 불러오기 (`past_reservations` 활용) """
        data = self.db.fetch_table_usage()
        table_data = []

        for row in data:
            start_time = row["start_time"].strftime("%Y-%m-%d %H:%M")
            end_time = row["end_time"].strftime("%H:%M")
            price = f"{row['price']:,}원"
            table_data.append([f"테이블 {row['table_id']}", f"{start_time} - {end_time}", price])

        self.populate_table(self.tableWidget_3, table_data)

    def load_sales_data(self):
        """ 📌 MySQL에서 매출 데이터 불러오기 (`past_reservations` 활용) """
        data = self.db.fetch_sales()
        table_data = []

        for row in data:
            start_time = row["start_time"].strftime("%Y-%m-%d %H:%M")
            # end_time = row["end_time"].strftime("%H:%M")
            price = f"{row['price']:,}원"
            table_data.append([f"테이블 {row['table_id']}", start_time, price])  # ✅ `start_time`만 추가

        self.populate_table(self.tableWidget_4, table_data)

        # **총 매출 계산**
        total_sales = sum(row["price"] for row in data)
        self.label_5.setText(f"{total_sales:,}원")

    def setup_robot_data(self):
        """ 로봇 상태 데이터 (임시) """
        pinky1_bettery = None
        pinky2_bettery = None
        pinky1_status = None
        pinky2_status = None

        with dict_lock:
            # 이전 값 유지 로직 추가
            pinky1_bettery = pinky1_latest_status.get('pinkyBettery') or self.prev_pinky1_bettery
            pinky2_bettery = pinky2_latest_status.get('pinkyBettery') or self.prev_pinky2_bettery
            pinky1_status = pinky1_latest_status.get('pinkyStatus') or self.prev_pinky1_status
            pinky2_status = pinky2_latest_status.get('pinkyStatus') or self.prev_pinky2_status

        # 기존 값을 업데이트
        self.prev_pinky1_bettery = pinky1_bettery
        self.prev_pinky2_bettery = pinky2_bettery
        self.prev_pinky1_status = pinky1_status
        self.prev_pinky2_status = pinky2_status

        # str(energy)
        data = [
            [str("RoboSpere1"), str(pinky1_status), str(pinky1_bettery)],
            [str("RoboSpere2"), str(pinky2_status), str(pinky2_bettery)]
        ]
        self.populate_table(self.tableWidget_2, data)

    def populate_table(self, table_widget, data):
        """ 📌 테이블에 데이터를 추가하는 함수 """
        table_widget.setRowCount(len(data))
        table_widget.setColumnCount(3)

        for row_idx, row in enumerate(data):
            for col_idx, value in enumerate(row):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                table_widget.setItem(row_idx, col_idx, item)

    def switch_page(self, page_index, button_name):
        """ 📌 페이지 이동 """
        self.stackedWidget.setCurrentIndex(page_index)
        self.update_button_styles(button_name)

    def update_button_styles(self, active_button):
        """ 📌 버튼 스타일 변경 """
        if active_button == "실시간 위치":
            self.pushButton.setStyleSheet("background-color: rgb(242, 186, 31);")
            self.pushButton_2.setStyleSheet("background-color: rgb(163, 163, 163);")
        else:
            self.pushButton.setStyleSheet("background-color: rgb(163, 163, 163);")
            self.pushButton_2.setStyleSheet("background-color: rgb(242, 186, 31);")

    def closeEvent(self, event):
        """ 프로그램 종료 시 데이터베이스 연결 종료 """
        self.db.close_connection()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()