import sys  # 시스템 관련 기능 (예: 프로그램 종료)
from PyQt5.QtWidgets import * #PyQt5에서 GUI 관련 기능을 사용
from PyQt5.QtCore import QTimer, Qt, QTime # 타이머 및 시간 관련 기능
from PyQt5.QtGui import * # 그래픽 관련 기능
from PyQt5 import uic # UI 파일을 불러오는 기능
import pymysql # MySQL 데이터베이스와 연결하는 기능
import serial  # RFID 리더기와 시리얼 통신을 하기 위한 라이브러리
import struct  # 바이너리 데이터 패킹 및 언패킹 (RFID 데이터 송수신에 사용)
import time    # 딜레이 설정을 위한 라이브러리
from datetime import datetime, timedelta



# UI파일을 로드하여 PyQt5에서 사용할 수 있도록 설정
from_class = uic.loadUiType("/home/kjj73/test_folder/src/GUI/GUI/Kiosk/Kiosk_v2.ui")[0]



class DatabaseManager:
    def __init__(self):
        self.conn = pymysql.connect(
            host="localhost",          # 데이터베이스 서버 주소 (현재 로컬호스트)
            user="lento",              # 데이터베이스 사용자 이름
            password="0819",           # 데이터베이스 비밀번호
            database="test2",          # 사용할 데이터베이스 이름
            charset="utf8mb4"          # 문자 인코딩 설정(한글 지원)
        )
        self.cursor = self.conn.cursor()  # SQL 실행을 위한 커서 생성

    def insert_user(self, phone, name, password, card_number):
        query = """
        INSERT INTO users (user_phone, user_name, password, card_number)
        VALUES (%s, %s, %s, %s)
        """   # SQL INSERT 문 (회원 정보를 저장)
        try:
            self.cursor.execute(query, (phone, name, password, card_number)) # SQL 실행
            self.conn.commit() # 변경 사항 저장
            return self.cursor.lastrowid  # 새로 추가된 회원 ID 반환
        except Exception as e:
            print(f"Error inserting user: {e}") # 예외 발생 시 오류 메시지 출력
            self.conn.rollback() # 오류 발생 시 데이터베이스 변경 취소
            return None # 실패 시 None 반환


### RFID 카드 번호를 사용하여 기존 회원 정보를 조회하는 함수
    def get_user_by_card(self, card_number):
        """카드번호로 기존 회원 정보 조회"""
        query = "SELECT user_id, user_phone, user_name, password FROM users WHERE card_number = %s" # 회원 조회 SQL문
        self.cursor.execute(query, (card_number,)) # 카드 번호를 바인딩하여 실행
        return self.cursor.fetchone()  # (전화번호, 이름, 비밀번호) 튜플 반환, 조회된 회원 정보 반환 (없으면 None)


### 기존 회원 정보를 업데이트하는 함수 

    def update_user(self, phone, name, password, card_number):
        query = """
        UPDATE users 
        SET user_phone = %s, user_name = %s, password = %s 
        WHERE card_number = %s
        """  # 회원 정보를 업데이트하는 SQL문
        try:
            self.cursor.execute(query, (phone, name, password, card_number))  # SQL 실행
            self.conn.commit() # 변경 사항 저장
            return True # 성공 시 true 반환 
        except Exception as e:
            print(f"Error updating user: {e}") # 예외 발생 시 오류 메시지 출력
            self.conn.rollback()  # 오류 발생 시 변경 취소
            return False   # 실패 시 False 반환


### RFID 카드 번호로 기존 회원을 삭제하는 함수 
    def delete_user_by_card(self, card_number):
        query = "DELETE FROM users WHERE card_number = %s" # 회원 삭제 SQL문
        try:
            self.cursor.execute(query, (card_number,)) # SQL 실행
            self.conn.commit() # 변경 사항 저장
            print(f"회원 정보 삭제 완료: {card_number}") # 삭제 완료 메시지 출력
        except Exception as e:
            print(f"Error deleting user: {e}") # 예외 발생 시 오류 메시지 출력
            self.conn.rollback() # 오류 발생 시 변경 취소 


# 새로 넣은 부분 
    def update_table_status_in_db(self, table_id, status_id):
        """ 테이블 상태를 데이터베이스에서 업데이트 """
        query = "UPDATE pp_table SET table_status_id = %s WHERE table_id = %s"
        try:
            self.cursor.execute(query, (status_id, table_id))
            self.conn.commit()
            print(f"[DEBUG] DB 업데이트: 테이블 {table_id} → 상태 {status_id}")
        except Exception as e:
            print(f"Error updating table status: {e}")
            self.conn.rollback()

    def reset_all_tables(self):
        """ 프로그램 종료 시 모든 테이블을 '비어있음(1)'으로 초기화 """
        query = "UPDATE pp_table SET table_status_id = 1"
        try:
            self.cursor.execute(query)
            self.conn.commit()
            print("[DEBUG] 프로그램 종료: 모든 테이블 상태 초기화")
        except Exception as e:
            print(f"Error resetting tables: {e}")
            self.conn.rollback()




### 데이터 베이스 연결을 종료하는 함수 

    def close_connection(self):
        self.cursor.close() # 커서 닫기
        self.conn.close() # 연결 종료 



# 테이블 상태를 표시하는 위젯 (QWidget을 상속받아 UI 요소로 사용)

class TableStatusWidget(QWidget):
    def __init__(self, table_number, status):
        super().__init__()
        self.table_number = table_number # 테이블 번호 저장
        self.setFixedSize(200, 100) # 위젯 크기 설정 (200x100 픽셀)

        # 레이아웃 설정
        self.layout = QVBoxLayout(self) # 세로 정렬 레이아웃 설정

        # 테이블 번호 표시 (라벨)
        self.table_label = QLabel(f"테이블 {table_number}") # "테이블 1" 형식으로 표시
        self.table_label.setAlignment(Qt.AlignCenter) # 중앙 정렬
        self.layout.addWidget(self.table_label) # 레이아웃에 추가 

        # 상태표시 (라벨)
        self.status_label = QLabel(status) # 초기 상태 표시 ("비어 있음", "사용중", "청소중")
        self.status_label.setAlignment(Qt.AlignCenter) # 중앙정렬
        self.layout.addWidget(self.status_label) # 레이아웃에 추가 

        # 남은시간 표시(라벨)
        self.timer_label = QLabel("남은 시간: --:--") # 기본 값 "--:--"
        self.timer_label.setAlignment(Qt.AlignCenter) # 중앙정렬
        self.layout.addWidget(self.timer_label) # 레이아웃에 추가

        # 이용 시작 및 종료 시간 범위 표시(라벨)
        self.time_range_label = QLabel("--:-- ~ --:--") # 기본 값"--:-- ~ --:--"
        self.time_range_label.setAlignment(Qt.AlignCenter) # 중앙 정렬
        self.layout.addWidget(self.time_range_label)  # 레이아웃에 추가

        # 자동 상태 업데이트를 위한 타이머
        self.timer = QTimer(self) # 타이머 생성
        self.timer.timeout.connect(self.update_remaining_time) # 타이머 이벤트 연결

        # 청소 시간이 끝나면 상태를 변경하기 위한 타이머
        self.cleanup_timer = QTimer(self)  # 청소 타이머 생성
        self.cleanup_timer.setSingleShot(True)  # 한 번만 실행되도록 설정
        self.cleanup_timer.timeout.connect(self.cleanup_finished) # 타이머 완료 시 cleanup_finished 실행

        # 내부 변수 초기화
        self.remaining_time = 0 # 남은 시간 (초 단위)
        self.start_time = None # 시작 시간
        self.end_time = None   # 종료 시간

        # 초기 배갱색 설정(테이블 상태에 따라 다름)
        self.set_background_color(status)


    # 테이블 상태를 변경하는 함수 
    def set_status(self, status, duration=None):
        self.status_label.setText(status) # 상태 라벨 업데이트
        self.set_background_color(status) # 배경색 업데이트

        # 메인 윈도우에서 테이블 상태 UI 업데이트
        parent_window = QApplication.instance().activeWindow()
        if isinstance(parent_window, WindowClass):
            parent_window.update_table_labels()        


        # "사용중" 상태일 경우 타이머 시작
        if status == "사용중" and duration:
            self.start_time = QTime.currentTime() # 현재 시간 저장 (시작 시간)
            self.end_time = self.start_time.addSecs(duration * 60) # 종료 시간 계산
            self.remaining_time = duration * 60  # 남은 시간을 초 단위로 설정
            self.timer.start(1000)  # 1초 간격으로 타이머 업데이트
            self.update_time_labels() # 시간 라벨 업데이트
            db_status_id = 2 # '사용중'이면 2
            print(f"[DEBUG] 타이머 시작: {duration}분")
        elif status == "청소중": # "청소중" 상태일 경우 청소 타이머 시작# **DB 상태 업데이트**라벨 비우기
            self.remaining_cleanup_time = 60  # 청소 시간을 1분(60초)으로 설정
            self.timer.start(1000)  # 1초 간격으로 카운트다운 시작
            self.update_cleanup_time_labels()
            db_status_id = 3 # '청소중'이면 3
        else: # 비어있음" 상태일 경우 초기화
            self.timer_label.setText("남은 시간: --:--")
            self.time_range_label.setText("--:-- ~ --:--")
            db_status_id = 1  # '비어있음'이면 1


        # **DB 상태 업데이트 (parent_window가 None이 아닐 때만 실행)**
        if parent_window and hasattr(parent_window, 'db'):
            parent_window.db.update_table_status_in_db(self.table_number, db_status_id)
        else:
            print("[DEBUG] parent_window is None or not WindowClass")

    # 청소 시간이 감소하면서 UI에 업데이트하는 함수
    def update_cleanup_time_labels(self):
        if self.remaining_cleanup_time > 0:
            self.remaining_cleanup_time -= 1 # 남은 시간 감소 
            minutes, seconds = divmod(self.remaining_cleanup_time, 60) # 분, 초 계산
            self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}") # UI 업데이트
            self.time_range_label.setText("")
            print(f"[DEBUG] 청소중 남은 시간: {minutes:02}:{seconds:02}")

            # 메인 윈도우 업데이트
            parent_window = QApplication.instance().activeWindow()
            if isinstance(parent_window, WindowClass):
                parent_window.update_table_labels()
            self.cleanup_timer.start(1000)  # 1초 후 다시 호출
        else:
            self.cleanup_finished() # 청소 완료


    # 테이블 상태에 따라 배경색을 변경하는 함수
    def set_background_color(self, status): 
        palette = self.palette() # 현재 팔레트 가져오기
        if status == "사용중":
            palette.setColor(QPalette.Window, QColor(255, 0, 0, 128))  # 빨간색 (반투명)
        elif status == "청소중":
            palette.setColor(QPalette.Window, QColor(255, 165, 0, 128))  # 주황색 (반투명)
        else:
            palette.setColor(QPalette.Window, QColor(0, 0, 255, 128))  # 파랑색 (반투명)
        self.setAutoFillBackground(True) # 배경색 활성화
        self.setPalette(palette) # 적용

    # 시작 시간과 종료 시간을 UI에 업데이트 하는 함수 
    def update_time_labels(self):
        if self.start_time and self.end_time:
            self.time_range_label.setText(
                f"{self.start_time.toString('HH:mm')} ~ {self.end_time.toString('HH:mm')}"
            ) # "시작시간 ~ 종료 시간" 형태로 표시

    # 남은 시간을 1초씩 감소 시키고 UI에 업데이트하는 함수 
    def update_remaining_time(self):
        if self.status_label.text() == "사용중":
            if self.remaining_time > 0:
                self.remaining_time -= 1 # 남은 시간 감소 
                minutes, seconds = divmod(self.remaining_time, 60) # 분, 초 계산
                self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}") # UI업데이트
                print(f"[DEBUG] 남은 시간: {minutes:02}:{seconds:02}")

                # 메인 윈도우에서 UI 업데이트
                # parent_window = self.parentWidget()
                # if parent_window is None:
                parent_window = QApplication.instance().activeWindow() 
                if isinstance(parent_window, WindowClass):
                    parent_window.update_table_labels()
            else:
                self.set_status("청소중") # 시간이 다 되면 "청소중" 상태로 변경
                parent_window = QApplication.instance().activeWindow()
                if parent_window and hasattr(parent_window, 'db'):
                    parent_window.db.update_table_status_in_db(self.table_number, 3)  # 청소중(3)으로 변경

        elif self.status_label.text() == "청소중":
            self.update_cleanup_time_labels()

    # 청소 시간이 끝난 후 테이블 "비어있음" 상태로 변경하는 함수 
    def cleanup_finished(self):
        self.timer.stop() # 타이머 정지
        self.set_status("비어있음") # 상태 변경
        print("[DEBUG] 청소 완료, 상태: 비어있음")




### 메인 GUI 클래스 (QMainWindow 상속) PyQt5의 UI 파일을 로드하고, 여러 기능을 수행하는 메서드를 포함
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__() # 부모 클래스 초기화
        self.setupUi(self) # UI 설정 (Qt Designer에서 만든 UI 파일 적용)
        
        self.last_tag_time = {}  # RFID 태깅 시간 저장 (각 테이블별)
        self.tag_delay = 5  # ✅ 5초 동안 재태깅 방지 (필요하면 조절 가능)

        # ✅ 퇴실한 사용자 목록 저장 (퇴실하면 여기에 추가)
        self.checked_out_users = set()



        # ✅ 타이머를 통해 모든 페이지에서 RFID 태깅 감지
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_rfid_card)  # 태깅 감지
        self.timer.start(500)  # 0.5초 간격으로 RFID 데이터 확인


        self.setWindowTitle("Kiosk") # 창 제목 설정
    

        # 시스템 초기화 버튼 클릭 시 'reset_system' 메서드 실행
        self.pushButton_21.clicked.connect(self.reset_system)

        # 현재 인식된 RFID UID 저장 변수
        self.current_uid = None


        # MySQL 연결
        self.db = DatabaseManager()

        # 회원가입 버튼 클릭 시 register_user 실행
        self.pushButton_5.clicked.connect(self.register_user)


        # #RFID 리더기와 시리얼 통신 설정
        # self.conn = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=1)
        # self.conn = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=1)

        # RFID 리더기 2개 연결
        self.rfid1 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=1)
        self.rfid2 = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=1)


        #RFID 카드 체크 타이머 설정
        self.timer = QTimer()
        
        self.timer.timeout.connect(self.check_rfid_card) # 타이머가 만료가 되면  `check_rfid_card` 실행
        self.timer.start() # 타이머 시작 

        # 페이지 변경 감지 및 처리
        self.stackedWidget.currentChanged.connect(self.handle_page_change)

        # 선택된 이용 시간, 테이블, 결제 금액 초기화
        self.selected_time = None
        self.selected_table = None
        self.total_price = 0

        # 테이블 상태 위젯 저장용 딕셔너리
        self.table_widgets = {}
        self.initialize_table_status_widgets()

        # 각 버튼을 클릭하면 해당 페이지로 이동하도록 설정
        self.pushButton.clicked.connect(self.go_to_page2)
        self.pushButton_2.clicked.connect(self.go_to_page3)
        self.pushButton_4.clicked.connect(self.go_to_page1)
        self.pushButton_3.clicked.connect(self.go_to_page3)
        self.pushButton_5.clicked.connect(self.go_to_page4)
        self.pushButton_7.clicked.connect(self.go_to_page1)
        self.pushButton_6.clicked.connect(self.go_to_page1)
        self.pushButton_9.clicked.connect(self.go_to_page6)
        self.pushButton_10.clicked.connect(self.go_to_page5)
        self.pushButton_8.clicked.connect(self.go_to_page7)
        self.pushButton_13.clicked.connect(self.go_to_page5)
        self.pushButton_21.clicked.connect(self.go_to_page1)
        self.pushButton_19.clicked.connect(self.go_to_page7)
        self.pushButton_20.clicked.connect(self.go_to_page9)
        self.pushButton_22.clicked.connect(self.go_to_page10)
        self.pushButton_23.clicked.connect(self.go_to_page1)

        # 이동 시간 선택 버튼 설정 (버튼 클릭시 'select_time' 메서드 실행)
        self.pushButton_11.clicked.connect(lambda: self.select_time(1, 7000)) # 30분 7000원
        self.pushButton_12.clicked.connect(lambda: self.select_time(60, 13000)) # 60분 13000원
        
        # 테이블 선택 버튼 설정 (버튼 클릭시 'select_table' 메서드 실행)
        self.pushButton_14.clicked.connect(lambda: self.select_table(1))
        self.pushButton_15.clicked.connect(lambda: self.select_table(2))
        self.pushButton_16.clicked.connect(lambda: self.select_table(3))
        self.pushButton_17.clicked.connect(lambda: self.select_table(4))
        
        # 확인 버튼 클릭 시 'self.confirm_selection' 실행
        self.pushButton_18.clicked.connect(self.confirm_selection)
        
        # 결제 버튼 클릭 시 `handle_payment` 실행
        self.pushButton_20.clicked.connect(self.handle_payment)


    # 페이지 이동 관련 메서드, 메인 페이지(0)로 이동
    def go_to_page1(self):
        self.stackedWidget.setCurrentIndex(0)

    # RFID 초기화 후 페이지(1)로 이동
    def go_to_page2(self):
        self.reset_system() #  시스템 리셋 (이전 RFID 초기화)
        self.stackedWidget.setCurrentIndex(1) # UI 업데이트
        QApplication.processEvents() # UI 업데이트


        print("[DEBUG] RFID 카드 감지 시작")
        self.timer.start()  # RFID 타이머 즉시 시작, 감지 시작


    # 페이지(2)로 이동
    def go_to_page3(self):
        self.stackedWidget.setCurrentIndex(2)
    
    # 페이지(4)로 이동 (5초 후 메인 페이지로 자동 이동)
    def go_to_page4(self):
        self.stackedWidget.setCurrentIndex(4)
        QTimer.singleShot(5000, self.go_to_page1)

    # 페이지(5)로 이동
    def go_to_page5(self):
        self.stackedWidget.setCurrentIndex(5)

    # 페이지(6)로 이동
    def go_to_page6(self):
        self.stackedWidget.setCurrentIndex(6)

    # 페이지(7)로 이동
    def go_to_page7(self):
        self.stackedWidget.setCurrentIndex(7)

    # 페이지(10)로 이동
    def go_to_page10(self):
        self.stackedWidget.setCurrentIndex(10)



    def check_rfid_card(self):
        """RFID 리더기에서 데이터를 감지하여 기능 수행"""

        # ✅ RFID1과 RFID2에 'GS' 명령을 보내고 데이터를 요청
        for rfid_reader in [self.rfid1, self.rfid2]:
            print(f"[DEBUG] {rfid_reader.port}에 'GS' 명령 전송")
            rfid_reader.write(b'GS\n')  # RFID 리더기에 UID 요청
            time.sleep(0.1)  # 데이터를 받을 시간 확보

        print(f"[DEBUG] RFID1 대기 중 데이터 개수: {self.rfid1.in_waiting}")
        print(f"[DEBUG] RFID2 대기 중 데이터 개수: {self.rfid2.in_waiting}")

        current_page = self.stackedWidget.currentWidget().objectName()

        # ✅ RFID1 (회원 확인 & 회원가입) → page_2, page_3에서만 실행
        if self.rfid1.in_waiting > 0 and current_page in ["page_2", "page_3"]:
            res = self.rfid1.read_until(b'\n')
            print(f"[DEBUG] RFID1 수신 데이터: {res}")

            if res.startswith(b'GS') and len(res) >= 7:
                uid = res[3:7].hex().upper()
                print(f"[DEBUG] RFID1 감지됨: {uid}")
                self.handle_rfid1(uid)  

        # ✅ RFID2 (입실/퇴실) → 모든 페이지에서 실행
        if self.rfid2.in_waiting > 0:
            res = self.rfid2.read_until(b'\n')
            print(f"[DEBUG] RFID2 수신 데이터: {res}")

            if res.startswith(b'GS') and len(res) >= 7:
                uid = res[3:7].hex().upper()
                print(f"[DEBUG] RFID2 감지됨: {uid}")
                self.handle_rfid2(uid)






    def handle_rfid1(self, uid):
        """RFID1: 기존 회원 확인 및 회원가입 관련 기능"""
        self.lineEdit_4.setText(uid)  # UI에 UID 표시
        current_page = self.stackedWidget.currentWidget().objectName()

        if current_page == "page_3":
            # DB에서 해당 UID 조회
            user_info = self.db.get_user_by_card(uid)
            if user_info:
                print(f"[DEBUG] 기존 회원 정보 존재, 초기화 후 등록: {user_info}")
                self.db.delete_user_by_card(uid)  # 기존 회원 삭제
            else:
                print("[DEBUG] 신규 카드 감지됨, 회원가입 진행")

            self.stackedWidget.setCurrentIndex(3)  # page_4로 이동 (회원가입)

        elif current_page == "page_2":
            user_info = self.db.get_user_by_card(uid)
            if user_info:
                print(f"[DEBUG] 기존 회원 확인: {user_info}")
                self.stackedWidget.setCurrentIndex(5)  # 기존 회원이면 page_6으로 이동
            else:
                print("[DEBUG] 신규 카드 감지됨, 회원가입 필요")

    def handle_rfid2(self, uid):
        """모든 페이지(page ~ page_11)에서 RFID2 태깅 시 입실/퇴실 가능하도록 설정"""

        print(f"[DEBUG] RFID2 태그 감지: {uid}")

        # 현재 페이지 이름 가져오기
        current_page = self.stackedWidget.currentWidget().objectName()
        print(f"[DEBUG] 현재 페이지: {current_page}")

        # ✅ 모든 페이지에서 RFID 태깅이 가능하도록 범용적으로 처리
        if current_page.startswith("page"):  # 모든 page에서 작동
            print("[INFO] 모든 페이지에서 RFID 태깅 감지됨 → 입실/퇴실 확인 진행")

            # ✅ 사용자 정보 확인
            user_info = self.db.get_user_by_card(uid)
            if not user_info:
                QMessageBox.warning(self, "오류", "등록되지 않은 카드입니다.")
                return

            user_id = user_info[0]  # user_id 가져오기

            # ✅ 퇴실한 사용자라면 다시 입실 불가능
            if uid in self.checked_out_users:
                QMessageBox.warning(self, "입실 제한", "이용 종료된 사용자입니다. 새로운 결제를 진행해주세요.")
                return

            # ✅ 현재 사용 중인지 확인
            for table_number, widget in self.table_widgets.items():
                if widget.status_label.text() == "사용중":
                    print(f"[DEBUG] {uid} 사용자가 사용 중인 테이블 {table_number} 확인됨")

                    # ✅ 최근 태깅 이후 5초 이상 지났는지 확인
                    current_time = time.time()
                    if table_number in self.last_tag_time and (current_time - self.last_tag_time[table_number]) < self.tag_delay:
                        print(f"[DEBUG] {self.tag_delay}초 내 재태깅 감지됨 → 무시")
                        return  # 5초 내 재태깅이면 무시
                    
                    self.last_tag_time[table_number] = current_time  # 마지막 태깅 시간 갱신

                    # ✅ **퇴실 처리: 다시 태그하면 "비어 있음"으로 변경**
                    self.update_table_status(table_number, "비어있음")
                    self.db.update_table_status_in_db(table_number, 1)  # DB에서 '비어있음(1)'으로 변경
                    QMessageBox.information(self, "퇴실 완료", f"테이블 {table_number} 퇴실 완료.")

                    # ✅ 퇴실한 사용자 저장 → 다시 입실 불가능
                    self.checked_out_users.add(uid)
                    print(f"[DEBUG] 퇴실한 사용자 목록: {self.checked_out_users}")

                    return  # 종료

            # ✅ **입실 처리: 사용 중으로 변경**
            if not self.selected_time:
                QMessageBox.warning(self, "시간 선택 오류", "이용 시간을 선택하세요 (30분 또는 1시간).")
                return

            if not self.selected_table:
                QMessageBox.warning(self, "테이블 선택 오류", "테이블을 선택해주세요.")
                return

            remaining_time = self.selected_time  
            print(f"[DEBUG] 선택된 시간 {remaining_time}분, 테이블 {self.selected_table} 이용 시작")

            # ✅ UI 상태 변경 및 남은 시간 설정
            self.update_table_status(self.selected_table, "사용중", remaining_time)

            # ✅ DB 업데이트 (상태: 사용중)
            self.db.update_table_status_in_db(self.selected_table, 2)  # 상태 ID 2: "사용중"

            # ✅ 태깅 시간 저장 (퇴실 방지)
            self.last_tag_time[self.selected_table] = time.time()

            # ✅ 이용 시작 알림
            QMessageBox.information(self, "입실 확인", f"테이블 {self.selected_table} 사용 시작. 이용 시간: {remaining_time}분")

        else:
            print(f"[WARNING] 현재 페이지 {current_page}에서는 RFID 태깅 기능이 없음")





    def handle_page_change(self, index):
        """페이지 변경 시 RFID 감지 활성화/비활성화 설정"""
        current_page = self.stackedWidget.currentWidget().objectName()
        
        # ✅ handle_rfid1(uid)는 page_2, page_3에서만 감지
        if current_page in ["page_2", "page_3"]:
            print(f"[DEBUG] {current_page}: RFID1 감지 활성화 (회원 확인)")
        else:
            print(f"[DEBUG] {current_page}: RFID1 감지 중지")

        # ✅ handle_rfid2(uid)는 모든 페이지에서 감지 가능 → RFID 감지는 항상 실행
        print(f"[DEBUG] {current_page}: RFID2 감지 유지 (입실/퇴실 가능)")

        # ✅ 타이머 항상 실행 (RFID2 감지를 위해)
        QApplication.processEvents()
        self.timer.start()





    def register_user(self):
        """회원가입 버튼 클릭 시 기존 회원 정보 삭제 후 새 정보 등록"""
        phone = self.lineEdit.text().strip()
        name = self.lineEdit_2.text().strip()
        password = self.lineEdit_3.text().strip()
        card_number = self.lineEdit_4.text().strip()  # RFID 카드 UID

        print(f"[DEBUG] 회원가입 버튼 클릭됨")
        print(f"[DEBUG] 입력된 정보: {phone}, {name}, {password}, {card_number}")

        if not phone or not name or not password or not card_number:
            QMessageBox.warning(self, "오류", "모든 필드를 입력하세요!")
            return

        existing_user = self.db.get_user_by_card(card_number)

        if existing_user:
            # 기존 회원 정보 삭제 후 새 정보 저장
            self.db.delete_user_by_card(card_number)
            print(f"[DEBUG] 기존 회원 정보 삭제됨: {card_number}")

        # 새 회원 등록
        user_id = self.db.insert_user(phone, name, password, card_number)
        if user_id:
            QMessageBox.information(self, "회원가입 완료", f"회원가입이 완료되었습니다! 사용자 ID: {user_id}")
            print(f"[DEBUG] 회원가입 성공, page_5로 이동")
            self.stackedWidget.setCurrentIndex(4)  # ✅ 회원가입 완료 후 page_5로 이동
        else:
            QMessageBox.critical(self, "오류", "회원가입에 실패했습니다.")
            print(f"[DEBUG] 회원가입 실패")




    def reset_system(self):
        """초기화 버튼 클릭 시 시스템 리셋"""
        print("[DEBUG] 초기화 버튼 클릭됨 → 모든 입력 필드 초기화 및 RFID 감지 리셋")

        # 입력 필드 초기화
        self.lineEdit.clear()
        self.lineEdit_2.clear()
        self.lineEdit_3.clear()
        self.lineEdit_4.clear()

        # RFID 데이터 초기화
        self.current_uid = None
        print("[DEBUG] RFID UID 초기화 완료")

        # RFID 시리얼 버퍼 클리어
        for rfid_reader in [self.rfid1, self.rfid2]:
            if rfid_reader.is_open:
                rfid_reader.reset_input_buffer()  # 읽기 버퍼 초기화
                rfid_reader.reset_output_buffer()
                print(f"[DEBUG] {rfid_reader.port} 시리얼 버퍼 초기화 완료")

        # RFID 감지 정지 후 다시 시작
        self.timer.stop()
        time.sleep(0.1)  # 짧은 딜레이 추가
        self.timer.start()

        # 첫 페이지로 이동
        self.stackedWidget.setCurrentIndex(0)

    #RFID 카드에 데이터를 기록하는 함수 
    def write_to_rfid(self, block, data): # 데이터 길이제한 
        if len(data) > 16:
            raise ValueError("데이터는 16바이트 이내로 제한됩니다.")

        # 데이터 패킹하여 RFID 리더기로 전송 
        req_data = struct.pack('<2s4s16sB', b'ST', self.current_uid, data.ljust(16, b'\x00'), block)
        self.conn.write(req_data)
        time.sleep(0.1)
        
        # 응답 확인
        res = self.conn.read_until(b'\n')
        if not (res.startswith(b'ST') and res[2] == 0):
            raise ValueError("카드 데이터 저장 실패") # 저장 실패 시 예외 발생

    # 회원가입 페이지로 이동 
    def sign_up(self):
        self.stackedWidget.setCurrentIndex(3)

    # 이용 시간을 선택하는 함수 
    def select_time(self, time, price):
        self.selected_time = time # 선택한 시간 저장
        self.total_price = price # 선택한 요금 저장 
        print(f"선택된 시간: {time}분, 금액: {price}원") # 디버깅 메시지 출력

    # 이용할 테이블을 선택하는 함수
    def select_table(self, table_number):
        self.selected_table = table_number # 선택한 테이블 번호 저장
        print(f"선택된 테이블: {table_number}번") # 디버깅 메시지 출력

    # 선택한 이용 시간과 테이블을 확인하는 함수 
    def confirm_selection(self):
        if self.selected_time and self.selected_table:
            button_name_page_8 = f"pushButton_{13 + self.selected_table}" # 선택한 테이블의 버튼 이름
            button_page_8 = getattr(self, button_name_page_8) # 해당 버튼 객체 가져오기


            if not button_page_8.isEnabled(): # 테이블이 사용 중이면 경고 메시지 출력 
                QMessageBox.warning(self, "오류", "선택한 테이블은 이미 사용 중입니다.")
                return            

            # 선택한 테이블 및 이용 시간 정보 업데이트 
            self.label_selected_table.setText(f"테이블 {self.selected_table}")
            self.label_selected_time.setText(f"{self.selected_time // 60}시간" if self.selected_time == 60 else "30분")
            self.label_price.setText(f"{self.total_price:,}원")

            # 결제 확인 페이지로 이동 
            self.stackedWidget.setCurrentIndex(8)
        else:
            QMessageBox.warning(self, "선택 오류", "이용 시간과 테이블을 모두 선택해주세요.") # 선택 누락 시 경고 

    # 페이지 9로 이동 후 5초 후 메인 페이지로 자동 이동 
    def go_to_page9(self):
        self.stackedWidget.setCurrentIndex(9)
        QTimer.singleShot(5000, self.go_to_page1) # 5초 후 메인 페이지(0)로 이동

    # 테이블 상태를 표시하는 위젯을 초기화하는 함수 
    def initialize_table_status_widgets(self):
        for table_number in range(1, 5): # 테이블 1~4번 초기화
            widget = TableStatusWidget(table_number, "비어있음") # "비어있음" 상태로 초기화
            self.table_widgets[table_number] = widget # 위젯을 딕셔너리에 저장

    # 결제 완료 후 테이블 상태를 '사용중'으로 변경하는 함수
    def handle_payment(self):
        if self.selected_time and self.selected_table:
            # 현재 시간 기준 예약 시작 시간 설정
            start_time = datetime.now()
            end_time = start_time + timedelta(minutes=self.selected_time)

            # RFID 카드 번호 가져오기
            card_number = self.lineEdit_4.text().strip()
            if not card_number:
                QMessageBox.warning(self, "결제 오류", "RFID 카드가 인식되지 않았습니다.")
                return

            # 사용자 정보 조회
            user_info = self.db.get_user_by_card(card_number)
            if not user_info:
                QMessageBox.warning(self, "결제 오류", "해당 카드로 등록된 사용자가 없습니다.")
                return

            user_id = user_info[0]  # user_phone, user_name, password 순서이므로 0번째 값이 user_id

            # ✅ 새로운 결제 후 퇴실 제한 해제
            if card_number in self.checked_out_users:
                self.checked_out_users.remove(card_number)
                print(f"[DEBUG] {card_number} 퇴실 제한 해제됨 (새로운 결제 완료)")



            # 예약 정보 저장
            query = """
            INSERT INTO reservations (user_id, table_id, start_time, end_time)
            VALUES (%s, %s, %s, %s)
            """
            try:
                self.db.cursor.execute(query, (user_id, self.selected_table, start_time, end_time))
                self.db.conn.commit()
                QMessageBox.information(self, "예약 완료", "예약이 정상적으로 완료되었습니다.")
                print(f"[DEBUG] 예약 완료: 사용자 {user_id}, 테이블 {self.selected_table}, 시간 {start_time} ~ {end_time}")
                self.stackedWidget.setCurrentIndex(9)  # 결제 완료 페이지로 이동
            except Exception as e:
                print(f"Error inserting reservation: {e}")
                self.db.conn.rollback()
                QMessageBox.critical(self, "예약 오류", "예약을 저장하는 중 문제가 발생했습니다.")
        else:
            QMessageBox.warning(self, "결제 오류", "이용 시간과 테이블을 모두 선택해주세요.")

    # 테이블 상태를 업데이트 하는 함수 
    def update_table_status(self, table_number, status, duration=None):
        if table_number in self.table_widgets:
            self.table_widgets[table_number].set_status(status, duration)
            print(f"테이블 {table_number}: {status} 상태로 변경 (타이머: {duration}분)")

        self.update_table_labels() # 테이블 상태 UI 업데이트 

    # UI의 테이블 상태를 업데이트하는 함수
    def update_table_labels(self):
        for table_number, widget in self.table_widgets.items(): # 페이지 7과 11의 테이블 상태 라벨 가져오기
            label_name_page_7 = f"label_{7 + table_number}"
            label_page_7 = getattr(self, label_name_page_7)

            label_name_page_11 = f"label_{26 + table_number}"
            label_page_11 = getattr(self, label_name_page_11)

            button_name_page_8 = f"pushButton_{13 + table_number}" # 페이지 8의 테이블 선택 버튼 가져오기
            button_page_8 = getattr(self, button_name_page_8)

            status = widget.status_label.text()  # 테이블 상태 가져오기


            # 상태에 따라 남은 시간과 시간 범위 설정
            if status == "비어있음":
                remaining_time = ""
                time_range = ""
            else:
                remaining_time = widget.timer_label.text()
                time_range = widget.time_range_label.text()

            # UI에 상태 정보 업데이트
            label_page_7.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")
            label_page_11.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")
            button_page_8.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")

            # 상태에 따라 버튼 UI 색상 변경 
            if status == "사용중":
                label_page_7.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                label_page_11.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                button_page_8.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                button_page_8.setEnabled(False) # 사용중이면 버튼 비활성화 
            elif status == "청소중":
                label_page_7.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                label_page_11.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                button_page_8.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                button_page_8.setEnabled(False) # 청소 중이면 버튼 비활성화
            else:
                label_page_7.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                label_page_11.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                button_page_8.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                button_page_8.setEnabled(True) #  비어있으면 버튼 활성화


    def closeEvent(self, event):
        """ 창 닫기 이벤트 → 모든 테이블 상태를 '비어있음(1)'으로 초기화 """
        self.db.reset_all_tables()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)  # PyQt 애플리케이션 실행
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_()) # 프로그램 종료 시 시스템 종료 
    
