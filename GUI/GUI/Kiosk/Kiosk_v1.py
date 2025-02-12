import sys  # 시스템 관련 기능 (예: 프로그램 종료)
from PyQt5.QtWidgets import * #PyQt5에서 GUI 관련 기능을 사용
from PyQt5.QtCore import QTimer, Qt, QTime # 타이머 및 시간 관련 기능
from PyQt5.QtGui import * # 그래픽 관련 기능
from PyQt5 import uic # UI 파일을 불러오는 기능
import pymysql # MySQL 데이터베이스와 연결하는 기능
import serial  # RFID 리더기와 시리얼 통신을 하기 위한 라이브러리
import struct  # 바이너리 데이터 패킹 및 언패킹 (RFID 데이터 송수신에 사용)
import time    # 딜레이 설정을 위한 라이브러리
from datetime import datetime, timedelta # 날짜와 시간을 다루는 기능
from PyQt5.QtCore import QDateTime  # 날짜 및 시간 관리



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


    # 사용자를 데이터베이스에 추가하는 기능
    def insert_user(self, phone, name, password, card_number, user_card_number):
        query = """
        INSERT INTO users (user_phone, user_name, password, card_number, user_card_number)
        VALUES (%s, %s, %s, %s, %s)
        """   # SQL INSERT 문 (회원 정보를 저장) 데이터베이스에 사용자의 전화번호, 이름, 비밀번호, 카드 번호를 저장하는  SQL 명령어
        try:
            self.cursor.execute(query, (phone, name, password, card_number, user_card_number)) # SQL 명령어 실행(사용자의 정보 저장)
            self.conn.commit() # 변경한 내용을 데이터베이스에 반영(저장)
            return self.cursor.lastrowid  # 새로 추가된 사용자의 ID(번호)를 반환
        except Exception as e: # 오류가 발생하면 실행되는 부분
            print(f"Error inserting user: {e}") # 예외 발생 시 오류 메시지 출력
            self.conn.rollback() # 오류가 발생하면 저장을 취소하고 원래 상태로 돌려놓음
            return None # 오류가 나면 아무 값도 반환하지 않음


    def sync_table_status(self, table_id, status): # 테이블(좌석)의 상태를 업데이트하는 기능(예: 사용 중, 비어 있음)
        query = "UPDATE pp_table SET table_status_id = %s WHERE table_id = %s"  # 테이블 상태가 바뀔 때, 데이터베이스에서도 똑같이 바꿔주는 기능, 테이블 상태를 변경하는 SQL 명령어
        try:
            self.cursor.execute(query, (status, table_id))  # 🔹 `self.db_manager.cursor` → `self.cursor` # SQL 명령어 실행 (해당 테이블의 상태를 업데이트)
            self.conn.commit()  # 🔹 `self.db_manager.conn` → `self.conn` # 변경한 내용을 저장 
            print(f"[DEBUG] 테이블 {table_id} → {status} 동기화 완료")

        except Exception as e:  # 오류가 발생하면 실행되는 부분
            print(f"Error syncing table status: {e}") # 오류 메시지 출력
            self.conn.rollback() # 오류 발생 시 변경 내용을 취소 



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
    def update_table_status_in_db(self, table_id, status_id): # 테이블 상태를 데이터베이스에서 업데이트 하는 기능
        query = "UPDATE pp_table SET table_status_id = %s WHERE table_id = %s"
        try:
            self.cursor.execute(query, (status_id, table_id)) # SQL 실행 (테이블 상태 업데이트)
            self.conn.commit() # 변경한 내용을 데이터베이스에 저장 
            print(f"[DEBUG] DB 업데이트: 테이블 {table_id} → 상태 {status_id}")   
        except Exception as e: # 오류가 발생하면 실행되는 부분
            print(f"Error updating table status: {e}") # 오류 메시지 출력
            self.conn.rollback() # 오류 발생 시 변경 내용을 취소

    def reset_all_tables(self): # 프로그램이 종료될 때 모든 테이블 상태를 '비어있음(1)으로 초기화 하는 기능
        query = "UPDATE pp_table SET table_status_id = 1"
        try:
            self.cursor.execute(query) # SQL 실행(모든 테이블 상태를 초기화)
            self.conn.commit() # 변경한 내용을 데이터베이스에 저장
            print("[DEBUG] 프로그램 종료: 모든 테이블 상태 초기화") 
        except Exception as e: # 오류가 발생하면 실행되는 부분 
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


    def set_status(self, status, duration=None): # 테이블 상태를 변경하고 필요한 경우 타이머를 설정하는 기능
        self.status_label.setText(status) # 상태에 따라 화면에 표시되는 글자 변경
        self.set_background_color(status) # 상태에 따라 배경색을 변경하는 함수 호출

        parent_window = QApplication.instance().activeWindow() # 현재 실행 중인 프로그램의 창을 가져옴
        if isinstance(parent_window, WindowClass): # 만약 현재 창이 `WindowClass`라면, 테이블 상태를 업데이트하는 함수 실행
            parent_window.update_table_labels()

        # 🔹 기본값 설정 (아직 상태가 정해지지 않았음)
        db_status_id = None  

        if status == "사용중" and duration: # 만약 테이블 상태가 "사용중"이고 시간이 주어졌다면
            self.start_time = QTime.currentTime() # 현재 시간을 저장
            self.end_time = self.start_time.addSecs(duration * 60) # 사용 시간이 끝나는 시간 설정
            self.remaining_time = duration * 60 # 남은 시간을 초 단위로 설정
            self.timer.start(1000) # 1초마다 시간을 업데이트하도록 타이머 시작
            self.update_time_labels() # 남은 시간을 화면에 업데이트
            db_status_id = 2  # 데이터베이스에서 "사용중" 상태를 나타내는 번호(2)
            print(f"[DEBUG] 테이블 {self.table_number}: 사용중 타이머 시작 ({duration}분)")

        elif status == "청소중": # 테이블 상태가 "청소중"이라면
            self.remaining_cleanup_time = 60 # 청소시간은 60초로 설정
            self.timer.start(1000) # 1초마다 시간을 업데이트하도록 타이머 시작
            db_status_id = 3  # 데이터베이스에서 "청소중" 상태를 나타내는 번호(3)
            print(f"[DEBUG] 테이블 {self.table_number}: 청소 타이머 시작 (60초)")

        else:  # 비어있음
            self.timer.stop() # 타이머 중지 (더 이상 시간 계산이 필요 없음)
            self.timer_label.setText("남은 시간: --:--")  #남은 시간을 초기화
            self.time_range_label.setText("--:-- ~ --:--") # 사용 시간 범위 초기화
            db_status_id = 1  # 데이터베이스에서 "비어있음" 상태를 나타내는 번호(1)
            print(f"[DEBUG] 테이블 {self.table_number}: 상태가 '비어있음'으로 변경됨 (db_status_id={db_status_id})")

        # **DB 상태 업데이트 (db_status_id가 설정된 경우에만 실행)**
        if db_status_id is not None: # 만약 현재 창이 존재하고 , 창에 'db'(데이터베이스 객체)가 있다면
            if parent_window and hasattr(parent_window, 'db'):
                #데이터베이스의 테이블 상태를 업데이트 
                parent_window.db.update_table_status_in_db(self.table_number, db_status_id)
                print(f"[DEBUG] 테이블 {self.table_number}: DB 상태 업데이트 완료 (db_status_id={db_status_id})")
            else:
                print(f"[DEBUG] 테이블 {self.table_number}: DB 업데이트 실패 (parent_window 없음)")


    # 청소 시간이 감소하면서 UI에 업데이트하는 함수
    def update_cleanup_time_labels(self):
        if self.remaining_cleanup_time > 0: #아직 청소 시간이 남아 있다면
            self.remaining_cleanup_time -= 1 # 남은 시간을 1초 줄임
            minutes, seconds = divmod(self.remaining_cleanup_time, 60) # 분과 초로 변환
            self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}") # 화면에 남은 시간 표시
            print(f"[DEBUG] 청소중 남은 시간: {minutes:02}:{seconds:02}") 

            # 메인 윈도우(프로그램의 큰 창) 가져오기
            parent_window = QApplication.instance().activeWindow()
            if isinstance(parent_window, WindowClass): # 만약 메인 윈도우가 'WindowClass`라면, 테이블 상태를 다시 업데이트 
                parent_window.update_table_labels()
        else: # 만약 청소 시간이 다 되었다면
            self.cleanup_finished()  # 🔹 테이블 상태를 '비어있음'으로 변경하는 함수 실행


    # 테이블 상태에 따라 배경색을 변경하는 함수
    def set_background_color(self, status): 
        palette = self.palette() # 현재 팔레트 가져오기
        if status == "사용중":
            palette.setColor(QPalette.Window, QColor(255, 0, 0, 128))  # 빨간색 (반투명)
        elif status == "청소중":
            palette.setColor(QPalette.Window, QColor(255, 165, 0, 128))  # 주황색 (반투명)
        else:
            palette.setColor(QPalette.Window, QColor(0, 0, 255, 128))  # 파랑색 (반투명)
        self.setAutoFillBackground(True) # 배경색을 적용할 수 있도록 설정
        self.setPalette(palette) # 변경한 색상을 적ㅇㅇ

    # 시작 시간과 종료 시간을 UI에 업데이트 하는 함수 
    def update_time_labels(self):
        if self.start_time and self.end_time: # 시작 시간과 종료 시간이 설정되었다면
            self.time_range_label.setText(
                f"{self.start_time.toString('HH:mm')} ~ {self.end_time.toString('HH:mm')}"
            ) # "시작시간 ~ 종료 시간" 형태로 표시

    # 남은 시간을 1초씩 감소 시키고 UI에 업데이트하는 함수 
    def update_remaining_time(self):
        if self.status_label.text() == "사용중":
            if self.remaining_time > 0: # 남은 시간이 0보다 크면
                self.remaining_time -= 1 # 남은 시간 감소 
                minutes, seconds = divmod(self.remaining_time, 60) # 분과 초로 변환 
                self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}") # 화면에 표시
                print(f"[DEBUG] 남은 시간: {minutes:02}:{seconds:02}")

                # 메인 윈도우에서 UI 업데이트
                # parent_window = self.parentWidget()
                # if parent_window is None:
                parent_window = QApplication.instance().activeWindow()  # 메인 윈도우 가져오기
                if isinstance(parent_window, WindowClass):# 메인 윈도우가 있다면
                    parent_window.update_table_labels() # 테이블 상태 업데이트
            else:
                self.set_status("청소중") # 시간이 다 되면 "청소중" 상태로 변경
                parent_window = QApplication.instance().activeWindow() 
                if parent_window and hasattr(parent_window, 'db'): # 데티터베이스가 있으면
                    parent_window.db.update_table_status_in_db(self.table_number, 3)  # 테이블 상태를 '청소중(3)'으로 변경 

        elif self.status_label.text() == "청소중": # 테이블 상태가 "청소중"이면 청소 시간을 업데이트 
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

        # 앱이 시작될 때 홈 버튼 숨기기
        self.reset_btn.setVisible(False)

        self.phone_input.setValidator(QIntValidator())  # 전화번호 숫자만 입력 가능
        self.password_input.setValidator(QIntValidator())  # 비밀번호 숫자만 입력 가능
        self.payment_card_input.setValidator(QIntValidator())  # 결제 카드번호 숫자만 입력 가능


        #  테이블과 UID를 저장할 딕셔너리 초기화 (예약 관리용)
        self.table_user_mapping = {}  # {table_number: UID} 형태로 매핑

        self.entry_time = {} # 입실한 시간 저장 {table_number: timestamp}
        
        self.tag_delay = 7  #  5초 동안 재태깅 방지 (필요하면 조절 가능)


        #  기타 필요한 변수 초기화
        self.selected_table = None
        self.selected_time = None

        self.last_tag_time = {}  # RFID 태깅 시간 저장 (각 테이블별)
        #  퇴실한 사용자 목록 저장 (퇴실하면 여기에 추가)
        self.checked_out_users = set()

        #  타이머를 통해 모든 페이지에서 RFID 태깅 감지
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_rfid_card)  # 태깅 감지
        self.timer.start(500)  # 0.5초 간격으로 RFID 데이터 확인


        self.setWindowTitle("Kiosk") # 창 제목 설정
   
    

        # 시스템 초기화 버튼 클릭 시 'reset_system' 메서드 실행
        self.reset_btn.clicked.connect(self.reset_system)

        # 현재 인식된 RFID UID 저장 변수
        self.current_uid = None


        # MySQL 연결
        self.db = DatabaseManager()

        # 회원가입 버튼 클릭 시 register_user 실행
        self.signup_complete_btn.clicked.connect(self.register_user)


        # RFID 리더기 2개 연결
        self.rfid1 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=1)
        self.rfid2 = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=1)


        #RFID 카드 체크 타이머 설정
        self.timer = QTimer()
        
        self.timer.timeout.connect(self.check_rfid_card) # 타이머가 만료가 되면  `check_rfid_card` 실행
        self.timer.start() # 타이머 시작 

        # 페이지 변경 감지 및 처리
        self.stackedWidget.currentChanged.connect(self.handle_page_change)

        # 선택된 이용 시간, 테이블, 
        self.selected_time = None
        self.selected_table = None
  
        self.check_in_status = {}

        # 테이블 상태 위젯 저장용 딕셔너리 (1~4번 테이블)
        self.table_widgets = {}
        self.initialize_table_status_widgets() # 테이블 상태 초기화(모두 "비어있음")


        #  RFID 태깅 후 최소 5초 동안 퇴실 방지 (기본값: 0)
        self.last_check_in_time = {}  # {table_id: datetime}

        # 각 버튼을 클릭하면 해당 페이지로 이동하도록 설정
        self.use_btn.clicked.connect(self.go_to_page2) # 실시간 테이블 확인 (Page2)
        self.signup_btn.clicked.connect(self.go_to_page3) # 회원가입 페이지 (Page3)
        self.prev_btn.clicked.connect(self.go_to_page1) # 처음 화면 (Page1)
        self.signup_btn_2.clicked.connect(self.go_to_page3) # 회원가입 페이지 (Page3)
        self.signup_complete_btn.clicked.connect(self.go_to_page4) # 회원가입 완료 (Page4)
        self.prev_btn_2.clicked.connect(self.go_to_page1) # 처음 화면 (Page1)
        self.prev_btn_3.clicked.connect(self.go_to_page1)  # 처음 화면 (Page1)
        self.realtime_table_btn_2.clicked.connect(self.go_to_page6)  # 실시간 테이블 정보 (Page6)
        self.prev_btn_4.clicked.connect(self.go_to_page5) # 예약 화면 (Page5)
        self.reserve_btn.clicked.connect(self.go_to_page7) # 예약 확인 페이지 (Page7)
        self.prev_btn_5.clicked.connect(self.go_to_page5)  # 예약 화면 (Page5)
        # self.reset_btn.clicked.connect(self.go_to_page1) # 초기화 후 처음 화면 (Page1)
        self.prev_btn_6.clicked.connect(self.go_to_page7) # 예약 확인 페이지 (Page7)
        self.payment_btn.clicked.connect(self.go_to_page9) # 결제 완료 화면 (Page9)
        self.realtime_table_btn.clicked.connect(self.go_to_page10) # 실시간 테이블 화면 (Page10)
        self.prev_btn_7.clicked.connect(self.go_to_page1) # 처음 화면 (Page1)

        # 이동 시간 선택 버튼 설정 (버튼 클릭시 'select_time' 메서드 실행)
        self.pushButton_11.clicked.connect(lambda: self.select_time(1, 7000)) # 30분 7000원
        self.pushButton_12.clicked.connect(lambda: self.select_time(60, 13000)) # 60분 13000원
        
        # 테이블 선택 버튼 설정 (버튼 클릭시 'select_table' 메서드 실행)
        self.pushButton_14.clicked.connect(lambda: self.select_table(1))
        self.pushButton_15.clicked.connect(lambda: self.select_table(2))
        self.pushButton_16.clicked.connect(lambda: self.select_table(3))
        self.pushButton_17.clicked.connect(lambda: self.select_table(4))
        
        # 확인 버튼 클릭 시 'self.confirm_selection' 실행
        self.confirm_btn.clicked.connect(self.confirm_selection)
        
        # 결제 버튼 클릭 시 `handle_payment` 실행
        self.payment_btn.clicked.connect(self.handle_payment)


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

        #  RFID1과 RFID2에 'GS' 명령을 보내고 데이터를 요청
        for rfid_reader in [self.rfid1, self.rfid2]:
            print(f"[DEBUG] {rfid_reader.port}에 'GS' 명령 전송")
            rfid_reader.write(b'GS\n')  # RFID 리더기에 UID 요청
            time.sleep(0.1)  # 데이터를 받을 시간 확보

        print(f"[DEBUG] RFID1 대기 중 데이터 개수: {self.rfid1.in_waiting}")
        print(f"[DEBUG] RFID2 대기 중 데이터 개수: {self.rfid2.in_waiting}")

        current_page = self.stackedWidget.currentWidget().objectName()

        #  RFID1 (회원 확인 & 회원가입) → page_2, page_3에서만 실행
        if self.rfid1.in_waiting > 0 and current_page in ["page_2", "page_3"]:
            res = self.rfid1.read_until(b'\n')
            print(f"[DEBUG] RFID1 수신 데이터: {res}")

            if res.startswith(b'GS') and len(res) >= 7:
                uid = res[3:7].hex().upper()
                print(f"[DEBUG] RFID1 감지됨: {uid}")
                self.handle_rfid1(uid)  

        #  RFID2 (입실/퇴실) → 모든 페이지에서 실행
        if self.rfid2.in_waiting > 0:
            res = self.rfid2.read_until(b'\n')
            print(f"[DEBUG] RFID2 수신 데이터: {res}")

            if res.startswith(b'GS') and len(res) >= 7:
                uid = res[3:7].hex().upper()
                print(f"[DEBUG] RFID2 감지됨: {uid}")
                self.handle_rfid2(uid)




    def handle_rfid1(self, uid):
        """RFID1: 기존 회원 확인 및 회원가입 관련 기능"""
        self.card_input.setText(uid)  # UI에 UID 표시
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
        """RFID 태깅으로 테이블 입실/퇴실 처리 (퇴실 후 5초 동안 RFID 태깅 무시)"""
        print(f"[DEBUG] RFID2 태그 감지: {uid}")

        current_time = QDateTime.currentDateTime()

        # 🔹 **퇴실 후 5초 이내 태깅 방지**
        if hasattr(self, "last_tag_time") and uid in self.last_tag_time:
            elapsed_time = self.last_tag_time[uid].secsTo(current_time)
            if elapsed_time < 5:
                print(f"[DEBUG] {uid} 퇴실 후 {elapsed_time}초 경과 → 태깅 무시")
                return  # 5초 이내면 태깅 무시

        # **태깅 시간 기록 (최신 태깅 시간 저장)**
        if not hasattr(self, "last_tag_time"):
            self.last_tag_time = {}

        self.last_tag_time[uid] = current_time

        # **DB에서 최신 데이터를 가져오도록 커넥션을 갱신**
        self.db.conn.ping(reconnect=True)  # 강제로 DB 연결을 새로고침하여 최신 데이터 반영
        self.db.conn.commit()

        # **self.check_in_status가 없으면 초기화 (안전한 실행 보장)**
        if not hasattr(self, "check_in_status"):
            self.check_in_status = {}

        # **RFID 태깅할 때마다 항상 최신 예약 정보를 DB에서 조회**
        query = """
        SELECT table_id, TIMESTAMPDIFF(MINUTE, NOW(), end_time) 
        FROM reservations 
        WHERE user_id = (SELECT user_id FROM users WHERE card_number = %s LIMIT 1)
        AND start_time <= NOW() AND end_time >= NOW()
        """
        self.db.cursor.execute(query, (uid,))
        result = self.db.cursor.fetchone()
        print(f"[DEBUG] 최신 예약 조회 결과: {result}")

        if result:
            table_id = result[0]
            remaining_time = result[1]
            print(f"[DEBUG] 최신 예약된 테이블 {table_id}, 남은 시간: {remaining_time}분")

            self.selected_table = table_id
        else:
            print(f"[DEBUG] RFID {uid}: 예약된 테이블 없음")
            QMessageBox.warning(self, "입실 오류", "예약된 테이블이 없습니다. 새로운 예약이 필요합니다.")
            return  # 🚫 기존 예약이 없으면 입실 불가

        widget = self.table_widgets.get(self.selected_table)

        # ✅ **입실 후 다시 태깅하면 퇴실되도록 수정**
        if self.check_in_status.get(self.selected_table, False):
            print(f"[DEBUG] UID {uid} → 테이블 {self.selected_table} 퇴실 처리")

            self.update_table_status(self.selected_table, "비어있음")
            self.db.update_table_status_in_db(self.selected_table, 1)  # '비어있음'으로 DB 업데이트

            self.db.sync_table_status(self.selected_table, 1)

            # ✅ 퇴실 완료 후 입실 상태 초기화
            self.check_in_status[self.selected_table] = False
            print(f"[DEBUG] 퇴실 상태 확인 (퇴실 후): {self.check_in_status}")

            # **예약 정보 이동 (past_reservations로 백업)**
            move_query = """
            INSERT INTO past_reservations (user_id, table_id, start_time, end_time, status, price)
            SELECT user_id, table_id, start_time, end_time, '완료됨', price FROM reservations WHERE table_id = %s
            """
            try:
                self.db.cursor.execute(move_query, (self.selected_table,))
                self.db.conn.commit()
                print(f"[DEBUG] 예약 데이터 이동 완료: 테이블 {self.selected_table}")
            except Exception as e:
                print(f"Error moving reservation: {e}")
                self.db.conn.rollback()
                return

            # **이동 후 기존 reservations 데이터 삭제**
            delete_query = "DELETE FROM reservations WHERE table_id = %s"
            try:
                self.db.cursor.execute(delete_query, (self.selected_table,))
                self.db.conn.commit()
                print(f"[DEBUG] 기존 예약 삭제 완료: 테이블 {self.selected_table}")
            except Exception as e:
                print(f"Error deleting reservation: {e}")
                self.db.conn.rollback()

            # ✅ **UI 강제 갱신 추가 (퇴실 반영)**
            QApplication.processEvents()  # UI 즉시 반영

            QMessageBox.information(self, "퇴실 완료", f"테이블 {self.selected_table} 퇴실 완료. 예약 정보가 기록되었습니다.")

            # 🚫 **퇴실 후 같은 UID로 다시 태깅해도 입실 불가 (새로운 예약 필요)**
            self.selected_table = None  # ✅ 퇴실 후 테이블 선택 해제
            return  # 🚫 퇴실 처리 후 종료

        # **입실 처리 (비어있음 → 사용중)**
        if widget and widget.status_label.text() == "비어있음":
            print(f"[DEBUG] UID {uid} → 테이블 {self.selected_table} 입실 처리")

            self.update_table_status(self.selected_table, "사용중", remaining_time)
            self.db.update_table_status_in_db(self.selected_table, 2)  # '사용중'으로 DB 업데이트
            self.last_check_in_time[self.selected_table] = current_time

            # ✅ 입실 체크 상태 업데이트 (입실 중)
            self.check_in_status[self.selected_table] = True

            self.db.sync_table_status(self.selected_table, 2)

            QMessageBox.information(self, "입실 완료", f"테이블 {self.selected_table} 사용 시작. 남은 시간: {remaining_time}분")




    def handle_rfid_check_out(self, user_id, table_id):
        # 현재 활성화된 예약 정보 가져오기
        query = """
        SELECT id, start_time, end_time, price FROM reservations
        WHERE user_id = %s AND table_id = %s
        """
        self.db_manager.cursor.execute(query, (user_id, table_id))
        reservation = self.db_manager.cursor.fetchone()

        if reservation:
            reservation_id, start_time, end_time, price = reservation

            # 완료된 예약을 past_reservations 테이블로 이동
            query = """
            INSERT INTO past_reservations (user_id, table_id, start_time, end_time, price, status)
            VALUES (%s, %s, %s, %s, %s, %s)
            """
            try:
                self.db_manager.cursor.execute(query, (user_id, table_id, start_time, end_time, price, '완료됨'))
                self.db_manager.conn.commit()
                print(f"[DEBUG] past_reservations로 이동 완료: reservation_id={reservation_id}")

                # 기존 reservations에서 해당 예약 삭제 (체크아웃 완료)
                query = "DELETE FROM reservations WHERE id = %s"
                self.db_manager.cursor.execute(query, (reservation_id,))
                self.db_manager.conn.commit()
                print(f"[DEBUG] reservations에서 삭제 완료: reservation_id={reservation_id}")

            except Exception as e:
                print(f"Error moving reservation to past_reservations: {e}")
                self.db_manager.conn.rollback()





    def handle_page_change(self, index):
        """페이지 변경 시 RFID 감지 활성화/비활성화 설정"""
        current_page = self.stackedWidget.currentWidget().objectName()
        
        # page1(메인 화면)에서는 reset_btn(홈 버튼) 숨김
        if current_page == "page":
            self.reset_btn.setVisible(False)
        else:
            self.reset_btn.setVisible(True)

        #  handle_rfid1(uid)는 page_2, page_3에서만 감지
        if current_page in ["page_2", "page_3"]:
            print(f"[DEBUG] {current_page}: RFID1 감지 활성화 (회원 확인)")
        else:
            print(f"[DEBUG] {current_page}: RFID1 감지 중지")

        #  handle_rfid2(uid)는 모든 페이지에서 감지 가능 → RFID 감지는 항상 실행
        print(f"[DEBUG] {current_page}: RFID2 감지 유지 (입실/퇴실 가능)")

        #  타이머 항상 실행 (RFID2 감지를 위해)
        QApplication.processEvents()
        self.timer.start()





    def register_user(self):#회원가입 버튼 클릭 시 기존 회원 정보 삭제 후 새 정보 등록
        phone = self.phone_input.text().strip()
        name = self.name_input.text().strip()
        password = self.password_input.text().strip()
        card_number = self.card_input.text().strip()  # RFID 카드 UID
        user_card_number = self.payment_card_input.text().strip()  # 사용자가 입력한 결제 카드

        print(f"[DEBUG] 회원가입 버튼 클릭됨")
        print(f"[DEBUG] 입력된 정보: {phone}, {name}, {password}, {card_number}, {user_card_number}")

        if not phone or not name or not password or not card_number or not user_card_number:
            QMessageBox.warning(self, "오류", "모든 필드를 입력하세요!")
            return

        existing_user = self.db.get_user_by_card(card_number)

        if existing_user:
            # 기존 회원 정보 삭제 후 새 정보 저장
            self.db.delete_user_by_card(card_number)
            print(f"[DEBUG] 기존 회원 정보 삭제됨: {card_number}")

        # 새 회원 등록
        user_id = self.db.insert_user(phone, name, password, card_number, user_card_number)
        if user_id:
            QMessageBox.information(self, "회원가입 완료", f"회원가입이 완료되었습니다! 사용자 ID: {user_id}")
            print(f"[DEBUG] 회원가입 성공, page_5로 이동")
            self.stackedWidget.setCurrentIndex(4)  #  회원가입 완료 후 page_5로 이동
        else:
            QMessageBox.critical(self, "오류", "회원가입에 실패했습니다.")
            print(f"[DEBUG] 회원가입 실패")




    def reset_system(self):
        """초기화 버튼 클릭 시 시스템 리셋"""
        print("[DEBUG] 초기화 버튼 클릭됨 → 모든 입력 필드 초기화 및 RFID 감지 리셋")

        # 입력 필드 초기화
        self.phone_input.clear()
        self.name_input.clear()
        self.password_input.clear()
        self.card_input.clear()

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


    def handle_payment(self):
        if self.selected_time and self.selected_table:
            # 현재 시간 기준 예약 시작 시간 설정
            start_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            end_time = (datetime.now() + timedelta(minutes=self.selected_time)).strftime('%Y-%m-%d %H:%M:%S')

            # RFID 카드 번호 가져오기
            card_number = self.card_input.text().strip()
            if not card_number:
                QMessageBox.warning(self, "결제 오류", "RFID 카드가 인식되지 않았습니다.")
                return

            # 사용자 정보 조회
            user_info = self.db.get_user_by_card(card_number)
            if not user_info:
                QMessageBox.warning(self, "결제 오류", "해당 카드로 등록된 사용자가 없습니다.")
                return

            user_id = user_info[0]  # user_id 가져오기

            # 새로운 결제 후 퇴실 제한 해제
            if card_number in self.checked_out_users:
                self.checked_out_users.remove(card_number)
                print(f"[DEBUG] {card_number} 퇴실 제한 해제됨 (새로운 결제 완료)")

            # SQL 실행 전 확인
            print(f"[DEBUG] 실행할 SQL: INSERT INTO reservations (user_id, table_id, start_time, end_time, status) VALUES ({user_id}, {self.selected_table}, '{start_time}', '{end_time}', '예약됨')")

            # 예약 정보 저장
            query = """
            INSERT INTO reservations (user_id, table_id, start_time, end_time, status, price)
            VALUES (%s, %s, %s, %s, '예약됨', %s)
            """
            try:
                self.db.cursor.execute(query, (user_id, self.selected_table, start_time, end_time, self.total_price))
                self.db.conn.commit()
                QMessageBox.information(self, "예약 완료", "예약이 정상적으로 완료되었습니다.")
                print(f"[DEBUG] 예약 완료: 사용자 {user_id}, 테이블 {self.selected_table}, 시간 {start_time} ~ {end_time}, 요금 {self.total_price}원")
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
        #  duration이 None이면 최소 1분 이상 설정 (남은 시간이 0분으로 표시되지 않도록)
            if duration is None or duration <= 0:
                duration = 1  # 기본 최소 1분 적용

            self.table_widgets[table_number].set_status(status, duration)
            print(f"[DEBUG] 테이블 {table_number}: {status} 상태로 변경 (타이머: {duration}분)")

        #  UI 강제 업데이트
        self.update_table_labels()

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

            #  UI 업데이트 즉시 반영
            QApplication.processEvents()

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
    
