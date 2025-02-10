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


from_class =uic.loadUiType("/home/jieun/GUI/UserApp/app.ui")[0]

class DatabaseManager:
    def __init__(self):
        self.conn = pymysql.connect(
            host="localhost",
            user="lento",
            password="0819",
            database="test2",
            charset="utf8mb4"
        )
        self.cursor = self.conn.cursor()


    # 테이블 상태 변경 시, 다른 프로그램과 동기화
    def sync_table_status(self, table_id, status):
        query = "UPDATE pp_table SET table_status_id = %s WHERE table_id = %s"  # SQL 명령어: 테이블 상태를 변경하는 쿼리 (예: 사용 중 → 비어있음)
        try:
            self.cursor.execute(query, (status, table_id))  #  데이터베이스에서 위의 SQL 명령을 실행 (테이블 상태 업데이트)
            self.conn.commit()   # 변경된 내용을 데이터베이스에 저장 (실제로 반영)
            print(f"[DEBUG] 테이블 {table_id} → {status} 동기화 완료")

        except Exception as e:  # 만약 오류가 발생하면 실행되는 부분
            print(f"Error syncing table status: {e}")
            self.conn.rollback()  # 데이터베이스에 적용된 변경 사항을 취소 (롤백)



# 테이블 상태를 보여주는 작은 창 (예: 사용 중, 비어있음, 청소 중)
class TableStatusWidget(QWidget):
    def __init__(self, table_number, status):
        super().__init__()
        self.table_number = table_number # 테이블 번호
        self.setFixedSize(200, 100) # 창 크기 고정

        # 창에 넣을 여러 개의 글씨
        self.layout = QVBoxLayout(self)

         # 테이블 번호를 표시하는 라벨 (예: "테이블 1")
        self.table_label = QLabel(f"테이블 {table_number}") # "테이블1" 같은 글씨
        self.table_label.setAlignment(Qt.AlignCenter) # 가운데 정렬
        self.layout.addWidget(self.table_label) # 창에 추가 

        
        self.status_label = QLabel(status) # 테이블 상태 ("사용중", "비어있음" 등)
        self.status_label.setAlignment(Qt.AlignCenter) 
        self.layout.addWidget(self.status_label) 



        self.timer_label = QLabel("남은 시간: --:--") # 시간이 얼마나 남았는지 표시
        self.timer_label.setAlignment(Qt.AlignCenter) 
        self.layout.addWidget(self.timer_label)    


        
        self.time_range_label = QLabel("--:-- ~ --:--") # 언제 시작해서 언제 끝나는지 표시 
        self.time_range_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.time_range_label)  



        self.timer = QTimer(self) # 시간 줄어들게 하는 타이머 만듦 
        self.timer.timeout.connect(self.update_remaining_time) # 1초마다 남은 시간 업데이트

        self.cleanup_timer = QTimer(self)  # 테이블을 청소하는 시간을 정하는 타이머
        self.cleanup_timer.setSingleShot(True)  # 한 번만 실행되도록 설정
        self.cleanup_timer.timeout.connect(self.cleanup_finished) # 청소가 끝나면 실행되는 함수 연결



        # 내부 변수 (나중에 시간 관련 정보를 저장할 공간)
        self.remaining_time = 0 # 남은 시간
        self.start_time = None # 시작 시간
        self.end_time = None  # 끝나는 시간       

        
         # 현재 테이블 상태에 따라 배경색을 설정
        self.set_background_color(status) 



    # 테이블 상태를 변경하고 타이머를 설정하는 함수
    def set_status(self, status, duration=None):
        self.status_label.setText(status)  # 상태 라벨을 변경 (예: 사용중 → 비어있음)
        self.set_background_color(status)  # 상태에 맞게 배경색 변경

        # 현재 활성화된 창을 가져옴 (부모 창 찾기)
        parent_window = QApplication.instance().activeWindow()
        if isinstance(parent_window, WindowClass):
            parent_window.update_table_labels()  # 테이블 상태 업데이트

        # 기본값 설정 (미리 정의)
        db_status_id = None  

        if status == "사용중" and duration:
            self.start_time = QTime.currentTime()  # 시작 시간을 현재 시간으로 설정
            self.end_time = self.start_time.addSecs(duration * 60) # 종료 시간을 시작 시간 + duration(분) 후로 설정
            self.remaining_time = duration * 60   # 남은 시간을 duration(분)을 초 단위로 변환해서 저장

            # 남은 시간을 "분:초" 형식으로 변환해서 화면에 표시
            minutes, seconds = divmod(self.remaining_time, 60)
            self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}")  
            self.update_time_labels() # 시작~종료 시간 표시

            # 화면 업데이트 (즉시 반영)
            QApplication.processEvents()

            # 1초마다 남은 시간을 줄이도록 타이머 시작
            self.timer.start(1000)
            self.update_time_labels()
            db_status_id = 2   # '사용중' 상태는 DB에서 2로 저장
            print(f"[DEBUG] 테이블 {self.table_number}: 사용중 타이머 시작 ({duration}분)")

        elif status == "청소중": 
            self.remaining_cleanup_time = 60   # 청소 시간은 60초 (1분)
            self.timer.start(1000)  # 1초마다 실행되도록 설정
            db_status_id = 3   # '청소중' 상태는 DB에서 3으로 저장
            print(f"[DEBUG] 테이블 {self.table_number}: 청소 타이머 시작 (60초)")

        else:  # 비어있음
            self.timer.stop()  # 타이머 중지
            self.timer_label.setText("남은 시간: --:--")  # 남은 시간 초기화
            self.time_range_label.setText("--:-- ~ --:--")  # 시간 범위 초기화
            db_status_id = 1  # '비어있음'이면 1
            print(f"[DEBUG] 테이블 {self.table_number}: 상태가 '비어있음'으로 변경됨 (db_status_id={db_status_id})")

        #  **DB 상태 업데이트 (db_status_id가 설정된 경우에만 실행)**
        if db_status_id is not None:
            if parent_window and hasattr(parent_window, 'db'):  # 데이터베이스에서 테이블 상태 업데이트 실행
                parent_window.db.update_table_status_in_db(self.table_number, db_status_id)
                print(f"[DEBUG] 테이블 {self.table_number}: DB 상태 업데이트 완료 (db_status_id={db_status_id})")
            else:
                print(f"[DEBUG] 테이블 {self.table_number}: DB 업데이트 실패 (parent_window 없음)")


    # 청소 중 남은 시간을 감소시키고 화면(UI)에 반영하는 함수
    def update_cleanup_time_labels(self):

        if self.remaining_cleanup_time > 0:  # 아직 청소 시간이 남아 있다면
            self.remaining_cleanup_time -= 1  # 남은 시간을 1초 줄이기
            minutes, seconds = divmod(self.remaining_cleanup_time, 60)  # 남은 시간을 '분:초'로 변환
            self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}") # 화면에 표시
            print(f"[DEBUG] 청소중 남은 시간: {minutes:02}:{seconds:02}") 

            # 현재 실행 중인 메인 윈도우를 가져옴
            parent_window = QApplication.instance().activeWindow()  
            if isinstance(parent_window, WindowClass):  # 메인 윈도우가 맞다면
                parent_window.update_table_labels()   # 테이블 상태 업데이트
        else:  # 만약 남은 시간이 0이 되면
            self.cleanup_finished()  # 청소 완료 함수 실행 (테이블을 '비어있음'으로 변경)




    def set_background_color(self, status): # 테이블 상태에 따라 배경색을 바꾸는 함수 
        palette = self.palette() # 현재 창의 색상 설정을 가져오기
        if status == "사용중":
            palette.setColor(QPalette.Window, QColor(255, 0, 0, 128))  # 빨강색(사용중)
        elif status == "청소중":
            palette.setColor(QPalette.Window, QColor(255, 165, 0, 128))  # 주황색(청소중)
        else:  # "비어있음"
            palette.setColor(QPalette.Window, QColor(0, 0, 255, 128))  # 파랑색(비어있음)
        self.setAutoFillBackground(True) # 배경색 적용
        self.setPalette(palette)  # 색상 변경 적용

    def update_time_labels(self): # 테이블 이용 시간(시작~종료)을 화면에 표시하는 함수 
        if self.start_time and self.end_time: # 시작 시간과 종료 시간이 있을 때만 실행
            self.time_range_label.setText(
                f"{self.start_time.toString('HH:mm')} ~ {self.end_time.toString('HH:mm')}"
            ) # "시작시간 ~ 종료시간" 형태로 표시



    def update_remaining_time(self): # 테이블이 '사용중' 또는 '청소중'일 때 남은 시간을 업데이트하는 함수
        if self.status_label.text() == "사용중":   # 상태가 '사용중'이면
            if self.remaining_time > 0:  # 남은 시간이 있으면
                self.remaining_time -= 1  # 1초 감소
                minutes, seconds = divmod(self.remaining_time, 60)  # 분과 초로 나누기
                self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}") # 화면에 표시
                print(f"[DEBUG] 사용중 남은 시간: {minutes:02}:{seconds:02}")
               
                QApplication.processEvents() # UI 강제 업데이트 (화면 즉시 반영)

                # 메인 윈도우를 가져와서 테이블 상태 업데이트
                parent_window = QApplication.instance().activeWindow()
                if isinstance(parent_window, WindowClass):
                    parent_window.update_table_labels()

            else:  # 시간이 다 되면
                print(f"[DEBUG] 테이블 {self.table_number}: 사용시간 종료 → 청소중으로 변경")
                self.set_status("청소중")  # 테이블 상태를 '청소중'으로 변경

        elif self.status_label.text() == "청소중":  # 상태가 '청소중'이면
            if self.remaining_cleanup_time > 0:  # 청소 시간이 남아 있으면
                self.update_cleanup_time_labels()  # 청소 시간 업데이트 함수 실행
            else:   # 청소 시간이 끝났으면
                self.cleanup_finished()  #  테이블을 '비어있음'으로 변경

    def cleanup_finished(self):  # 청소 시간이 끝나면 실행되는 함수
            self.timer.stop() # 타이머 정지
            self.set_status("비어있음") # 테이블을 "비어있음"으로 변경
            print("[DEBUG] 청소 완료, 상태: 비어있음")  # 디버깅 메시지


    def handle_timer_finished(self): # 테이블 이용 시간이 끝나면 자동으로 '청소중' 상태로 변경하는 함수
        print(f"[DEBUG] 테이블 {self.table_number} → 청소중 시작")
        self.set_status("청소중") # 테이블 상태를 '청소중'으로 변경

        # 🔹 청소 타이머 시작 (1분 후 '비어있음'으로 변경)
        self.cleanup_timer.start(60000)  # 60초 후 `cleanup_finished` 실행
        print(f"[DEBUG] 테이블 {self.table_number} 청소 타이머 시작 (60초)")



class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self) # UI 불러오기

        self.setWindowTitle("App")
        self.db_manager = DatabaseManager()  # 데이터베이스 연결 객체 생성
        self.previous_table_status = {}  # 이전 테이블 상태 저장
        self.check_in_status = {} #입실 여부를 저장하는 딕셔너리 초기화


        # 테이블 상태 감지용 타이머 설정 (1초마다 체크)
        self.status_check_timer = QTimer(self)
        self.status_check_timer.timeout.connect(self.check_table_status)
        self.status_check_timer.start(1000)  # 1초마다 실행

        # # 예약 ID를 저장할 속성 초기화
        # self.selected_reservation_id = None


        # # 결제 버튼 클릭 시 order_payment() 실행
        # self.order_payment_btn.clicked.connect(self.order_payment)


        # 선택된 정보 저장용 변수
        self.selected_time = None # 예약된 시간(30분/60분)
        self.selected_table = None # 예약한 테이블 번호
        self.total_price = 0 # 결제 금액


        # 테이블 상태를 저장할 딕셔너리(테이블 1~4)
        self.table_widgets = {}
        self.initialize_table_status_widgets()


        # 상품 목록 (버튼과 가격)
        self.products = {
            "사이다": 1000,
            "콜라": 1000,
            "환타": 1000,
            "커피": 1000,
            "초코파이" :500,
            "에너지바": 1000,
            "단백질바" : 1000,
            "다크초콜릿" : 1000,
        }


        # 버튼 이벤트 연결
        self.realtime_table_btn.clicked.connect(self.go_to_page2)  # 실시간 테이블 확인
        self.reserve_btn.clicked.connect(self.go_to_page3)  # 예약 페이지 읻동
        self.reset_btn.clicked.connect(self.go_to_page1)  # 처음 페이지 이동
        self.confirm_btn.clicked.connect(self.validate_login) # 로그인 확인
        self.prev_btn_2.clicked.connect(self.go_to_page1) # 이전 페이지
        self.prev_btn.clicked.connect(self.go_to_page5) # 이전 페이지
        self.prev_btn_3.clicked.connect(self.go_to_page3) # 예약 페이지로 이동
        self.payment_btn.clicked.connect(self.go_to_page7) # 결제 완료 메시지
        self.snack_order_btn.clicked.connect(self.go_to_page8)  # 음료/간식 주문 버튼 클릭 시 페이지 이동
        self.clean_service_btn.clicked.connect(self.go_to_page10) # 전체 청소, 부분 청소
        self.order_payment_btn.clicked.connect(self.go_to_page9)  # 음료/간식결제 버튼
        self.clean_all_btn.clicked.connect(self.go_to_page11) # 전체 청소
        self.clean_partial_btn.clicked.connect(self.go_to_page12) # 부분 청소 

        self.stop_all_btn.clicked.connect(self.go_to_page10) # 전체 청소 종료 버튼
        self.stop_partial_btn.clicked.connect(self.go_to_page10) # 부분 청소 종료 버튼





        self.pushButton_11.clicked.connect(lambda: self.select_time(1, 7000))  # 30분 선택
        self.pushButton_12.clicked.connect(lambda: self.select_time(60, 13000))  # 1시간 선택
        self.pushButton_14.clicked.connect(lambda: self.select_table(1))  # 테이블 1 선택
        self.pushButton_15.clicked.connect(lambda: self.select_table(2))  # 테이블 2 선택
        self.pushButton_16.clicked.connect(lambda: self.select_table(3))  # 테이블 3 선택
        self.pushButton_17.clicked.connect(lambda: self.select_table(4))  # 테이블 4 선택
        self.confirm_btn_2.clicked.connect(self.confirm_selection)  # 예약 확인 버튼       
        self.payment_btn.clicked.connect(self.handle_payment)  # 결제 버튼



        self.logout_btn.clicked.connect(self.logout)  # 로그아웃 버튼 연결

        # 처음에는 로그아웃 버튼을 비활성화 상태로 설정
        self.logout_btn.setEnabled(False)

        # 상품 주문 결제 완료
        self.order_payment_btn.clicked.connect(self.order_payment)


        # 상품 주문 버튼과 `add_to_order()` 함수 연결 (누르면 해당 상품이 장바구니에 추가됨)
        self.cider_btn.clicked.connect(lambda: self.add_to_order("사이다"))
        self.cola_btn.clicked.connect(lambda: self.add_to_order("콜라"))
        self.fanta_btn.clicked.connect(lambda: self.add_to_order("환타"))  
        self.coffee_btn.clicked.connect(lambda: self.add_to_order("커피"))  
        self.choco_btn.clicked.connect(lambda: self.add_to_order("초코파이"))
        self.energy_btn.clicked.connect(lambda: self.add_to_order("에너지바"))
        self.protein_btn.clicked.connect(lambda: self.add_to_order("단백질바"))  
        self.darkchoco_btn.clicked.connect(lambda: self.add_to_order("다크초콜릿")) 

        # 테이블 설정 - 열 너비 고정 (테이블 화면에서 상품명이 너무 길거나 짧아지지 않도록 설정)
        self.tableWidget.setColumnWidth(0, 150)  #  상품명 열 너비 150px 고정
        self.tableWidget.setColumnWidth(1, 50)   #  수량 열 너비 50px 고정
        self.tableWidget.setColumnWidth(5, 105)  #  금액 열 너비 105px 고정
        self.tableWidget.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)  #  상품명 열 크기 고정
        self.tableWidget.horizontalHeader().setSectionResizeMode(1, QHeaderView.Fixed)  #  수량 열 크기 고정
        self.tableWidget.horizontalHeader().setSectionResizeMode(5, QHeaderView.Fixed)  #  금액 열 크기 고정

        # 모든 열을 꽉 차게 확장 (남는 공간을 자동으로 채움)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def check_table_status(self): # MySQL에서 테이블 상태 변화를 감지하고, UI 업데이트
        #  MySQL 최신 데이터 반영 (연결 유지 & 커밋)
        self.db_manager.conn.ping(reconnect=True) # 연결이 끊겼다면 다시 연결
        self.db_manager.conn.commit() # 변경 사항 저장

        # 현재 테이블 상태 조회 (테이블 ID와 상태 ID 가져오기)
        query = "SELECT table_id, table_status_id FROM pp_table"
        self.db_manager.cursor.execute(query) # 데이터베이스에서 쿼리 실행
        table_statuses = self.db_manager.cursor.fetchall() # 모든 데이터 가져오기

        for table_id, status_id in table_statuses: # 가져온 데이터 반복 처리
            status_text = self.get_status_text(status_id)   # 상태 ID를 상태 텍스트로 변환 (예: 1 → "비어있음")

            #  이전 상태와 비교 후 변경된 경우만 UI 업데이트
            if self.previous_table_status.get(table_id) != status_id:
                print(f"[DEBUG] 테이블 {table_id} 상태 변경 감지 → {status_text}")
                
                self.update_table_status(table_id, status_text) # 테이블 상태 업데이트 실행
                
                #  UI 강제 업데이트 (변경이 보장되도록)
                QApplication.processEvents()                
                self.previous_table_status[table_id] = status_id   # 현재 상태를 저장하여 다음 비교 시 활용

    def get_status_text(self, status_id):  # 테이블 상태 ID를 텍스트로 변환하는 함수
        status_mapping = {1: "비어있음", 2: "사용중", 3: "청소중"} # 상태 ID를 상태 이름으로 변환
        return status_mapping.get(status_id, "알 수 없음")  # 상태 ID가 없으면 '알 수 없음' 반환


     # 화면(페이지) 이동 함수 (각 버튼을 누르면 다른 화면으로 이동)
    def go_to_page1(self): # 처음 페이지로 이동
        self.stackedWidget.setCurrentIndex(0)
        self.phone_input.clear()
        self.password_input.clear()
        self.tableWidget.setRowCount(0)  # 모든 행 삭제 (헤더 유지)



    def go_to_page2(self): # 실시간 테이블 상태 확인 페이지로 이동
        self.stackedWidget.setCurrentIndex(1)  

    def go_to_page3(self): # 예약 페이지로 이동 
        self.stackedWidget.setCurrentIndex(2)
        self.phone_input.clear()
        self.password_input.clear() 
        

    def go_to_page4(self): # 로그인 후 이동하는 페이지
        self.stackedWidget.setCurrentIndex(3)  
          

    def go_to_page5(self): # 이전 페이지 이동
        self.stackedWidget.setCurrentIndex(0) # 

    def go_to_page6(self): 
        self.stackedWidget.setCurrentIndex(4) # 

    def go_to_page7(self):
        self.stackedWidget.setCurrentIndex(5) 
        QTimer.singleShot(5000, self.go_to_page1) # 5초 후 처음 페이지로 이동

    def go_to_page8(self):
        self.stackedWidget.setCurrentIndex(6)

    def go_to_page9(self):
        self.stackedWidget.setCurrentIndex(7)
        QTimer.singleShot(5000, self.go_to_page1) # 5초 후 처음 페이지로 이동

    def go_to_page10(self):
        self.stackedWidget.setCurrentIndex(8)

    def go_to_page11(self):
        self.stackedWidget.setCurrentIndex(9)

    def go_to_page12(self):
        self.stackedWidget.setCurrentIndex(10)
        
   # 로그인 확인 함수
    def validate_login(self): #사용자의 전화번호와 비밀번호를 확인하는 함수 
        user_phone = self.phone_input.text().strip()  # 입력된 휴대폰 번호
        password = self.password_input.text().strip()  # 입력된 비밀번호

        if not user_phone or not password: # 입력하지 않았다면
            QMessageBox.warning(self, "로그인 오류", "휴대폰 번호와 비밀번호를 입력하세요.")
            return

        # 데이터베이스에서 사용자 조회
        query = "SELECT user_id FROM users WHERE user_phone = %s AND password = %s"
        self.db_manager.cursor.execute(query, (user_phone, password))
        user = self.db_manager.cursor.fetchone() # 결과 가져오기

        if user: # 사용자가 존재하면 (로그인 성공)
            self.logged_in_user_id = user[0]  # 로그인된 사용자의 ID 저장
            QMessageBox.information(self, "로그인 성공", "로그인이 완료되었습니다.")
            self.go_to_page4()  # 페이지 이동

            self.logout_btn.setEnabled(True) # 로그아웃 버튼 활성화
        else:
            QMessageBox.warning(self, "로그인 실패", "휴대폰 번호 또는 비밀번호가 올바르지 않습니다.")



    # 로그아웃 함수(로그아웃 버튼 클릭시 실행)
    def logout(self):
        QMessageBox.information(self, "로그아웃", "로그아웃되었습니다.")
        self.logged_in_user_id = None # 로그인 정보 초기화
        self.logout_btn.setEnabled(False)  # 로그아웃 버튼 비활성화
        self.snack_order_btn.setEnabled(False) # 간식 주문 버튼 비활성화
        self.clean_service_btn.setEnabled(False) # 청소 서비스 버튼 비활성화
        self.go_to_page1()  # 처음 페이지로 이동


    # 시간 선택 함수
    def select_time(self, time, price): # 사용자 예약 시간을 선택하면 저장
        self.selected_time = time # 선택한 시간을 저장
        self.total_price = price # 선택한 시간에 맞는 가격 저장
        print(f"선택된 시간: {time}분, 금액: {price}원")

    # 테이블 선택 함수 
    def select_table(self, table_number): # 사용자가 예약할 테이블을 선택하면 저장
        self.selected_table = table_number #선택한 테이블 번호 저장
        print(f"선택된 테이블: {table_number}번")

    # 예약 확인 함수
    def confirm_selection(self): # 선택한 시간과 테이블을 확인하는 함수
        if self.selected_time and self.selected_table:
        # 버튼 상태 확인 , 선택한 테이블이 이미 사용 중인지 확인
            button_name_page_8 = f"pushButton_{13 + self.selected_table}"  # 선택한 테이블의 버튼 찾기
            button_page_8 = getattr(self, button_name_page_8)
            
            if not button_page_8.isEnabled(): # 버튼이 비활성화(이미 사용 중)라면 예약 불가능, 선택한 테이블이 사용 중이라면 
                QMessageBox.warning(self, "오류", "선택한 테이블은 이미 사용 중입니다.")
                return            
            
            
            # 선택된 정보 표시 (예약된 테이블 번호, 시간, 가격)
            self.label_selected_table.setText(f"테이블 {self.selected_table}")
            self.label_selected_time.setText(f"{self.selected_time // 60}시간" if self.selected_time == 60 else "30분")
            self.label_price.setText(f"{self.total_price:,}원")

            # 예약 완료 페이지로 이동
            self.stackedWidget.setCurrentIndex(4)
        else:
            QMessageBox.warning(self, "선택 오류", "이용 시간과 테이블을 모두 선택해주세요.")


    # 테이블 상태 초기화 함수 
    def initialize_table_status_widgets(self): # 각 테이블(1~4)의 상태를 '비어있음'으로 설정하는 함수 
        for table_number in range(1, 5):
            widget = TableStatusWidget(table_number, "비어있음") # 처음에는 모두 비어있음
            self.table_widgets[table_number] = widget # 테이블 정보를 저장

    # 결제 처리 함수 
    def handle_payment(self): # 사용자가 예약하고 결제하면 예약 정보를 DB에 저장
        if self.selected_time and self.selected_table:  # 선택한 시간과 테이블이 있는 경우
            start_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S') # 현재 시간 가져오기
            end_time = (datetime.now() + timedelta(minutes=self.selected_time)).strftime('%Y-%m-%d %H:%M:%S')  # 종료 시간 계산

            # 사용자 정보 가져오기
            user_phone = self.phone_input.text().strip()   # 사용자의 휴대폰 번호 가져오기
            if not user_phone: # 전화번호가 입력되지 않았다면
                QMessageBox.warning(self, "결제 오류", "휴대폰 번호가 입력되지 않았습니다.")
                return # 함수 종료 

            # 사용자 ID 가져오기
            query = "SELECT user_id FROM users WHERE user_phone = %s"  # 전화번호로 사용자 ID
            self.db_manager.cursor.execute(query, (user_phone,))  # SQL 실행
            user = self.db_manager.cursor.fetchone() # 사용자 ID 가져오기
            print(f"[DEBUG] user_id 조회 결과: {user}")


            if not user: # 사용자가 데이터베이스에 없으면 
                QMessageBox.warning(self, "결제 오류", "등록된 사용자가 아닙니다.")
                return # 함수 종료 

            user_id = user[0]

            # 예약 정보를 데이터베이스에 저장
            query = """
            INSERT INTO reservations (user_id, table_id, start_time, end_time, price)
            VALUES (%s, %s, %s, %s, %s)
            """
            try:
                self.db_manager.cursor.execute(query, (user_id, self.selected_table, start_time, end_time, self.total_price))
                self.db_manager.conn.commit()  # 변경 사항 저장 (데이터베이스 업데이트 )
                print(f"[DEBUG] 예약 정보 저장 완료: user_id={user_id}, table_id={self.selected_table}, start_time={start_time}, end_time={end_time}")

            except Exception as e: # 오류가 발생하면 
                print(f"Error inserting reservation: {e}") # 오류 메시지 출력
                self.db_manager.conn.rollback() # 데이터베이스 변경 취소(원래 상태로 되돌림)

                
     # 테이블 상태 업데이트 함수 (MySQL에서 테이블 상태를 가져와 UI 업데이트)
    def update_table_status(self, table_number, status, duration=None):
        if table_number in self.table_widgets:  # 선택한 테이블이 리스트 안에 있는지 확인
            widget = self.table_widgets[table_number] # 해당 테이블의 위젯 가져오기

            #  "사용중" 상태일 경우, MySQL에서 예약 시간 조회
            if status == "사용중":
                query = """
                SELECT start_time, end_time FROM reservations WHERE table_id = %s ORDER BY start_time DESC LIMIT 1
                """
                self.db_manager.cursor.execute(query, (table_number,)) # SQL 실행
                reservation = self.db_manager.cursor.fetchone() # 결과 가져오기

                if reservation: # 예약 정보가 있다면 
                    start_time, end_time = reservation  # 시작 시간과 종료 시간 가져오기
                    now = datetime.now() # 현재 시간 가져오기

                    #  남은 시간 계산 (초 단위)
                    remaining_time = int((end_time - now).total_seconds())
                    duration = max(remaining_time // 60, 1)  # **최소 1분 유지 (음수 방지)

                    # 남은 시간이 음수여도 "사용중"으로 유지**
                    widget.set_status(status, duration)
                    widget.start_time = QTime.fromString(start_time.strftime('%H:%M'), 'HH:mm') # 시작 시간 표시
                    widget.end_time = QTime.fromString(end_time.strftime('%H:%M'), 'HH:mm') #  종료 시간 표시
                    widget.update_time_labels()  # UI에 시간 표시
                    QApplication.processEvents() # UI 즉시 업데이트
                else: # 예약 정보가 없으면 
                    print(f"[DEBUG] 테이블 {table_number}: 예약 정보 없음")
                    widget.set_status(status, duration) # 테이블 상태만 변경

                # "사용중" 상태일 때 , 간식 주문과 청소 서비스 버튼 활성화
                self.snack_order_btn.setEnabled(True)
                self.clean_service_btn.setEnabled(True)


            else: # 테이블이 "사용중"이 아닐 때 (비어있음, 청소중 등)
                widget.set_status(status, duration) # 상태 변경


                # 비어있음" 상태라면 간식 주문과 청소 서비스 버튼 비활성화 
                if status == "비어있음":
                    self.snack_order_btn.setEnabled(False)
                    self.clean_service_btn.setEnabled(False)
            #  UI 강제 업데이트 추가
            self.update_table_labels()
            QApplication.processEvents()


    # 테이블 상태 표시 함수 
    def update_table_labels(self): # 화면에 테이블 상태 (사용중, 비어있음 등)를 업데이트하는 함수
        for table_number, widget in self.table_widgets.items(): # 모든 테이블 위젯에 대해 반복
            label_name_page_2 = f"label_{7 + table_number}"  # 페이지 2에서 사용할 라벨 이름 만들기 (예: label_8, lable_9,..)
            label_page_2 = getattr(self, label_name_page_2) # 해당 라벨 가져오기


            button_name_page_4 = f"pushButton_{13 + table_number}"  # 페이지 4에서 사용할 버튼 이름 만들기 
            button_page_4 = getattr(self, button_name_page_4) # 해당 버튼 가져오기

            # 현재 테이블 상태 가져오기
            status = widget.status_label.text()

            # 상태가 "비어있음"일 경우 남은 시간과 시간 범위를 숨김
            if status == "비어있음":
                remaining_time = "" # 남은 시간을 표시하지 않음
                time_range = "" # 시간 범위를 표시하지 않음
            else:
                remaining_time = widget.timer_label.text() # 남은 시간 가져오기
                time_range = widget.time_range_label.text() # 시간 범위 가져오기

            #  UI 즉시 반영
            QApplication.processEvents()

            # 화면 업데이트 (테이블 상태를 텍스트로 표시)
            label_page_2.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")
            button_page_4.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")


            # 상태에 따라 스타일 변경 (버튼 색상 및 활성화 여부)
            if status == "사용중": 
                label_page_2.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                button_page_4.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                button_page_4.setEnabled(False)  # 사용 중인 테이블은 버튼 비활성화
            elif status == "청소중":
                label_page_2.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                button_page_4.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                button_page_4.setEnabled(False)  # 청소 중이면 버튼 비활성화

            else:  # "비어있음"
                label_page_2.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                button_page_4.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                button_page_4.setEnabled(True)  # 예약가능 버튼 활성화


    #  테이블 열 너비 설정 함수 (상품명 열 고정, 가로 스크롤바 제거)
    def update_table_width(self):
        self.tableWidget.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)  #  가로 스크롤바 비활성화
        self.tableWidget.setSizeAdjustPolicy(self.tableWidget.AdjustToContents)  #  크기 자동 조정

        #  테이블 전체 크기에 맞게 설정 (너비 조정)
        self.tableWidget.horizontalHeader().setStretchLastSection(False)  
        self.tableWidget.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)  # 상품명 자동 확장
        self.tableWidget.horizontalHeader().setSectionResizeMode(1, QHeaderView.Fixed)    # 수량 고정
        self.tableWidget.horizontalHeader().setSectionResizeMode(2, QHeaderView.Fixed)    # 감소 버튼
        self.tableWidget.horizontalHeader().setSectionResizeMode(3, QHeaderView.Fixed)    # 추가 버튼
        self.tableWidget.horizontalHeader().setSectionResizeMode(4, QHeaderView.Fixed)    # 삭제 버튼
        self.tableWidget.horizontalHeader().setSectionResizeMode(5, QHeaderView.Fixed)    # 금액 고정

        #  열 크기 조정 (너비가 초과되지 않도록 조절)
        self.tableWidget.setColumnWidth(0, 180)  # 상품명 (기본보다 넓게 설정)
        self.tableWidget.setColumnWidth(1, 50)   # 수량
        self.tableWidget.setColumnWidth(2, 50)   # 감소 버튼
        self.tableWidget.setColumnWidth(3, 50)   # 추가 버튼
        self.tableWidget.setColumnWidth(4, 50)   # 삭제 버튼
        self.tableWidget.setColumnWidth(5, 200)  # 금액

        QApplication.processEvents()  #  UI 업데이트 즉시 반영

    def remove_from_order(self, button): # 주문 목록에서 상품을 삭제하는 함수
        row = self.tableWidget.indexAt(button.pos()).row() # 버튼이 위치한 행 번호 찾기
        if row >= 0: # 올바른 행 번호라면
            self.tableWidget.removeRow(row) # 해당 행 삭제

        if self.tableWidget.rowCount() == 0:  # 모든 행이 삭제된 경우
            self.tableWidget.clearContents()  #  내용만 삭제하고 헤더 유지
            self.tableWidget.setRowCount(1)  #  빈 행을 추가해 테이블 크기 유지
            self.tableWidget.setRowHidden(0, True)  #  빈 행을 숨김
            
            #  모든 열을 꽉 차게 확장
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

            QApplication.processEvents()  #  UI 업데이트 강제 적용
            self.tableWidget.updateGeometry()  #  테이블 크기 재설정

        self.update_total_price() # 총 금액 업데이트 


    #  주문 목록에 상품 추가 (가운데 정렬 적용)
    def add_to_order(self, product_name):  
        price = self.products[product_name]  # 상품 가격 가져오기
        row_count = self.tableWidget.rowCount() # 현재 테이블의 행 개수 가져오기

        # 기존 상품이 있는지 확인
        for row in range(row_count):  
            item = self.tableWidget.item(row, 0)  # 첫 번째 열(상품명)가져오기
            if item and item.text() == product_name: # 같은 상품이 있다면
                qty_item = self.tableWidget.item(row, 1) # 수량 가져오기
                price_item = self.tableWidget.item(row, 5) # 가격 가져오기

                new_qty = int(qty_item.text()) + 1 # 수량 +1 증가
                qty_item.setText(str(new_qty)) # 업데이트된 수량 표시
                price_item.setText(f"{new_qty * price:,}원")  #  "1,000원" 형식 적용
                self.update_total_price() # 총 금액 업데이트
                return

        # 새 행 추가
        self.tableWidget.insertRow(row_count)

        # 상품명 추가
        item_name = QTableWidgetItem(product_name)
        item_name.setTextAlignment(Qt.AlignCenter)  #  상품명 가운데 정렬


        # 수량 추가 (기본값: 1)
        item_qty = QTableWidgetItem("1")
        item_qty.setTextAlignment(Qt.AlignCenter)  #  수량 가운데 정렬

        # 가격 추가("1,000원" 형식)
        formatted_price = QTableWidgetItem(f"{price:,}원")
        formatted_price.setTextAlignment(Qt.AlignCenter)  #  금액 가운데 정렬


        # 테이블에 추가된 값 넣기
        self.tableWidget.setItem(row_count, 0, item_name)
        self.tableWidget.setItem(row_count, 1, item_qty)
        self.tableWidget.setItem(row_count, 5, formatted_price)

        # 감소 버튼 추가
        btn_decrease = QPushButton("➖")
        btn_decrease.clicked.connect(lambda _, btn=btn_decrease: self.decrease_quantity(btn))
        self.tableWidget.setCellWidget(row_count, 2, btn_decrease)

        # 추가 버튼 추가
        btn_increase = QPushButton("➕")
        btn_increase.clicked.connect(lambda _, btn=btn_increase: self.increase_quantity(btn))
        self.tableWidget.setCellWidget(row_count, 3, btn_increase)

        # 삭제 버튼 추가
        btn_delete = QPushButton("🗑️")
        btn_delete.clicked.connect(lambda _, btn=btn_delete: self.remove_from_order(btn))
        self.tableWidget.setCellWidget(row_count, 4, btn_delete)

        self.update_total_price()  # 총 금액 업데이트


    # 총 금액 업데이트 (1,000원 형식 적용)
    def update_total_price(self):
        total = 0
        for row in range(self.tableWidget.rowCount()): # 모든 행 확인
            price_item = self.tableWidget.item(row, 5) # 5번째 열(가격) 가져오기
            if price_item:
                total += int(price_item.text().replace("원", "").replace(",", ""))  #  "1,000원" 처리
        self.label_total.setText(f"{total:,}원")  #  "1,000원" 적용

    # # 상품 삭제 감소 함수 
    # def remove_from_order(self, button):
    #     row = self.tableWidget.indexAt(button.pos()).row()
    #     if row >= 0:
    #         self.tableWidget.removeRow(row)
    #     self.update_total_price()

    # 수량 감소 (금액 형식 적용 + 가운데 정렬)
    def decrease_quantity(self, button):
        row = self.tableWidget.indexAt(button.pos()).row() # 버튼이 위치한 행 번호 찾기
        qty_item = self.tableWidget.item(row, 1) # 수량 가져오기
        price_item = self.tableWidget.item(row, 5) # 가격 가져오기
        product_name = self.tableWidget.item(row, 0).text() # 상품명 가져오기
        price = self.products[product_name] # 해당 상품의 가격 가져오기

        new_qty = int(qty_item.text()) - 1 # 수량 1 감소
        if new_qty > 0: # 수량이 0보다 크면 업데이트
            qty_item.setText(str(new_qty))
            qty_item.setTextAlignment(Qt.AlignCenter)  #  수량 가운데 정렬
            price_item.setText(f"{new_qty * price:,}원")  #  "1,000원" 형식 적용
            price_item.setTextAlignment(Qt.AlignCenter)  #  금액 가운데 정렬
        else: # 수량이 0이면 행 삭제
            self.tableWidget.removeRow(row)
        self.update_total_price() # 총 금액 업데이트

    # 수량 증가 (금액 형식 적용 + 가운데 정렬)
    def increase_quantity(self, button):
        row = self.tableWidget.indexAt(button.pos()).row() # 버튼이 위치한 행 번호 찾기
        qty_item = self.tableWidget.item(row, 1) # 수량 가져오기
        price_item = self.tableWidget.item(row, 5) # 가격 가져오기
        product_name = self.tableWidget.item(row, 0).text() # 상품명 가져오기
        price = self.products[product_name] # 해당 상품의 가격 가져오기

        new_qty = int(qty_item.text()) + 1 # 수량 1증가
        qty_item.setText(str(new_qty))
        qty_item.setTextAlignment(Qt.AlignCenter)  #  수량 가운데 정렬
        price_item.setText(f"{new_qty * price:,}원")  #  "1,000원" 형식 적용
        price_item.setTextAlignment(Qt.AlignCenter)  #  금액 가운데 정렬
        self.update_total_price()   # 총 금액 업데이트



    def order_payment(self):
        #  로그인된 사용자의 ID 확인
        if not hasattr(self, "logged_in_user_id") or self.logged_in_user_id is None:
            QMessageBox.warning(self, "결제 오류", "로그인이 필요합니다.")
            return

        user_id = self.logged_in_user_id  # 저장된 user_id 사용

        # 3️ 장바구니(주문 목록)에서 총 금액 계산
        total_amount = 0
        for row in range(self.tableWidget.rowCount()):
            price_item = self.tableWidget.item(row, 5)  # 5번째 열(가격) 가져오기
            if price_item:
                total_amount += int(price_item.text().replace("원", "").replace(",", ""))  # 금액 정수 변환

        if total_amount == 0:
            QMessageBox.warning(self, "결제 오류", "주문한 상품이 없습니다.")
            return

        try:
            #  orders 테이블에 새 주문 삽입 (결제 대기 상태)
            query = "INSERT INTO orders (user_id, total_amount, payment_status_id) VALUES (%s, %s, %s)"
            self.db_manager.cursor.execute(query, (user_id, total_amount, 1))  # 초기 상태는 1 (결제 대기)
            self.db_manager.conn.commit()

            #  방금 생성된 order_id 가져오기
            order_id = self.db_manager.cursor.lastrowid  # 가장 최근 삽입된 주문 ID

            # 결제 완료로 상태 변경 (1 → 2)
            query = "UPDATE orders SET payment_status_id = 2 WHERE order_id = %s"
            self.db_manager.cursor.execute(query, (order_id,))
            self.db_manager.conn.commit()

            QMessageBox.information(self, "결제 완료", "결제가 정상적으로 처리되었습니다.")
            self.go_to_page9()  # 결제 완료 페이지로 이동

        except Exception as e:
            self.db_manager.conn.rollback()
            QMessageBox.critical(self, "결제 오류", f"결제 중 오류가 발생했습니다: {e}")




if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())        
