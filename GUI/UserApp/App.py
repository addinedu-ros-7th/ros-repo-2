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


from_class =uic.loadUiType("/home/jieun/robosphere/app.ui")[0]

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



class TableStatusWidget(QWidget):
    """Widget to manage and display the status of a table."""
    def __init__(self, table_number, status):
        super().__init__()
        self.table_number = table_number
        self.setFixedSize(200, 100)

        # Layout
        self.layout = QVBoxLayout(self)

        # Table Label
        self.table_label = QLabel(f"테이블 {table_number}")
        self.table_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.table_label)

        # Status Label
        self.status_label = QLabel(status)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.status_label)


        # Timer Label (남은 시간)
        self.timer_label = QLabel("남은 시간: --:--")
        self.timer_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.timer_label)    


        # Time Range Label (시작 ~ 종료 시간)
        self.time_range_label = QLabel("--:-- ~ --:--")
        self.time_range_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.time_range_label)  


        # Timer for auto status update
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_remaining_time)



        self.cleanup_timer = QTimer(self)  # Timer for cleanup
        self.cleanup_timer.setSingleShot(True)  # Trigger only once
        self.cleanup_timer.timeout.connect(self.cleanup_finished)



        # Internal variables
        self.remaining_time = 0
        self.start_time = None
        self.end_time = None        

        # Set initial background color
        self.set_background_color(status)


    def set_status(self, status, duration=None):
        """Set table status and start timer if needed."""
        self.status_label.setText(status)
        self.set_background_color(status)

        # UI 갱신
        parent_window = QApplication.instance().activeWindow()
        if isinstance(parent_window, WindowClass):
            parent_window.update_table_labels()        

        if status == "사용중" and duration:
            self.start_time = QTime.currentTime()
            self.end_time = self.start_time.addSecs(duration * 60)
            self.remaining_time = duration * 60  # 남은 시간을 초 단위로 설정
            self.timer.start(1000)  # 1초 간격으로 타이머 업데이트
            self.update_time_labels()
            print(f"[DEBUG] 타이머 시작: {duration}분")  # 디버깅용 메시지 추가
 
        elif status == "청소중":
            self.time_range_label.setText("")
            self.remaining_cleanup_time = 60  # 청소 시간을 1분(60초)으로 설정
            self.timer.start(1000)  # 1초 간격으로 카운트다운 시작
            #self.timer_label.setText(f"남은 시간: 01:00")
            self.update_cleanup_time_labels()

        else:
            #self.timer.stop()
            self.timer_label.setText("남은 시간: --:--")
            self.time_range_label.setText("--:-- ~ --:--")

    def update_cleanup_time_labels(self):
        """Update the timer label during cleanup."""
        if self.remaining_cleanup_time > 0:
            self.remaining_cleanup_time -= 1
            minutes, seconds = divmod(self.remaining_cleanup_time, 60)
            self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}")
            print(f"[DEBUG] 청소중 남은 시간: {minutes:02}:{seconds:02}")  # 디버깅 메시지

            # WindowClass의 테이블 레이블 업데이트 호출
            parent_window = QApplication.instance().activeWindow()
            if isinstance(parent_window, WindowClass):
                parent_window.update_table_labels()
            self.cleanup_timer.start(1000)  # 1초 후 다시 호출
        else:
            self.cleanup_finished()



    def set_background_color(self, status):
        """Set the background color based on status."""
        palette = self.palette()
        if status == "사용중":
            palette.setColor(QPalette.Window, QColor(255, 0, 0, 128))  # Red
        elif status == "청소중":
            palette.setColor(QPalette.Window, QColor(255, 165, 0, 128))  # Orange
        else:  # "비어있음"
            palette.setColor(QPalette.Window, QColor(0, 0, 255, 128))  # Blue
        self.setAutoFillBackground(True)
        self.setPalette(palette)

    def update_time_labels(self):
        """Update the time range label."""
        if self.start_time and self.end_time:
            self.time_range_label.setText(
                f"{self.start_time.toString('HH:mm')} ~ {self.end_time.toString('HH:mm')}"
            )

    def update_remaining_time(self):
        """Handle both '사용중' and '청소중' timer updates."""
        if self.status_label.text() == "사용중":
            if self.remaining_time > 0:
                self.remaining_time -= 1
                minutes, seconds = divmod(self.remaining_time, 60)
                self.timer_label.setText(f"남은 시간: {minutes:02}:{seconds:02}")
                print(f"[DEBUG] 남은 시간: {minutes:02}:{seconds:02}")  # 디버깅 메시지

                # WindowClass의 테이블 레이블 업데이트 호출
                parent_window = self.parentWidget()
                if parent_window is None:
                    parent_window = QApplication.instance().activeWindow()
                if isinstance(parent_window, WindowClass):
                    parent_window.update_table_labels()
            else:
                self.set_status("청소중")  # 사용중 타이머 종료 후 청소중 상태로 전환
        elif self.status_label.text() == "청소중":
            self.update_cleanup_time_labels()

    def cleanup_finished(self):
            """Handle cleanup timer expiration and reset status to '비어있음'."""
            self.timer.stop()
            self.set_status("비어있음")
            print("[DEBUG] 청소 완료, 상태: 비어있음")  # 디버깅 메시지


    def handle_timer_finished(self):
        """Handle timer expiration and change status to '청소중'."""
        self.set_status("청소중")


class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("App")
        self.db_manager = DatabaseManager()  # 데이터베이스 연결 객체 생성



        # 선택된 정보 저장용 변수
        self.selected_time = None
        self.selected_table = None
        self.total_price = 0


        # 테이블 상태 위젯 초기화
        self.table_widgets = {}
        self.initialize_table_status_widgets()


        # 버튼 이벤트 연결
        self.pushButton.clicked.connect(self.go_to_page2)  # 실시간 테이블
        self.pushButton_2.clicked.connect(self.go_to_page3)  # 회원가입
        self.pushButton_3.clicked.connect(self.go_to_page1)  # 처음 페이지
        self.pushButton_5.clicked.connect(self.validate_login)
        self.pushButton_6.clicked.connect(self.go_to_page1) # 이전3
        self.pushButton_10.clicked.connect(self.go_to_page5) # 이전4
        self.pushButton_13.clicked.connect(self.go_to_page3)
        self.pushButton_20.clicked.connect(self.go_to_page7) # 결제 완료 메시지

        self.pushButton_11.clicked.connect(lambda: self.select_time(1, 7000))  # 30분 선택
        self.pushButton_12.clicked.connect(lambda: self.select_time(60, 13000))  # 1시간 선택
        self.pushButton_14.clicked.connect(lambda: self.select_table(1))  # 테이블 1
        self.pushButton_15.clicked.connect(lambda: self.select_table(2))  # 테이블 2
        self.pushButton_16.clicked.connect(lambda: self.select_table(3))  # 테이블 3
        self.pushButton_17.clicked.connect(lambda: self.select_table(4))  # 테이블 4
        self.pushButton_18.clicked.connect(self.confirm_selection)  # 확인 버튼       
        self.pushButton_20.clicked.connect(self.handle_payment)  # 결제 버튼


    def go_to_page1(self):
        self.stackedWidget.setCurrentIndex(0)  

    def go_to_page2(self):
        self.stackedWidget.setCurrentIndex(1)  

    def go_to_page3(self):
        self.stackedWidget.setCurrentIndex(2) 
        

    def go_to_page4(self):
        self.stackedWidget.setCurrentIndex(3)  
          

    def go_to_page5(self):
        self.stackedWidget.setCurrentIndex(0) # 

    def go_to_page6(self):
        self.stackedWidget.setCurrentIndex(4) # 

    def go_to_page7(self):
        self.stackedWidget.setCurrentIndex(5)
        QTimer.singleShot(5000, self.go_to_page1)

    def validate_login(self):
        """로그인 검증 함수"""
        user_phone = self.lineEdit.text().strip()  # 입력된 휴대폰 번호
        password = self.lineEdit_2.text().strip()  # 입력된 비밀번호

        if not user_phone or not password:
            QMessageBox.warning(self, "로그인 오류", "휴대폰 번호와 비밀번호를 입력하세요.")
            return

        # 데이터베이스에서 사용자 조회
        query = "SELECT user_id FROM users WHERE user_phone = %s AND password = %s"
        self.db_manager.cursor.execute(query, (user_phone, password))
        user = self.db_manager.cursor.fetchone()

        if user:
            QMessageBox.information(self, "로그인 성공", "로그인이 완료되었습니다.")
            self.go_to_page4()  # 페이지 이동
        else:
            QMessageBox.warning(self, "로그인 실패", "휴대폰 번호 또는 비밀번호가 올바르지 않습니다.")




    def select_time(self, time, price):
        """이용 시간 선택"""
        self.selected_time = time
        self.total_price = price
        print(f"선택된 시간: {time}분, 금액: {price}원")

    def select_table(self, table_number):
        """테이블 선택"""
        self.selected_table = table_number
        print(f"선택된 테이블: {table_number}번")

    def confirm_selection(self):
        """선택 확인 및 page_9로 이동"""
        if self.selected_time and self.selected_table:
        # 버튼 상태 확인
            button_name_page_8 = f"pushButton_{13 + self.selected_table}"  # pushButton_14 ~ pushButton_17
            button_page_8 = getattr(self, button_name_page_8)
            
            if not button_page_8.isEnabled():
                QMessageBox.warning(self, "오류", "선택한 테이블은 이미 사용 중입니다.")
                return            
            
            
            # 선택된 정보 표시
            self.label_selected_table.setText(f"테이블 {self.selected_table}")
            self.label_selected_time.setText(f"{self.selected_time // 60}시간" if self.selected_time == 60 else "30분")
            self.label_price.setText(f"{self.total_price:,}원")

            # page_8로 이동
            self.stackedWidget.setCurrentIndex(4)
        else:
            QMessageBox.warning(self, "선택 오류", "이용 시간과 테이블을 모두 선택해주세요.")


    def initialize_table_status_widgets(self):
        for table_number in range(1, 5):
            widget = TableStatusWidget(table_number, "비어있음")
            self.table_widgets[table_number] = widget


    def handle_payment(self):
        """Handle the payment and start the timer."""
        if self.selected_time and self.selected_table:
            # 선택된 테이블을 "사용중" 상태로 변경
            self.update_table_status(self.selected_table, "사용중", self.selected_time)
            print(f"[DEBUG] 결제 완료: 테이블 {self.selected_table}, 시간 {self.selected_time}분")  # 디버깅 메시지


            self.stackedWidget.setCurrentIndex(6)
        else:
            QMessageBox.warning(self, "결제 오류", "이용 시간과 테이블을 모두 선택해주세요.")
                

    def update_table_status(self, table_number, status, duration=None):
        """Update the status of a specific table."""
        if table_number in self.table_widgets:
            self.table_widgets[table_number].set_status(status, duration)
            print(f"테이블 {table_number}: {status} 상태로 변경 (타이머: {duration}분)")


        self.update_table_labels()

    def update_table_labels(self):
        for table_number, widget in self.table_widgets.items():
            label_name_page_2 = f"label_{7 + table_number}"  # label_8 ~ label_11
            label_page_2 = getattr(self, label_name_page_2)



            button_name_page_4 = f"pushButton_{13 + table_number}"  # pushButton_14 ~ pushButton_17
            button_page_4 = getattr(self, button_name_page_4)

            # 테이블 상태 가져오기
            status = widget.status_label.text()

            # 상태가 "비어있음"일 경우 남은 시간과 시간 범위를 숨김
            if status == "비어있음":
                remaining_time = ""
                time_range = ""
            else:
                remaining_time = widget.timer_label.text()
                time_range = widget.time_range_label.text()


            label_page_2.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")
            button_page_4.setText(f"테이블 {table_number}\n{status}\n{remaining_time}\n{time_range}")


            # 상태에 따라 스타일 변경 
            if status == "사용중":
                label_page_2.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                button_page_4.setStyleSheet("border: 2px solid #007bff; background-color: lightcoral;")
                button_page_4.setEnabled(False)  # 버튼 비활성화
            elif status == "청소중":
                label_page_2.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                button_page_4.setStyleSheet("border: 2px solid #007bff; background-color: orange;")
                button_page_4.setEnabled(False)  # 버튼 활성화

            else:  # "비어있음"
                label_page_2.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                button_page_4.setStyleSheet("border: 2px solid #007bff; background-color: #724DE1;")
                button_page_4.setEnabled(True)  # 버튼 활성화


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())        