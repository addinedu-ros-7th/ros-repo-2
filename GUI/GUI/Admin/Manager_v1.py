import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5 import uic
from datetime import datetime
import pymysql  # MySQL 데이터베이스 연동 라이브러리

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
        self.setup_robot_data() 

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
        data = [
            ["로봇 1", "청소 중", "80%"],
            ["로봇 2", "충전 중", "50%"]
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())
