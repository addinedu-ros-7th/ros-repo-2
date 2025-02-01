import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import urllib.request
from PyQt5 import uic
import numpy as np
import heapq
import numpy as np
import time
from PIL import Image
import yaml

# ui
from_class = uic.loadUiType("/home/kjj73/dev_ws/Project/final_project/pgmtest.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("pgm test!!")

        self.pixelSize = 5                              # 원래 픽셀 마다의 크기 0.05 cm
        self.collisionArea = 2                          # 로봇의 사이즈를 고려한 장애물 영역 설정
        self.mapRatio = self.collisionArea * 8          # 맵의 크기를 8배 늘려서 비율을 맞춰줌
        self.realAreaSize = int(self.mapRatio)          
        # self.realAreaSize = 8

        self.pgmMap.lower()
        self.gridMap.raise_()
        self.collisionMap.raise_()
        self.collisionMap.raise_()
        self.pointMap.raise_()
        self.pointMap.raise_()
        self.pointMap.raise_()

        with Image.open('/home/kjj73/dev_ws/Project/final_project/map1.pgm') as pgm_image:
            self.mapWidth, self.mapHeight = pgm_image.size
            self.img_array = np.array(pgm_image)

        with open('/home/kjj73/dev_ws/Project/final_project/map1.yaml', 'r') as file:
            self.yaml = yaml.safe_load(file)

        self.pgmMap.setGeometry(10,100,self.mapWidth * 8, self.mapHeight * 8)
        self.gridMap.setGeometry(10,100,self.mapWidth * 8, self.mapHeight * 8)
        self.collisionMap.setGeometry(10,100,self.mapWidth * 8, self.mapHeight * 8)
        self.pointMap.setGeometry(10,100,self.mapWidth * 8, self.mapHeight * 8)

        self.pixmap = QPixmap()
        self.pixmap.load('/home/kjj73/dev_ws/Project/final_project/map1.pgm')
        self.pixmap = self.pixmap.scaled(self.pgmMap.width(), self.pgmMap.height())
        self.pgmMap.setPixmap(self.pixmap)

        self.pixmap2 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.gridMap.setPixmap(self.pixmap2)
        self.pixmap3 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.collisionMap.setPixmap(self.pixmap3)
        self.pixmap4 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.pointMap.setPixmap(self.pixmap3)

        # self.pgmMap.pixmap().fill(Qt.transparent)
        self.gridMap.pixmap().fill(Qt.transparent)
        self.collisionMap.pixmap().fill(Qt.transparent)
        self.pointMap.pixmap().fill(Qt.transparent)

        self.pixmapWidth = self.pgmMap.width()
        self.pixmapHeight = self.pgmMap.height()
        print(self.mapWidth, self.mapHeight)
        print(self.pixmapWidth, self.pixmapHeight, self.realAreaSize)
        # self.labelPixmap.resize(self.pixmap.width(), self.pixmap.height())

        self.drawGrid()
        print(np.unique(self.img_array))
        self.mapArray = np.zeros((int(self.pgmMap.width() / self.realAreaSize) + 1, int(self.pgmMap.height() / self.realAreaSize) + 1))
        print(self.mapArray.shape)

        self.drawCollision()

        print(self.yaml)
        print(self.yaml['origin'])
        print(self.yaml['origin'][1])
        self.drawOrigin()
        self.comboboxAdd()

        self.mouse_x, self.mouse_y = None, None
        self.drawList = []
        self.pathPoint = []
        self.dataCreate.clicked.connect(self.createMap)
        self.dataClear.clicked.connect(self.clearMap)
        self.dataSave.clicked.connect(self.saveMap)
        self.dataOpen.clicked.connect(self.openMap)

    def saveMap(self):
        npPathPoint = np.array(self.pathPoint)
        npPathPoint = ((npPathPoint / 8) / 20)

        if not np.any(npPathPoint):
            print("Empty")
        else :
            npPathPoint[:,1] = ((self.pgmMap.height() / 8 / 20) - npPathPoint[:,1])
            npPathPoint = npPathPoint * -1

        # print(self.npPathPoint)
        
        why = self.requestTime.currentText()
        where = self.tableNumber.currentText()
        index = self.fileIndex.currentText()
        name = (why + "_" + where + "_" + index)
        print(name)
        np.savetxt("/home/kjj73/dev_ws/Project/final_project/"+name+".txt", npPathPoint, fmt='%.3f')

    def openMap(self):
        why = self.requestTime.currentText()
        where = self.tableNumber.currentText()
        index = self.fileIndex.currentText()
        name = (why + "_" + where + "_" + index)
        
        arr = np.loadtxt("/home/kjj73/dev_ws/Project/final_project/"+name+".txt")
        # print(arr)

        pathPainter = QPainter(self.pointMap.pixmap())
        pathPainter.setPen(QPen(Qt.red, 1, Qt.SolidLine))
        pathPainter.setBrush(QColor(255, 255, 0, 100))  # 투명한 노란색

        if not np.any(arr):
            print("Empty")
        else :
            arr = arr * -1
            arr[:,0] = (arr[:,0] * 8 * 20)
            arr[:,1] = (self.pgmMap.height() - (arr[:,1] * 8 * 20))
            arr = np.round(arr).astype(int)

        print(arr)
        for i in range(0, int(len(arr))):
            x = int(arr[i][0] / self.realAreaSize) * self.mapRatio
            y = int(arr[i][1] / self.realAreaSize) * self.mapRatio
            pathPainter.drawRect(x, y, self.mapRatio - 1, self.mapRatio - 1)

        self.update()

        pathPainter.end()


    def clearMap(self):
        self.pathPoint = []
        self.drawGrid()
        self.pointMap.pixmap().fill(Qt.transparent)
        self.drawOrigin()
        self.drawGrid()
        self.update()
        print(self.pathPoint)

    def createMap(self):
        pathPainter = QPainter(self.pointMap.pixmap())
        pathPainter.setPen(QPen(Qt.red, 1, Qt.SolidLine))
        pathPainter.setBrush(QColor(0, 255, 255, 100))  # 투명한 하늘색

        print(self.pathPoint)
        for i in range(0, int(len(self.pathPoint))):
            # for j in range(0, int(self.pathPoint.shape[1])):
            # if(self.mapArray[i][j] == 255):
            x = int(self.pathPoint[i][0] / self.realAreaSize) * self.mapRatio
            y = int(self.pathPoint[i][1] / self.realAreaSize) * self.mapRatio
            pathPainter.drawRect(x, y, self.mapRatio - 1, self.mapRatio - 1)

            self.update()

        pathPainter.end()

    def mouseMoveEvent(self, event):
        if self.mouse_x is None:
            self.mouse_x = event.x()
            self.mouse_y = event.y()
            return
        
        painter = QPainter(self.pointMap.pixmap())
        painter.setPen(QPen(Qt.blue, 2, Qt.SolidLine))
        painter.drawLine(self.mouse_x - self.pointMap.x(), self.mouse_y - self.pointMap.y(), event.x() - self.pointMap.x(), event.y() - self.pointMap.y())
        self.update()
        painter.end()

        self.mouse_x = event.x()
        self.mouse_y = event.y()

        self.pathPoint.append([self.mouse_x - self.pointMap.x(), self.mouse_y - self.pointMap.y()])
        # print([self.mouse_x, self.mouse_y])

    def mouseReleaseEvent(self, event):
        self.mouse_x = None 
        self.mouse_y = None

    def comboboxAdd(self):
        self.requestTime.addItem(str("request"))
        self.requestTime.addItem(str("routine"))
        self.requestTime.addItem(str("exit"))

        self.tableNumber.addItem(str("1"))
        self.tableNumber.addItem(str("2"))
        self.tableNumber.addItem(str("3"))
        self.tableNumber.addItem(str("4"))

        for i in range(10):
            self.fileIndex.addItem(str(i))
    
    def drawOrigin(self):
        originPainter = QPainter(self.pointMap.pixmap())
        originPainter.setPen(QPen(Qt.blue, 3, Qt.SolidLine))

        x = int((self.yaml['origin'][0] * -1 * 100) / 5) * 8
        y = int((self.yaml['origin'][1] * -1 * 100) / 5) * 8
        y = self.pgmMap.height() - y
        x = x - (8)
        y = y - (8)

        print("origin : ", self.yaml['origin'][0], self.yaml['origin'][1], x, y)
        originPainter.drawRect(x, y, self.mapRatio, self.mapRatio)

        originPainter.end()
        self.pointMap.update()

    def drawCollision(self):
        collisionPainter = QPainter(self.collisionMap.pixmap())
        collisionPainter.setBrush(QColor(255, 0, 255, 100))  # 투명한 분홍색
        collisionPainter.setPen(Qt.NoPen)  # 테두리 없음

        # PGM 데이터를 기반으로 흰색이 아닌 부분 찾기
        for y in range(self.mapHeight):
            for x in range(self.mapWidth):
                pixel_value = self.img_array[y, x]
                if pixel_value < 250:  # 흰색이 아닌 영역 감지 (보통 255가 완전한 흰색)
                    # mapArray 좌표 변환
                    map_x = int(x / self.collisionArea)
                    map_y = int(y / self.collisionArea)

                    # **배열 크기 초과 방지**
                    map_x = min(map_x, self.mapArray.shape[1] - 1)
                    map_y = min(map_y, self.mapArray.shape[0] - 1)

                    # 장애물 표시
                    self.mapArray[map_y, map_x] = 1  

        for y in range(self.mapArray.shape[1]):
            for x in range(self.mapArray.shape[0]):
                # CollisionMap에 분홍색으로 표시
                if self.mapArray[x][y] == 1:
                    collisionPainter.drawRect(y * self.mapRatio, x * self.mapRatio, self.mapRatio, self.mapRatio)  # 8x8 크기로 그림
        
        collisionPainter.end()
        self.collisionMap.update()

    def drawGrid(self):
        gridPainter = QPainter(self.gridMap.pixmap())
        gridPainter.setPen(QPen(Qt.green, 1, Qt.DashLine))

        # gridPainter.drawLine(0, 0, 700, 700)
        for i in range(0, self.pixmapWidth, self.realAreaSize):
            gridPainter.drawLine(i, 0, i, self.pgmMap.width())
        for i in range(0, self.pixmapHeight, self.realAreaSize):
            gridPainter.drawLine(0, i, self.pgmMap.height(), i)

        gridPainter.end()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec())