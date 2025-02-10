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
from_class = uic.loadUiType("/home/kjj73/test_folder/src/PathMaker/PathMaker.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("pgm test!!")

        self.filepath = "/home/kjj73/test_folder/src/PathMaker/"
        self.pgmpath = '/home/kjj73/test_folder/src/PathMaker/fourtable.pgm'
        self.yamlpath = '/home/kjj73/test_folder/src/PathMaker/fourtable.yaml'

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

        with Image.open(self.pgmpath) as pgm_image:
            self.mapWidth, self.mapHeight = pgm_image.size
            self.img_array = np.array(pgm_image).T
            #self.img_array = np.flip(self.img_array)
            print("img_array:", self.img_array.shape)

        self.mapLimitX = (self.mapWidth * 5) / 100
        self.mapLimitY = (self.mapHeight * 5) / 100

        with open(self.yamlpath, 'r') as file:
            self.yaml = yaml.safe_load(file)

        self.pgmMap.setGeometry(15,150,self.mapWidth * 8, self.mapHeight * 8)
        self.gridMap.setGeometry(15,150,self.mapWidth * 8, self.mapHeight * 8)
        self.collisionMap.setGeometry(15,150,self.mapWidth * 8, self.mapHeight * 8)
        self.pointMap.setGeometry(15,150,self.mapWidth * 8, self.mapHeight * 8)
        self.pathMap.setGeometry(15,150,self.mapWidth * 8, self.mapHeight * 8)

        self.pixmap = QPixmap()
        self.pixmap.load(self.pgmpath)
        self.pixmap = self.pixmap.scaled(self.pgmMap.width(), self.pgmMap.height())
        self.pgmMap.setPixmap(self.pixmap)

        self.pixmap2 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.gridMap.setPixmap(self.pixmap2)
        self.pixmap3 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.collisionMap.setPixmap(self.pixmap3)
        self.pixmap4 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.pointMap.setPixmap(self.pixmap4)
        self.pixmap5 = QPixmap(self.pgmMap.width(), self.pgmMap.height())
        self.pathMap.setPixmap(self.pixmap5)

        # self.pgmMap.pixmap().fill(Qt.transparent)
        self.gridMap.pixmap().fill(Qt.transparent)
        self.collisionMap.pixmap().fill(Qt.transparent)
        self.pointMap.pixmap().fill(Qt.transparent)
        self.pathMap.pixmap().fill(Qt.transparent)

        self.pixmapWidth = self.pgmMap.width()
        self.pixmapHeight = self.pgmMap.height()
        print("map width, height : ", self.mapWidth, self.mapHeight)
        print(self.pixmapWidth, self.pixmapHeight, self.realAreaSize)
        # self.labelPixmap.resize(self.pixmap.width(), self.pixmap.height())

        self.drawGrid()
        print(np.unique(self.img_array))
        self.mapArray = np.zeros((int(self.pgmMap.width() / self.realAreaSize) + 1, int(self.pgmMap.height() / self.realAreaSize) + 1))
        self.collision_cell_list = np.zeros((int(self.pgmMap.width()/self.mapRatio) + 1, int(self.pgmMap.height()/self.mapRatio) + 1), dtype=np.int32)
        print("collision_cell_list:", self.collision_cell_list.shape)
        print("mapArray:", self.mapArray.shape)

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

        self.startPoint = [None, None]
        self.endPoint = [None, None]
        self.collisionShow.clicked.connect(self.showCollision)
        self.pathAction.clicked.connect(self.astarStart)
        self.pathClear.clicked.connect(self.clearPath)

        # PGM 데이터를 기반으로 흰색이 아닌 부분 찾기
        for x in range(self.mapWidth):
            for y in range(self.mapHeight):
                pixel_value = self.img_array[x, y]
                if pixel_value < 240:  # 흰색이 아닌 영역 감지 (보통 255가 완전한 흰색)
                    # mapArray 좌표 변환
                    map_x = int(x / self.collisionArea)
                    map_y = int(y / self.collisionArea)

                    # **배열 크기 초과 방지**
                    map_x = min(map_x, self.mapArray.shape[0] - 1)
                    map_y = min(map_y, self.mapArray.shape[1] - 1)

                    # 장애물 표시
                    self.mapArray[map_x, map_y] = 1
                    self.collision_cell_list[map_x, map_y] = 1
                    # print("self.mapArray:", self.mapArray.shape)

    def clearPath(self):
        # pathPainter = QPainter(self.pathMap.pixmap())
        self.pathMap.pixmap().fill(Qt.transparent)
        self.collision_cell_list = np.zeros((int(self.pgmMap.width()/self.mapRatio) + 1, int(self.pgmMap.height()/self.mapRatio) + 1), dtype=np.int32)
        self.drawGrid()
        self.pointMap.pixmap().fill(Qt.transparent)
        #self.collision_cell_list.pixmap().fill(Qt.transparent)
        self.drawOrigin()
        self.drawGrid()
        self.update()

        # PGM 데이터를 기반으로 흰색이 아닌 부분 찾기
        for x in range(self.mapWidth):
            for y in range(self.mapHeight):
                pixel_value = self.img_array[x, y]
                if pixel_value < 240:  # 흰색이 아닌 영역 감지 (보통 255가 완전한 흰색)
                    # mapArray 좌표 변환
                    map_x = int(x / self.collisionArea)
                    map_y = int(y / self.collisionArea)

                    # **배열 크기 초과 방지**
                    map_x = min(map_x, self.mapArray.shape[0] - 1)
                    map_y = min(map_y, self.mapArray.shape[1] - 1)

                    # 장애물 표시
                    self.mapArray[map_x, map_y] = 1
                    self.collision_cell_list[map_x, map_y] = 1
                    # print("self.mapArray:", self.mapArray.shape)
        
    def astarStart(self):
        self.startPoint[0] = int(self.x_StartPoint.text())
        self.startPoint[1] = int(self.y_StartPoint.text())

        self.endPoint[0] = int(self.x_EndPoint.text())
        self.endPoint[1] = int(self.y_EndPoint.text())

        point_painter = QPainter(self.pathMap.pixmap())
        
        if self.startPoint[0] != "" and self.startPoint[1] != "":
            point_painter.setPen(QPen(Qt.green, 1, Qt.SolidLine))
            point_painter.setBrush(QBrush(Qt.green))
            point_painter.drawRect((int(self.startPoint[0])*self.mapRatio + 1), (int(self.startPoint[1])*self.mapRatio + 1), 8, 8)

        if self.endPoint[0] != "" and self.endPoint[1] != "":
            point_painter.setPen(QPen(Qt.blue, 1, Qt.SolidLine))
            point_painter.setBrush(QBrush(Qt.blue))
            point_painter.drawRect((self.endPoint[0]*self.mapRatio + 1), (self.endPoint[1]*self.mapRatio + 1), 8, 8)

        self.update()
        point_painter.end()

        start = (self.startPoint[0], self.startPoint[1])
        goal = (self.endPoint[0], self.endPoint[1])
        grid = self.collision_cell_list

        self.path = self.astar(grid, start, goal)
        # print("경로:", self.path)
            # print(i[0], i[1])
        self.drawPath()

    def drawPath(self):
        path_painter = QPainter(self.pathMap.pixmap())
        if self.path != None:
            for i in self.path:
                path_painter.setBrush(QColor(139, 69, 19, 100))  # 투명한 갈색
                path_painter.drawRect((int(i[0])*self.mapRatio + 1), (int(i[1])*self.mapRatio + 1), self.mapRatio, self.mapRatio)
                self.update()
                # time.sleep(0.125)

        path_painter.setPen(QPen(Qt.green, 1, Qt.SolidLine))
        path_painter.setBrush(QBrush(Qt.green))
        path_painter.drawRect((self.startPoint[0]*self.mapRatio + 1), (self.startPoint[1]*self.mapRatio + 1), self.mapRatio, self.mapRatio)

        path_painter.setPen(QPen(Qt.blue, 1, Qt.SolidLine))
        path_painter.setBrush(QBrush(Qt.blue))
        path_painter.drawRect((self.endPoint[0]*self.mapRatio + 1), (self.endPoint[1]*self.mapRatio + 1), self.mapRatio, self.mapRatio)
        self.update()
        path_painter.end()
        pass

    def showCollision(self):
        collisionPainter = QPainter(self.pathMap.pixmap())
        collisionPainter.setBrush(QColor(128, 0, 128, 100))  # 투명한 보라색
        collisionPainter.setPen(Qt.NoPen)  # 테두리 없음

        for x in range(self.collision_cell_list.shape[0]):
            for y in range(self.collision_cell_list.shape[1]):
                # CollisionMap에 분홍색으로 표시
                if self.collision_cell_list[x][y] == 1:
                    collisionPainter.drawRect(x * self.mapRatio, y * self.mapRatio, self.mapRatio, self.mapRatio)  # 16x16 크기로 그림
                    self.pathMap.update()
        
        collisionPainter.end()

    # Manhattan Distance 휴리스틱 함수 (목표까지의 예상 거리)
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
    # A* 알고리즘 구현
    def astar(self, grid, start, goal):
        # 이동할 수 있는 방향 (상, 하, 좌, 우)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # 오픈 리스트: (f_score, (x, y)) 형태로 저장
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        # 각 노드에 대해 f, g, h 값 저장
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 경로를 추적할 부모 노드
        came_from = {}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            # 목표 지점에 도달하면 경로를 반환
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # 역순으로 경로 반환
            
            # 현재 노드에서 갈 수 있는 이웃 탐색
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                # 이웃이 경계 내에 있고, 장애물이 아니어야 함
                if (0 <= neighbor[0] < len(grid)) and (0 <= neighbor[1] < len(grid[0])) and grid[neighbor[0]][neighbor[1]] != 1:
                    tentative_g_score = g_score[current] + 1  # 이동 비용 (상, 하, 좌, 우는 동일하게 1로 설정)
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        return None  # 경로가 없으면 None 반환

    def saveMap(self):
        npPathPoint = np.array(self.pathPoint)
        npPathPoint = ((npPathPoint / 8) / 20)

        if not np.any(npPathPoint):
            print("Empty")
        else :
            print("limit x , y : ", self.mapLimitX, self.mapLimitY)
            # npPathPoint[:, 0] = npPathPoint[:, 0] + (self.yaml['origin'][0])
            # npPathPoint[:, 1] = npPathPoint[:, 1] + (self.yaml['origin'][1])
            npPathPoint[:, 0] = npPathPoint[:, 0] - (self.mapLimitX - (self.mapLimitX - abs(self.yaml['origin'][0])))
            npPathPoint[:, 1] = (npPathPoint[:, 1] - (self.mapLimitY - abs(self.yaml['origin'][1]))) * -1

        # npPathPoint[:,0]

        # print(self.pathPoint)
        npPathPoint = np.round(npPathPoint, 6)
        print(npPathPoint)
        
        why = self.requestTime.currentText()
        where = self.tableNumber.currentText()
        index = self.fileIndex.currentText()
        name = (why + "_" + where + "_" + index)
        print(name)
        np.savetxt(self.filepath+name+".txt", npPathPoint, fmt='%.3f')

    def openMap(self):
        why = self.requestTime.currentText()
        where = self.tableNumber.currentText()
        index = self.fileIndex.currentText()
        name = (why + "_" + where + "_" + index)
        
        arr = np.loadtxt(self.filepath+name+".txt")
        # print(arr)

        pathPainter = QPainter(self.pointMap.pixmap())
        pathPainter.setPen(QPen(Qt.red, 1, Qt.SolidLine))
        pathPainter.setBrush(QColor(255, 255, 0, 100))  # 투명한 노란색

        if not np.any(arr):
            print("Empty")
        else :
            arr[:, 0] = (arr[:, 0] + (self.mapLimitX - (self.mapLimitX - abs(self.yaml['origin'][0]))))
            arr[:, 1] = (abs(arr[:, 1]) + (self.mapLimitY - abs(self.yaml['origin'][1])))

            # arr = arr * -1
            arr[:,0] = (arr[:,0] * 8 * 20)
            arr[:,1] = (arr[:,1] * 8 * 20)
            arr = np.round(arr).astype(int)
            print(arr)

        print(arr//self.realAreaSize)
        for i in range(0, int(len(arr))):
            x = int(arr[i][0] / self.realAreaSize) * self.mapRatio # Cell Size Complete
            y = int(arr[i][1] / self.realAreaSize) * self.mapRatio
            #x = int(arr[i][0])
            #y = int(arr[i][1])
            #print("x, y:", x, y)
            self.collision_cell_list[x//self.realAreaSize, y//self.realAreaSize] = 1
            
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
        # print(self.pathPoint)

    def createMap(self):
        pathPainter = QPainter(self.pointMap.pixmap())
        pathPainter.setPen(QPen(Qt.red, 1, Qt.SolidLine))
        pathPainter.setBrush(QColor(0, 255, 255, 100))  # 투명한 하늘색

        # print(self.pathPoint)
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
        time.sleep(0.05)
        # print([self.mouse_x, self.mouse_y])

    def mouseReleaseEvent(self, event):
        self.mouse_x = None 
        self.mouse_y = None

    def comboboxAdd(self):
        self.requestTime.addItem(str("request"))
        self.requestTime.addItem(str("routine"))
        self.requestTime.addItem(str("exit"))
        self.requestTime.addItem(str("collision"))

        self.tableNumber.addItem(str("1"))
        self.tableNumber.addItem(str("2"))
        self.tableNumber.addItem(str("3"))
        self.tableNumber.addItem(str("4"))
        self.tableNumber.addItem(str("5"))

        for i in range(10):
            self.fileIndex.addItem(str(i))
    
    def drawOrigin(self):
        originPainter = QPainter(self.pointMap.pixmap())
        originPainter.setPen(QPen(Qt.red, 3, Qt.SolidLine))
        originPainter.drawLine(self.pgmMap.width() // 2, 0, self.pgmMap.width() // 2, self.pgmMap.height())
        originPainter.drawLine(0, self.pgmMap.height() // 2, self.pgmMap.width(), self.pgmMap.height() // 2)

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
        for x in range(self.mapWidth):
            for y in range(self.mapHeight):
                pixel_value = self.img_array[x, y]
                if pixel_value < 240:  # 흰색이 아닌 영역 감지 (보통 255가 완전한 흰색)
                    # mapArray 좌표 변환
                    map_x = int(x / self.collisionArea)
                    map_y = int(y / self.collisionArea)

                    # **배열 크기 초과 방지**
                    map_x = min(map_x, self.mapArray.shape[0] - 1)
                    map_y = min(map_y, self.mapArray.shape[1] - 1)

                    # 장애물 표시
                    self.mapArray[map_x, map_y] = 1
                    self.collision_cell_list[map_x, map_y] = 1
                    # print("self.mapArray:", self.mapArray.shape)

        for x in range(self.mapArray.shape[0]):
            for y in range(self.mapArray.shape[1]):
                # CollisionMap에 분홍색으로 표시
                if self.mapArray[x][y] == 1:
                    collisionPainter.drawRect(x * self.mapRatio, y * self.mapRatio, self.mapRatio, self.mapRatio)  # 16x16 크기로 그림
                    self.collisionMap.update()
        
        collisionPainter.end()

    def drawGrid(self):
        gridPainter = QPainter(self.gridMap.pixmap())
        gridPainter.setPen(QPen(Qt.green, 1, Qt.DashLine))

        # gridPainter.drawLine(0, 0, 700, 700)
        for i in range(0, self.pixmapWidth, self.realAreaSize):
            gridPainter.drawLine(i, 0, i, self.pgmMap.width())
        for i in range(0, self.pixmapHeight, self.realAreaSize):
            gridPainter.drawLine(0, i, self.pgmMap.height(), i)

        gridPainter.end()

if __name__ == "__main__":#
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec())