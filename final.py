import cv2
import numpy as np
import serial
import time
import statistics

COL, ROW = 640, 480
CV_FONT = cv2.FONT_HERSHEY_SIMPLEX
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
PURPLE = (128, 0, 128)

LINE_MIN_LEN = 0
CY = 330 

COM_PORT = '/dev/ttyUSB0'
BAUD_RATES = 57600
ser = serial.Serial(COM_PORT, BAUD_RATES)

# self.line : 0-no line  1-left line  2-right line  3:both lines
# self.if_tuen, self.if_turn2 : 0-go straight  1-turn left  2-turn right

class LINES:
    def __init__(self):
        self.frame = 0                  # 影片
        self.prevPosCol = int(COL/2)    # 前一個左右線中點
        self.prevOffsetL = 0            # 左線offset
        self.prevOffsetR = 0            # 右線offset
        self.prevL = 0                  # 左邊線在y=330時的x
        self.prevR = 0                  # 右邊線在y=330時的x
        self.line = 0                   # 線的數目 
        self.if_turn = 0                # 看到標示左右轉  
        self.if_turn2 = 0               # 左邊是否有斜率為負的線或右邊是否有斜率為正的線
        
    def calcCenterPos(self, allLines, indexL, indexR):
        angle = 0
        # 左右線都有偵測到
        if indexL > -1 and indexR > -1:
            xsL = allLines[indexL][2]  # 左邊線在y=330時的x
            xsR = allLines[indexR][2]  # 右邊線在y=330時的x
            posCol = int((xsL + xsR) / 2)  # y=330時的左右線中心點
            self.prevOffsetL = posCol - xsL  # 左線offset
            self.prevOffsetR = posCol - xsR  # 右線offset
            self.line = 3
        # 只有左線
        elif indexL > -1:
            xsL = allLines[indexL][2]  # 中心位置設為現在左線位置再加上剛剛的offset
            posCol = xsL + self.prevOffsetL  # 中心位置設為現在左線位置再加上剛剛的offset
            xsR = posCol - self.prevOffsetR  # 右線位置維持
            self.line = 1
            angle = allLines[indexL][3]  # 線段角度
        # 只有右線
        else:
            xsR = allLines[indexR][2]
            posCol = xsR + self.prevOffsetR
            xsL = posCol - self.prevOffsetL
            self.line = 2
            angle = allLines[indexR][3]
        return posCol, xsL, xsR, angle

    # 把左右線畫到影片上
    def drawallLines(self, allLines, indexL, indexR):
        lines = [indexL, indexR]
        # 遍歷左右兩線
        for i in range(2):
            # 線存在
            if lines[i] != -1:
                line = allLines[lines[i]][1][0] # the line of the index
                cv2.line(self.frame, (line[0], line[1]), (line[2], line[3]), GREEN, 2) # 把線畫到圖上
            c1x, c1y = int(COL/2), 0
            c2x, c2y = int(COL/2), ROW
            cv2.line(self.frame, (c1x, c1y), (c2x, c2y), YELLOW, 2)  # 畫出畫面中線
        return

    # 找線
    def findLines(self, signal_detected):
        # 轉化為灰度圖
        img_gray = cv2. cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        
        # 二值化
        ret, img_binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
        
        # 侵蝕
        kernel = np.ones((7, 7), np.uint8)
        img_erode = cv2.dilate(img_binary, kernel, iterations=4)
        cv2.imshow('canny', img_erode)
        
        # Canny
        img_canny = cv2.Canny(img_erode, 50, 150)
        
        # 有偵測到障礙物，開始找三角形
        if signal_detected:
            contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 取出影像中輪廓
            tri = 0 # 計算三角形變數
            for contour in contours:
                epsilon = 0.1 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 3 and cv2.contourArea(contour) > 5500: # 檢查是否為三角形，且面積不可太小
                    tri = tri + 1
                    print(cv2.contourArea(contour))
                    cv2.drawContours(self.frame, [approx], 0, (0, 255, 0), 2)
                    x1, x2, x3 = approx[0][0][0], approx[1][0][0], approx[2][0][0]  # 在影像上畫出三角形
                    x_arr = [x1, x2, x3]  # 取得三角形三個頂點的x座標
                    # 左轉三角形
                    if statistics.median(x_arr)-min(x_arr) > max(x_arr)-statistics.median(x_arr):  
                        self.if_turn = 1
                    # 右轉三角形
                    else:  
                        self.if_turn = 2
        
        # ROI
        ROI = np.array([[(0, 0), (0, 150), (640, 150), (640, 0)]])
        cv2.fillPoly(img_canny, ROI, (0,0,0))
        
        # show roi boundary
        cv2.line(self.frame, (0, 480), (640, 480), PURPLE, 1)
        cv2.line(self.frame, (640, 480), (640, 150), PURPLE, 1)
        cv2.line(self.frame, (640, 150), (0, 150), PURPLE, 1)
        cv2.line(self.frame, (0, 150), (0, 480), PURPLE, 1)
        
        # 霍夫找線
        lines = cv2.HoughLinesP(img_canny, 1, np.pi/180, threshold=50, minLineLength=10, maxLineGap=250)
        
        # 存取找到的線
        allLines = [] # element:[length, linePosition[x1, y1, x2, y2], cx, angle]
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if (y2-y1)!=0:
                    lineLen = np.linalg.norm(np.array([x2-x1, y2-y1])) # length of the line
                    angle = np.arctan2(y1-y2, x2-x1)  # angle of the line
                    cx = 0
                    allLines.append([lineLen, line, cx, angle])  # 把線存到allLines
                    
        return allLines
    
    def Position(self, frame, signal_detected):
        self.frame = frame
        self.if_turn = 0
        self.if_turn2 = 0
        angle = 0

        # 找線
        allLines = self.findLines(signal_detected)

        # 選擇想要的線
        lineStat, indexL, indexR = self.selectLines(allLines, signal_detected)

        # 如果有找到線
        if lineStat:
            self.drawallLines(allLines, indexL, indexR)  # 把線畫出來
            posCol, xsL, xsR, angle = self.calcCenterPos(allLines, indexL, indexR)  # 計算中心點等
        # 如果沒有找到線
        else:
            posCol = self.prevPosCol  # 維持前一次偵測到時的設定
            xsL, xsR = self.prevL, self.prevR  
            angle = 0
        cv2.circle(self.frame, (int(posCol), CY), 4, RED, -1)  # 畫出中心點
        strData = "{:5d}".format(int(posCol - COL/2))  # 計算offset
        cv2.putText(self.frame, strData, (posCol, 50), CV_FONT, 0.5, GREEN, 1, cv2.LINE_AA, False)  # 在影片上顯示出offset
        self.prevPosCol = posCol
        self.prevL = xsL
        self.prevR = xsR
        return angle, self.if_turn2, self.if_turn, self.line, posCol, self.frame
    
    # 選想要的線
    def selectLines(self, allLines, signal_detected):
        
        vLines = [] # element:[length, index]
        # vLines存下每一條線對應在allLines時的index
        for k in range(len(allLines)):
            lineLen = allLines[k][0] # length
            if lineLen >= LINE_MIN_LEN:  # 不要找到太短的線
                vLines.append([lineLen, k])
        vLines.sort(reverse=True) # from long to short
        
        # 偵測到兩條以上的線
        if len(vLines) >= 2:
            i=0
            cx = -1
            cx1 = -1
            k = -1
            k0 = -1
            k1 = -1
            slope = 0
            while(1):
                # 遍歷完全部的線了
                if i == len(vLines):
                    break
                k = vLines[i][1]
                x1, y1, x2, y2 = allLines[k][1][0]  # 線的xy
                slope = (y1-y2) / (x2-x1) if (x2 - x1) != 0 else np.inf  # 斜率

                 # 線有通過CY(左上到右下)
                if y2-y1>0 and y2>CY and y1<CY:
                    m = (y2-CY) / (y2-y1)  # 線斜率
                    allLines[k][2] = int(x2 - m * (x2-x1))  # 在CY時的x
                # 線有通過CY(右上到左下)
                elif y2-y1<0 and y2<CY and y1>CY: 
                    m = (y1-CY) / (y1-y2)
                    allLines[k][2] = int(m * (x2-x1) + x1)
                # 線沒有通過CY
                else: 
                    allLines[k][2] = -1  # 這一條線不選，cx=-1
                cx = allLines[k][2]  # 剛剛算出y=330時的x

                # line on the left and angle is minus, then turn left
                if slope < 0 and slope > -1 and signal_detected==False and cx < COL/2: 
                    self.if_turn2 = 1
                # line on the right and angle is minus, then turn right
                elif slope > 0 and slope < 1 and signal_detected==False and cx > COL/2:
                    self.if_turn2 = 2
                
                i=i+1

                # 通過CY，找到線了
                if cx != -1: 
                    k0=k
                    break

            # 沒有符合條線的線
            if cx == -1: 
                lineStat = False
                indexL, indexR = -1, -1
            # 剛剛找到一條線了，繼續找第二條線
            else:
                k0=k
                while(1):
                    # 沒線了
                    if i==len(vLines):
                        break
                    k = vLines[i][1]  # 線的index
                    x1, y1, x2, y2 = allLines[k][1][0]  # 線兩端的xy

                    # 線有通過CY(左上到右下)
                    if y2-y1>0 and y2>CY and y1<CY:
                        m = (y2-CY) / (y2-y1)  # 線斜率
                        allLines[k][2] = int(x2 - m * (x2-x1))  # 在CY時的x
                    # 線有通過CY(右上到左下)
                    elif y2-y1<0 and y2<CY and y1>CY:
                        m = (y1-CY) / (y1-y2)  
                        allLines[k][2] = int(m * (x2-x1) + x1)  
                    # 線沒有通過CY高度，不選
                    else:
                        allLines[k][2] = -1
                    # 在CY時的x
                    cx1 = allLines[k][2]
                    
                    # 最長的線是左線
                    if cx < COL/2:
                        # 第二長的線是找到右線
                        if cx1 > COL/2 and abs(cx-cx1)>150:
                            k1 = k
                            break  # 找到需要的線了，跳出迴圈
                    # 最長的線是右線
                    elif cx>COL/2:
                        # 第二長的線是找到左線
                        if cx1 < COL/2 and cx1 != -1 and abs(cx-cx1)>150:
                            k1 = k
                            break  # 找到需要的線了，跳出迴圈
                    i=i+1  # 沒有找到對邊的線，再繼續找下一條線
                lineStat = True  # 有找到線
                # 存取左右線的index
                if cx > COL/2:
                    indexR, indexL = k0, k1
                else:
                    indexR, indexL = k1, k0

        # 霍夫只有找到一條線
        elif len(vLines) == 1: 
            k0 = vLines[0][1]
            # the line is on the right side
            if allLines[k0][2] >= COL/2:
                indexR, indexL = k0, -1 
            # the line is on the left side
            else:
                indexR, indexL = -1, k0  
            lineStat = True  # 有線
        # 霍夫沒有找到線
        else:
            lineStat, indexL, indexR = False, -1, -1
        return lineStat, indexL, indexR
        
# s : go straight
# p : pause
# 1~5 : 右轉(數字越大轉幅越大)
# -1~-5 : 左轉(絕對值越大轉幅越大)
# L : turn left
# R : turn right
# n : new loop -> measure distance

def Drive(angle, if_turn2, if_turn, linenum, offset):
    # 看到右轉標示
    if if_turn == 2:
        return 'R\n'
    # 看到左轉標示
    elif if_turn == 1:
        return 'L\n'
    # 右半畫面有斜率為正的線要右轉
    if if_turn2 == 2:
        return '5\n'
    # 左半畫面有斜率為負的線要左轉
    elif if_turn2 == 1:
        return '-5\n'
    # right line -> turn left
    elif linenum == 2: 
        if angle > np.deg2rad(-40):
            return '-4\n'
        elif np.deg2rad(-40) >= angle > np.deg2rad(-60):
            return '-3\n'
        elif np.deg2rad(-60) >= angle > np.deg2rad(-75):
            return '-2\n'
        elif np.deg2rad(-75) >= angle:
            return '-1\n'
    # left line -> turn right
    elif linenum == 1: 
        if np.deg2rad(40) >= angle:
            return '4\n'
        elif np.deg2rad(60) >= angle > np.deg2rad(40):
            return '3\n'
        elif np.deg2rad(75) >= angle > np.deg2rad(60):
            return '2\n'
        elif angle > np.deg2rad(75):
            return '1\n'
    # 兩線中心比畫面中心偏右大於10cm，向左靠一點
    elif offset < -10:
        return '-1\n'
    # 兩線中心比畫面中心偏左大於10cm，向右靠一點
    elif offset > 10:
        return '1\n'
    # 其他狀況直走
    else:
        return 's\n'
        

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    Lines = LINES()  # 創建一個class來找線
    time.sleep(2)
    ser.write('s\n'.encode())  # 最一開始先直走
    datas = ''
    data = ser.readline().decode('utf-8').strip()  # 接收arduino傳回來的訊息
    signal_detected = False
    while(True):
        ret, frame = cap.read()  # 讀取影像
        ser.write('n\n'.encode())  # new loop -> measure distance
        datas = ser.readline().decode('utf-8').strip()
        # 有障礙物
        if datas == 'y':
            datas = ''
            signal_detected = True
            print("Signal detected")
        angle, if_turn2, if_turn, linenum, posCol, frame = Lines.Position(frame, signal_detected)  # 開始找線和全部的計算
        signal_detected = False
        offset = int(posCol - COL/2)  # 兩線中心和畫面中心的偏移量
        cv2.imshow('image', frame)  # 顯示影像
        command = Drive(angle, if_turn2, if_turn, linenum, offset)  # 判斷自走車怎麼走
        ser.write(command.encode())  # 傳送自走車走向給arduino
        data = ser.readline().decode('utf-8').strip()
        print(data)
        
        if data == "turn left":
            break
        
        k = cv2.waitKey(1) & 0xFF
        # 按'q'結束程式
        if k == ord('q'):  
            ser.write('q'.encode())
            break
    
    ser.close()  # 關閉端口
    cap.release()
    cv2.destroyAllWindows()
