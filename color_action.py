#!/usr/bin/env python3
# coding=utf-8
import cv2 as cv
import time
from DOGZILLALib import DOGZILLA

# V1.1.1
class Dogzilla_Tracking_Forward(object):

    def __init__(self, video_id=0, width=640, height=480, debug=False):
        self.__debug = debug
        self.__video_id = video_id
        self.__state = False
        self.__width = width
        self.__height = height

        self.__video = cv.VideoCapture(self.__video_id)
        # success, _ = self.__video.read()
        success = self.__video.isOpened()
        if not success:
            self.__video_id = (self.__video_id + 1) % 2
            self.__video = cv.VideoCapture(self.__video_id)
            # success, _ = self.__video.read()
            success = self.__video.isOpened()
            if not success:
                if self.__debug:
                    print("---------Camera Init Error!------------")
                return
        self.__state = True

        self.__config_camera()
        
        #我的改动-------------------------------------------------------
        self.dog = DOGZILLA()
        self.dog_init()

        if self.__debug:
            print("---------Video%d Init OK!------------" % self.__video_id)

    def __del__(self):
        if self.__debug:
            print("---------Del Camera!------------")
        self.__video.release()
        self.__state = False

    def __config_camera(self):
        cv_edition = cv.__version__
        if cv_edition[0]=='3':
            self.__video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else:
            self.__video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        
        # self.__video.set(cv.CAP_PROP_BRIGHTNESS, 30)  # 设置亮度 -64 - 64  0.0
        # self.__video.set(cv.CAP_PROP_CONTRAST, 50)  # 设置对比度 -64 - 64  2.0
        # self.__video.set(cv.CAP_PROP_EXPOSURE, 156)  # 设置曝光值 1.0 - 5000  156.0
        self.__video.set(cv.CAP_PROP_FRAME_WIDTH, self.__width)  # 640
        self.__video.set(cv.CAP_PROP_FRAME_HEIGHT, self.__height)  # 480

    # 摄像头是否打开成功
    # Check whether the camera is enabled successfully
    def isOpened(self):
        return self.__video.isOpened()

    # 释放摄像头 Release the camera
    def clear(self):
        self.__video.release()

    # 重新连接摄像头 
    # Reconnect the camera
    def reconnect(self):
        self.__video = cv.VideoCapture(self.__video_id)
        success, _ = self.__video.read()
        if not success:
            self.__video_id = (self.__video_id + 1) % 2
            self.__video = cv.VideoCapture(self.__video_id)
            success, _ = self.__video.read()
            if not success:
                if self.__debug:
                    self.__state = False
                    print("---------Camera Reconnect Error!------------")
                return False
        if not self.__state:
            if self.__debug:
                print("---------Video%d Reconnect OK!------------" % self.__video_id)
            self.__state = True
            self.__config_camera()
        return True

    # 获取摄像头的一帧图片 
    # Gets a frame of the camera
    def get_frame(self):
        success, image = self.__video.read()
        if not success:
            return success, bytes({1})
        return success, image

    # 获取摄像头的jpg图片 
    # Gets the JPG image of the camera
    def get_frame_jpg(self, text="", color=(0, 255, 0)):
        success, image = self.__video.read()
        if not success:
            return success, bytes({1})
        if text != "":
            # 各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
            # The parameters are: image, added text, top left coordinate, font, font size, color, font size  
            cv.putText(image, str(text), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        success, jpeg = cv.imencode('.jpg', image)
        return success, jpeg.tobytes()

    # 初始化机器狗  init DOGZILLA
    def dog_init(self):
        self.dog.stop()
        time.sleep(.01)
        self.dog.pace("slow")
        time.sleep(.01)
        self.dog.translation('z', 90)
        time.sleep(.01)
        self.dog.attitude('p', 10)
        
    def get_color_range(self, rgb_color):
        red = rgb_color[0]
        green = rgb_color[1]
        blue = rgb_color[2]
        
        if red > 230 and green > 230 and blue > 230:
            return "white"
        elif red < 25 and green < 25 and blue < 25:
            return "black"
        elif red > green and red > blue:
            return "red"
        elif green > red and green > blue:
            return "green"
        elif blue > red and blue > green:
            return "blue"
        else:
            return "mix"
    
    def action_mode(self, L):
        if len(L) != 6:
            print("error")
            return 2    #站立
        largest = max(L)
        if largest == L[0] or largest == L[1]:
            return 2    #站立
        elif largest == L[2]:
            return 5    #原地踏步
        elif largest == L[3]:
            return 16   #左右摇摆
        elif largest == L[4]:
            return 15   #波浪
        else:
            return 11   #撒尿
    
    def timekeeping(self,ctn = 1):
        start_time = time.time()
        while True:
            current_time = time.time()
            delta_time = current_time - start_time
            if delta_time > ctn:
                break
        
    #检测红色前进（密度）
    #Detect red forward (density)
    def action_density(self, frame, width=640, height=480):
        i = j = 0
        cntWhite = 0
        cntBlack = 0
        cntRed = 0
        cntGreen = 0
        cntBlue = 0
        cntMix = 0
        for i in range(0, height, 4):
            for j in range(0, width, 4):
                r = frame[i,j][0]
                g = frame[i,j][1]
                b = frame[i,j][2]
                color = (r, g, b)
                color_range = self.get_color_range(color)
                if color_range == "white":
                    cntWhite = cntWhite + 1
                elif color_range == "black":
                    cntBlack = cntBlack + 1
                elif color_range == "red":
                    cntRed = cntRed + 1
                elif color_range == "green":
                    cntGreen = cntGreen + 1
                elif color_range == "blue":
                    cntBlue = cntBlue + 1
                else :
                    cntMix = cntMix + 1
        L = [cntWhite, cntBlack, cntRed, cntGreen, cntBlue, cntMix]
        mode = self.action_mode(L)
        self.dog.action(mode)
        return mode
    
    #检测红色前进（色块）
    #Detect red forward (color block)
    def trace_block(self, frame, width=640, height=480, edge = 40):
        grid = []
        x = int(width/edge)
        y = int(height/edge)
        cntRedBlock = 0
        cntRed = 0
        mode = "stop"
        for i in range(y):
            grid.append([])
            for j in range (x):
                grid[i].append(False)
        for i in range(y):
            for j in range(x):
                cntRed = 0
                num = 8
                interval = int(edge / num)
                for k in range(0, edge, interval):
                    for l in range(0, edge, interval):
                        pixel = frame[i*edge + k, j*edge + l]
                        r = pixel[0]
                        g = pixel[1]
                        b = pixel[2]
                        bright = 180
                        if (r>=bright and ((g<=3*r/4 and g<=b<=r) or (b<=3*r/4 and b<=g<=r))) or (b>=bright and g<=3*bright/4 and r>=bright) or (g>=bright and b<=3*bright/4 and r>=bright):
                            cntRed = cntRed + 1
                if cntRed >= num*num/9:
                    grid[i][j] = True
                    cntRedBlock = cntRedBlock + 1
        if x*y/16 <= cntRedBlock <= x*y/4:
            mode = "forward"
            self.dog.forward(10)
        else:
            mode = "stop"
            self.cancel()
        return mode, cntRedBlock, grid
    #将640*480的帧拆分成16*12的192个小格子，并对每个格子进行抽取num*num个样本进行检测
    #如果每个格子抽取的样本中有大于1/9的像素为红色，则将此小格子标记为true并计数
    #被标记为true的小格子总数大于1/16且小于1/4则认定该帧中有红色物体，并前进，在距离红色过近时停下

    #检测红色色块分布调整方向
    def turn(self, grid):
        cntL = 0
        cntR = 0
        for i in range(len(grid)):
            for j in range(len(grid[i])):
                if grid[i][j]:
                    if j < len(grid[i])/2:
                        cntL = cntL + 1
                    else:
                        cntR = cntR + 1
        if cntL >= cntR + 10:
            mode = "left"
            self.dog.left(10)
        elif cntR >= cntL + 10:
            mode = "right"
            self.dog.right(10)
        else:
            self.cancel()#-------------------可能会造成机器狗停止移动
            mode = "NoTurn"
        return mode
    
    # 停止 stop
    def cancel(self):
        self.dog.reset()

if __name__ == '__main__':
    camera = Dogzilla_Tracking_Forward(debug=True)
    average = False
    m_fps = 0
    t_start = time.time()
    while camera.isOpened():
        if average:
            ret, frame = camera.get_frame() 
            m_fps = m_fps + 1
            fps = m_fps / (time.time() - t_start)
            
        else:
            start = time.time()
            ret, frame = camera.get_frame()
            end = time.time()
            fps = 1 / (end - start)
        
        #我的改动————————————————————————————————————————————
        mode = camera.action_density(frame)
        
        text="FPS:" + str(int(fps)) + " mode: " + str(mode)
        #text="FPS:" + str(int(fps))
        
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)
        cv.imshow('frame', frame)
        
        k = cv.waitKey(1) & 0xFF
        if k == 27 or k == ord('q'):
            break
        camera.timekeeping(2)
        camera.dog.action(2)
        camera.timekeeping(1)
    del camera
    cv.destroyAllWindows()
