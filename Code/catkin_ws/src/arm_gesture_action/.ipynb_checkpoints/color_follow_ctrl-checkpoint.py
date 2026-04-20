# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np
from color_follow_move import color_follow_move
import random


class color_follow:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.follow_move = color_follow_move()
        self.color_name = None
        self.image = None
        self.color_hsv = {"red": ((0, 150, 160), (10, 255, 255)),
                          "green": ((53, 80, 36), (80, 255, 160)),
                          "blue": ((116, 124, 100), (130, 255, 255)),
                          "yellow": ((20, 98, 130), (40, 255, 255))}

    def Image_Processing(self, img):
        '''
        形态学变换去出细小的干扰因素
        :param img: 输入初始图像
        :return: 检测的轮廓点集(坐标)
        '''
        # 将图像转为灰度图
        gray_img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
#         cv.imshow("th", binary)
        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours

    def draw_ctrl(self, contours):
        '''
        采用多边形逼近的方法绘制轮廓
        '''
        for i, cnt in enumerate(contours):
            # 计算多边形的矩
            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']
            # 获取多边形的中心
            (x, y) = (np.int(cx), np.int(cy))
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积⼤于10000
            if area > 800:
                # 绘制中⼼
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)
                # 计算最小矩形区域
                rect = cv.minAreaRect(cnt)
                # 获取盒⼦顶点
                box = cv.boxPoints(rect)
                # 转成long类型
                box = np.int0(box)
                # 绘制最小矩形
                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                # cv.circle(self.image,(int(x), int(y)),10,(255,0,0),2)
                # 在图片中绘制结果
                cv.putText(self.image, self.color_name, (int(box[1][0] - 15), int(box[1][1]) - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                # print(x,y)
                # 调用舵机移动函数
                self.follow_move.move(x, y)

    def color_follow_ctrl(self, color_hsv):
        (lowerb, upperb) = color_hsv
        # 复制原始图像,避免处理过程中干扰
        color_mask = self.image.copy()
        # 将图像转换为HSV。
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        color = cv.inRange(hsv_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        color_mask[color == 0] = [0, 0, 0]
#         cv.imshow("mask", color_mask)
        # 检测轮廓点集
        color_contours = self.Image_Processing(color_mask)
        # 绘制检测图像,并控制跟随
        self.draw_ctrl(color_contours)

    def run(self, img, name):
        '''
        颜色跟随控制函数
        :param img: 输入图像
        :param name: 选择跟随的颜色
        :return: 输出处理后的图像
        '''
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480), )
        if name == "red":
            self.color_name = "red"
            self.color_follow_ctrl(self.color_hsv["red"])
        elif name == "blue":
            self.color_name = "blue"
            self.color_follow_ctrl(self.color_hsv["blue"])
        elif name == "yellow":
            self.color_name = "yellow"
            self.color_follow_ctrl(self.color_hsv["yellow"])
        elif name == "green":
            self.color_name = "green"
            self.color_follow_ctrl(self.color_hsv["green"])
        return self.image

    def get_hsv(self, img):
        '''
        获取某一区域的HSV的范围
        :param img: 彩色图
        :return: 图像和HSV的范围
        '''
        H = [];S = [];V = []
        img = cv.resize(img, (640, 480), )
        # 将彩色图转成HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 画矩形框
        cv.rectangle(img, (290, 190), (350, 250), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
        for i in range(280, 360):
            for j in range(180, 260):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小
        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)
        # HSV范围调整
        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min - 15 < 0:S_min = 0
        else:S_min -= 15
        if V_min - 15 < 0:V_min = 0
        else:V_min -= 15
        S_max = 255;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        # color = [[random.randint(0, 255) for _ in range(3)] for _ in range(254)]
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range

    def learning_follow(self, img, hsv_lu):
        if hsv_lu == None: return
        self.color_name = "learning_color"
        self.image = cv.resize(img, (640, 480), )
        self.color_follow_ctrl(hsv_lu)
        return self.image
