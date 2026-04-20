#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
from color_follow_ctrl import color_follow

if __name__ == '__main__':
    follow = color_follow()
    capture = cv.VideoCapture(0)
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
    # capture.set(cv.CAP_PROP_CONTRAST, 50) #设置对比度 -64 - 64  2.0
    # capture.set(cv.CAP_PROP_EXPOSURE, 156) #设置曝光值 1.0 - 5000  156.0
    while capture.isOpened():
        _, img = capture.read()
        pid = (kp, ki, kd)
        lowerb = (s_min, h_min, v_min)
        upperb = (s_max, h_max, v_max)
        t0 = (t1, t2)
        img = follow.run(img, "red")
        cv.line(img, (320, 0), (320, 480), (255, 0, 0), 1)
        cv.line(img, (0, 240), (680, 240), (255, 0, 0), 1)
        cv.imshow("img", img)
        action = cv.waitKey(10) & 0xff
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
    cv.destroyAllWindows()
    capture.release()