import cv2 as cv
from color_identify import color_identify

if __name__ == '__main__':
    color_identify = color_identify()
    capture = cv.VideoCapture(0)
    while capture.isOpened():
        _, image = capture.read()
        image, pos = color_identify.select_color(image, "3101")
        color_identify.identify_grap(pos)
        cv.imshow("image", image)
        cv.waitKey(10)
    cv.destroyAllWindows()
