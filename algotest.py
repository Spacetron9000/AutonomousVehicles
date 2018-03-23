import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math


cap = cv2.VideoCapture('vidja.avi')
frame_count = 0
cv2.namedWindow("name")

while True:
    ret,frame = cap.read()


    if frame_count == cap.get(cv2.CAP_PROP_FRAME_COUNT):
		frame_count = 0
		cap.set(cv2.CAP_PROP_POS_FRAMES,0)

    frame_count +=1

    cv2.imshow("name", frame)

    ch = cv2.waitKey(5)
    if(ch == 27):
        break

cv2.destroyAllWindows()
