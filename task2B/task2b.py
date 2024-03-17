#The aim of this task is to detect object from the given image and find its pixel co-ordinates.

import numpy as np
import cv2

img = cv2.imread('yellow_detect.jpeg', -1)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

lower_yellow = np.array([15, 100, 150])
upper_yellow = np.array([24, 255, 255])
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

result = cv2.bitwise_and(img, img, mask = mask)

gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

corners = cv2.goodFeaturesToTrack(gray, 5, 0.35, 17)
corners = np.int0(corners)

for corner in corners:
	x, y = corner.ravel()
	cv2.circle(result, (x, y), 3, (0, 0, 255), -1)

centre = ((corners[0]+corners[2]+corners[3]+corners[4])/4)
centre = np.int0(centre)

string = ""
for p in centre:
	string+=str(p)
for a in range(1, 8):
	print(string[a], end="")
