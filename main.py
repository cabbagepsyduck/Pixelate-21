import numpy as np
import cv2
import math

img = cv2.imread('sample.jpg')
img = cv2.resize(img, (360,360))
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower = np.array([[55,150,150],[145,0,0],[227,227,0],[211,114,211]])  #0.Green , 1.Red , 2Yellow ,3Pink (Patient)
upper = np.array([[65,255,255],[145,0,0],[227,227,0],[211,114,211]])  #0.Green , 1.Red , 2Yellow ,3Pink (Patient)
mat = np.zeros((6,6))
maskg=cv2.inRange(hsv, lower[0], upper[0])
Gmask = cv2.bitwise_and(img, img, mask = maskg)
greencont,_ = cv2.findContours(maskg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
for cnt in greencont:
	M = cv2.moments(cnt)
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])
	print('{} {}'.format(cx, cy))
	x = math.ceil(cx/60) - 1
	y = math.ceil(cy/60) - 1
	mat[y][x] = 1
	
print(mat)
print(len(greencont))
cv2.drawContours(img, greencont, 0, (50,50,50),3)
cv2.imshow('original', img)
cv2.imshow('maskg', Gmask)
cv2.waitKey(0)	