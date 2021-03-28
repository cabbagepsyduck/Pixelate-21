import gym
import pix_sample_arena
import time
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np
import math



env = gym.make("pix_sample_arena-v0")
time.sleep(3)


img = env.camera_feed()
img = img[100:412,100:412]
img = cv2.resize(img, (360,360))
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
colorBGR = np.array([[0,227,0],[0,0,145],[0,227,227],[211,114,211],[227,227,227]])  #0.Green , 1.Red , 2Yellow ,3Pink (Patient) ,4.White
mat = np.zeros((6,6))
for i in range(5):
	
		mask=cv2.inRange(img, colorBGR[i], colorBGR[i])
		Gmask = cv2.bitwise_and(img, img, mask = mask)
		cv2.imshow("test", Gmask)
		contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			# print('{} {}'.format(cx, cy))
			x = math.ceil(cx/60) - 1
			y = math.ceil(cy/60) - 1
			if(cv2.contourArea(cnt)>2000.0):
				mat[y][x] = i+1
			if(i==4):
				for cnt in contours:

					print(cv2.contourArea(cnt))
	
print(mat)
print(len(contours))
cv2.drawContours(img, contours, 0, (50,50,50),3)
cv2.imshow('original', img)
cv2.imshow('maskg', Gmask)
cv2.waitKey(0)	
