import cv2.aruco as aruco
import cv2
import gym
import pix_sample_arena
import time
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np
import math



ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

rvecs, tvecs = None, None

board = aruco.GridBoard_create(
		markersX=2,
		markersY=2,
		markerLength=0.09,
		markerSeparation=0.01,
		dictionary=ARUCO_DICT)
def forward():
	env.move_husky(0.2, 0.2, 0.2, 0.2)

def reverse():
	env.move_husky(-0.2, -0.2, -0.2, -0.2)	

def clockwise():
	env.move_husky(0.3, -0.3, 0.3, -0.3)

def anti_clockwise():
	env.move_husky(-0.2, 0.2, -0.2, 0.2)


def facing(husky_front, husky_centre_coords):

	if(husky_front[1] < husky_centre_coords[1] and abs(husky_front[0]-husky_centre_coords[0])<40):
		return 'left'
	elif(husky_front[1] > husky_centre_coords[1] and abs(husky_front[0]-husky_centre_coords[0])<40):
		return 'right'
	elif(husky_front[0] < husky_centre_coords[0] and abs(husky_front[1]-husky_centre_coords[1])<40):
		return 'up'
	elif(husky_front[0] > husky_centre_coords[0] and abs(husky_front[1]-husky_centre_coords[1])<40):
		return 'down'
	else:
		return None


if __name__=="__main__":
  
	path = np.array([[5,5],[5,4],[5,3],[5,2],[4,2],[3,2]])
	pos = [0,0]
	i=0
	k=1
	env = gym.make("pix_sample_arena-v0")
	direction = None


    
	while True:


		p.stepSimulation()

		i=i+1
		

		if(i%500==0):
			img = env.camera_feed()
			img = img[80:432,80:432]
			img = cv2.resize(img, (360,360))
			# img = cv2.resize(img, (360,360))
			
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
			if ids is not None:
			            # Print corners and ids to the console
				for i, corner in zip(ids, corners):
					print('ID: {}; Corners: {}'.format(i, corner))

			            # Outline all of the markers detected in our image

			img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255))

			husky_centre_coords = [(corners[0][0][0][1]+corners[0][0][2][1])*0.5, (corners[0][0][0][0]+corners[0][0][2][0])*0.5]

			husky_centre = np.array([math.ceil((corners[0][0][0][1]+corners[0][0][2][1])/140), math.ceil((corners[0][0][0][0]+corners[0][0][2][0])/140)])

			husky_front = [(corners[0][0][0][1]+corners[0][0][1][1])*0.5,corners[0][0][0][0]]

			print(path[k],husky_centre, husky_centre_coords, husky_front, k)		# img = cv2.resize(img, (360,360))

			
			cv2.imshow("img", img)
			cv2.waitKey(1)
			
			

			if(path[k][1] == husky_centre[1]-1):
				direction = 'left'

			elif(path[k][1] == husky_centre[1]+1):
				direction = 'right'

			elif(path[k][0] == husky_centre[0]+1):
				direction = 'down'

			elif(path[k][0] == husky_centre[0]-1):
				direction = 'up'
			print(direction)	

			if(direction == 'left'):
				if(facing(husky_front,husky_centre_coords)=='left'):   #abs(husky_front[1]-husky_centre_coords[1])<50 and 

					if(husky_centre[0] != path[k][0] or husky_centre[1] != path[k][1]):
						
						forward()
						print('forward')
					else:
						
						k=k+1
				else:
					clockwise()


			elif(direction == 'right'):
				if(facing(husky_front,husky_centre_coords)=='right'):  #abs(husky_front[1]-husky_centre_coords[1])<50 and 
					if(husky_centre[0] != path[k][0] or husky_centre[1] != path[k][1]):
						forward()
					else:
						k=k+1
				else:
					clockwise()
					

			elif(direction == 'up'):
				if(facing(husky_front,husky_centre_coords)=='up'): #abs(husky_front[0]-husky_centre_coords[0])<50 and 
					if(husky_centre[0] != path[k][0] or husky_centre[1] != path[k][1]):
						forward()
						print('forward')
					else:
						k=k+1
				else:
					clockwise()
					


			elif(direction == 'down'):
				if(facing(husky_front,husky_centre_coords)=='down'):  #abs(husky_front[0]-husky_centre_coords[0])<50 and 
					if(husky_centre[0] != path[k][0] or husky_centre[1] != path[k][1]):
						forward()
					else:
						k=k+1
				else:
					clockwise()


    	
    	
