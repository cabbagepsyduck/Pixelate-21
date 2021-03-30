import gym
import pix_sample_arena
import time
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np
import math
from collections import defaultdict
from numpy import inf



env = gym.make("pix_sample_arena-v0")
time.sleep(3)


img = env.camera_feed()
img = img[100:412,100:412]
img = cv2.resize(img, (360,360))
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
colorBGR = np.array([[227,227,227],[0,227,0],[0,227,227],[0,0,145],[211,114,211],[227,227,0]])  #0.White , 1.Green , 2.Yellow ,3.Red,4.Pink (Patient) ,4.White
mat = np.zeros((6,6))																	# 5.Blue
for i in range(6):
	
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
				if(i==5):
					mat[y][x] = 1	
				else :
					mat[y][x] = i+1
			if(i==4):
				for cnt in contours:

					print(cv2.contourArea(cnt))
	
print(mat)
print(len(contours))
graph = defaultdict(dict)
costs = {}

for i in range(6):
	for j in range(6):
		if j<5:
			if mat[i][j+1] > 0:
				n=str(i)+ str(j)
				m=str(i)+ str(j+1)
				item = {m : mat[i][j+1]}
				graph[n].update(item)
		if j>0:
			if mat[i][j-1] > 0:
				n=str(i)+ str(j)
				m=str(i)+ str(j-1)
				item = {m : mat[i][j-1]}
				graph[n].update(item)
		if i<5:
			if mat[i+1][j] > 0:
				n=str(i)+ str(j)
				m=str(i+1)+ str(j)
				item = {m : mat[i+1][j]}
				graph[n].update(item)
		if i>0:
			if mat[i-1][j] > 0:
				n=str(i)+ str(j)
				m=str(i-1)+ str(j)
				item = {m : mat[i-1][j]}
				graph[n].update(item)
print(graph)

for nodes in graph:
	costs[nodes] = inf	
	costs['00'] = 0

print(costs)

parents = {}

def search(source, target, graph, costs, parents):
    
    nextNode = source
    
    while nextNode != target:
        
        for neighbor in graph[nextNode]:
            
            if graph[nextNode][neighbor] + costs[nextNode] < costs[neighbor]:
                
                costs[neighbor] = graph[nextNode][neighbor] + costs[nextNode]
                
                parents[neighbor] = nextNode
                
            del graph[neighbor][nextNode]
            
        del costs[nextNode]
        
        nextNode = min(costs, key=costs.get)
        
    return parents

result = search('00', '12', graph, costs, parents)

def backpedal(source, target, searchResult):
    
    node = target
    
    backpath = [target]
    
    path = []
    
    while node != source:
        
        backpath.append(searchResult[node])
        
        node = searchResult[node]
        
    for i in range(len(backpath)):
        
        path.append(backpath[-i - 1])
        
    return path

print('parent dictionary={}'.format(result))

print('longest path={}'.format(backpedal('00', '12', result)))
