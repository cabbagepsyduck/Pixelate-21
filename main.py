import gym
import pix_sample_arena
import time
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np



if __name__=="__main__":
    lower = np.array([[0,227,0],[145,0,0],[227,227,0],[211,114,211]])  # 1.Green , 2.Red , 3.Yellow ,4. Pink (Patient)
    

    env = gym.make("pix_sample_arena-v0")
    time.sleep(3)
    img = env.camera_feed()
    img = img[100:412,100:412]
    cv2.imshow("img", img)
    # img = img[]
    img = cv2.resize(img, (600,600))
    # cv2.imshow("test", img2)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",hsv)

    mask = cv2.inRange(img , lower[0],lower[0])
    cv2.imshow("threshold", mask)
    Fmask = cv2.bitwise_and(img, img, mask = mask)
    # Fgreen[green == 0] = 255
    cv2.imshow("masked", Fmask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mat = np.zeros((6,6))                                                                  #Square area = 46
  

    print(cv2.contourArea(contours[0]))
    i=0
    j=0
    p=0

    for i in range(6):
        
        for j in range(6):     
            print(img[50 + 100*i,50+100*j])                   
            if(img[50 + 100*i,50+100*j].all()==lower[0].all()):
                mat[j][i]=1
            j=j+1 

    i=i+1






   
    print(mat)
    print(np.shape(img))
    # print(matrix)
    # cv2.drawContours(gray, contours, -1, (0, 255, 0), 3)
    # cv2.imshow("img2",gray)

  
    # i = 0




    



    cv2.waitKey(0)

    
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    time.sleep(100)


