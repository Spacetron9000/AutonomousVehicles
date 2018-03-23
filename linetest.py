import numpy as np
import cv2

# Read in the image desired

#image = cv2.imread("solidYellowCurve.jpg")
video = cv2.VideoCapture("vidja.avi")

newvid = cv2.VideoWriter('testvideo.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 29, (int(video.get(3)),int(video.get(4))))


while(video.isOpened()):
     idk, image = video.read()
     image3 = image.copy()
     #Find the edges of the image which will be used to find the lane lines.
     greyimage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
     blurredimage = cv2.GaussianBlur(greyimage,(5,5),0)
     edgesimage = cv2.Canny(blurredimage, 90, 180)
    

 

     #Define four points for a trapezoid Then it fills that trapezoid and uses that to create a mask for a copy of the image
     
     left_bottom =[image.shape[1]/9,image.shape[0]]
     left_top=[200,image.shape[0]/2]
     right_bottom=[8.7*image.shape[1]/9,image.shape[0]]
     right_top =[530,image.shape[0]/2]
     vertices = np.array([[left_bottom, left_top, right_top, right_bottom]], dtype=np.int32)
     image2 = image.copy() #Need to copy the image before I fill it. 
     cv2.fillPoly(image,vertices,255) # this 
     mask=cv2.inRange(image,np.array([255,0,0]),np.array([255,0,0]))
     
     testimage = cv2.bitwise_and(image2,image2, mask=mask) 

     hsv = cv2.cvtColor(testimage, cv2.COLOR_BGR2HSV)
     lowerblue = np.array([90,60,50])
     upperblue = np.array([130,255,255])
     bluemask = cv2.inRange(hsv, lowerblue, upperblue)
     blueimage = cv2.bitwise_and(testimage,testimage, mask = bluemask)


     newvid.write(blueimage)
     if idk == True:
          cv2.imshow('frame', blueimage)
          if cv2.waitKey(25) & 0xFF == ord('q'):
              break
     else:
          break

video.release()
newvid.release()









