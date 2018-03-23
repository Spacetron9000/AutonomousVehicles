import rospy as rp
import numpy as np
import cv2
from sensor_msgs.msg import Image
#from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# Read in the image desired

#preprocess image

video = cv2.VideoCapture("vidja.avi")



def scanPixelLength(img1, leftLine, rightLine):

#Start: starting column

    img = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    scanningRow = 185 #Dummy Value, change according to whereabout the ROI is
    foundit = False
    pixellength = 0
    endOfScreenLength = 250	#this is a dummy value to tell the function to stop if there are no blue pixels in the ROI

    counterL = leftLine #the starting pixel columns
    counterR = rightLine


	#Loop will continuously search left and right for the lane, and return direction to turn
    while foundit == False:
        scanPixelL=img[scanningRow,counterL]
        scanPixelR=img[scanningRow,counterR]

        if (np.all(scanPixelL > (90,60,50)) and np.all(scanPixelL < (130,255,255))):
        #    print (scanPixelL[0], scanPixelL[1])
            foundit = True
            direction = 1

            print ("The lane is on the left!")

        elif (np.all(scanPixelR > (90,60,50)) and np.all(scanPixelR < (130,255,255))):
            #print(scanPixelR)
            foundit = True
            direction = -1
            print ("The lane is on the right!")

        elif (pixellength > endOfScreenLength):
            print ("There's no lane on the screen!")
            return (-1,0)



        counterL = counterL - 1
        counterR = counterR + 1
        #print (counterL, counterR)

        pixellength+=1

    #print ("The lane is "+ str(pixellength)+" pixels away.")

    return (pixellength,direction)


def process_image(img):
    greyimage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurredimage = cv2.GaussianBlur(greyimage,(5,5),0)
    edgesimage = cv2.Canny(blurredimage, 90, 180)
    left_bottom =[img.shape[1]/9,img.shape[0]]
    left_top=[200,img.shape[0]/2]
    right_bottom=[8.7*img.shape[1]/9,img.shape[0]]
    right_top =[530,img.shape[0]/2]
    vertices = np.array([[left_bottom, left_top, right_top, right_bottom]], dtype=np.int32)
    image2 = img.copy() #Need to copy the image before I fill it.
    cv2.fillPoly(img,vertices,255) # this
    mask=cv2.inRange(img,np.array([255,0,0]),np.array([255,0,0]))

    testimage = cv2.bitwise_and(image2,image2, mask=mask)

    hsv = cv2.cvtColor(testimage, cv2.COLOR_BGR2HSV)
    lowerblue = np.array([90,60,50])
    upperblue = np.array([130,255,255])
    bluemask = cv2.inRange(hsv, lowerblue, upperblue)
    blueimage = cv2.bitwise_and(testimage,testimage, mask = bluemask)

    return blueimage



#MAIN LOOP TO RUN PID
def pid():

    frame_count = 0
    kp = 0.3
    ki = 0.1
    kd = 0.01

    proportional_error = 0
    integral_error = 0
    derivative_error = 0
    previous_error = 0
    outputAngle = 0

    while True:

        frame_count+=1
        idk, image = video.read()
        blue_img = process_image(image)

        #Fix the following line with appropriate values
        proportional_error, direction = scanPixelLength(image, 285,335)
        integral_error = integral_error + proportional_error

        if (proportional_error == 0):
            integral_error = 0
        if (proportional_error > 30):
            integral_error= 0

        derivative_error = proportional_error - previous_error
        previous_error = proportional_error

        outputAngle = direction*(kp*proportional_error+ki*integral_error+kd*derivative_error)


        print(direction, proportional_error,integral_error,derivative_error,outputAngle)

		#publish the angle to the steering system


        if frame_count == video.get(cv2.CAP_PROP_FRAME_COUNT):
            frame_count = 0
            video.set(cv2.CAP_PROP_POS_FRAMES,0)


        ch = cv2.waitKey(5)


        cv2.imshow("frame",blue_img)
        if(ch == 27):
            break

    video.release()


pid()
