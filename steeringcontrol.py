#Get input from the ZED camera (or from our test video)

#Error will be the pixel distance from where the lane is to where we want it
#to be (Region of Interest)

#We'll algorithmically control the speed of the turn in order to make sure
#it's turning smoothly and not in a jerky manner

#if the lane is to the left of the ROI, a positive signal (left turn) will
#be sent to the ackermann steer control topic

#otherwise, negative (right)

#then we'll need to tune the constants (Kp, Ki, Kd) in order to get it
#turning smoothly

#output will be published to the steering topic AND printed to the terminal

import rospy as rp
import cv2
import math
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

'''
def whateverSideTheLaneIsOn(img, boundL, boundR, hl, sl, vl, hh, sh, vh):
	scanningRow = 1
	foundit = False
	counter1 = boundL #starting pixel column
	counter2 = boundR
	fff = True
	while (fff):
		if (img[scanningRow, counter1] >= [hl,sl,vl] && img [scanningRow, counter1] <= [hh, sh, vh]):
			return 1
		if (img[scanningRow, counter2] >= [hl,sl,vl] && img [scanningRow, counter2] <= [hh, sh, vh]):
			return -1

		counter1-=1
		counter2+=1
'''


'''

def findMeSomeLines(img, hl, sl, vl, hh, sh, vh):
	x = 0
	lline = -1
	rline = -1
	while( x < 672 ):
		if (img[row,x] >= [hl, sl, vl] && img[row,x] <= [hh, sh, vh]):
			lline = x
			break
		x+=1
	while( x < 672 ):
		if (img[row,x] <= [hl, sl, vl] || img[row,x] >= [hh, sh, vh]):
			rline = x
			break
		x+=1
	return {lline, rline}
'''

def scanPixelLength(img, leftLine, rightLine, hl, sl, vl, hh, sh, vh):

#Start: starting column

#hsv(l,h): pretty self explanatory. the hsv value of the pixel

	scanningRow = 0 #Dummy Value, change according to whereabout the ROI is
	foundit = False
	pixellength = 0
	endOfScreenLength = 100	#this is a dummy value to tell the function to stop if there are no blue pixels in the ROI


	counterL = leftLine #the starting pixel columns
	counterR = rightLine

	#Loop will continuously search left and right for the lane, and return direction to turn
	while (foundit == False):
		if (img[scanningRow, counterL] >= [hl,sl,vl] && img [scanningRow, counterL] <= [hh, sh, vh]):
			foundit = True
			direction = 1

		if (img[scanningRow, counterR] >= [hl,sl,vl] && img [scanningRow, counterR] <= [hh, sh, vh]):
			foundit = True
			direction = -1

		if (pixellength > endOfScreenLength)
			return (-1,0)

		counterL+=1
		counterR-=1

		pixellength+=1

	return (pixellength,direction)

def pidLoop ():

	kp = 0
	ki = 0
	kd = 0

	proportional_error = 0
	integral_error = 0
	derivative_error = 0

	previous_error = 0
	outputAngle = 0

	while (1):

		#Fix the following line with appropriate values
		proportional_error, direction = scanPixelLength(image, 100,400, 90, 60, 50, 130, 255, 255)
		integral_error = integral_error + proportional_error

		if (proportional_error == 0):
			integral_error = 0
		if (proportional_error > 30):
			integral_error= 0

		derivative_error = proportional_error - previous_error
		previous_error = proportional_error

		outputAngle = direction*(kp*proportional_error+ki*integral_error+kd*derivative_error)

		#publish the angle to the steering system




#Function to determine if there's no lane on the camera at all









#Image receiving

class image_sub:
    def __init__(self):
        self.image_sub = rospy.Subscriber("zedLeft", Image, self.callback)

    def callback(self, data):
        #print("callback")
        cv2.imshow('cv_somethign',cv2.imdecode('.jpg', data))





#Main Function
def main(args):
    rospy.init_node('image_feature', anonymous=True)
    ic = image_sub()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
