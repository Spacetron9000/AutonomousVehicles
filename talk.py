import rospy as rp
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def talker():
  left=rp.Publisher("left", Image, queue_size=10)
  right=rp.Publisher("right", Image, queue_size=10)
  rp.init_node('image_converter', anonymous=True)
  rate=rp.Rate(60)
  bridge=CvBridge()
  cap=cv2.VideoCapture(0)
  while not rp.is_shutdown():
    ret,img=cap.read()
    left_i=img[0:367,0:672]
    right_i=img[0:367,672:1344]
    left.publish(bridge.cv2_to_imgmsg(left_i, encoding="passthrough"))
    right.publish(bridge.cv2_to_imgmsg(right_i, encoding="passthrough"))
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rp.ROSInterruptException:
    pass
