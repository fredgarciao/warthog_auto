#!/usr/bin/env python
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
class image_converter:
  def __init__(self):
    self.flag_1_pub = rospy.Publisher("/flag_1", Bool,  queue_size=1)
    self.flag_2_pub = rospy.Publisher("/flag_2", Bool,  queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/warthog/camera/image_raw",Image,self.callback)
    self.flag_1 = Bool()
    self.flag_1 = False
    self.flag_2 = Bool()
    self.flag_2 = False
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    # Variable of image
    frame = cv_image
    # Converting to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Using the dictionary of arucos
    res = cv2.aruco.detectMarkers(gray, dictionary)
    # Logic to not send empty values
    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
        #logic to reconize
        if res[1][0] == 100:
          self.flag_1 = True
          self.flag_1_pub.publish(self.flag_1)
          self.flag_2 = False
          self.flag_2_pub.publish(self.flag_2)
          cv2.imwrite('/home/fair/warthog_ws/src/warthog_auto/Pictures/aruco_1.jpeg', cv_image)
        if res [1][0] == 99:
          self.flag_2 = True
          self.flag_2_pub.publish(self.flag_2)
          # self.flag_1 = False
          # self.flag_1_pub.publish(self.flag_1)
    cv2.imshow('frame',cv_image)
    cv2.waitKey(3)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)
