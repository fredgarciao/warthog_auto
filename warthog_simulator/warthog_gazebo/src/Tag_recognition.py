#!/usr/bin/env python
from __future__ import print_function
import sys
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult


#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)


class image_converter:


  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,  queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/warthog/camera/image_raw",Image,self.callback)
    # Place to goal
    self.goal_pose_pub = rospy.Publisher("/goal_pose_aruco", Point, queue_size=10)

    self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                Twist, queue_size=10)
    self._laser = rospy.Subscriber("/warthog/laser/scan", LaserScan, self.callback_laser)

    self.pub_movegoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    self.lista = []

    self.ama = False
    self.fred = [0,0]


  def callback_laser (self, msg):
    self.lista = list(msg.ranges)
    
  
  def velo (self):
    if self.ama == False:
      velocity = Twist()
      for x in self.lista:

          if x >= 2.5:
              velocity.linear.x = 0.5

              self.velocity_publisher.publish(velocity)
          else:
              velocity.linear.x = 0.6
              velocity.angular.z = -3.6
              self.velocity_publisher.publish(velocity)

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
          self.ama = True           

    cv2.imshow('frame',cv_image)
    cv2.waitKey(3)
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
  def stop (self):
    if self.ama == True:
      goal = PoseStamped()
      goal.header.frame_id = "odom"
      goal.pose.orientation.w = 0.2
      goal.pose.position.x = self.fred[0]
      goal.pose.position.y = self.fred[1]
      self.pub_movegoal.publish(goal)
      rospy.spin()

# def main(args):
#   rospy.init_node('image_converter', anonymous=True)
#   ic = image_converter()
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()

if __name__ == '__main__':
    # main(sys.argv)        
    rospy.init_node('image_converter', anonymous=True)                   
    ic = image_converter()

    while not rospy.is_shutdown():        
        ic.velo()
        ic.stop()
