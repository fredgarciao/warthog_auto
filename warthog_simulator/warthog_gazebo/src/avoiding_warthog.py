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
from std_msgs.msg import Bool
import time

class run:
  def __init__(self):
    # Place to goal
    self.goal_pose_pub = rospy.Publisher("/goal_pose_aruco", Point, queue_size=10)
    self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                Twist, queue_size=10)
    self._laser = rospy.Subscriber("/warthog/laser/scan", LaserScan, self.callback_laser)

    self.pub_movegoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    self.flag_1_sub = rospy.Subscriber("/flag_1" , Bool, self.callback_flag_1)
    self.flag_2_sub = rospy.Subscriber("/flag_2" , Bool, self.callback_flag_2)
    self.lista = []
    # Variables
    self.flag_1 = Bool()
    self.flag_1 = False
    self.flag_2 = Bool()
    self.flag_2 = False
    self.pose1 = [0,0]
    self.pose2 = [2,2]

    # self.time = rospy.Time.from_sec(time.time())

    # self.right()

  def callback_flag_1 (self,data):
    self.flag_1 = data.data
  def callback_flag_2 (self, data):
    self.flag_2 = data.data
  def callback_laser (self, msg):
    self.lista = list(msg.ranges)
  # Function to go right
  def right (self):
      velocity = Twist()
      # for self.i in range (80000):
      if (self.time < 2): 
        velocity = Twist()
        velocity.linear.x = 0.6
        velocity.angular.z = -3.6
        self.velocity_publisher.publish(velocity)
        print (self.time)

  def velo (self):
    if self.flag_1 == False:
      velocity = Twist()
      for x in self.lista:
          if x >= 2.2: 
              # print("oi")
              velocity.linear.x = 0.5
              self.velocity_publisher.publish(velocity)
          else:
              velocity.linear.x = 0.6
              velocity.angular.z = -3.6
              self.velocity_publisher.publish(velocity)
  def goal (self):
    if self.flag_1 == True:
      print("Aruco 1")
      goal = PoseStamped()
      goal.header.frame_id = "odom"
      goal.pose.orientation.w = 0.2
      goal.pose.position.x = self.pose1[0]
      goal.pose.position.y = self.pose1[1]
      self.pub_movegoal.publish(goal)
      rospy.spin()
    if self.flag_2 == True:
      print("Aruco 2")
      goal = PoseStamped()
      goal.header.frame_id = "odom"
      goal.pose.orientation.w = 0.2
      goal.pose.position.x = self.pose2[0]
      goal.pose.position.y = self.pose2[1]
      self.pub_movegoal.publish(goal)
      rospy.spin()



  # def aruco_not (self):
  #   velocity = Twist()
  #   if self.flag_2 == False and self.pose1 == [0,0]:
  #     velocity.linear.x = 0.6
  #     velocity.angular.z = -1.6
  #     self.velocity_publisher.publish(velocity)
  # def get_position (self):
  #     if self.flag_1 == True:
  #       print ("oi")
if __name__ == '__main__':
    # main(sys.argv)
    rospy.init_node('image_converter', anonymous=True)
    ic = run()
    while not rospy.is_shutdown():
        ic.velo()
        ic.goal()
