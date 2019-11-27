#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
class laser():
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                    Twist, queue_size=10)
        self._laser = rospy.Subscriber("/warthog/laser/scan", LaserScan, self.callback)
        self.lista = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/warthog/camera/image_raw",Image,self.callback)
        #dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        #dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    def callback (self, msg):
        self.lista = list(msg.ranges)
    def velo (self):
        velocity = Twist()
        for x in self.lista:
            if x >= 2.0:
                velocity.linear.x = 0.5
                self.velocity_publisher.publish(velocity)
            else:
                velocity.linear.x = 0.6
                velocity.angular.z = 5
                self.velocity_publisher.publish(velocity)
# A classe Image é para a identificação do Aruco.

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
        res = cv2.aruco.detectMarkers(gray, self.dictionary)
        # if res[1][0] == 99:
            # print("oi")
        #cv2.namedWindow('cv_image', cv2.WINDOW_NORMAL)
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit()
if __name__ == "__main__":    
    rospy.init_node('lasertest', anonymous=True)                           
    la = laser()

    while not rospy.is_shutdown():        
        la.velo()












