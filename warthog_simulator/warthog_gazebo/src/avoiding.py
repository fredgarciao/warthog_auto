#!/usr/bin/env python
import rospy
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class laser():

    def __init__(self):

        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                    Twist, queue_size=10)
        self._laser = rospy.Subscriber("/warthog/laser/scan", LaserScan, self.callback)
        self.lista = []


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
                velocity.angular.z = -5
                self.velocity_publisher.publish(velocity)


if __name__ == "__main__":    
    rospy.init_node('lasertest', anonymous=True)                           
    la = laser()
    while not rospy.is_shutdown():        
        la.velo()