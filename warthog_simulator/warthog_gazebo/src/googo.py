#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool
class Move():
    def __init__(self):
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.ama = [-1,-15]
        
    def move_base(self):
        
            goal = PoseStamped()
            goal.header.frame_id = "odom"
            goal.pose.orientation.w = 0.2
            goal.pose.position.x = self.ama[0]
            goal.pose.position.y = self.ama[1]
            self.pub_goal.publish(goal)

            
            
if __name__ == "__main__":
    rospy.init_node("move_node",anonymous=True)
    move = Move()
    while not rospy.is_shutdown():
        move.move_base()
