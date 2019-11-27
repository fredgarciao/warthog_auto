#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool
class Move():
    def __init__(self):
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.sub_result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.callback_result)
        self.sub_aruco_found = rospy.Subscriber("/warthog/aruco", Bool, self.callback_aruco_found)
        self.result = True
        self.aruco_found = False
        self.pose = [[2, -5], [6, -4], [5,  8], [-6, -4],[0, 6], [0, 0]]
        self.pose_id = 0
    def callback_result(self, msg):
        self.result = True
    def callback_aruco_found(self, data):
        self.aruco_found = data
    def move_base(self):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.orientation.w = 0.2
        rospy.sleep(1)
        if self.result and self.pose_id < len(self.pose):
            self.result = False
            goal.pose.position.x = self.pose[self.pose_id][0]
            goal.pose.position.y = self.pose[self.pose_id][1]
            self.pose_id +=1
            self.pub_goal.publish(goal)
            rospy.loginfo("Moving to pose " + str(self.pose_id))
if __name__ == "__main__":
    rospy.init_node("move_node",anonymous=True)
    move = Move()
    while not rospy.is_shutdown():
        move.move_base()
        if move.aruco_found:
            break