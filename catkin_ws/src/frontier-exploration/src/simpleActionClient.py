#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from threading import Thread
from geometry_msgs.msg import Pose


class ActionClient() :
    def __init__(self):
        self.init_node()
        self.init_action_client()

    def init_node(self):
        rospy.init_node('simpleActionClient')
    
    def init_action_client(self) :
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
    
    def coordinate_callback(self, x, y) :
        thread = Thread(target=self.coordinate_callback_thread, args=(x, y))
        thread.start()

    def coordinate_callback_thread(self, x, y) :
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 1.0
        goal = MoveBaseGoal()
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        self.client.wait_for_result()


if __name__ == '__main__' :
    goalSet = ActionClient()
    goalSet.init_node()
    goalSet.init_action_client()
    goalSet.coordinate_callback(1.0,1.0)



