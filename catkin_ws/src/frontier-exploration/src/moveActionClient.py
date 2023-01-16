#!/usr/bin/env python

import rospy
import actionlib
import getopt, sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from threading import Thread
from geometry_msgs.msg import Pose


class moveActionClient() :
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

    
def main(argv) :
    '''
    #####  Input Key #####
    -x : goal position in x (from base frame of robot)
    -y : goal position in y (from base frame of robot)
    '''
    goalSet = moveActionClient()

    x = 0.0
    y = 0.0
    

    if len(argv) == 0 :
        rospy.loginfo('You can enter (x,y) commands in terminal\n \
            example, \n \t $ rosrun frontier-exploration moveActionClient -x <goal position in x> -y <goal position in y> \n')

    if len(argv) == 2 :
        try :
            x = float(argv[0])
            y = float(argv[1])
            argv = []
        except TypeError('Cannot convert x,y terminal to float') :
            sys.exit(2)


    try :
        opts, args = getopt.getopt(argv, "x:y:",[])
    except getopt.GetoptError:
      rospy.loginfo('Unaccepted input: \n \t -x <goal position in x> \n \t -y <goal position in y>')

    for opt , arg in opts :
        if opt == '-x' :
            if type(arg) not in [type(1), type(1.1), type('1')] :
                raise TypeError('x coordinate must be of type float or int')
            x = float(arg)
        if opt == '-y' :
            if type(arg) not in [type(1), type(1.1), type('1')] :
                raise TypeError('y coordinate must be of type float or int')
            y = float(arg)

    rospy.loginfo(f'applying translation goal: x = {x}, y = {y}')

    goalSet.coordinate_callback(x,y)

if __name__ == '__main__' :
    '''
    #####  Input Key #####
    -x : goal position in x (from base frame of robot)
    -y : goal position in y (from base frame of robot)
    '''
    main(sys.argv[1:])

