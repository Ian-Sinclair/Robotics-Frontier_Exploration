#!/usr/bin/env python

import rospy
import actionlib
import getopt, sys

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from threading import Thread
from geometry_msgs.msg import Pose


'''
    Action Client to move the base of the robot relative to its own frame. (base_footprint)
    This means that translations are localized to the robot, x = 1, y = 1 will move the robot up 1
    and right 1 unit. Not to the map coordinate (1,1).
    To create a moveActionClient initialize the class to a variable and call coordinate_callback.
    Parameters are translation in (x,y,z,w) and rotation (nx,ny,nz).

    The main client program takes input arguments for x and y from the terminal.
    Examples of this are,
    $ rosrun frontier-exploration moveActionClient -x <translation in x> -y <translation in y>
'''

class moveActionClient() :
    def __init__(self):
        '''
            Inits the internal node and action client with rospy.
        '''
        self.init_node()
        self.init_action_client()

    def init_node(self):
        '''
            Rospy node initiation
        '''
        rospy.init_node('simpleActionClient')
    
    def init_action_client(self) :
        '''
            Initializes rospy action client to /move_base topic.
        '''
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
    
    def coordinate_callback(self, x = 0, y = 0, z = 0, w = 1, nx = 0, ny = 0, nz = 1) :
        '''
            Starts a thread for translation with coordinate callback thread function.
        '''
        thread = Thread(target=self.coordinate_callback_thread, args=(x, y, z, w, nx, ny, nz))
        thread.start()

    def coordinate_callback_thread(self, x = 0, y = 0, z = 0, w = 1, nx = 0, ny = 0, nz = 1) :
        '''
            Creates a pose variable with translation information,
            then sends that translation to the robot with respect to
            the base_footprint frame.
        '''
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = nx
        pose.orientation.y = ny
        pose.orientation.z = nz
        pose.orientation.w = w
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        self.client.wait_for_result()

    
def main(argv) :
    '''
        Main client to accept robot translation information from the terminal.
    '''
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
    
    goalSet.coordinate_callback(x=x,y=y)

if __name__ == '__main__' :
    '''
        #####  Input Key #####
        -x : goal position in x (from base frame of robot)
        -y : goal position in y (from base frame of robot)
    '''
    main(sys.argv[1:])

