#!/usr/bin/env python

import rospy
import numpy as np
from util import point, dilate, dilate_subset
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray
import random


cache = {'Occupancy' : None , 'dilatedGrid' : []}

class occupancyGridSubscriber() :
    def __init__( self ) :
        self.init_node()
        self.init_Subscriber()

    
    def init_node(self) :
        rospy.init_node('gridSub', anonymous=True)

    def init_Subscriber(self) :
        rospy.Subscriber("map", OccupancyGrid, self.callback)
        rospy.spin()

    def callback(self, data) :
        '''
            INFO:
                --> resolution = 0.05   width = 384     height = 384
                --> start x: -10.0,     y: -10.0,   z:0.0
        '''
        if cache['Occupancy'] == None :
            cache['Occupancy'] = [0]*len(data.data)


        step_diff = [a for b,a in zip(data.data,cache['Occupancy']) if a!=b]
        step_diff = []
        new_100s = []
        for i in range(len(data.data)) :
            if data.data[i]!= cache['Occupancy'][i] :
                step_diff += [data.data[i]]
                if data.data[i] == 100 :
                    new_100s += [i]
        rospy.loginfo(new_100s)

        if len(step_diff) > 10 :
            rospy.loginfo('Updating cache')
            cache['Occupancy'] = data.data

            occuGrid = np.fromiter(data.data, int).reshape(384,384)  # for big 1D array np.fromiter is faster than np.asarray

            if len(cache['dilatedGrid']) == 0 :
                cache['dilatedGrid'] = occuGrid

            new_100s = [(int(i/384),i%384 ) for i in new_100s]
            cache['dilatedGrid'] = dilate_subset(cache['dilatedGrid'], (3,3), new_100s)
            
            self.publish_occupied_points(cache['dilatedGrid'])
        else : rospy.loginfo('callback already cached...')
        

    def publish_occupied_points(self, Grid) :
        a,b = Grid.shape
        msg = PoseArray()
        points = []
        for r in range(a) :
            for c in range(b) :
                if Grid[r][c] == 100 :
                    p = Pose()
                    p.position.x = ((c/384)*19.2) - 10
                    p.position.y = ((r/384)*19.2) - 10
                    p.position.z = 0
                    points.append(p)
        msg.poses = points
        pub = rospy.Publisher('points', PoseArray, queue_size=10)
        pub.publish(msg)
        rospy.loginfo('running....')
        


if __name__ == '__main__' :
    grid = occupancyGridSubscriber()


