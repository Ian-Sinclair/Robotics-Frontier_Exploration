#!/usr/bin/env python

import rospy
import numpy as np
import util
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray

class occupancyGridSubscriber() :
    cache = {'Occupancy' : None , 
             'frontierGrid' : [],
             'obstacle_record' : {}}
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
        if occupancyGridSubscriber.cache['Occupancy'] == None :
            occupancyGridSubscriber.cache['Occupancy'] = [0]*len(data.data)

        step_diff = []
        for i in range(len(data.data)) :
            if data.data[i] != occupancyGridSubscriber.cache['Occupancy'][i] :
                step_diff += [data.data[i]]
                if data.data[i] == 100 :
                    occupancyGridSubscriber.cache['obstacle_record'].add([i])

        if len(step_diff) > 0 :
            rospy.loginfo('Updating cache')
            occupancyGridSubscriber.cache['Occupancy'] = data.data

            occupancyGrid = np.fromiter(data.data, int).reshape(384,384)  # for big 1D array np.fromiter is faster than np.asarray

            obstacles = [map(lambda x: (int(x/384),x%384 ), occupancyGridSubscriber.cache['obstacle_record'])]

            ExpandedOccupancyGrid = util.informed_dilate(occupancyGrid, (3,3), obstacles)
            
            #  Finding frontiers
            frontiersGrid = util.edge_detection(ExpandedOccupancyGrid)
            frontiersGrid = frontiersGrid.flatten()

            #  Need to publish frontiers occupancy grid.


        else : rospy.loginfo('Callback received but already cached')        






if __name__ == '__main__' :
    grid = occupancyGridSubscriber()


