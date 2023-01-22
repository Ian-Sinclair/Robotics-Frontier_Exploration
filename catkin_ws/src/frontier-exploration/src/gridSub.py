#!/usr/bin/env python

import rospy
import numpy as np
import util
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray

class occupancyGridSubscriber() :
    cache = {'Occupancy' : None , 
             'dilatedGrid' : [] , 
             'frontierGrid' : []}
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


        #step_diff = [a for b,a in zip(data.data,occupancyGridSubscriber.cache['Occupancy']) if a!=b]
        step_diff = []
        new_100s = []
        new_0s = []
        for i in range(len(data.data)) :
            if data.data[i] != occupancyGridSubscriber.cache['Occupancy'][i] :
                step_diff += [data.data[i]]
                if data.data[i] == 100 :
                    new_100s += [i]
                if data.data[i] == -1 :
                    new_0s += [i]
        print(new_0s)

        if len(step_diff) > 0 :
            rospy.loginfo('Updating cache')
            occupancyGridSubscriber.cache['Occupancy'] = data.data

            occuGrid = np.fromiter(data.data, int).reshape(384,384)  # for big 1D array np.fromiter is faster than np.asarray

            if len(occupancyGridSubscriber.cache['dilatedGrid']) == 0 :
                occupancyGridSubscriber.cache['dilatedGrid'] = occuGrid

            new_100s = [(int(i/384),i%384 ) for i in new_100s]
            new_0s = [(int(i/384),i%384 ) for i in new_0s]
            occupancyGridSubscriber.cache['dilatedGrid'] = util.dilate_subset(occupancyGridSubscriber.cache['dilatedGrid'], (3,3), new_100s)
            
            #  Finding frontiers
            occupancyGridSubscriber.cache['frontierGrid'] = util.edge_detection(occuGrid , occupancyGridSubscriber.cache['dilatedGrid'])
            frontierGrid = occupancyGridSubscriber.cache['frontierGrid'].flatten()

            dilatedOccuGrid = occupancyGridSubscriber.cache['dilatedGrid'].flatten()
            obstacles_pub = rospy.Publisher('dilatedOccupancyGrid', OccupancyGrid, queue_size=1)
            #obstacles_pub.publish(data.header, data.info, dilatedOccuGrid)
            obstacles_pub.publish(data.header, data.info, frontierGrid)

        else : rospy.loginfo('Callback received but already cached')        


if __name__ == '__main__' :
    grid = occupancyGridSubscriber()


