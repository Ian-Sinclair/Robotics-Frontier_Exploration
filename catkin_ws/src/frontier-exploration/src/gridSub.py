#!/usr/bin/env python

import rospy
import numpy as np
import util
import random
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray

class occupancyGridSubscriber() :
    cache = {'Occupancy' : None , 
             'frontierGrid' : [],
             'obstacle_record' : set([])}
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
                    occupancyGridSubscriber.cache['obstacle_record'].add(i)

        if len(step_diff) > 0 :
            rospy.loginfo('Updating cache')
            occupancyGridSubscriber.cache['Occupancy'] = data.data

            occupancyGrid = np.fromiter(data.data, int).reshape(384,384)  # for big 1D array np.fromiter is faster than np.asarray

            obstacles = [*map(lambda x: (int(x/384),x%384 ), occupancyGridSubscriber.cache['obstacle_record'])]

            ExpandedOccupancyGrid, _ = util.informed_dilate(occupancyGrid, (3,3), obstacles)
            
            #  Finding frontiers
            frontiersGrid, frontiersPoints = util.edge_detection(ExpandedOccupancyGrid)

            #  Remove outlier points.
            frontiersGrid, frontiersPoints = util.informed_erode(frontiersGrid,frontiersPoints,(2,2),ExpandedOccupancyGrid, tr=15)

            #  Expand frontiers.
            frontiersGrid, _ = util.informed_dilate(frontiersGrid,(1,1), frontiersPoints)

            frontiers = util.connection_component_analysis(frontiersGrid , frontiersPoints , (2,2))
            print(len(frontiers))

            frontiersGrid = frontiersGrid.flatten()

            #  Need to publish frontiers occupancy grid.
            pub = rospy.Publisher('/frontiers_map' , OccupancyGrid, queue_size=1)
            pub.publish(data.header,data.info,frontiersGrid)
            rospy.loginfo('Publishing')

            #  make markers
            map_frontiers = [util.tf_occuGrid_to_map(f) for f in frontiers]

            publish_deletaALLMarkers(namespace='frontier_points')
            publish_deletaALLMarkers(namespace='centroids')

            maps = MarkerArray()

            maps.markers = [
                            convert_marker(f, 
                                            r=random.random() ,
                                            g=random.random() ,
                                            b=random.random() , 
                                            id=i ,
                                            namespace='frontier_points'
                                            ) 
                                            for i,f in enumerate(map_frontiers)
                                            ]

            #  get centroids
            centroids = [util.get_centroid(f) for f in map_frontiers]

            centroid_RviZ = MarkerArray()

            centroid_RviZ.markers = [
                                    convert_marker(points=[f],
                                                    r=0,
                                                    g=1,
                                                    b=0, 
                                                    id=i+len(maps.markers), 
                                                    sx=0.25,
                                                    sy=0.25,
                                                    sz=0.25, 
                                                    type=7,
                                                    namespace='centroids'
                                                    ) 
                                                    for i,f in enumerate(centroids)
                                                    ]

            maps.markers = centroid_RviZ.markers + maps.markers


            frontiersPub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
            frontiersPub.publish(maps)


        else : rospy.loginfo('Callback received but already cached')        




def publish_deletaALLMarkers(topic = '/visualization_marker_array', namespace = 'marker') :
    marker_array = MarkerArray()
    marker = Marker()
    marker.id = 0
    marker.ns = namespace
    marker.action = Marker.DELETEALL
    marker_array.markers = [marker]
    pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    pub.publish(marker_array)




def convert_marker(points = None, 
                    w=1 , 
                    nx=0 , 
                    ny=0 , 
                    nz=0 , 
                    r=1.0, 
                    g=0.0, 
                    b=0.0, 
                    alpha=1.0, 
                    id=0, 
                    namespace='marker', 
                    type = 8, 
                    sx = 0.03, 
                    sy = 0.03, 
                    sz = 0.03 ) :
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # marker type = 8 should be a point, refer to the msg/Marker config for details.
    marker.ns = namespace
    marker.type = type
    marker.id = id

    marker.pose.orientation.x = nx
    marker.pose.orientation.y = ny
    marker.pose.orientation.z = nz
    marker.pose.orientation.w = w

    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    # Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = float(r)
    marker.color.g = float(g)
    marker.color.b = float(b)
    marker.color.a = float(alpha)

    if points == None :
        rospy.logerr('list of points need not be none in RviZ Publisher -> pub_point')
        return None

    if points is not None : 
        marker.points = points


    marker.lifetime = rospy.Duration()

    return marker






if __name__ == '__main__' :
    grid = occupancyGridSubscriber()
    #publish_deletaALLMarkers()


