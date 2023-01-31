#!/usr/bin/env python

import rospy
import numpy as np
import util
import random
from moveActionClient import moveActionClient
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

'''
    Class to subscribe to the map occupancy grid and detect frontier clusters
    then publish the location and centroid of each frontier cluster.

    - A frontier is an area where known unoccupied space meets unknown space.
    - The goal of this script is to identify distinct frontier regions that
        can be used as goal for the robot to explore its environment.
    
    - Procedure :
        1) Subscribes to /map topic and takes an occupancy grid object
                        - when something is published on the /map topic is calls the
                            'callback' method in this class.

        2) Detects the location of known obstacles and increases their
            size based on the cspace of the robot (discovered a priori).
                        - This is done to prevent the algorithm from finding frontiers that
                            are to close to walls for the robot to explore.
                        - Increasing the wall size is done with a standard morphological algorithm, dilation. (in util.py) 

        3) Detect frontier regions with edge detection
                        - This is done with standard edge detection filtering
                            where an adjustable kernel is convoluted against 
                            the occupancy grid to detect locations where known
                            unoccupied tiles (0) meets known space (-1).
                            Walls are excluded as edges.

        4) Remove false regions and simplify frontiers topology
                        - the edge detection method may mis-classify sensor errors as a new frontier.
                            This is a case where a small unknown region is completely surrounded by 
                            known unoccupied space. And seems to be most commonly caused by gaps in the
                            lidar sensors on the robot. (So something should be visible but isn't, and will
                            likely become visible on the next time step.)
                        - To remove these regions, each frontier is eroded based on the amount of known space
                                that surrounds it. (If a region is next to a large block of unknown space it will not be
                                eroded. However, if a frontier is surrounded by known space, the size of its topology will 
                                be reduced and in the event of a false frontier, be eroded into nothingness.)
                        - After eroding, all remaining frontiers are dilated, (or have their area increased) to both fill holes
                                and join frontier candidates that are extremely close but not connected. This is done to prevent
                                over classification of frontier regions during connected component analysis.

        5 ) Frontier segmentation
                        - Here the list of all candidate frontier points are segmented into distinct clusters.
                        - Features of a good segmentation can include, a continuous topological region,
                            (or at least semi-continuous with small jumps.) distance from walls or in general
                            are reachable by the robot.
                        - Here connected component analysis is used to segment each frontier as semi-continuous 
                            topological regions. 
                                By semi-continuous we mean that the region is either fully connected or any gaps between 
                                components are 'small'.
                            This is done by running breadth first search (BFS) sequentially on each frontier point
                            candidate and connecting the visited points.
                            Successors in the BFS for each point are found by the neighbors of that point on a grid.
                            Neighbors are determined by overlaying an (n X n) kernel. 
'''


class occupancyGridSubscriber() :
    cache = { 'Occupancy' : None , 
             'frontierGrid' : [] ,
             'obstacle_record' : set([]) }
    def __init__( self ) :
        self.init_node()
        self.init_Subscriber()

    
    def init_node( self ) :
        rospy.init_node( 'gridSub' , anonymous = True )

    def init_Subscriber( self ) :
        rospy.Subscriber("map", OccupancyGrid, self.callback)
        rospy.spin()

    def callback( self , data ) : 
        '''
            callback function:
                Runs every time something is published to /map topic.
                Data is an occupancy grid.
        '''
        '''
            INFO: occupancy grid :: data
                --> resolution = 0.05   width = 384     height = 384
                --> start x: -10.0,     y: -10.0,   z:0.0
        '''
        #  Initializes the cache to compare callback calls between times steps. (1st time step only)
        if occupancyGridSubscriber.cache[ 'Occupancy' ] == None :
            occupancyGridSubscriber.cache[ 'Occupancy' ] = [0]*len( data.data )


        #  Determines if the occupancy grid changed sense the last time step.
        step_diff = []
        for i in range( len( data.data ) ) :
            if data.data[ i ] != occupancyGridSubscriber.cache[ 'Occupancy' ][ i ] :
                step_diff += [ data.data[ i ] ]
                if data.data[ i ] == 100 :  # might as well collect the location of obstacles too.
                    occupancyGridSubscriber.cache[ 'obstacle_record' ].add( i )

        if len( step_diff ) > 0 :
            #  Update cache with new occupancy grid
            rospy.loginfo('Updating cache')
            occupancyGridSubscriber.cache[ 'Occupancy' ] = data.data

            occupancyGrid = np.fromiter( data.data , int ).reshape(384 , 384)  #  converts occupancy grid to grid structure/ numpy array
            obstacles = [*map( lambda x: (int(x/384),x%384 ), occupancyGridSubscriber.cache['obstacle_record'])]

            #  Expands obstacles using approximate cspace of the robot
            ExpandedOccupancyGrid, _ = util.informed_dilate( grid = occupancyGrid , 
                                                             mask_size = (3,3) , 
                                                             array = obstacles )
            
            #  Finding Frontier Candidates
            frontiersGrid, frontiersPoints = util.edge_detection( Grid = ExpandedOccupancyGrid )

            #  Remove outlier points and false frontiers.
            frontiersGrid, frontiersPoints = util.informed_erode( Grid = frontiersGrid, 
                                                                  array = frontiersPoints, 
                                                                  mask_size = (2,2), 
                                                                  key = ExpandedOccupancyGrid, 
                                                                  tr=15 )

            #  Expand frontiers.
            frontiersGrid, _ = util.informed_dilate( grid = frontiersGrid, 
                                                     mask_size = (1,1) , 
                                                     array = frontiersPoints )

            #  Segment frontiers.      frontiers : list[list[]]
            frontiers = util.connection_component_analysis( grid = frontiersGrid , 
                                                            array = frontiersPoints , 
                                                            kernel_size = (2,2) )
            rospy.loginfo(f'Number of distinct frontiers detected. {len(frontiers)}')

                        
            #  Transforms frontier coordinates from occupancy grid frame to map frame.
            map_frontiers = [ util.tf_occuGrid_to_map( f ) for f in frontiers ]

            #  Gets centroid coordinates for each frontier cluster
            centroids = [ ( util.get_centroid( f ) , f ) for f in map_frontiers ]  #  returns a list of 'Point' types

            targets = sorted(centroids, key=lambda a: len( a[ 1 ] ) )

            goal = targets[-1][0]

            goal_client = moveActionClient()        
            rospy.loginfo('sending goal')
            print(goal)
            goal_client.coordinate_callback( x = goal[0] , y = goal[1] )


            frontiersGrid = frontiersGrid.flatten()

            #  Publishes occupancy grid of non-segmented frontier points.
            pub = rospy.Publisher( '/frontiers_map' , OccupancyGrid , queue_size=1 , latch=True )
            pub.publish( data.header, data.info , frontiersGrid )
            rospy.loginfo( 'Publishing' )

            #  Deletes all markers on a namespace
            publish_deletaALLMarkers( namespace='frontier_points' )
            publish_deletaALLMarkers( namespace='centroids' )

            #  Creates marker objects for frontier points.
            frontier_markerArray = MarkerArray()

            frontier_markerArray.markers = [
                                            convert_marker( f, 
                                                            r=random.random() ,
                                                            g=random.random() ,
                                                            b=random.random() , 
                                                            id=i ,
                                                            namespace='frontier_points' ) 
                                                            for i,f in enumerate(map_frontiers)
                                                            ]

            centroid_RviZ = MarkerArray()

            centroid_RviZ.markers = [
                                    convert_marker( points=[f[0]],
                                                    r=0,
                                                    g=1,
                                                    b=0, 
                                                    z=1,
                                                    id=i+len(frontier_markerArray.markers), 
                                                    sx=0.25,
                                                    sy=0.25,
                                                    sz=0.25, 
                                                    type=7,
                                                    namespace='centroids' ) 
                                                    for i,f in enumerate(targets)
                                                    ]

            frontier_markerArray.markers = centroid_RviZ.markers + frontier_markerArray.markers


            frontiersPub = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1 , latch=True )
            frontiersPub.publish( frontier_markerArray )


        else : rospy.loginfo( 'Callback received but already cached' )        



'''
    Method to delete all markers on a topic and namespace pair.
'''
def publish_deletaALLMarkers(topic = '/visualization_marker_array', namespace = 'marker') :
    marker_array = MarkerArray()
    marker = Marker()
    marker.id = 0
    marker.ns = namespace
    marker.action = Marker.DELETEALL
    marker_array.markers = [ marker ]
    pub = rospy.Publisher( topic , MarkerArray , queue_size=1 )
    pub.publish( marker_array )



'''
    args: marker config details
    output: marker object
'''
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
                    sx = 0.05, 
                    sy = 0.05, 
                    sz = 0.05,
                    z = 0 ) :
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
        marker.points = [Point(p[0],p[1],z) for p in points]


    marker.lifetime = rospy.Duration()

    return marker






if __name__ == '__main__' :
    grid = occupancyGridSubscriber()



