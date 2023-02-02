#!/usr/bin/env python

import util
import tf2_ros
import rospy
import actionlib
from moveActionClient import moveActionClient


from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from threading import Thread
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseArray, Pose

import tf2_geometry_msgs


class navigation() :
    cache = {'robot_position' : None, 
            'active centroids' : []}
    def __init__(self) :
        self.init_action_client()

    def init_node() :
        rospy.init_node('Frontier Navigation')


    def init_frame_listener(self , source = 'map' , target = 'base_footprint') :
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        rate = rospy.Rate(10.0)
        count = 0
        while not rospy.is_shutdown() :
            try:
                trans = self.tfBuffer.lookup_transform(source, target, rospy.Time(0))
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
                rate.sleep()
                count += 1
                if count > 200 :
                    rospy.logerr('ERROR: Could not find map to base_footprint transform')
                continue

    def init_action_client(self) :
        '''
            Initializes rospy action client to /move_base topic.
        '''
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        #self.client.wait_for_server()

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
        pose = util.to_pose(x,y,z,w,nx,ny,nz)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        self.client.wait_for_result() # Note sure wait for result is what I want
    
    def pushGoal_max_priority(self,x,y) :
        self.client.cancel_all_goals()
        x,y=self.transform_pose(x,y , 'base_footprint')
        self.coordinate_callback(x=x,y=y)

    def auto_navigation(self, Occupancy_grid, map_frontiers ) :
        '''
            1) Get position of the robot
            2) Filter centroids and extract new goal
            3) Determine if goal should be updated or not
        '''
        #  Gets centroid coordinates for each frontier cluster
        centroids = [ util.get_centroid( f )  for f in map_frontiers ]

        navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
        p,q=navigation.cache['robot_position'].transform.translation.x,navigation.cache['robot_position'].transform.translation.y
        grid_goal = util.tf_map_to_occuGrid([(q,p)])[0]
        centroid_RviZ = MarkerArray()

        grid_goals = util.tf_map_to_occuGrid(centroids)


        cc = [(y,x) for x,y in grid_goals]
        paths, heatmap = util.ExpandingWaveForm(Occupancy_grid['data'], grid_goal , cc)
        # paths = { centroid : (distance to robot , path list) }

        #  Publishes occupancy grid of non-segmented frontier points.
        for key,sum_and_path in paths.items() :
            for i,j in sum_and_path[1] : 
                heatmap[i][j] = 100
        heatmap = heatmap.flatten()
        for i,v in enumerate(heatmap) : 
            if v > 100 : heatmap[i] = 100
        pub = rospy.Publisher( '/energy_map' , OccupancyGrid , queue_size=1 , latch=True )
        pub.publish( Occupancy_grid['header'], Occupancy_grid['info'] , heatmap )


        maxi = 10000
        for key,path in paths.items() :
            if path[0]<maxi :
                maxi = path[0]
                goal = key
        '''
        goal = util.tf_occuGrid_to_map([goal])[0]
        a,b = goal
        goal = self.transform_pose(a,b , source = 'map', target_frame='base_footprint')
        x,y = goal.position.x , goal.position.y
        #self.coordinate_callback( x , y )
        ''' 

        centroid_RviZ.markers = [
                                self.convert_marker( points=[f],
                                                r=0,
                                                g=1,
                                                b=0, 
                                                z=1,
                                                id=i,
                                                sx=0.25,
                                                sy=0.25,
                                                sz=0.25, 
                                                type=7,
                                                namespace='centroids' ) 
                                                for i,f in enumerate(centroids)
                                                ]
        
        frontiersPub = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1 , latch=True )
        frontiersPub.publish( centroid_RviZ )




    '''
        args: marker config details
        output: marker object
    '''
    def convert_marker(self,points = None, 
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

    def transform_pose( self , x , y , source = 'map', target_frame = 'base_footprint') :
        trans = self.init_frame_listener(source , target_frame)

        pose = tf2_geometry_msgs.PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()

        try :
            out_pose = tf2_geometry_msgs.do_transform_pose(pose, trans)
            return out_pose.pose
        except :
            raise

