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
            'active centroids' : [],
            'goal' : None}
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

        self.client.send_goal( goal, done_cb=self.done_callback , active_cb= self.active_callback , feedback_cb= self.feedback_callback )

        self.client.wait_for_result() # Note sure wait for result is what I want
    
    def done_callback() :
        # Get next centroid target....
        pass
    def active_callback() :
        #  callback that activated as soon as a goal is received...
        #  remove centroid from list of centroids
        pass
    def feedback_callback() :
        pass


    def recast_goal() :
        #  if a centroid is unreachable, cast rays around it until you find something that looks reachable...
        #  Probably can use the wavefront path...
        pass


    def pushGoal_max_priority(self,x,y) :
        self.client.cancel_all_goals()
        x,y=self.transform_pose(x,y , 'base_footprint')
        self.coordinate_callback(x=x,y=y)


    def auto_navigation(self , Occupancy_grid , map_frontiers ) :
        '''
            1) Get position of the robot
            2) Filter centroids and extract new goal
            3) Determine if goal should be updated or not
        '''
        #  Gets centroid coordinates for each frontier cluster
        centroids = [ util.get_centroid( f )  for f in map_frontiers ]

        navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')

        p = navigation.cache['robot_position'].transform.translation.x
        q = navigation.cache['robot_position'].transform.translation.y

        robot_grid_pos = util.tf_map_to_occuGrid([(q,p)])[0]
        centroid_grid_poses = util.tf_map_to_occuGrid(centroids)
        centroid_grid_poses = [(y,x) for x,y in centroid_grid_poses]

        paths, heatmap = util.ExpandingWaveForm(Occupancy_grid['data'], robot_grid_pos , centroid_grid_poses)
        # paths = { centroid : (distance to robot , path list) }
        centroids_entropy = util.random_entropy_sample( Occupancy_grid['data'] , map_frontiers , centroid_grid_poses )
        # centroids_entropy = { centroid : map entropy around cluster corresponding to centroid }

        centroids_utility = sorted( centroid_grid_poses , key = lambda centroid : centroids_entropy[centroid]/paths[centroid][0] )


        #  Publishes occupancy grid of non-segmented frontier points.
        for key,sum_and_path in paths.items( ) :
            for i,j in sum_and_path[ 1 ] : 
                heatmap[ i ][ j ] = 100
        heatmap = heatmap.flatten( )
        heatmap = heatmap.clip( 0 , 100 )
        pub = rospy.Publisher( '/energy_map' , OccupancyGrid , queue_size=1 , latch=True )
        pub.publish( Occupancy_grid['header'], Occupancy_grid['info'] , heatmap )



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

