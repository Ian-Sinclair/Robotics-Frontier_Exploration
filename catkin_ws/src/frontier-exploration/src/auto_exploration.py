#!/usr/bin/env python
import util
import numpy as np
import random
import tf2_ros
import rospy
import actionlib
import tf2_geometry_msgs
import tf2_geometry_msgs.tf2_geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid





class navigation() :
    cache = {'incoming occupancy grid' : None ,
             'occupancy grid' : None , 
             'obstacle_record' : None ,
             'robot position' : None ,
             'active centroids' : None ,
             'target' : None ,
             'expanded occupancy grid' : None ,
             'frontiers' : None ,
             'ranked centroids' : None, 
             'robot origin' : None, 
             'paths' : None}

    def __init__(self) -> None:
        self.init_node()
        self.init_Subscriber()
        self.init_action_client()
        self.auto_navigation()

    def init_node( self ) -> None :
        rospy.init_node( 'frontiers_exploration' , anonymous = True )

    def init_Subscriber( self ) -> None :
        rospy.Subscriber("map", OccupancyGrid, self.occupancy_grid_callback)
    

    def init_frame_listener(self , source = 'map' , target = 'base_footprint') -> None :
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
        self.navigation_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.navigation_client.wait_for_server()
    
    def done_nav_callback( self , terminal_state , result ) :
        pass
    def active_nav_callback( self ) :
        #  callback that activated as soon as a goal is received...
        #  remove centroid from list of centroids
        pass
    def feedback_nav_callback( self , feedback ) :
        pass

    def occupancy_grid_callback( self , data ) :
        rospy.loginfo(f'Map callback received')
        navigation.cache['incoming occupancy grid'] = data


    def navigation_client_push_goal(self, pose : Pose = None , x = 0, y = 0, z = 0, w = 1, nx = 0, ny = 0, nz = 1) :
        if pose == None : pose = util.to_pose( x , y , z , w , nx , ny , nz )
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose

        self.navigation_client.send_goal( goal )
        self.wait_for_result_non_stalled()


    def wait_for_result_non_stalled( self ) :
        rate = rospy.Rate(10.0)
        result = None
        try :
            while result == None and not rospy.is_shutdown() :
                result = self.navigation_client.get_result()
                print(self.navigation_client.get_state())
                frontiers , expanded_occupancy_grid = self.update_frontiers( navigation.cache['incoming occupancy grid'] )
                if frontiers != None :
                    navigation.cache['expanded occupancy grid'] = expanded_occupancy_grid
                    navigation.cache['frontiers'] = frontiers
                    centroids = self.update_centroids( frontiers )
                    if centroids != None :
                        navigation.cache['active centroids'] = centroids
                        self.publish_frontiers(frontiers , centroids)
                        target = navigation.cache['target']
                        habitability = self.check_habitable(expanded_occupancy_grid , target  )
                        if habitability != 'Habitable' :
                            rospy.loginfo(f'Target has {habitability} modality --> refactoring target to Habitable')
                            goal = self.reconstruct_inhabitable_target( expanded_occupancy_grid , target )
                            navigation.cache['target'] = goal
                            self.navigation_client.stop_tracking_goal()
                
                self.publish_goal( navigation.cache['target'] )
                navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
                if self.epsilon_containment(navigation.cache['robot_position'] , util.tf_occuGrid_to_map( [navigation.cache['target'] ])[0], 0.2 ) :
                    rospy.loginfo(f'-------------------------------------------------------------')
                    #self.navigation_client.cancel_all_goals()
                    self.navigation_client.stop_tracking_goal()
                    navigation.cache['target'] = None
                    break
                rate.sleep()
            navigation.cache['target'] = None
            self.navigation_client.stop_tracking_goal()
        except :
            raise 


    def init_auto_navigation( self ) :
        flag_map_data_counter = 0
        while navigation.cache['incoming occupancy grid'] == None and not rospy.is_shutdown() :
            rospy.loginfo(f'......Waiting for map data.......')
            rospy.sleep(3)
            flag_map_data_counter += 1
            if flag_map_data_counter >= 5 : return False

        frontiers , expanded_occupancy_grid = self.update_frontiers( navigation.cache['incoming occupancy grid'] )
        if frontiers != None :
            if len(frontiers) == 0 : return False
            centroids = self.update_centroids( frontiers )
            if centroids != None :
                if len(centroids) == 0 : return False
                self.publish_frontiers(frontiers , centroids)
                ranked_centroids, paths , _ = self.rank_centroids( frontiers , centroids, expanded_occupancy_grid )
                navigation.cache['active centroids'] = centroids
                navigation.cache['paths'] = paths
                if ranked_centroids != None :
                    if len(ranked_centroids) == 0 : return False
                    self.publish_goal( ranked_centroids[0] )
                    navigation.cache['active centroids'] = ranked_centroids
                    navigation.cache['target'] = ranked_centroids[0]
                    return True
                else :
                    rospy.logerr(f'cannot find goal from centroids')
                    return False
            else :
                rospy.logerr(f'cannot find centroid')
                navigation.cache['active centroids'] = []
                return False
        else : 
            rospy.loginfo(f'Cannot find any frontiers')
            return False

    def auto_navigation( self ) :
        rospy.loginfo(f'Starting automatic navigation')
        flag_nav_initialized = False
        for i in range(10) :
            rospy.loginfo(f'Initializing Navigation Software: attempt: {i+1}/{10}') 
            if self.init_auto_navigation() == True :
                flag_nav_initialized = True
                break
            rospy.sleep(2)
        if flag_nav_initialized == False :
            rospy.loginfo(f'fAILURE to initialize auto nav software  \
                            \t it is possible that the map is already explored')
            rospy.on_shutdown(self.no_action_shutdown)
            return False

        if navigation.cache['active centroids'] == None : 
            rospy.logerr(f'Cannot initialize navigation: inability to detect/segment/ and/or rank frontiers')

        while len( navigation.cache['active centroids'] ) > 0 and not rospy.is_shutdown() :
            if navigation.cache['target'] == None :
                if len(navigation.cache['frontiers']) > 0 :
                    ranked_centroids, paths , _ = self.rank_centroids( navigation.cache['frontiers'] , 
                                                            navigation.cache['active centroids'] , 
                                                            navigation.cache['expanded occupancy grid'] )
                    navigation.cache['ranked centroids'] = ranked_centroids[1:]
                    navigation.cache['target'] = ranked_centroids[0]
                    navigation.cache['paths'] = paths
                    self.publish_goal( ranked_centroids[0] )
                #self.publish_frontiers(navigation.cache['frontiers'] , navigation.cache['active centroids'])
            self.init_nav_procedure( navigation.cache['target'] )
            rospy.sleep(0)
        rospy.on_shutdown(self.shutdown)
        return True

    def init_nav_procedure( self , target : tuple ) :
        x,y = util.tf_occuGrid_to_map( [ navigation.cache['target'] ] )[0]
        rospy.loginfo(f'Targeting point in map frame x = {x} , y = {y}')
        pose = self.transform_object( x , y , source = 'map' , target_frame = 'base_footprint' )
        self.navigation_client_push_goal( pose )

    def no_action_shutdown( self ) :
        rospy.loginfo(f'-------Shutting Down--------')

    def shutdown( self ) :
        rospy.loginfo(f'Navigation Complete')
        '''
        rospy.loginfo(f'--------RETURNING TO ORIGIN----------')
        navigation.cache['target'] = util.tf_map_to_occuGrid( [(0,0)] )[0]
        #x,y = navigation.cache['target']
        self.publish_goal(navigation.cache['target'])
        for i in range(5) :
            self.init_nav_procedure( navigation.cache['target'] )
            rospy.sleep(0)
        '''
        self.navigation_client.cancel_all_goals()
        self.publish_deleteALLMarkers(namespace='goal')
        rospy.loginfo(f'Ending ROSPY')
        rospy.loginfo(f'Application Complete')



    def transform_object( self , x , y , source = 'map', target_frame = 'base_footprint') :
        trans = self.init_frame_listener(target_frame , source)

        pose = tf2_geometry_msgs.PoseStamped()
        pose.pose = util.to_pose(x,y)
        pose.header.frame_id = source
        pose.header.stamp = rospy.Time.now()

        try :
            out_pose = tf2_geometry_msgs.do_transform_pose(pose, trans)
            return out_pose.pose
        except :
            raise


    def epsilon_containment( self , source , target , r ) :
        '''
            Is the source within an epsilon neighborhood of radius r 
            from the target
        '''
        xs,ys = source.transform.translation.x, source.transform.translation.y
        xt,yt = target
        if ((xs-xt)**2 + (ys-yt)**2)**0.5 <= r : 
            return True
        return False


    def update_frontiers( self , data , tolerance = 0 ) :
        if navigation.cache[ 'incoming occupancy grid' ] == None :
            return None , None

        if data == None :
            return None , None

        if navigation.cache[ 'occupancy grid' ] == None :
            navigation.cache[ 'occupancy grid' ] = OccupancyGrid( header = data.header , info = data.info , data = [-1]*len( data.data ) )
        
        if navigation.cache['obstacle_record'] == None :
            navigation.cache['obstacle_record'] = []
        
        step_diff = []
        for i in range( len( navigation.cache[ 'incoming occupancy grid' ].data ) ) :
            if navigation.cache[ 'incoming occupancy grid' ].data[ i ] != navigation.cache[ 'occupancy grid' ].data[ i ] :
                step_diff += [ navigation.cache[ 'incoming occupancy grid' ].data[ i ] ]
                if data.data[ i ] == 100 :  # might as well collect the location of obstacles too.
                    navigation.cache[ 'obstacle_record' ] += [ i ]

        if not sum( step_diff ) > tolerance :
            return None , None
        
        navigation.cache[ 'occupancy grid' ] = data
        shape = ( data.info.height , data.info.width )
        occupancyGrid = np.fromiter( data.data , int ).reshape( shape )
        obstacles = [*map( lambda x: (int(x/shape[0]),x%shape[1] ), navigation.cache['obstacle_record'])]

        #  Expands obstacles using approximate cspace of the robot
        ExpandedOccupancyGrid, _ = util.informed_dilate( grid = occupancyGrid , 
                                                            mask_size = (3,3) , 
                                                            array = obstacles )
        #  Finding Frontier Candidates
        frontiersGrid, frontiersPoints = util.edge_detection( Grid = ExpandedOccupancyGrid )

        if len(frontiersPoints) == 0 :
            return [] , ExpandedOccupancyGrid
        
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

        frontiersGrid = frontiersGrid.flatten()

        #  Publishes occupancy grid of non-segmented frontier points.
        pub = rospy.Publisher( '/frontiers_map' , OccupancyGrid , queue_size=1 , latch=True )
        pub.publish( data.header, data.info , frontiersGrid )

        return frontiers , ExpandedOccupancyGrid
        
    def publish_frontiers( self , frontiers , centroids ) :
        #  Creates marker objects for frontier points.
        frontier_markerArray = MarkerArray()
        #  Transforms frontier coordinates from occupancy grid frame to map frame.
        map_frontiers = [ util.tf_occuGrid_to_map( f ) for f in frontiers ]

        map_centroids = util.tf_occuGrid_to_map( centroids )
        centroid_marker_array = MarkerArray()
        centroid_marker_array.markers = [
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
                                                for i,f in enumerate(map_centroids)
                                                ]
        self.publish_deleteALLMarkers(namespace='centroids')

        frontier_markerArray.markers = [
                                        self.convert_marker( f, 
                                                        r=random.random() ,
                                                        g=random.random() ,
                                                        b=random.random() , 
                                                        id=i ,
                                                        namespace='frontier_points' ) 
                                                        for i,f in enumerate(map_frontiers)
                                                        ]
        #  Deletes all markers on a namespace
        self.publish_deleteALLMarkers( namespace='frontier_points' )
        frontier_markerArray.markers = centroid_marker_array.markers + frontier_markerArray.markers
        frontiersPublisher = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1 , latch=True )
        frontiersPublisher.publish( frontier_markerArray )


    def update_centroids( self , frontiers ) -> list :
        if frontiers == None :
            return None
        if len(frontiers) == 0 :
            return []
        centroids = [ util.get_centroid( f )  for f in frontiers ]
        return centroids

    def rank_centroids( self , frontiers , centroids , occupancy_grid) :
        if navigation.cache['robot position'] == None :
            navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
        
        if type(frontiers) == type(None) or type(centroids) == type(None) or type(occupancy_grid) == type(None) :
            rospy.loginfo(f'Not enough information to rank targets. \
                            Fronters : {type(frontiers)} , \
                            centroids : {type(centroids)} , \
                            occupancy grid : {type(occupancy_grid)}')

        if len(frontiers) == 0 or len(centroids) == 0 or len(occupancy_grid) == 0 :
            return None
        
        x = navigation.cache['robot_position'].transform.translation.x
        y = navigation.cache['robot_position'].transform.translation.y

        robot_grid_pos = util.tf_map_to_occuGrid([(y,x)])[0]

        paths, heatmap = util.ExpandingWaveForm( occupancy_grid , robot_grid_pos , centroids )
        # paths = { centroid : (distance to robot , path list) }

        centroids_entropy = util.random_entropy_sample( occupancy_grid , frontiers , centroids )

        centroids_utility = sorted( centroids , key = lambda centroid : centroids_entropy[centroid]/paths[centroid][0] , reverse=True )
        #  Ordered most useful to least


        navigation.cache['active centroids'] = centroids_utility

        centroids_utility = [self.refactor_by_habitability(occupancy_grid , centroid , paths[centroid][1]) for centroid in centroids_utility ]

        #  Publishes occupancy grid of non-segmented frontier points.
        for key,sum_and_path in paths.items( ) :
            for i,j in sum_and_path[ 1 ] : 
                heatmap[ i ][ j ] = 100
        heatmap = heatmap.flatten( )
        heatmap = heatmap.clip( 0 , 100 )
        pub = rospy.Publisher( '/energy_map' , OccupancyGrid , queue_size=1 , latch=True )
        pub.publish( navigation.cache['occupancy grid'].header , navigation.cache['occupancy grid'].info , heatmap )

        return centroids_utility , paths , heatmap


    def reconstruct_inhabitable_target(self,expanded_occupancy_grid,target) :
        navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
        x = navigation.cache['robot_position'].transform.translation.x
        y = navigation.cache['robot_position'].transform.translation.y
        robot_grid_pos = util.tf_map_to_occuGrid([(y,x)])[0]
        paths, _ = util.ExpandingWaveForm( expanded_occupancy_grid , robot_grid_pos , [navigation.cache['target']] )
        return self.refactor_by_habitability(expanded_occupancy_grid , target , paths[target][1] )

    def check_habitable( self , occupancy_grid, centroid, filter = None) :
        if filter == None :
            fs =(3,3)
            filter = [(i,j) for i in range(-fs[0] , fs[0]) for j in range(-fs[1] , fs[1])]
        x,y = centroid
        conv = [ (x+a,y+b) for a,b in filter if occupancy_grid[x+a][y+b] == 100 ]

        if len(conv)/len(filter) > 0.5 :
            return 'Granular'
        if len(conv)/len(filter) > 0.15 :
            return 'Restricted'
        return 'Habitable'


    def refactor_by_habitability( self, occupancy_grid, target_centroid, path ) :

        if type(target_centroid) == type(None) : return target_centroid
        if type(occupancy_grid) == type(None) : return target_centroid
        if type(path) == type(None) : return target_centroid

        x,y = target_centroid
        fs =(3,3)
        filter = [(i,j) for i in range(-fs[0] , fs[0]) for j in range(-fs[1] , fs[1])]
        
        if self.check_habitable(occupancy_grid, (x,y) , filter) == 'Habitable' :
            return target_centroid
        
        p=1
        while self.check_habitable(occupancy_grid , (x,y) , filter) not in ['Habitable'] and not rospy.is_shutdown() :
            x,y = path[ p ]
            p+=1
            if p > len(path)/2 :
                break
        return (x,y)



    def publish_goal( self , goal ) :
        goal = util.tf_occuGrid_to_map( [goal] )[0]

        #  Creates marker objects for frontier points.
        goal_markerArray = MarkerArray()


        goal_markerArray.markers = [
                                        self.convert_marker( [ goal ], 
                                                        r=1 ,
                                                        g=0 ,
                                                        b=0 , 
                                                        z = 2 ,
                                                        id=100 ,
                                                        sx = 0.3 ,
                                                        sy = 0.3 ,
                                                        sz = 0.3 ,
                                                        type = 7,
                                                        namespace='goal' )
                                                        ]
        #self.publish_deleteALLMarkers(namespace='goal')
        GoalPublisher = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1 , latch=True )
        GoalPublisher.publish( goal_markerArray )
        return True


    '''
        Method to delete all markers on a topic and namespace pair.
    '''
    def publish_deleteALLMarkers(self , topic = '/visualization_marker_array', namespace = 'marker') :
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





if __name__ == '__main__' :
    # add flags to enable or disable auto navigation and type of utility selection, etc...
    navigation()





