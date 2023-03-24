#!/usr/bin/env python
import math
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
from tf.transformations import euler_from_quaternion, quaternion_from_euler




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
        #  Inits navigation node (single node for all frontier navigation behavior)
        rospy.init_node( 'frontiers_exploration' , anonymous = True )


    def init_Subscriber( self ) -> None :
        #  Subscribes to map
        #  --> takes occupancy grid --> updated at self.occupancy_grid_callback
        rospy.Subscriber("map", OccupancyGrid, self.occupancy_grid_callback)
    

    def init_frame_listener(self , source = 'map' , target = 'base_footprint') -> None :
        """Summary
        -
            Searches for a transformation from the source frame to the target frame.
            Returns a transformation object that can be used to switch points.
            Note: for default values source = 'map' and target = 'base_footprint'
            the translation of the transformation is the location of the robot in the
            map frame.

        Args:
        -
            source (str, optional): Original frame. Defaults to 'map'.
            target (str, optional): Target frame. Defaults to 'base_footprint'.

        Returns:
        -
            _type_: Transformation object taking source frame to target frame
        """        
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        rate = rospy.Rate(10.0)
        count = 0
        #  Tries to lookup transform matrix for a a maximum of 200 times.
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
        """Summary
        -
            Initializes action client, called navigation_client, on the /move_base topic.
        """
        self.navigation_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.navigation_client.wait_for_server()


    def occupancy_grid_callback( self , data ) :
        """Summary
        -
            Takes incoming occupancy grid data and updates a cached object
            <navigation.cache['incoming occupancy grid']>
            This is later checked against navigation.cache['incoming occupancy grid']
            to check for updates in the map information.

        Args:
            data (OccupancyGrid): Map information containing .header, .info and .data
        """        
        rospy.loginfo(f'Map callback received')
        navigation.cache['incoming occupancy grid'] = data


    def navigation_client_push_goal(self, pose : Pose = None , x = 0, y = 0, z = 0, w = 1, nx = 0, ny = 0, nz = 1) :
        """Summary
        -
            Takes a target position in the base_footprint frame (either in the form of a pose object
            or positional arguments).
            Sends the goal to the navigation client
            Then waits for the result of the navigation attempt.
            NOTE: This is a non blocking wait, and so while waiting
                a pipeline that updates frontier and goal information will be
                running.
                <self.wait_for_result_non_stalled>

        Args:
            pose (Pose, optional): Pose type object containing goal information in base_footprint frame. Defaults to None.
            x (int, optional): translation in x (base_footprint frame). Defaults to 0.
            y (int, optional): translation in y (base_footprint frame). Defaults to 0.
            z (int, optional): translation in z (base_footprint frame). Defaults to 0.
            w (int, optional): quaternion in w (base_footprint frame). Defaults to 1.
            nx (int, optional): quaternion in x (base_footprint frame). Defaults to 0.
            ny (int, optional): quaternion in y (base_footprint frame). Defaults to 0.
            nz (int, optional): quaternion in z (base_footprint frame). Defaults to 1.
        """        
        if pose == None : pose = util.to_pose( x , y , z , w , nx , ny , nz )
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose

        self.navigation_client.send_goal( goal )
        self.wait_for_result_non_stalled()


    def wait_for_result_non_stalled( self ) :
        """Summary
        -
            Given that the navigation client has a goal, this waiting function
            runs a pipeline to check for map updates, then updates information about
            frontiers and centroids then publishes new markers.
            Also, checks the quality of the current goal mid-flight:

        Procedure
        -
            1) Checks if the goal has been reached or terminated
            2) Checks if there is any new map information
                2.a) If yes: Updates cache values for frontiers and centroids.
                2.b) If yes: Uses the new map information to determine of the current
                             goal is still applicable / high quality.
                             (This is done by checking its habitability or rather the 
                             density of surrounding walls)
                             2.b.a) If the current goal is found to be restricted, it will be moved
                                    along the safest path to the robot until its habitable. Or halfway along
                                    the safest path to the robot if no other habitable location is possible.
                2.c) If No: Do nothing
            3) Checks if the base_frame is within a tolerable limit of the goal
                3.a) If yes: stop tracking goal and exit waiting pipeline
                3.b) If No: No nothing
        """        
        rate = rospy.Rate( 10.0 )
        result = None
        try :
            #  Iterates until goal is reached or plan terminated
            rospy.loginfo(f'---------Navigating---------')
            while result == None and not rospy.is_shutdown() :

                #  Updates cache with new information is available
                result = self.navigation_client.get_result()

                frontiers , expanded_occupancy_grid = self.update_frontiers( navigation.cache[ 'incoming occupancy grid' ] )

                if frontiers != None :

                    #  If there are new frontier locations, find their centroids and update cache
                    navigation.cache[ 'expanded occupancy grid' ] = expanded_occupancy_grid
                    navigation.cache[ 'frontiers' ] = frontiers
                    centroids = self.update_centroids( frontiers )

                    if centroids != None :

                        #  If finding centroids was successful, publish all new information as markers to map
                        navigation.cache[ 'active centroids' ] = centroids
                        #self.publish_frontiers( frontiers , centroids )
                        self.publish_markers( navigation.cache[ 'target' ] , frontiers , centroids )

                        #  Because new information is known about the map
                        #  we may learn about new challenges impacting the robots
                        #  ability to reach the goal.
                        #  Here the habitability of the goal is checked,
                        #  Or rather how easy it is for the robot to reach the goal
                        #  If its difficult to reach, the goal is moved to something easier.
                        target = navigation.cache[ 'target' ]
                        habitability = self.check_habitable( expanded_occupancy_grid , target  )
                        if habitability != 'Habitable' :
                            rospy.loginfo(f'Target has {habitability} modality --> refactoring target to nearest Habitable zone')
                            #  Retrieves the nearest habitable goal
                            goal = self.reconstruct_inhabitable_target( expanded_occupancy_grid , target )
                            navigation.cache[ 'target' ] = goal
                            #self.navigation_client.stop_tracking_goal()
                            self.navigation_client.cancel_all_goals()
                            self.publish_markers( navigation.cache[ 'target' ] , frontiers , centroids )

                #  Checks if the robot is sufficiently close to the goal
                #  If it is, exits pipeline.
                navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
                if self.epsilon_containment(navigation.cache['robot_position'] , util.tf_occuGrid_to_map( [navigation.cache['target'] ])[0], 0.2 ) :
                    x,y = navigation.cache[ 'target' ]
                    rospy.loginfo(f'------------------------------------------')
                    rospy.loginfo(f'------ Goal at ({x} , {y}) Reached -------')
                    rospy.loginfo(f'------------------------------------------')
                    self.navigation_client.stop_tracking_goal()
                    navigation.cache[ 'target' ] = None
                    break
                rate.sleep()
            
            #  Exit condition when goal is found
            #navigation.cache[ 'target' ] = None
            self.navigation_client.stop_tracking_goal()
        except :
            raise 


    def init_auto_navigation( self ) :
        """Summary
        -
            Called while auto navigation software is attempting to connect to the map
            and find frontiers for the first time.  
        
        Procedure
        -
            1) Wait to connect to the map: waits for a maximum of 1 minute then fails
            2) Calls function to find frontier locations
            3) Calls function to find centroids
            4) Calls function to rank centroids based on entropy/distance metric
            5) Establishes a habitable goal/target
            6) Publishes frontiers/centroids/goal markers

        Returns:
            Boolean: True : if completes entire pipeline, finds frontiers/centroids, ranks centroids/ establishes goal
                     False : otherwise.
        """        
        rospy.loginfo(f'---Attempting To Initialize---')
        
        #  Waits for map subscriber to send Occupancy Map object
        #  If no map is received for 1 minute, fails the program.
        flag_map_data_counter = 0
        while navigation.cache['incoming occupancy grid'] == None and not rospy.is_shutdown() :
            rospy.loginfo(f'......Waiting for map data.......')
            rospy.sleep(3)
            flag_map_data_counter += 1
            if flag_map_data_counter >= 5 : return False

        rospy.loginfo(f'----- Map Data Found ------')
        #  Attempts to update frontier information
        frontiers , expanded_occupancy_grid = self.update_frontiers( navigation.cache['incoming occupancy grid'] )

        #  If non-zero number of frontiers are found
        if frontiers != None :

            if len( frontiers ) == 0 : return False

            rospy.loginfo('----Frontiers Found----')
            #  Attempts to update centroid information
            centroids = self.update_centroids( frontiers )
            if centroids != None :

                if len( centroids ) == 0 : return False

                rospy.loginfo('----Centroids Found----')
                #  Publishes markers for frontiers and centroids
                self.publish_frontiers( frontiers , centroids )

                #  Ranks centroids using entropy/distance metric
                    #  distance is calculated by expanding wavefront configured for safe paths
                    #  Entropy is calculated using monte-carlo sampling
                ranked_centroids , paths , _ = self.rank_centroids( frontiers , centroids, expanded_occupancy_grid )

                #  Updates cache
                navigation.cache['active centroids'] = centroids
                navigation.cache['paths'] = paths

                if ranked_centroids != None :

                    if len(ranked_centroids) == 0 : return False

                    rospy.loginfo('----Centroids Ranked----')
                    #  Publishes marker at goal location
                    self.publish_markers( ranked_centroids[0] , frontiers , centroids )

                    navigation.cache['active centroids'] = ranked_centroids
                    navigation.cache['target'] = ranked_centroids[0]

                    #  Runs mini-navigation procedure to rotate the robot
                    #  in the direction of the safest path found by expanding wavefront
                    #  This helps guide the navigation software in environments with
                    #  granular modality.
                    #path_key = navigation.cache['target']
                    #if len(paths[path_key][1]) > 2 :
                    #    self.nav_procedure_rotation_goal(paths[path_key][1][-2])
                    
                    #  Completes pipeline
                    rospy.loginfo(f'--------------------------------------------')
                    rospy.loginfo(f'------Navigation Software Initialized-------')
                    rospy.loginfo(f'--------------------------------------------')
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
        """Summary
        -
            Primary navigation loop: 
                manages creating new goals based on cached map information
                Runs until no new centroids can be found as targets  

                At each loop checks if the cache has a target already
                    A)  If navigation.cache['target'] has a target send that target to the navigation client

                    B)  If navigation.cache['target'] does not have a target, finds a new target by ranking the 
                        currently cached centroids.

        Returns:
            Boolean: Success or failure of navigation software
        """        

        rospy.loginfo(f'Starting automatic navigation')

        #  Attempts to connect navigation software to non-redundant map data
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


        #  Redundancy Checks if there are any centroids during startup
        if navigation.cache['active centroids'] == None : 
            rospy.logerr(f'Cannot initialize navigation: inability to detect/segment/ and/or rank frontiers')


        #  Iterates until there are no new non-redundant centroids visible by the robot
        #  (And so no new frontiers)
        while (len( navigation.cache['active centroids'] ) > 0 or navigation.cache['target'] != None) and not rospy.is_shutdown() :

            #  Simple logic:
                #  If cache has a target, send target to navigation client
                #  If cache doesn't have a target, find a new target and send it to the navigation client.

            if navigation.cache['target'] == None :
                if len(navigation.cache['frontiers']) > 0 :

                    #  Ranking centroids and assigning a new target
                    ranked_centroids, paths , _ = self.rank_centroids( navigation.cache['frontiers'] , 
                                                            navigation.cache['active centroids'] , 
                                                            navigation.cache['expanded occupancy grid'] )
                    navigation.cache['ranked centroids'] = ranked_centroids[1:]
                    navigation.cache['target'] = ranked_centroids[0]
                    navigation.cache['paths'] = paths
                    #self.publish_goal( ranked_centroids[0] )

                    #  Starts blocking mini-navigation task to rotate the robot in the direction 
                    #  of the safest path to the target.
                    #path_key = navigation.cache['target']
                    #self.nav_procedure_rotation_goal(paths[path_key][1][-2])

                    self.publish_markers(navigation.cache['target'] ,navigation.cache['frontiers'] , navigation.cache['active centroids'])

            #  Starts non-blocking navigation procedure to go to target.
            self.init_nav_procedure( navigation.cache['target'] )
            rospy.sleep(0)
        
        #  Terminating line after all frontiers have been explored.
        rospy.on_shutdown(self.shutdown)
        return True


    def init_nav_procedure( self , target : tuple ) :
        """Summary
        -
            Non-blocking navigation procedure to move the robot to the target location

        Args:
            target (tuple): (x,y) goal location as index in 2D occupancy grid
        """        
        x,y = util.tf_occuGrid_to_map( [ target ] )[0]
        rospy.loginfo(f'------------------------------------------------------')
        rospy.loginfo(f'Targeting point in map frame x = {round(x,2)} , y = {round(y,2)}')
        rospy.loginfo(f'------------------------------------------------------')

        pose = self.transform_object( x , y , source = 'map' , target_frame = 'base_footprint' )
        self.navigation_client_push_goal( pose )


    def nav_procedure_rotation_goal(self , target : tuple ) :
        """Summary
        -
            Blocking navigation procedure to rotate the robot to face target

        Args:
            target ( tuple ): index in occupancy grid
        """        
        rospy.loginfo(f'Beginning Rotation Navigation Procedure')
        #  Gets the robots position and orientation in the map frame
        navigation.cache['robot_position'] = self.init_frame_listener( source='map' , target='base_footprint' )
        source = navigation.cache['robot_position']
        orientation = source.transform.rotation
        nx,ny,nz,w = orientation.x,orientation.y,orientation.z,orientation.w
        roll,pitch,_ = euler_from_quaternion( [nx,ny,nz,w] )

        #  converts the target to base_footprint frame
        #  Finds the minimum angle of attack from the robots position
        s,t = util.tf_occuGrid_to_map( [ target ] )[0]
        target_pose = self.transform_object(s,t,source='map',target_frame='base_footprint')
        s,t = target_pose.position.x , target_pose.position.y
        theta = math.atan2( t , s )

        #  Rotates the robot by delta_theta radians until aligns with goal.
        delta_theta = 0.5
        orientation = quaternion_from_euler( roll , pitch , delta_theta )
        nx,ny,nz,w = orientation[0],orientation[1],orientation[2],orientation[3]
        count = 0
        while theta**2 > 0.2 and (theta-math.pi)**2 > 0.2 and not rospy.is_shutdown() :
            s,t = util.tf_occuGrid_to_map( [ target ] )[0]
            target_pose = self.transform_object(s,t,source='map',target_frame='base_footprint')
            s,t = target_pose.position.x , target_pose.position.y
            theta = math.atan2( t , s )
            pose = util.to_pose(0,0,0,w,nx,ny,nz)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'base_footprint'
            goal.target_pose.pose = pose
            
            result = self.navigation_client.send_goal_and_wait( goal,rospy.Duration(3) )
            if result == 9 :
                count += 1
            if count > 3 :
                rospy.loginfo(f'Rotation Failed : attempt {count+1}/{5}')
                self.navigation_client.cancel_all_goals()
                break
            #self.navigation_client.send_goal( goal )
            #  Blocking wait
            #self.navigation_client.wait_for_result()

            rospy.sleep(0)


    def no_action_shutdown( self ) :
        """Summary
        -
            runs when rospy is shutdown
        """        
        rospy.loginfo(f'-------Shutting Down--------')


    def shutdown( self ) :
        """Summary
        -
            Called on rospy shutdown
            Clears all goals, deletes all markers, closes application
        """        
        rospy.loginfo(f'----------Navigation Complete------------')
        
        rospy.loginfo(f'--------RETURNING TO ORIGIN----------')
        navigation.cache['target'] = util.tf_map_to_occuGrid( [(0,0)] )[0]
        robot_pos = self.init_frame_listener(source='map' , target='base_footprint')
        a,b = robot_pos.transform.translation.x,robot_pos.transform.translation.y
        a,b = util.tf_map_to_occuGrid([(b,a)])[0]
        x,y = navigation.cache['target']
        
        paths , _ = util.ExpandingWaveForm(navigation.cache['expanded occupancy grid'],(a,b),[(x,y)])
        navigation.cache['target'] = self.refactor_by_habitability(navigation.cache['expanded occupancy grid'],navigation.cache['target'],paths[navigation.cache['target']][1])
        self.publish_markers(navigation.cache['target'] ,navigation.cache['frontiers'] , navigation.cache['active centroids'])       
        self.init_nav_procedure( navigation.cache['target'] )
        rospy.sleep(0)
        
        self.navigation_client.cancel_all_goals()
        self.publish_deleteALLMarkers(namespace='goal')
        rospy.loginfo(f'Ending ROSPY')
        rospy.loginfo(f'Application Complete')


    def transform_object( self , x=0, y=0, z=0, nx=0, ny=0,nz=0,w=1, source = 'map', target_frame = 'base_footprint') :
        """Summary
        -
            Transforms source frame information to target frame 

        Args:
        -
            x (int, optional): Translation in x. Defaults to 0.
            y (int, optional): Translation in y. Defaults to 0.
            z (int, optional): Translation in z. Defaults to 0.
            nx (int, optional): Rotation in x. Defaults to 0.
            ny (int, optional): Rotation in y. Defaults to 0.
            nz (int, optional): Rotation in z. Defaults to 0.
            w (int, optional): Rotation in w. Defaults to 1.
            source (str, optional): _description_. Defaults to 'map'.
            target_frame (str, optional): _description_. Defaults to 'base_footprint'.

        Returns:
            Pose: pose object in robot frame
        """        

        #  Constructs transformation object from target to source
        trans = self.init_frame_listener(target_frame , source)

        #  Constructs and stamps pose object with source information
        pose = tf2_geometry_msgs.PoseStamped()
        pose.pose = util.to_pose(x,y,z,w,nx,ny,nz)
        pose.header.frame_id = source
        pose.header.stamp = rospy.Time.now()

        #  Try to transform pose to target frame
        try :
            out_pose = tf2_geometry_msgs.do_transform_pose(pose, trans)
            return out_pose.pose
        except :
            raise


    def epsilon_containment( self , source , target , r ) :
        """Summary
        -
            Checks if source is within an r radius ball around target
        Args:
            source ( tuple ): source
            target ( tuple ): target
            r (_type_): radius

        Returns:
            Boolean: Is contained or not
        """        
        xs,ys = source.transform.translation.x, source.transform.translation.y
        xt,yt = target
        if ((xs-xt)**2 + (ys-yt)**2)**0.5 <= r : 
            return True
        return False


    def update_frontiers( self , data , tolerance = 0 ) :
        ''' Summary
        -
        Class to subscribe to the map occupancy grid and detect frontier clusters
        then publish the location and centroid of each frontier cluster.

        - A frontier is an area where known unoccupied space meets unknown space.
        - The goal of this script is to identify distinct frontier regions that
            can be used as goal for the robot to explore its environment.
        
        Procedure :
        -
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
        #rospy.loginfo(f'Number of distinct frontiers detected. {len(frontiers)}')

        frontiersGrid = frontiersGrid.flatten()

        #  Publishes occupancy grid of non-segmented frontier points.
        pub = rospy.Publisher( '/frontiers_map' , OccupancyGrid , queue_size=1 , latch=True )
        pub.publish( data.header, data.info , frontiersGrid )


        return frontiers , ExpandedOccupancyGrid


    def publish_frontiers( self , frontiers , centroids ) :
        """Summary
        -
            Takes lists of frontier and centroid objects in occupancy grid format
            and publishes markers in corresponding map frame

        Args:
        -
            frontiers (list): List of lists of frontier indices in occupancy grid format.
            centroids (list): list of centroids as indices in occupancy grid.
        """        
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
        frontiersPublisher = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1, latch = True )
        frontiersPublisher.publish( frontier_markerArray )


    def update_centroids( self , frontiers ) -> list :
        """Summary
        -
            Given a list of lists of frontier points as indices in occupancy grid format
            finds centroid of each frontier region

        Args:
        -
            frontiers (list[list]): list containing segmented frontiers

        Returns:
        -
            list: list of centroid locations
        """        
        if frontiers == None :
            return None
        if len(frontiers) == 0 :
            return []
        centroids = [ util.get_centroid( f )  for f in frontiers ]
        return centroids


    def rank_centroids( self , frontiers , centroids , occupancy_grid) :
        """Summary
        -
            Sorts the list of centroids from highest expected utility to lowest

                - The expected utility of a centroid is an approximation of how quickly 
                    the entire map can be explored if that centroid is discovered next.

                - Here we calculate the utility of a centroid (i,j) by the expected map 
                    entropy around (i,j) over the sum of the 'safest' path from the robot 
                    to the centroid.

                            U(i,j) = H(i,j)/d(i,j)

                - This rewards exploring paths with high entropy (uncertainty) but discourages
                    exploration to points that are either far away to challenging for the robot
                    to reach.

                - Distance is calculated using expanding wavefront algorithm from the robot to each
                    centroid where the magnitude of the gradient between points corresponds to the 
                    points distance from walls.

                    - expanding wavefront returns a dictionary of paths for each centroid where each
                        path contains both the sum of weights along that path and the indices within the path.

                - Map entropy of a frontier, F, is found using random sampling in monte carlo simulation

                    - Define a fixed 2D region, R, to sample the map.

                    - For frontier F, randomly select v points as anchors

                    - For each anchor v_i, fix origin of region R at v_i 

                    - randomly sample map points, w_j within region R centered at v_i

                    - Sum and normalize the entropy for each random sample for each v_i.

            After the centroids are ranked their habitability is determine and position adjusted accordingly.

        Args:
        -
            frontiers (list): list of frontier clusters, each contains a list of indices belonging to that frontier
            centroids (list): list of centroids for corresponding frontiers in frontiers param
            occupancy_grid (list): 2D grid map with convention, 
                                    100 = known occupied space  --> entropy = 0
                                    0 = known unoccupied space  --> entropy = 0
                                    -1 = unknown space          --> entropy = 1

        Returns:
        -
            list: list of ranked centroids
            dict : paths = { centroid : (distance to robot , path list) }
            list : heatmap from expanding wavefront
        """        

        #  Gets position of robot in map frame
        if navigation.cache['robot position'] == None :
            navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
        

        #  Ensured type compatibility of frontiers and centroids
        if type(frontiers) == type(None) or type(centroids) == type(None) or type(occupancy_grid) == type(None) :
            rospy.loginfo(f'Not enough information to rank targets. \
                            Fronters : {type(frontiers)} , \
                            centroids : {type(centroids)} , \
                            occupancy grid : {type(occupancy_grid)}')

        #  Check for Null call
        if len(frontiers) == 0 or len(centroids) == 0 or len(occupancy_grid) == 0 :
            return None
        
        x = navigation.cache['robot_position'].transform.translation.x
        y = navigation.cache['robot_position'].transform.translation.y

        robot_grid_pos = util.tf_map_to_occuGrid([(y,x)])[0]

        #  runs expanding wavefront algorithm to find safe paths to each centroid.
        paths, heatmap = util.ExpandingWaveForm( occupancy_grid , robot_grid_pos , centroids )
        # paths = { centroid : (distance to robot , path list) }

        #  returns a list of centroids adjoined with their expected map entropy
        centroids_entropy = util.random_entropy_sample( occupancy_grid , frontiers , centroids )

        #  sorts each centroid by its expected utility H(f)/d(f)
        centroids_utility = sorted( centroids , key = lambda centroid : centroids_entropy[centroid]/paths[centroid][0] , reverse=True )
        #  Ordered most useful to least


        #  Checks the habitability of each centroid location re factors along the safest path if necessary
        navigation.cache['active centroids'] = centroids_utility
        habitable_centroids = []
        for centroid in centroids_utility :
            if self.check_habitable(occupancy_grid,centroid) :

                habitable_centroids += [self.refactor_by_habitability(occupancy_grid , centroid , paths[centroid][1])]
                paths[habitable_centroids[-1]] = paths[centroid]

        #  Publishes occupancy grid of non-segmented frontier points.
        for key,sum_and_path in paths.items( ) :
            for i,j in sum_and_path[ 1 ] : 
                heatmap[ i ][ j ] = 100
        heatmap = heatmap.flatten( )
        heatmap = heatmap.clip( 0 , 100 )
        pub = rospy.Publisher( '/energy_map' , OccupancyGrid , queue_size=1 , latch=True )
        pub.publish( navigation.cache['occupancy grid'].header , navigation.cache['occupancy grid'].info , heatmap )

        return habitable_centroids , paths , heatmap


    def reconstruct_inhabitable_target(self,expanded_occupancy_grid,target) :
        """Summary
        -
            If a target is difficult for the robot to navigate to, the target is 
            moved along the safest path (determined by expanding wavefront) towards
            the robot.
        Args:
            expanded_occupancy_grid (_type_): _description_
            target (_type_): _description_

        Returns:
            _type_: _description_
        """        
        navigation.cache['robot_position'] = self.init_frame_listener(source='map' , target='base_footprint')
        x = navigation.cache['robot_position'].transform.translation.x
        y = navigation.cache['robot_position'].transform.translation.y
        robot_grid_pos = util.tf_map_to_occuGrid([(y,x)])[0]
        paths, _ = util.ExpandingWaveForm( expanded_occupancy_grid , robot_grid_pos , [navigation.cache['target']] )
        return self.refactor_by_habitability(expanded_occupancy_grid , target , paths[target][1] )


    def check_habitable( self , occupancy_grid, centroid, filter = None) :
        """Summary
        -
            Determines the difficulty in modality for the robot to navigate to target.
            Returns either

                Granular    --> Highly restricted movement
                Restricted  --> Restricted movement
                Habitable   --> Non-restricted movement
        Args:
        -
            occupancy_grid (_type_): _description_
            centroid (_type_): _description_
            filter (_type_, optional): _description_. Defaults to None.

        Returns:
            literal: ['Granular' , 'Restricted' , 'Habitable']
        """        
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
        """Summary
        -
            Checks of a target is in a habitable zone,

            if Yes: return target

            if No: move target along path until its in a habitable zone
                    if none exists, move target halfway through the path.
        Args:
        -
            occupancy_grid (_type_): _description_
            target_centroid (_type_): _description_
            path (_type_): _description_

        Returns:
            _type_: _description_
        """
        #  Type assurance
        if type(target_centroid) == type(None) : return target_centroid
        if type(occupancy_grid) == type(None) : return target_centroid
        if type(path) == type(None) : return target_centroid

        #  constructs filter
        x,y = target_centroid
        fs =(3,3)
        filter = [(i,j) for i in range(-fs[0] , fs[0]) for j in range(-fs[1] , fs[1])]

        #  Checks if the target is habitable already
        if self.check_habitable(occupancy_grid, (x,y) , filter) == 'Habitable' :
            return target_centroid
        
        #  while the target is not in a habitable region, move it along the path.
        p=1
        while self.check_habitable(occupancy_grid , (x,y) , filter) not in ['Habitable'] and not rospy.is_shutdown() :
            x,y = path[ p ]
            p+=1
            if p > len(path)/2 :
                break
        return (x,y)


    def publish_markers( self , goal , frontiers , centroids) :
        """Summary
        -
            Publishes markers for the goal, frontiers, and centroids at once

        Args:
            goal (tuple): occupancy indices of goal location
            frontiers (list): list of lists of tuples for each cluster of frontiers in occupancy grid format
            centroids (list): list of tuples in occupancy grid location of centroids.

        Returns:
            _type_: _description_
        """        
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
        self.publish_deleteALLMarkers(namespace='centroids')
        self.publish_deleteALLMarkers( namespace='frontier_points' )
        frontier_markerArray.markers = centroid_marker_array.markers + frontier_markerArray.markers + goal_markerArray.markers
        frontiersPublisher = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1, latch = True )
        frontiersPublisher.publish( frontier_markerArray )






        #self.publish_deleteALLMarkers(namespace='goal')
        #GoalPublisher = rospy.Publisher( "/visualization_marker_array" , MarkerArray , queue_size=1, latch=True )
        #GoalPublisher.publish( goal_markerArray )
        return True


    def publish_deleteALLMarkers(self , topic = '/visualization_marker_array', namespace = 'marker') :
        """Summary
        -
            Deletes all markers on topic and namespace
        Args:
            topic (str, optional): _description_. Defaults to '/visualization_marker_array'.
            namespace (str, optional): _description_. Defaults to 'marker'.
        """        
        marker_array = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        marker_array.markers = [ marker ]
        pub = rospy.Publisher( topic , MarkerArray , queue_size=1 )
        pub.publish( marker_array )
    

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
        """Summary
        -
            Creates marker object with entry information

        Args:
            points (_type_, optional): _description_. Defaults to None.
            w (int, optional): _description_. Defaults to 1.
            nx (int, optional): _description_. Defaults to 0.
            ny (int, optional): _description_. Defaults to 0.
            nz (int, optional): _description_. Defaults to 0.
            r (float, optional): _description_. Defaults to 1.0.
            g (float, optional): _description_. Defaults to 0.0.
            b (float, optional): _description_. Defaults to 0.0.
            alpha (float, optional): _description_. Defaults to 1.0.
            id (int, optional): _description_. Defaults to 0.
            namespace (str, optional): _description_. Defaults to 'marker'.
            type (int, optional): _description_. Defaults to 8.
            sx (float, optional): _description_. Defaults to 0.05.
            sy (float, optional): _description_. Defaults to 0.05.
            sz (float, optional): _description_. Defaults to 0.05.
            z (int, optional): _description_. Defaults to 0.

        Returns:
            _type_: _description_
        """        
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
    navigation()





