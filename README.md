# COMP-4510-project-2

# Table of Contents

- catkin_ws
  - The main workspace for the project
- frontier_exploration
  - The primary package including nodes and launch files for the project
- frames.pdf
  - A pdf displaying the transform view frame connections
- waffle_tf_listener.py         (NOT USED FOR TASK 3)
  - Python transformation listener file to display the current position of the robots
    base_footprint in reference to the map.
- moveActionClient.py           (NOT USED FOR TASK 3)
  - TeleOp system action client to move the robot with respect to its base_footprint reference frame.
  - Translation parameters (goals) are entered in the terminal, see setup files tutorial.
- RvizProjectTwoConfig.rviz
  - setup config file for RviZ, includes robot camera, global/local path markers and markers to display frontiers.
- auto_exploration.py
  - Finds all frontier clusters and their centroids.
  - Subscribes to /map to take an occupancy grid and locate candidates for frontiers (locations where unoccupied known space meets unknown space).
  - Publishes an occupancy map showing the location of each frontier
  - Publishes color coordinated points to display the distinct clusters of segmented frontiers and their centroids.
  - Automatically navigates to goal locations on the map until the entire map is explore ( or at minimum entropy )
- util.py
  - utility function for auto_exploration.py contains morphological functions for dilation, erosion and line detection on images (occupancy grids)
  - contains connected component analysis algorithms to detect continuous or semi-continuos regions to binary occupancy grids.
- Video of setup files at
[![Watch the video]](https://www.youtube.com/watch?v=dVd2kIACvE8)
- Video of frontier location algorithm (part 2) here
[![Watch the video for part 2]](https://youtu.be/aaL37uBaii4)

-Video for TASK 3
[![Watch the video for part 3]](https://youtu.be/OnBdq9bTtGM)

# Summary  

The following is instructions on running frontiers based auto-navigation software  

- A frontier is an area where known unoccupied space meets unknown space.  

The goal of this project is to construct a 'smart' navigation system that can
detect frontier locations, segment them into distinct regions, then navigate
/ explore each region in a way that explores the entire map in the least amount of time.  

The majority of the work here is done in auto_exploration.py  

To detect and segment frontiers:

## Procedure

1) Subscribes to /map topic and takes an occupancy grid object  

    - when something is published on the /map topic an internal cache is updated  

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

5) Frontier segmentation  

    - Here the list of all candidate frontier points are segmented into distinct clusters.  

    - Features of a good segmentation can include, a continuous topological region,  
      (or at least semi-continuous with small jumps.) distance from walls or in general  
      are reachable by the robot.  

    - Here connected component analysis is used to segment each frontier as semi-continuous
      topological regions.

    - By semi-continuous we mean that the region is either fully connected or any gaps between
      components are 'small'.  
      This is done by running breadth first search (BFS) sequentially on each frontier point  
      candidate and connecting the visited points.  
      Successors in the BFS for each point are found by the neighbors of that point on a grid.  
      Neighbors are determined by overlaying an (n X n) kernel.

## Then to rank frontier segments

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

# Running Files for task 3

In their own terminal run the following

```console
roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

This should open a Gazebo and RviZ window.

Next, open the included RviZ config file  

```console
frontier-exploration / RvizProkectTwoConfig.rviz
```

It is important that the RviZ window config loads correctly,  
the displays tab should contain,  

- Map/local path/ global path from the start up files section (PART 1)
- frontiers_map  
        - Subscribed to /frontiers_map
        - Displays the occupancy grid for the group of all frontiers
- MarkerArray  
        - Subscribed to /visualization_marker_array  
        - Displays color coordinated dotes representing distinct clusters of frontiers, centroids and targets.
- EnergyMap  
        - Displays the paths found from robot to centroids created by expanding wavefront
        - This should initially be checked 'OFF' because this marker is distracting (but kinda cool)

Next, in a new terminal run,

```console
roslaunch frontier-exploration turtlebot3_navigation.launch
```

If this command errors you may need to resource the terminal

```console
cd catkin_ws
source devel/setup.bash
roslaunch frontier-exploration turtlebot3_navigation.launch
```

Next, run the frontiers identification script

```console
rosrun frontier-exploration auto_exploration.py
```

(you may need to resource the terminal)  
This will begin an automatic navigation software where the robot will  
find regions of unknown space and navigate to them in a way that optimally  
explores the entire map.  

# Running Additional Files Procedure (Not used for task 3)

See the position of the robot with,

```console
rosrun tf tf_echo /map /base_footprint
```

Another way to display the position of the robot is with a listener script in terminal

```console
rosrun frontier-exploration waffle_tf_listener.py
```

This will display the current position of the base of the robot with respect to the map.  
Next, we want to issue a command to the robot to move.  
This can be done with the moveActionClient.  
Running  

```console
rosrun frontier-exploration moveActionClient.py -x <goal in x> -y <goal in y>
```

will translate the position of the robots base frame by (x,y) units.  
Note: this translation is with respect to the robots frame not the map so for example
translating (x=1,y=1) will move the robot right and up one unit. (instead of moving the coordinate (1,1) on the map.)  
Here are a few examples to run.

```console
rosrun frontier-exploration moveActionClient.py -x -1 -y 1
rosrun frontier-exploration moveActionClient.py -x -1 -y -1
```

You can also issue commands without specifying the argument, in the order (x,y)

```console
rosrun frontier-exploration moveActionClient.py -1 1
rosrun frontier-exploration moveActionClient.py -1 -1
```

The pdf file frames.pdf displays a tf tree of the objects on the module.  
To generate a new pdf, run,

```console
cd catkin_ws
sudo apt install ros-noetic-tf2-tools
rosrun tf2_tools view_frames.py
```

# Troubleshooting

- Cannot find package error
  - This is a problem that is liklely caused by the terminal not being sourced correctly.
    Which can be resourced by

```console
cd catkin_ws
source devel/setup.bash
```

- Problems with tf package for view_frames.py
  - many of the original tf functions are depriciated and so tf2 has been used in exchange.

```console
sudo apt install ros-noetic-tf2-tools
rosrun tf2_tools view_frames.py
```

- If the application is having trouble connecting to the robot try running the following to change the environment to use the waffle_pi robot.

```console
export TURTLEBOT3_MODEL=waffle_pi
```

- If anything is added to the package, re-make the workspace

```console
catkin_make
cd project/catkin_ws
source devel/setup.bash
```

- If you want to ensure a publisher is publishing

```console
rostopic echo <topic name>
```
