# COMP-4745-project-2

## Table of Contents

- catkin_ws
    - The main workspace for the project
- frontier-exploration
    - The primary package including a node and launch files for the project
- frames.pdf
    - A pdf displaying the transform view frame connections
- waffle_tf_listener.py
    - Python transformation listener file to display the current position of the robots
    base_footprint in reference to the map.
- moveActionClient.py
    - Python action client to move the robot with respect to its base_footprint reference frame.
    - Translation parameters (goals) are entered in the terminal, see setup files tutorial.
- RvizProjectTwoConfig.rviz
    - setup config file for RviZ, includes robot camera and global/local path markers.


## Running setup files

In their own terminal run the following

```console
$ roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

This should open a Gazebo and RviZ window.
The RviZ window config should include a pre-saved camera display and global/local path markers.
If the RviZ config does not laod, it can be added by opening the config.
```console
RvizProkectTwoConfig.rviz
```

Next, in a new terminal run,
```console
    $ roslaunch frontier-exploration turtlebot3_navigation.launch
```
If this command errors you may need to resource the terminal
```console
    $ cd catkin_ws
    $ source devel/setup.bash
    $ roslaunch frontier-exploration turtlebot3_navigation.launch
```
The pdf file frames.pdf displays a tf tree of the objects on the module.
To generate a new pdf, run,
```console
    $ cd catkin_ws
    $ sudo apt install ros-noetic-tf2-tools
    $ rosrun tf2_tools view_frames.py
```
Next, we can see the position of the robot with,
```console
    $ rosrun tf tf_echo /map /base_footprint
```
Another way to display the position of the robot is with a listener script in terminal
```console
    $ rosrun frontier-exploration waffle_tf_listener.py
```
This will display the current position of the base of the robot with respect to the map.
Next, we want to issue a command to the robot to move.
This can be done with the moveActionClient.
Running
```console
    $ rosrun frontier-exploration moveActionClient.py -x <goal in x> -y <goal in y>
```
will translate the position of the robots base frame by (x,y) units.
Note: this translation is with respect to the robots frame not the map so for example
translating (x=1,y=1) will move the robot right and up one unit. (instead of moving the coordinate (1,1) on the map.)
Here are a few examples to run.
```console
    $ rosrun frontier-exploration moveActionClient.py -x -1 -y 1
    $ rosrun frontier-exploration moveActionClient.py -x -1 -y -1
```
You can also issue commands without specifying the argument, in the order (x,y)
```console
    $ rosrun frontier-exploration moveActionClient.py -1 1
    $ rosrun frontier-exploration moveActionClient.py -1 -1
```
This concludes the setup for the project.

##  Troubleshooting



## To make an update to a package run....


catkin_make
cd project/catkin_ws
source devel/setup.bash  ## (YOu need to do this everytime you open a new terminal....)

1) setup ROS package

2) Change env to waffle
    export TURTLEBOT3_MODEL=waffle_pi
3) Copy directory
    cp /opt/ros/noetic/share/turtlebot3_navigation/launch/turtlebot3_navigation.launch src/frontier-exploration/launch/
    Updated added launch file.

4) rebuild catkin_make

5) Run
    roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    roslaunch frontier-exploration turtlebot3_navigation.launch

6) setup Rviz
    rosrun image_view image_view image:=/camera/rgb/image_raw

    /move_base/DWAPlannerROS/global_plan
    /move_base/DWAPlannerROS/local_plan

    ### Figure out how to save an rviz setup...

 7) where am I?
    View the tf tree rosrun tf view_frames
    sudo apt install ros-noetic-tf2-tools
    rosrun tf2_tools view_frames.py

8) Current transform between robot and map
    rosrun tf tf_echo /map /base_footprint

9) setting up waffle_tf_listener.py
    rosrun frontier-exploration waffle_tf_listener.py
    (You may need to resource the terminal before running....
        cd catkin_ws
        source devel/setup.bash)

10) setting up 
    







