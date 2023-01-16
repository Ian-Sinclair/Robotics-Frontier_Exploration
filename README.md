# COMP-4745-project-2


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
    







