# RTMR-LOAM
The code for RTMR-LOAM paper
# Setup

Dependencies:

+ ros melodic

Installation steps:

+ `mkdir ~/catkin_ws/src`
+ ` cd ~/catkin_ws/src`
+ `git clone https://github.com/lishuai-cau/RTMR-LOAM.git`
+ `cd ../`
+ `rosdep install -iry --from-paths src`
+ `cd ~/catkin_ws/`
+ `catkin make`


After successful compilation, run the following command for VLS-128 line LIDAR reconstruction：
+ ` roslaunch aloam_velodyne loam_velodyne_128.launch`
