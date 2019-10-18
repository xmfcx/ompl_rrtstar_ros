# OMPL RRTSTAR ROS

A quick and very dirty implementation of RRT* on ROS.
Used https://ompl.kavrakilab.org/optimalPlanningTutorial.html and many other tutorials
on StateValidityChecker, StateSampler and MotionValidator.

## Dependencies
 * ROS Kinetic or above?
 * sudo apt-get install ros-`rosversion -d`-ompl

## Building and Running

 * mkdir -p ~/catkin_ws/src/
 * cd ~/catkin_ws/src/
 * git clone https://github.com/xmfcx/ompl_rrtstar_ros.git
 * cd ..
 * catkin_make
 * source devel/setup.bash
 * roslaunch ompl_rrtstar_ros ompl_rrtstar_ros.launch
 
You should see the following

![RViz Screenshot](https://github.com/xmfcx/ompl_rrtstar_ros/blob/master/docs/rviz_screenshot.png)