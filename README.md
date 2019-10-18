# OMPL RRTSTAR ROS

A quick and very dirty implementation of RRT* on ROS.
Used https://ompl.kavrakilab.org/optimalPlanningTutorial.html and many other tutorials
on StateValidityChecker, StateSampler and MotionValidator.

## Running

 * Open an rviz window with the configuration file .../ompl_rrtstar_ros/rviz_stuff/ompl.rviz
 * Build it using catkin_make
 * source devel/setup.bash
 * rosrun ompl_rrtstar_ros ompl_rrtstar_ros
 * in another terminal navigate to .../ompl_rrtstar_ros/maps
 * rosrun map_server map_server map4.yaml
 
You should see the following

![RViz Screenshot](https://github.com/xmfcx/ompl_rrtstar_ros/blob/master/docs/rviz_screenshot.png)