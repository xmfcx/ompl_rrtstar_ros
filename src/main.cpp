#include <ros/ros.h>
#include "Planner.h"

int main(int argc, char **argv) {
  std::cout << std::fixed;
  std::cout << std::setprecision(10);
  ros::init(argc, argv, "ompl_rrtstar_ros");
  std::cout << "ompl_rrtstar_ros instance has started!" << std::endl;

  ros::NodeHandle nh("~");

  Planner planner(nh);

  ros::spin();
  ros::waitForShutdown();
  return 0;
}