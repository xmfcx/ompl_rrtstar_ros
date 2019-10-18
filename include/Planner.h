#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <limits>
#include <boost/math/constants/constants.hpp>

#include <ros/ros.h>
#include <thread>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "MarkerStuff.h"
#include <nav_msgs/OccupancyGrid.h>
#include "ValidityChecker.h"
#include "MyMotionValidator.h"
#include "MyStateSampler.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


class Planner {
public:
  explicit Planner(ros::NodeHandle &nh);

private:

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
  ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si) {
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
  }


  ros::NodeHandle &nh_;

  ros::Publisher pub_marker;
  ros::Publisher pub_marker_shortest;
  ros::Subscriber sub_map;

  void CallbackMap(const nav_msgs::OccupancyGrid::ConstPtr &msg_map);

};


#endif //SRC_PLANNER_H
