#ifndef SRC_VALIDITYCHECKER_H
#define SRC_VALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ob = ompl::base;

class ValidityChecker : public ob::StateValidityChecker {
public:
  explicit ValidityChecker(const ob::SpaceInformationPtr &si);

  // Returns whether the given state's position overlaps the
  // circular obstacle
  bool isValid(const ob::State *state) const override;

  void setMap(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &map);

private:
  float width_;
  float height_;
  float res_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
};


#endif //SRC_VALIDITYCHECKER_H
