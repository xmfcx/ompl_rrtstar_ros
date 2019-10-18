#ifndef SRC_MOTIONVALIDATOR_H
#define SRC_MOTIONVALIDATOR_H

#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ob = ompl::base;

class MyMotionValidator : public ob::MotionValidator {
public:

  explicit MyMotionValidator(const ob::SpaceInformationPtr &si);


  /** \brief Check if the path between two states (from \e s1 to \e s2) is valid.
   * This function assumes \e s1 is valid.

      \note This function updates the number of valid and invalid segments. */
  bool checkMotion(const ob::State *state1, const ob::State *state2) const override;

  void setMap(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &map);

  bool checkMotion(const ompl::base::State *s1,
                   const ompl::base::State *s2,
                   std::pair<ompl::base::State *, double> &lastValid) const override;

private:
  float width_;
  float height_;
  float res_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
};


#endif //SRC_MOTIONVALIDATOR_H
