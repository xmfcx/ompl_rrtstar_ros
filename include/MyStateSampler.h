#ifndef SRC_MYSTATESAMPLER_H
#define SRC_MYSTATESAMPLER_H

#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>
#include <ompl/base/SpaceInformation.h>

#include <nav_msgs/OccupancyGrid.h>
#include <queue>

namespace ob = ompl::base;
//namespace og = ompl::geometric;

/// @cond IGNORE
// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
class MyStateSampler : public ob::StateSampler {
public:
  MyStateSampler(const ob::StateSpace *space);

//  // Generate a sample in the valid part of the R^3 state space
//  // Valid states satisfy the following constraints:
//  // -1<= x,y,z <=1
//  // if .25 <= z <= .5, then |x|>.8 and |y|>.8
  void sampleUniform(ob::State *state) override;

  // We don't need this in the example below.
  void sampleUniformNear(ob::State *state, const ob::State *near, const double distance) override;

  void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev) override;



  void setMap(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &map);

protected:
  ompl::RNG rng_;
private:
  float width_;
  float height_;
  float res_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
  std::vector<std::pair<int,int>> valid_states_;

};

#endif //SRC_MYSTATESAMPLER_H
