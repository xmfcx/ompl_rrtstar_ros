#include "ValidityChecker.h"

ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr &si) :
  ob::StateValidityChecker(si) {
}

bool ValidityChecker::isValid(const ob::State *state) const {

  const ob::RealVectorStateSpace::StateType *state2D =
    state->as<ob::RealVectorStateSpace::StateType>();
  // Extract the robot's (x,y) position from its state
  double x = state2D->values[0];
  double y = state2D->values[1];

  auto is_white = [this](double x, double y) {
    int disc_x = std::floor(x / res_);
    int disc_y = std::floor(y / res_);
//    std::cout << "disc_x: " << disc_x << std::endl;
//    std::cout << "disc_y: " << disc_y << std::endl;
    int ind = disc_x + disc_y * map_->info.width;
    int numm = map_->data[ind];
//    std::cout << "data: " << numm << std::endl;
    bool is_white = numm < 50;
//    std::cout << "is_white: " << is_white << std::endl;
    return is_white;
  };

  bool aa;
//  if(x * res_)

  return y < height_ && is_white(x, y);
}

void ValidityChecker::setMap(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &map) {
  map_ = map;
  res_ = map_->info.resolution;
  width_ = map_->info.width * res_;
  height_ = map_->info.height * res_;
  std::cout << "width: " << map_->info.width << std::endl;
  std::cout << "height: " << map_->info.height << std::endl;
  std::cout << "resolution: " << map_->info.resolution << std::endl;
  std::cout << "origin.position.x: " << map_->info.origin.position.x << std::endl;
  std::cout << "origin.position.y: " << map_->info.origin.position.y << std::endl;
  std::cout << "origin.position.z: " << map_->info.origin.position.z << std::endl;
}


