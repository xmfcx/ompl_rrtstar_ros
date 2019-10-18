#include "MyMotionValidator.h"

MyMotionValidator::MyMotionValidator(const ob::SpaceInformationPtr &si) :
  ob::MotionValidator(si) {
}

void MyMotionValidator::setMap(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &map) {
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

bool MyMotionValidator::checkMotion(const ob::State *state1, const ob::State *state2) const {

  const ob::RealVectorStateSpace::StateType *s1 =
    state1->as<ob::RealVectorStateSpace::StateType>();
  double x1 = s1->values[0];
  double y1 = s1->values[1];
  const ob::RealVectorStateSpace::StateType *s2 =
    state2->as<ob::RealVectorStateSpace::StateType>();
  double x2 = s2->values[0];
  double y2 = s2->values[1];


  int disc_x1 = std::floor((x1 - res_ / 2) / res_);
  int disc_y1 = std::floor((y1 - res_ / 2) / res_);
  int disc_x2 = std::floor((x2 - res_ / 2) / res_);
  int disc_y2 = std::floor((y2 - res_ / 2) / res_);

//  std::cout << "disc_x1: " << disc_x1 << std::endl;
//  std::cout << "disc_y1: " << disc_y1 << std::endl;
//  std::cout << "disc_x2: " << disc_x2 << std::endl;
//  std::cout << "disc_y2: " << disc_y2 << std::endl;
//  auto is_white = [this](double x, double y) {
//    int disc_x = std::floor(x / res_);
//    int disc_y = std::floor(y / res_);
////    std::cout << "disc_x: " << disc_x << std::endl;
////    std::cout << "disc_y: " << disc_y << std::endl;
//    int ind = disc_x + disc_y * map_->info.width;
//    int numm = map_->data[ind];
////    std::cout << "data: " << numm << std::endl;
//    bool is_white = numm < 50;
////    std::cout << "is_white: " << is_white << std::endl;
//    return is_white;
//  };


//  if (!is_white(x2, y2))
//    return false;
  if (y2 >= height_)
    return false;


//  int disc_x1 = std::floor(x1 / res_);
//  int disc_y1 = std::floor(y1 / res_);
//
//  int disc_x2 = std::floor(x2 / res_);
//  int disc_y2 = std::floor(y2 / res_);

  int delta_x = std::floor(std::abs(disc_x1 - disc_x2));
  int delta_y = std::floor(std::abs(disc_y1 - disc_y2));

//  if (delta_x != 0 && delta_y != 0)
//    return false;
//
//  if (delta_x == 0 && delta_y == 0)
//    return false;

  auto is_white = [this](int i, int j) {
    int ind = i + j * map_->info.width;
    int numm = map_->data[ind];
    return numm < 50;
  };

  if (delta_y != 0) {
    float y_big;
    float y_small;
    if (disc_y2 > disc_y1) {
      y_big = disc_y2;
      y_small = disc_y1;
    } else {
      y_big = disc_y1;
      y_small = disc_y2;
    }

    for (size_t i = y_small; i <= y_big; ++i) {
      if (!is_white(disc_x1, i)) {
        return false;
      }
    }

  }

  if (delta_x > 0) {
    float x_big;
    float x_small;
    if (disc_x2 > disc_x1) {
      x_big = disc_x2;
      x_small = disc_x1;
    } else {
      x_big = disc_x1;
      x_small = disc_x2;
    }

    for (size_t i = x_small; i <= x_big; ++i) {
      if (!is_white(i, disc_y1)) {
        return false;
      }
    }

  }


//  std::cout << "disc_x1: " << disc_x1 << std::endl;
//  std::cout << "disc_y1: " << disc_y1 << std::endl;
//  std::cout << "disc_x2: " << disc_x2 << std::endl;
//  std::cout << "disc_y2: " << disc_y2 << std::endl << std::endl;

//  if (std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)) > res_)
//    return false;

//  std::cout << "true motion: " << std::endl;
  return true;
}

bool MyMotionValidator::checkMotion(const ompl::base::State *state1,
                                    const ompl::base::State *state2,
                                    std::pair<ompl::base::State *, double> &lastValid) const {
  std::cout << "check2222222" << std::endl;
  return checkMotion(state1, state2);
}
