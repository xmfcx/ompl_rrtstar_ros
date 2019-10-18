#include "MyStateSampler.h"

void MyStateSampler::sampleUniform(ob::State *state) {
  double *val = static_cast<ob::RealVectorStateSpace::StateType *>(state)->values;

//  int rows = map_->info.height;
//  int cols = map_->info.width;
////  std::cout << "called MyValidStateSampler::sample" << std::endl;
//
//  int random_row = rng_.uniformInt(0, rows - 1);
//  int random_col = rng_.uniformInt(0, cols - 1);
//  if (valid_states_.empty()) {
//    val[0] = res_ / 2;
//    val[1] = res_ / 2;
//    std::cout << "empty: " << std::endl;
//    return;
//  }

  std::pair<int, int> pair = valid_states_[rng_.uniformInt(0, valid_states_.size() - 1)];

  val[0] = pair.first * res_ + res_ / 2;
  val[1] = pair.second * res_ + res_ / 2;

}

MyStateSampler::MyStateSampler(const ob::StateSpace *space) : StateSampler(space) {
//  name_ = "my sampler";
}

void MyStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance) {
  std::cout << "aaaaaaaaaa: " << std::endl;
  throw ompl::Exception("MyStateSampler::sampleUniformNear", "not implemented");
}

void MyStateSampler::setMap(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &map) {
  std::cout << "setMap for MyValidStateSampler" << std::endl;
  map_ = map;
  res_ = map_->info.resolution;
  width_ = map_->info.width * res_;
  height_ = map_->info.height * res_;
  int rows = map_->info.height;
  int cols = map_->info.width;
  for (size_t i = 0; i < cols; ++i) {
    for (size_t j = 0; j < rows; ++j) {
      int ind = i + j * cols;
      int numm = map_->data[ind];
      if (numm < 50)
        valid_states_.push_back(std::make_pair(i, j));
    }
//    std::cout << "pushin " << i << std::endl;
  }
  std::cout << "valid_states_size: " << valid_states_.size() << std::endl;
//  std::cout << "width: " << map_->info.width << std::endl;
//  std::cout << "height: " << map_->info.height << std::endl;
//  std::cout << "resolution: " << map_->info.resolution << std::endl;
//  std::cout << "origin.position.x: " << map_->info.origin.position.x << std::endl;
//  std::cout << "origin.position.y: " << map_->info.origin.position.y << std::endl;
//  std::cout << "origin.position.z: " << map_->info.origin.position.z << std::endl;
}

void MyStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev) {
  std::cout << "bbbbbbbbbbbbbb: " << std::endl;
  throw ompl::Exception("MyStateSampler::sampleUniformNear", "not implemented");
}
