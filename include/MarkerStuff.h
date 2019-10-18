#ifndef SRC_MARKERSTUFF_H
#define SRC_MARKERSTUFF_H

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class MarkerStuff {
public:
  static visualization_msgs::Marker Lines(
    const std::vector<geometry_msgs::Point> &points,
    const std::string &frame_id, int r, int g, int b, float thicc);
};


#endif //SRC_MARKERSTUFF_H
