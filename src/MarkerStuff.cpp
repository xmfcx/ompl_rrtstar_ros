#include "MarkerStuff.h"

visualization_msgs::Marker MarkerStuff::Lines(
  const std::vector<geometry_msgs::Point> &points,
  const std::string &frame_id, int r, int g, int b, float thicc) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.points = points;

  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = thicc;

  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  marker.pose.position.z = 1;
  marker.pose.orientation.w = 1;

  return marker;
}
