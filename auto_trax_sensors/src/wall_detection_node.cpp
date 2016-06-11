#include "auto_trax_sensors/wall_detection_node.h"

namespace auto_trax {

WallDetectionNode::WallDetectionNode() {
  ros::NodeHandle pnh("~");

  std::string laser_scan_sub_topic;
  std::string wall_angle_pub_topic;
  pnh.param("laser_scan_topic", laser_scan_sub_topic, kDefaultLaserScanSubTopic);
  pnh.param("wall_angle_topic", wall_angle_pub_topic, kDefaultWallAnglePubTopic);

  laser_scan_sub_ = nh_.subscribe(laser_scan_sub_topic, 1, &WallDetectionNode::LaserScanCallback, this);
  wall_angle_pub_ = nh_.advertise<std_msgs::Float64>(wall_angle_pub_topic, 1);
}

WallDetectionNode::~WallDetectionNode() {

}

void WallDetectionNode::LaserScanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg) {
  // Initialize the scan hit points
  std::pair<float, float> first_point = std::pair<float, float>(0.0, 0.0);
  std::pair<float, float> second_point = std::pair<float, float>(0.0, 0.0);

  // Extract the scan parameters
  float range_min = scan_msg->range_min;
  float range_max = scan_msg->range_max;
  float angle_min = scan_msg->angle_min;
  float angle_max = scan_msg->angle_max;
  float angle_inc = scan_msg->angle_increment;

  // Initialize the highest detected wall angle (0.0 angle means we are parallel to the wall)
  float greatest_wall_angle = 0.0;

  // Assume that all the scan points are in front of the laser sensor
  assert(angle_min >= (-M_PI * 0.5) && angle_max <= (M_PI * 0.5));

  // Loop over all the points in the scan
  for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
    float range = scan_msg->ranges.at(i);

    // Only process points that are within the valid range
    if (range > range_min && range < range_max) {
      // Compute the x,y coordinates (in the robot frame) of the scan point
      float point_angle = angle_min + angle_inc * i;
      float x = fabs(range * cos(point_angle));
      float y = range * sin(point_angle);

      // Check if this is our first hit, only compute the angle once we have two hits
      if (first_point.first == 0.0 && first_point.second == 0.0) {
        first_point = std::pair<float, float>(x, y);
        continue;
      }
      else
        second_point = std::pair<float, float>(x, y);

      // Compute the angle between two hits
      float wall_angle;
      float dx = fabs(first_point.first - second_point.first);
      float dy = fabs(first_point.second - second_point.second);

      if (dx < 0.001)
        wall_angle = M_PI * 0.5;
      else if (dy < 0.001)
        wall_angle = 0.0;
      else
        wall_angle = atan(dy / dx);

      assert(wall_angle >= 0.0);

      // Check if the new angle is the highest one yet
      if (wall_angle > greatest_wall_angle)
        greatest_wall_angle = wall_angle;

      // Shift the points to check by one
      first_point = second_point;
    }
  }

  // Publish the newly detected wall angle
  std_msgs::Float64 wall_angle_msg;
  wall_angle_msg.data = greatest_wall_angle;
  wall_angle_pub_.publish(wall_angle_msg);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_detection_node");

  auto_trax::WallDetectionNode wall_detection_node;

  ros::spin();

  return 0;
}
