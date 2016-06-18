//
// Created by marius on 16.04.16.
//

#ifndef CATKINPKG_FRAMEWORK_PARAMETER_BAG_H
#define CATKINPKG_FRAMEWORK_PARAMETER_BAG_H

#include <string>

struct ParameterBag
{
  // Parameter specific to parameter bag
  std::string node_name;

  std::string subscribed_rostopic_scan;
  int queue_size_subscriber_scan;

  std::string pub_rostopic_dist;
  int queue_size_pub_dist;

  std::string pub_rostopic_merged_scan;
  int queue_size_pub_merged_scan;

  std::string frame_id_left;
  std::string frame_id_right;

  float angle_increment;
  float left_camera_offset;
  float left_camera_orientation;
  float right_camera_offset;
  float right_camera_orientation;
};

#endif //CATKINPKG_FRAMEWORK_PARAMETER_BAG_H
