//
// Created by frank on 10.07.16.
//

#ifndef AUTO_TRAX_PATH_PLANNING_OC_GRID_PARAMETERS_H
#define AUTO_TRAX_PATH_PLANNING_OC_GRID_PARAMETERS_H

#include <string>

struct OCGridParameterBag
{
  std::string node_name;

  std::string subscribed_rostopic_scan_summary;
  int queue_size_subscriber_scan_summary;

  std::string published_rostopic_oc_grid;
  int queue_size_pub_oc_grid;

  int grid_height, grid_width;

  float grid_resolution;      // ROS occupancy grid convention is row major

  float origin_position_x, origin_position_y, origin_position_z;

  float origin_quaternion_x, origin_quaternion_y, origin_quaternion_z, origin_quaternion_w;

  std::string frame_id;

  float obstacle_padding;

  float sensor_available_range;


};

#endif //AUTO_TRAX_PATH_PLANNING_OC_GRID_PARAMETERS_H
