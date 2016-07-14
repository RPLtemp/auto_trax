//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/occupancy_grid_processor.h"

OCGridProcessor::OCGridProcessor(const ros::NodeHandle& nh, const OCGridParameterBag& parameters)
: nh_(nh),
  oc_grid_param_(parameters)
{
  ROS_DEBUG("Occupancy Grid Processor started!");

  sub_scan_summary_ = nh_.subscribe(oc_grid_param_.subscribed_rostopic_scan_summary,
                                    oc_grid_param_.queue_size_subscriber_scan_summary,
                                    &OCGridProcessor::CallbackScanSummary,
                                    this);
  pub_oc_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>(oc_grid_param_.published_rostopic_oc_grid,
                                                        oc_grid_param_.queue_size_pub_oc_grid);

  result_.header.frame_id = oc_grid_param_.frame_id;
  result_.info.width = oc_grid_param_.grid_width;
  result_.info.height = oc_grid_param_.grid_height;

//  result_.info.resolution = oc_grid_param_.grid_resolution;
  result_.info.resolution = oc_grid_param_.sensor_available_range / oc_grid_param_.grid_width;
  oc_grid_param_.grid_resolution = result_.info.resolution;
  result_.info.origin.position.x = oc_grid_param_.origin_position_x;
  result_.info.origin.position.y = oc_grid_param_.origin_position_y;
  result_.info.origin.position.z = oc_grid_param_.origin_position_z;
  result_.info.origin.orientation.x = oc_grid_param_.origin_quaternion_x;
  result_.info.origin.orientation.y = oc_grid_param_.origin_quaternion_y;
  result_.info.origin.orientation.z = oc_grid_param_.origin_quaternion_z;
  result_.info.origin.orientation.w = oc_grid_param_.origin_quaternion_w;
  result_.data.resize(result_.info.width * result_.info.height);

  obstacle_padding_ = oc_grid_param_.obstacle_padding;

  if(obstacle_padding_ < result_.info.resolution)
  {
    ROS_WARN("Obstacle padding %.2f is smaller than the grid resolution %.2f. Effectively, there is no padding applied!", obstacle_padding_, result_.info.resolution);
  }
}

OCGridProcessor::~OCGridProcessor() {}

int OCGridProcessor::indexOf(int i, int j)
{
  return i * oc_grid_param_.grid_width + j;
}

void OCGridProcessor::SetIsObstacle(int i, int j)
{
  int data_index = indexOf(i,j);

  if (data_index < 0 || data_index > result_.data.size() ||
          result_.data.at(data_index) == -1 || result_.data.at(data_index) == 100)
  {
    return;
  }
  result_.data.at(data_index) = 100;

}

void OCGridProcessor::CallbackScanSummary(const auto_trax_msgs::MergedScan &scan_summary)
{
  ROS_DEBUG("Scan Summary received!");
  result_.data.resize(result_.info.width * result_.info.height);

  float scanned_point_angle = scan_summary.merged_scan.angle_min;

  for (int i = 0; i < scan_summary.merged_scan.ranges.size(); i++)
  {
    // To be appended later to account for robot pose

    float scanned_point_depth = scan_summary.merged_scan.ranges.at(i);
    float scanned_point_x = cos(scanned_point_angle) * scanned_point_depth
                            - oc_grid_param_.origin_position_x;
    float scanned_point_y = sin(scanned_point_angle) * scanned_point_depth
                            - oc_grid_param_.origin_position_y;
    int scanned_point_index_i = scanned_point_y / result_.info.resolution;
    int scanned_point_index_j = scanned_point_x / result_.info.resolution;

    int index_wise_padding = int(obstacle_padding_ / oc_grid_param_.grid_resolution);

    SetIsObstacle(scanned_point_index_i,scanned_point_index_j);

    for (int j = -index_wise_padding; j < index_wise_padding; j++){
      for (int k = -index_wise_padding; k < index_wise_padding; k++){
        SetIsObstacle(scanned_point_index_i+j,scanned_point_index_j+k);
      }
    }
    scanned_point_angle += scan_summary.merged_scan.angle_increment;
  }

  // Close the line between the origin and the left and right most corners from
  // the scan
  float wall_angles[] = {scan_summary.valid_angle_min,scan_summary.valid_angle_max};
  for (float scan_angle : wall_angles) {
    for (float scanned_point_depth = 0; scanned_point_depth < scan_summary.valid_range_max;) {
      // To be appended later to account for robot pose
      float scanned_point_x = cos(scan_angle) * scanned_point_depth
                              - oc_grid_param_.origin_position_x
                              - 2 * obstacle_padding_;
      float scanned_point_y = sin(scan_angle) * scanned_point_depth
                              - oc_grid_param_.origin_position_y;
      int scanned_point_index_i = scanned_point_y / result_.info.resolution;
      int scanned_point_index_j = scanned_point_x / result_.info.resolution;

      int index_wise_padding = int(obstacle_padding_ / oc_grid_param_.grid_resolution);

      SetIsObstacle(scanned_point_index_i, scanned_point_index_j);

      for (int j = -index_wise_padding; j < index_wise_padding; j++) {
        for (int k = -index_wise_padding; k < index_wise_padding; k++) {
          SetIsObstacle(scanned_point_index_i + j, scanned_point_index_j + k);
        }
      }

      scanned_point_depth += 0.1;
    }
  }

  pub_oc_grid_.publish(result_);
  result_.data.clear();
}



