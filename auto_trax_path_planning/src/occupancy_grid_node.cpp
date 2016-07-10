//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/occupancy_grid_processor.h"

void InitializeParameters(const ros::NodeHandle& nh, OCGridParameterBag* parameter){
  nh.param("subscribed_rostopic_scan_summary", parameter->subscribed_rostopic_scan_summary, kDefaultScanSummarySubTopic);
  nh.param("queue_size_subscriber_scan_summary", parameter->queue_size_subscriber_scan_summary, kDefaultScanSummarySubQueueSize);
  nh.param("published_rostopic_oc_grid", parameter->published_rostopic_oc_grid, kDefaultOCGridPubTopic);
  nh.param("queue_size_pub_oc_grid", parameter->queue_size_pub_oc_grid, kDefaultOCGridQueueSize);
  nh.param("grid_height", parameter->grid_height, kDefaultGridHeight);
  nh.param("grid_width", parameter->grid_width, kDefaultGridWidth);
  nh.param("grid_resolution", parameter->grid_resolution, kDefaultGridResolution);
  nh.param("origin_position_x", parameter->origin_position_x, kDefaultGridOriginPositionX);
  nh.param("origin_position_y", parameter->origin_position_y, kDefaultGridOriginPositionY);
  nh.param("origin_position_z", parameter->origin_position_z, kDefaultGridOriginPositionZ);
  nh.param("origin_quaternion_x", parameter->origin_quaternion_x, kDefaultGridOriginQuaternionX);
  nh.param("origin_quaternion_y", parameter->origin_quaternion_y, kDefaultGridOriginQuaternionY);
  nh.param("origin_quaternion_z", parameter->origin_quaternion_z, kDefaultGridOriginQuaternionZ);
  nh.param("origin_quaternion_w", parameter->origin_quaternion_w, kDefaultGridOriginQuaternionW);
  nh.param("frame_id", parameter->frame_id, kDefaultGridFrameID);
  nh.param("obstacle_padding", parameter->obstacle_padding, kDefaultObstaclePadding);
  nh.param("sensor_available_range", parameter->sensor_available_range, kDefaultSensorAvailableRange);

}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "auto_trax_oc_grid_node");
  ros::NodeHandle nh;

  OCGridParameterBag parameter;
  InitializeParameters(nh, &parameter);

  OCGridProcessor ocGridProcessor(nh, parameter);

  ros::spin();
}
