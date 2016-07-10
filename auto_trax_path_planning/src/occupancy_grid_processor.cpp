//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/occupancy_grid_processor.h"

OCGridProcessor::OCGridProcessor(const ros::NodeHandle& nh, OCGridParameterBag parameters)
: nh_(nh),
  oc_grid_param_(parameters)
{
  ROS_DEBUG("Occupancy Grid Processor started!");

  sub_scan_summary_ = nh_.subscribe("/scan_center",
                                    1,
                                    &OCGridProcessor::CallbackScanSummary,
                                    this);
  pub_oc_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>(oc_grid_param_.published_rostopic_oc_grid,
                                                        oc_grid_param_.queue_size_pub_oc_grid);
}

OCGridProcessor::~OCGridProcessor() {}

void OCGridProcessor::CallbackScanSummary(const auto_trax_msgs::MergedScan &scan_summary)
{
  ROS_DEBUG("Scan Summary received!");
  ROS_WARN("The limit on the right is %.2f", scan_summary.valid_angle_min);

}

