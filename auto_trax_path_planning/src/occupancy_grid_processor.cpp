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
}

OCGridProcessor::~OCGridProcessor() {}

void OCGridProcessor::CallbackScanSummary(const auto_trax_msgs::MergedScan &scan_summary)
{
  ROS_DEBUG("Scan Summary received!");

}

