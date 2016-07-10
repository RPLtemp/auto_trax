//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/rrt_path_processor.h"

RRTPathProcessor::RRTPathProcessor(const ros::NodeHandle& nh) : nh_(nh) {
  ROS_DEBUG("RRT Path Processor started!");

  sub_oc_grid_ = nh_.subscribe("/oc_grid",
                                    1,
                                    &RRTPathProcessor::CallbackOCGrid,
                                    this);
  pub_path_ = nh_.advertise<nav_msgs::Path>("rrt_path",
                                                        1);

}

RRTPathProcessor::~RRTPathProcessor(){ }

void RRTPathProcessor::CallbackOCGrid(const nav_msgs::OccupancyGrid &oc_grid){

}
