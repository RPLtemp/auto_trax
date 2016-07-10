//
// Created by frank on 10.07.16.
//

#ifndef AUTO_TRAX_PATH_PLANNING_RRT_PLANNER_H
#define AUTO_TRAX_PATH_PLANNING_RRT_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <auto_trax_msgs/MergedScan.h>

#include <iostream>
#include <vector>

class RRTPathProcessor{

public:
  RRTPathProcessor(const ros::NodeHandle& nh);
  virtual ~RRTPathProcessor();

  void CallbackOCGrid(const nav_msgs::OccupancyGrid &oc_grid);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_oc_grid_;
  ros::Publisher pub_path_;
  nav_msgs::Path result_;
};


#endif //AUTO_TRAX_PATH_PLANNING_RRT_PLANNER_H
