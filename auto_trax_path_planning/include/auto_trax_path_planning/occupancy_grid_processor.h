//
// Created by frank on 10.07.16.
//

#ifndef AUTO_TRAX_PATH_PLANNING_OCCUPANCY_GRID_CREATOR_H
#define AUTO_TRAX_PATH_PLANNING_OCCUPANCY_GRID_CREATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <auto_trax_msgs/MergedScan.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <vector>

#include "auto_trax_path_planning/parameter/occupancy_grid_parameters.h"

// Default values
static const std::string kDefaultOCGridPubTopic = "/oc_grid";
static const int kDefaultOCGridQueueSize = 1;
static const std::string kDefaultScanSummarySubTopic = "/scan_summary";
static const int kDefaultScanSummarySubQueueSize = 1;
static const int kDefaultGridHeight = 400;
static const int kDefaultGridWidth = 400;
static const float kDefaultGridResolution = 1;
static const float kDefaultGridOriginPositionX = 0;
static const float kDefaultGridOriginPositionY = 0;
static const float kDefaultGridOriginPositionZ = 0;
static const float kDefaultGridOriginQuaternionX = 0;
static const float kDefaultGridOriginQuaternionY = 0;
static const float kDefaultGridOriginQuaternionZ = 0;
static const float kDefaultGridOriginQuaternionW = 1;
static const std::string kDefaultGridFrameID = "/robot";
static const float kDefaultObstaclePadding = 0.1;
static const float kDefaultSensorAvailableRange = 4;

class OCGridProcessor
{
public:
  OCGridProcessor(const ros::NodeHandle& nh, const OCGridParameterBag& parameters);
  virtual ~OCGridProcessor();

  // Callback
  void CallbackScanSummary(const auto_trax_msgs::MergedScan &scan_center_msg);

  // Returns the row major single index equivalent of the Cartesian i,j
  int indexOf(int i, int j);

  void SetIsObstacle(int i, int j);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_scan_summary_;
  ros::Publisher pub_oc_grid_;
  OCGridParameterBag oc_grid_param_;
  nav_msgs::OccupancyGrid result_;
  float obstacle_padding_;

};

#endif //AUTO_TRAX_PATH_PLANNING_OCCUPANCY_GRID_CREATOR_H
