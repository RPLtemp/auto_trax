//
// Created by frank on 10.07.16.
//

#ifndef AUTO_TRAX_PATH_PLANNING_RRT_PLANNER_H
#define AUTO_TRAX_PATH_PLANNING_RRT_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <auto_trax_msgs/MergedScan.h>

#include <Eigen/Dense>
#include <rrt/2dplane/2dplane.hpp>
#include <rrt/planning/Path.hpp>
#include <rrt/2dplane/GridStateSpace.hpp>
#include <rrt/BiRRT.hpp>
#include <iostream>
#include <vector>
#include <cmath>


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

static const float kDefaultStartX = 0;
static const float kDefaultStartY = 0;
static const float kDefaultGoalX = 4;
static const float kDefaultGoalY = 0;
static const std::string kDefaultOCGridSubTopic = "/oc_grid";
static const std::string kDefaultPathPubTopic = "/path";
static const int kDefaultPathPubQueueSize = 1;

static const int kDefaultRRTStepSize = 10;
static const int kDefaultRRTMaxStepSize = 30;
static const int kDefaultGoalMaxDist = 20;

static const float kDefaultStartVelX = 1;
static const float kDefaultStartVelY = 0;
static const float kDefaultGoalVelX = 1;
static const float kDefaultGoalVelY = 0;

static const int kDefaultRRTWeightPointsSize = 20;
static const int kDefaultRRTMaxIteration = 1000;


class RRTPathProcessor{

public:
  RRTPathProcessor(const ros::NodeHandle& nh, const OCGridParameterBag& parameters);
  virtual ~RRTPathProcessor();

  void CallbackOCGrid(const nav_msgs::OccupancyGrid &oc_grid);

  void reset();
  void setObstacleAt(float x, float y);
  void clearObstacles();
  void step(int nTimes);
  bool findSolution();
  void printPath();
  void debugPrint();
  double getFirstSetpoint();

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_oc_grid_;
  ros::Publisher pub_path_;
  ros::Publisher pub_oc_grid_debug;
  ros::Publisher pub_setpoint_;
  ros::Publisher pub_path_found_;

  nav_msgs::OccupancyGrid rrt_oc_grid_;
  nav_msgs::Path result_;
  std_msgs::Float64 setpoint_;
  std_msgs::Bool path_found_;

  std::shared_ptr<RRT::GridStateSpace> _stateSpace;
  RRT::BiRRT<Eigen::Vector2f>* _biRRT;

  Eigen::Vector2f _startVel, _goalVel;

  std::vector<Eigen::Vector2f> _previousSolution;

  OCGridParameterBag rrt_param_;

  int rrt_weighpoints_size_, rrt_max_iteration_;
};


#endif //AUTO_TRAX_PATH_PLANNING_RRT_PLANNER_H
