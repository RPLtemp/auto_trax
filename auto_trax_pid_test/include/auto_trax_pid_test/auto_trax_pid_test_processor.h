//
// Created by marius on 16.04.16.
//

#ifndef CATKINPKG_FRAMEWORK_FRAMEWORKPROCESSOR_H
#define CATKINPKG_FRAMEWORK_FRAMEWORK_PROCESSOR_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <vector>

#include "auto_trax_pid_test/parameter/parameter_bag.h"

// Default values
static const std::string kDefaultDistPubTopic = "distance_result";
static const int kDefaultDistPubQueueSize = 1;
static const std::string kDefaultMergedScanPubTopic = "merged_scan";
static const int kDefaultMergedScanPubQueueSize = 1;
static const std::string kDefaultScanSubTopic = "/scan";
static const int kDefaultScanSubQueueSize = 1;
static const float kDefaultAngleIncrement = 0.01;
static const std::string kDefaultFrameIdLeft = "camera_left";
static const std::string kDefaultFrameIdRight = "camera_right";
static const float kDefaultLeftCameraOffset = 0.15;
static const float kDefaultLeftCameraOrientation = M_PI * 0.25;
static const float kDefaultRightCameraOffset = -0.15;
static const float kDefaultRightCameraOrientation = -M_PI * 0.25;

class AutoTraxPidTest
{
public:
  // Constructor with nodehandle and parameters
  AutoTraxPidTest (ros::NodeHandle nh, ParameterBag parameter);
  virtual ~AutoTraxPidTest();

  // Callback
  void CallbackScan (const sensor_msgs::LaserScan::ConstPtr &scan_msg);

  void AvgScanDistance (const sensor_msgs::LaserScan::ConstPtr &scan_msg);

 private:
  ros::NodeHandle nh_;
  ParameterBag parameter_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_dist_;
  ros::Publisher pub_merged_scan_;
  sensor_msgs::LaserScanConstPtr laser_scan_left_;
  sensor_msgs::LaserScanConstPtr laser_scan_right_;
};


#endif //CATKINPKG_FRAMEWORK_FRAMEWORKPROCESSOR_H
