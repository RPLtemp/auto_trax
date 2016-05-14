//
// Created by marius on 16.04.16.
//

#ifndef CATKINPKG_FRAMEWORK_FRAMEWORKPROCESSOR_H
#define CATKINPKG_FRAMEWORK_FRAMEWORK_PROCESSOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <vector>

#include "auto_trax_pid_test/parameter/parameter_bag.h"

// Default values
static const std::string kDefaultImageSubTopic = "/camera/depth/image";
static const int kDefaultImageSubQueueSize = 1;
static const std::string kDefaultDistPubTopic = "distance_result";
static const int kDefaultDistPubQueueSize = 1;
static const std::string kDefaultScanSubTopic = "/scan";
static const int kDefaultScanSubQueueSize = 1;

class AutoTraxPidTest
{
public:
  // Constructor with nodehandle and parameters
  AutoTraxPidTest (ros::NodeHandle nh, ParameterBag parameter);
  virtual ~AutoTraxPidTest();

  // Callback
  void CallbackImg (const sensor_msgs::ImageConstPtr &a_image_msg);
  void CallbackScan (const sensor_msgs::LaserScan::ConstPtr &a_scan_msg);

 private:
  ros::NodeHandle nh_;
  ParameterBag parameter_;
  ros::Subscriber sub_img_, sub_scan_;
  ros::Publisher pub_dist_;
};


#endif //CATKINPKG_FRAMEWORK_FRAMEWORKPROCESSOR_H
