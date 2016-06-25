//
// Created by marius on 16.04.16.
//

#ifndef WALL_FOLLOWING_TEST_PROCESSOR_H
#define WALL_FOLLOWING_TEST_PROCESSOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <vector>

#include "auto_trax_tests/parameter/wall_following_bag.h"

// Default values
static const std::string kDefaultDistPubTopic = "distance_result";
static const int kDefaultDistPubQueueSize = 1;
static const std::string kDefaultScanSubTopic = "/scan";
static const int kDefaultScanSubQueueSize = 1;

class WallFollowingTest
{
public:
  // Constructor with nodehandle and parameters
  WallFollowingTest (ros::NodeHandle nh, WallFollowingTestBag parameter);
  virtual ~WallFollowingTest();

  // Callback
  void CallbackScan (const sensor_msgs::LaserScan::ConstPtr &scan_msg);

 private:
  ros::NodeHandle nh_;
  WallFollowingTestBag parameter_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_dist_;
};

#endif //WALL_FOLLOWING_TEST_PROCESSOR_H
