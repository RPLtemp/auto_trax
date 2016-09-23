#pragma once
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <barc/Encoder.h>
#include <include/particle_filter.hpp>

class LocalizationNode{

public:
  LocalizationNode(ros::NodeHandle nh);

private:
  void depthScanCB(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void encoderCB(const barc::EncoderConstPtr& encoder_msg);

  ros::NodeHandle nh_;
  ros::Subscriber depthScanSub, encoderSub;
  ros::Publisher posePub;
  ParticleFilter particleFilter_;

};