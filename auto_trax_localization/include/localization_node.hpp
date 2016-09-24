#pragma once

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <barc/Encoder.h>
#include <include/particle_filter.hpp>

class LocalizationNode{

public:
  LocalizationNode(ros::NodeHandle nh);
  void publishParticlesRViz();


private:
  void depthScanCB(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void encoderCB(const barc::EncoderConstPtr& encoder_msg);
  void initializeParameters();
  visualization_msgs::Marker* generateMarker(boost::shared_ptr<WheelBot> particle);


  ros::NodeHandle nh_;
  ros::Subscriber depthScanSub, encoderSub;
  ros::Publisher posePub;
  ros::Publisher particles_pub_;
  ros::Publisher particles_poses_pub_;

  ParticleFilter particleFilter_;

  boost::shared_ptr<WheelBot> initial_pose_;

  ParticleVisualProperties particleVisualProperties;

  float prev_encoder_FR_;
};
