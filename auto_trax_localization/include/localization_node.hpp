#pragma once

#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <barc/Encoder.h>
#include <particle_filter.hpp>

class LocalizationNode {

public:
  LocalizationNode(ros::NodeHandle nh);
  virtual ~LocalizationNode();

  void publishParticlesRViz();
  void publishParticleRViz(boost::shared_ptr<WheelBot> particle);

private:
  void depthScanCB(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void encoderCB(const barc::EncoderConstPtr& encoder_msg);
  void mapCB(const nav_msgs::OccupancyGridConstPtr& map_msg);
  void initializeParameters();
  visualization_msgs::Marker* generateMarker(boost::shared_ptr<WheelBot> particle);
  void publishPoseTF(boost::shared_ptr<WheelBot> particle);

  bool laserScanParamsInitialized = false;
  bool mapParamsInitialized = false;

  ros::NodeHandle nh_;

  ros::Subscriber depthScanSub;
  ros::Subscriber encoderSub;
  ros::Subscriber mapSub;

  ros::Publisher posePub;
  ros::Publisher particle_pub_;
  ros::Publisher particles_poses_pub_;
  ros::Publisher particle_laser_scan_pub_;

  tf::TransformBroadcaster pose_br_;

  ParticleFilter particleFilter_;
  sensor_msgs::LaserScanPtr last_scan_msg_ptr;
  boost::shared_ptr<WheelBot> initial_pose_;

  ParticleVisualProperties particleVisualProperties;

  float prev_encoder_FR_;
};
