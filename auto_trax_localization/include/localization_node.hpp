#pragma once

#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <auto_trax_msgs/IOSetpoint.h>
#include <Eigen/Core>

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
  bool SteeringServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                               auto_trax_msgs::IOSetpoint::Response &res);
  bool MotorServiceCallback(auto_trax_msgs::IOSetpoint::Request  &req,
                            auto_trax_msgs::IOSetpoint::Response &res);
  void ForwardKinematics(Eigen::Matrix<float, 3, 1>& xi_dot_robot);
  void GetGlobalPosition(geometry_msgs::PointPtr robot_position,
                         const geometry_msgs::Point& start_position,
                         const Eigen::Matrix<float, 3, 1>& robot_velocity,
                         const double& delte_time);
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
  ros::ServiceServer steering_service_;
  ros::ServiceServer motor_service_;

  std::string steering_service_name_;
  std::string motor_service_name_;

  double angle_in_radians_, speed_in_m_s_;



  ParticleFilter particleFilter_;
  sensor_msgs::LaserScanPtr last_scan_msg_ptr;
  boost::shared_ptr<WheelBot> initial_pose_;

  ParticleVisualProperties particleVisualProperties;

  float prev_encoder_FR_;
};
