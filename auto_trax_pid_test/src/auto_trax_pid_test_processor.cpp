//
// Created by marius on 16.04.16.
//

#include "auto_trax_pid_test/auto_trax_pid_test_processor.h"

AutoTraxPidTest::AutoTraxPidTest(ros::NodeHandle nodehandle, ParameterBag params_bag):
    nh_(nodehandle),
    parameter_(params_bag) {
  ROS_DEBUG("Framework Processor started!");

  sub_scan_= nh_.subscribe(parameter_.subscribed_rostopic_scan,
                          parameter_.queue_size_subscriber_scan,
                          &AutoTraxPidTest::CallbackScan,
                          this);

  // Create publisher for scan distance
  pub_dist_ = nh_.advertise<std_msgs::Float64>(parameter_.pub_rostopic_dist,
                                              parameter_.queue_size_pub_dist);

  // Create publisher for merged scan
  pub_merged_scan_ = nh_.advertise<sensor_msgs::LaserScan>(parameter_.pub_rostopic_merged_scan,
                                                           parameter_.queue_size_pub_merged_scan,
                                                           true);
}

AutoTraxPidTest::~AutoTraxPidTest(){
}

void AutoTraxPidTest::CallbackScan(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  ROS_DEBUG("Scan received!");

  if (!laser_scan_left_ && scan_msg->header.frame_id == parameter_.frame_id_left)
    laser_scan_left_ = scan_msg;
  else if(!laser_scan_right_ && scan_msg->header.frame_id == parameter_.frame_id_right)
    laser_scan_right_ = scan_msg;

  if (!laser_scan_left_ || !laser_scan_right_)
    return;

  sensor_msgs::LaserScanPtr merged_scan_msg(new sensor_msgs::LaserScan());

  // Get the left-most angle and range in robot frame
  float angle_max;
  int angle_max_index;
  double rot_cos = cos(parameter_.left_camera_orientation * M_PI / 180.0);
  double rot_sin = sin(parameter_.left_camera_orientation * M_PI / 180.0);

  Eigen::Vector2d translation_left(0.0, parameter_.left_camera_offset);
  Eigen::Matrix2d rotation_left;
  rotation_left << rot_cos, -rot_sin,
                   rot_sin, rot_cos;

  float left_angle_min = laser_scan_left_->angle_min;
  float left_angle_inc = laser_scan_left_->angle_increment;

  for (int i = laser_scan_left_->ranges.size() - 1; i >= 0; i--) {
    float range = laser_scan_left_->ranges.at(i);

    if (range < laser_scan_left_->range_max && range > laser_scan_left_->range_min) {
      float angle = left_angle_min + i * left_angle_inc;

      float scan_pt_x = range * cos(angle);
      float scan_pt_y = range * sin(angle);

      Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
      Eigen::Vector2d robot_pt = rotation_left * scan_pt + translation_left;
      angle_max = atan2(robot_pt[1], robot_pt[0]);
      angle_max_index = i;

      break;
    }
  }

  // Get the right-most angle and range in robot frame
  float angle_min;
  int angle_min_index;
  rot_cos = cos(parameter_.right_camera_orientation * M_PI / 180.0);
  rot_sin = sin(parameter_.right_camera_orientation * M_PI / 180.0);

  Eigen::Vector2d translation_right(0.0, parameter_.right_camera_offset);
  Eigen::Matrix2d rotation_right;
  rotation_right << rot_cos, -rot_sin,
                    rot_sin, rot_cos;

  float right_angle_min = laser_scan_right_->angle_min;
  float right_angle_inc = laser_scan_right_->angle_increment;

  for (int i = 0; i < laser_scan_right_->ranges.size(); i++) {
    float range = laser_scan_left_->ranges.at(i);

    if (range < laser_scan_right_->range_max && range > laser_scan_right_->range_min) {
      float angle = right_angle_min + i * right_angle_inc;

      float scan_pt_x = range * cos(angle);
      float scan_pt_y = range * sin(angle);

      Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
      Eigen::Vector2d robot_pt = rotation_right * scan_pt + translation_right;
      angle_min = atan2(robot_pt[1], robot_pt[0]);
      angle_min_index = i;

      break;
    }
  }

  // Initialize the ranges vector in the merged laser scan
  int ranges_size = (angle_max - angle_min) / parameter_.angle_increment;
  merged_scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  // Transform points from left scan into robot frame
  for (int i = 0; i <= angle_max_index; i++) {
    float angle = left_angle_min + i * left_angle_inc;
    float range = laser_scan_left_->ranges.at(i);

    if (range < laser_scan_left_->range_max && range > laser_scan_left_->range_min) {
      float scan_pt_x = range * cos(angle);
      float scan_pt_y = range * sin(angle);

      Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
      Eigen::Vector2d robot_pt = rotation_left * scan_pt + translation_left;

      float angle_robot = atan2(robot_pt[1], robot_pt[0]) - angle_min;
      float range_robot = robot_pt.norm();

      int angle_ind = angle_robot / parameter_.angle_increment - 1;

      if (range_robot < merged_scan_msg->ranges.at(angle_ind) ||
              isnan(merged_scan_msg->ranges.at(angle_ind))) {
        merged_scan_msg->ranges.at(angle_ind) = range_robot;
      }
    }
  }

  // Transform points from right scan into robot frame
  for (int i = angle_min_index; i < laser_scan_right_->ranges.size(); i++) {
    float angle = right_angle_min + i * right_angle_inc;
    float range = laser_scan_right_->ranges.at(i);

    if (range < laser_scan_right_->range_max && range > laser_scan_right_->range_min) {
      float scan_pt_x = range * cos(angle);
      float scan_pt_y = range * sin(angle);

      Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
      Eigen::Vector2d robot_pt = rotation_right * scan_pt + translation_right;

      float angle_robot = atan2(robot_pt[1], robot_pt[0]) - angle_min;
      float range_robot = robot_pt.norm();

      int angle_ind = angle_robot / parameter_.angle_increment + 1;

      if (range_robot < merged_scan_msg->ranges.at(angle_ind) ||
              isnan(merged_scan_msg->ranges.at(angle_ind))) {
        merged_scan_msg->ranges.at(angle_ind) = range_robot;
      }
    }
  }

  // Fill the new merged laser scan message
  merged_scan_msg->angle_min = angle_min;
  merged_scan_msg->angle_max = angle_max;
  merged_scan_msg->angle_increment = parameter_.angle_increment;
  merged_scan_msg->scan_time = laser_scan_right_->scan_time;
  merged_scan_msg->range_min = 0.2;
  merged_scan_msg->range_max = 2.0;
  merged_scan_msg->header.frame_id = "robot";

  // Publish the new merged laser scan message
  pub_merged_scan_.publish(merged_scan_msg);

  // Clear the individual laser scan messages
  laser_scan_left_.reset();
  laser_scan_right_.reset();
}

void AutoTraxPidTest::AvgScanDistance (const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  size_t effective_scan_size = 0;

  for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
    if (scan_msg->ranges.at(i) > scan_msg->range_min && scan_msg->ranges.at(i) < scan_msg->range_max) {
      effective_scan_size++;
    }
  }

  if(effective_scan_size == 0){
    ROS_WARN("SCAN SIZE = 0");
    return;
  }

  double sum = 0;
  for (int i = 0; i < effective_scan_size; ++i) {
    if (scan_msg->ranges.at(i) > scan_msg->range_min && scan_msg->ranges.at(i) < scan_msg->range_max) {
      sum += scan_msg->ranges[i];
    }
  }

  double average = sum/effective_scan_size;
  std_msgs::Float64 result;
  result.data = average;

  pub_dist_.publish(result);
}
