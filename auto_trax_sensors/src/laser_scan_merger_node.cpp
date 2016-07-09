#include "auto_trax_sensors/laser_scan_merger_node.h"

namespace auto_trax {

LaserScanMerger::LaserScanMerger() {
  ros::NodeHandle pnh("~");

  // Retrieve all parameters or set to default
  pnh.param("scan_center_pub_topic", scan_center_pub_topic_, kDefaultScanCenterPubTopic);
  pnh.param("merged_scan_pub_topic", merged_scan_pub_topic_, kDefaultMergedScanPubTopic);
  pnh.param("scan_sub_topic", scan_sub_topic_, kDefaultScanSubTopic);
  pnh.param("frame_id_left", frame_id_left_, kDefaultFrameIdLeft);
  pnh.param("frame_id_right", frame_id_right_, kDefaultFrameIdRight);
  pnh.param("angle_increment", angle_increment_, kDefaultAngleIncrement);
  pnh.param("left_camera_offset", left_camera_offset_, kDefaultLeftCameraOffset);
  pnh.param("left_camera_orientation", left_camera_orientation_, kDefaultLeftCameraOrientation);
  pnh.param("right_camera_offset", right_camera_offset_, kDefaultRightCameraOffset);
  pnh.param("right_camera_orientation", right_camera_orientation_, kDefaultRightCameraOrientation);

  scan_sub_= nh_.subscribe(scan_sub_topic_, 1, &LaserScanMerger::ScanCallback, this);

  center_pt_pub_ = nh_.advertise<geometry_msgs::PointStamped>(scan_center_pub_topic_, 1, true);
  merged_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(merged_scan_pub_topic_, 1, true);
}

LaserScanMerger::~LaserScanMerger() {

}

void LaserScanMerger::ScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg) {
  ROS_DEBUG("Scan received!");

  if (!laser_scan_left_ && scan_msg->header.frame_id == frame_id_left_)
    laser_scan_left_ = scan_msg;
  else if(!laser_scan_right_ && scan_msg->header.frame_id == frame_id_right_)
    laser_scan_right_ = scan_msg;

  if (!laser_scan_left_ || !laser_scan_right_)
    return;

  geometry_msgs::PointStampedPtr scan_center_msg(new geometry_msgs::PointStamped());
  sensor_msgs::LaserScanPtr merged_scan_msg(new sensor_msgs::LaserScan());

  angle_increment_ = laser_scan_left_->angle_increment;
  float left_angle_min = laser_scan_left_->angle_min;
  float left_angle_inc = laser_scan_left_->angle_increment;
  float right_angle_min = laser_scan_right_->angle_min;
  float right_angle_inc = laser_scan_right_->angle_increment;

  // Create the objects for processing the two scans
  LaserScanProcessor left_scan_processor(laser_scan_left_);
  LaserScanProcessor right_scan_processor(laser_scan_right_);

  // Get the left-most angle and range
  float angle_max;
  int angle_max_index;
  left_scan_processor.GetMaxValidAngle(angle_max, angle_max_index);

  // Transform the max angle from left scan into robot frame
  double rot_cos = cos(left_camera_orientation_ * M_PI / 180.0);
  double rot_sin = sin(left_camera_orientation_ * M_PI / 180.0);

  Eigen::Vector2d translation_left(0.0, left_camera_offset_);
  Eigen::Matrix2d rotation_left;
  rotation_left << rot_cos, -rot_sin,
                   rot_sin, rot_cos;

  float range = laser_scan_left_->ranges.at(angle_max_index);
  float scan_pt_x = range * cos(angle_max);
  float scan_pt_y = range * sin(angle_max);

  Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
  Eigen::Vector2d robot_pt = rotation_left * scan_pt + translation_left;
  angle_max = atan2(robot_pt[1], robot_pt[0]);

  // Get the right-most angle and range
  float angle_min;
  int angle_min_index;
  right_scan_processor.GetMinValidAngle(angle_min, angle_min_index);

  // Transform the min angle from right scan into robot frame
  rot_cos = cos(right_camera_orientation_ * M_PI / 180.0);
  rot_sin = sin(right_camera_orientation_ * M_PI / 180.0);

  Eigen::Vector2d translation_right(0.0, right_camera_offset_);
  Eigen::Matrix2d rotation_right;
  rotation_right << rot_cos, -rot_sin,
                    rot_sin, rot_cos;

  range = laser_scan_right_->ranges.at(angle_min_index);
  scan_pt_x = range * cos(angle_min);
  scan_pt_y = range * sin(angle_min);

  scan_pt = Eigen::Vector2d(scan_pt_x, scan_pt_y);
  robot_pt = rotation_right * scan_pt + translation_right;
  angle_min = atan2(robot_pt[1], robot_pt[0]);

  // Initialize the ranges vector in the merged laser scan
  int ranges_size = (angle_max - angle_min) / angle_increment_;
  merged_scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  // Initialize the scan point coordinate accumulator
  Eigen::Vector2d points_sum(0.0, 0.0);
  int points_count = 0;

  // Transform points from left scan into robot frame
  for (int i = 0; i <= angle_max_index; i++) {
    float angle = left_angle_min + i * left_angle_inc;
    float range = laser_scan_left_->ranges.at(i);

    if (left_scan_processor.IsRangeValid(range)) {
      float scan_pt_x = range * cos(angle);
      float scan_pt_y = range * sin(angle);

      Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
      Eigen::Vector2d robot_pt = rotation_left * scan_pt + translation_left;

      points_sum += robot_pt;
      points_count++;

      float angle_robot = atan2(robot_pt[1], robot_pt[0]) - angle_min;
      float range_robot = robot_pt.norm();

      int angle_ind = angle_robot / angle_increment_ - 1;

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

    if (right_scan_processor.IsRangeValid(range)) {
      float scan_pt_x = range * cos(angle);
      float scan_pt_y = range * sin(angle);

      Eigen::Vector2d scan_pt(scan_pt_x, scan_pt_y);
      Eigen::Vector2d robot_pt = rotation_right * scan_pt + translation_right;

      points_sum += robot_pt;
      points_count++;

      float angle_robot = atan2(robot_pt[1], robot_pt[0]) - angle_min;
      float range_robot = robot_pt.norm();

      int angle_ind = angle_robot / angle_increment_ + 1;

      if (angle_ind < 0 ||
              angle_ind > ((merged_scan_msg->ranges.size()) -1)) {
        std::cout << "Marius" << std::endl;
        continue;
      }

      if (range_robot < merged_scan_msg->ranges.at(angle_ind) ||
              isnan(merged_scan_msg->ranges.at(angle_ind))) {
        merged_scan_msg->ranges.at(angle_ind) = range_robot;
      }
    }
  }

  // Get the current time stamp
  ros::Time current_time = ros::Time::now();

  // Compute the center point of the scan
  Eigen::Vector2d center_pt = points_sum / points_count;
  scan_center_msg->header.frame_id = "robot";
  scan_center_msg->header.stamp.sec = current_time.sec;
  scan_center_msg->header.stamp.nsec = current_time.nsec;
  scan_center_msg->point.x = center_pt[0];
  scan_center_msg->point.y = center_pt[1];
  scan_center_msg->point.z = 0.0;

  // Fill the new merged laser scan message
  merged_scan_msg->header.frame_id = "robot";
  merged_scan_msg->header.stamp.sec = current_time.sec;
  merged_scan_msg->header.stamp.nsec = current_time.nsec;
  merged_scan_msg->angle_min = angle_min;
  merged_scan_msg->angle_max = angle_max;
  merged_scan_msg->angle_increment = angle_increment_;
  merged_scan_msg->scan_time = laser_scan_right_->scan_time;
  merged_scan_msg->range_min = 0.2;
  merged_scan_msg->range_max = 2.0;

  // Publish the new merged laser scan message
  merged_scan_pub_.publish(merged_scan_msg);

  // Publish the center point of the scan
  center_pt_pub_.publish(scan_center_msg);

  // Clear the individual laser scan messages
  laser_scan_left_.reset();
  laser_scan_right_.reset();
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_scan_merger_node");

  auto_trax::LaserScanMerger laser_scan_merger;

  ros::spin();

  return 0;
}
