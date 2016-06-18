#ifndef AUTO_TRAX_LASER_SCAN_MERGER_NODE_H
#define AUTO_TRAX_LASER_SCAN_MERGER_NODE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace auto_trax {
// Default values
static const std::string kDefaultMergedScanPubTopic = "merged_scan";
static const std::string kDefaultScanSubTopic = "/scan";
static const std::string kDefaultFrameIdLeft = "camera_left";
static const std::string kDefaultFrameIdRight = "camera_right";
static const float kDefaultAngleIncrement = 0.01;
static const float kDefaultLeftCameraOffset = 0.15;
static const float kDefaultLeftCameraOrientation = 45.0;
static const float kDefaultRightCameraOffset = -0.15;
static const float kDefaultRightCameraOrientation = -45.0;

class LaserScanMerger {
  public:
    LaserScanMerger();
    virtual ~LaserScanMerger();

    void ScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher merged_scan_pub_;

    sensor_msgs::LaserScanConstPtr laser_scan_left_;
    sensor_msgs::LaserScanConstPtr laser_scan_right_;

    std::string merged_scan_pub_topic_;
    std::string scan_sub_topic_;
    std::string frame_id_left_;
    std::string frame_id_right_;

    float angle_increment_;
    float left_camera_offset_;
    float left_camera_orientation_;
    float right_camera_offset_;
    float right_camera_orientation_;
};
}

#endif // AUTO_TRAX_LASER_SCAN_MERGER_NODE_H
