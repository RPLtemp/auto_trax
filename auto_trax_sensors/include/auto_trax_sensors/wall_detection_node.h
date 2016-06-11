#ifndef AUTO_TRAX_WALL_DETECTION_NODE_H
#define AUTO_TRAX_WALL_DETECTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

namespace auto_trax {
// Default values
static const std::string kDefaultLaserScanSubTopic = "scan";
static const std::string kDefaultWallAnglePubTopic = "/distance_result";

class WallDetectionNode {
  public:
    WallDetectionNode();
    virtual ~WallDetectionNode();

    void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_sub_;
    ros::Publisher wall_angle_pub_;
};
}

#endif // AUTO_TRAX_WALL_DETECTION_NODE_H
