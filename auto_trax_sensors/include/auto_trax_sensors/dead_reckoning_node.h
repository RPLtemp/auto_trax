#ifndef AUTO_TRAX_DEAD_RECKONING_NODE_H
#define AUTO_TRAX_DEAD_RECKONING_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace auto_trax {
// Constants
static const double kG = 9.80665;

// Default values
static const std::string kDefaultImuSubTopic = "imu/data";
static const std::string kDefaultOdometryPubTopic = "odometry";

class DeadReckoningNode {
  public:
    DeadReckoningNode();
    virtual ~DeadReckoningNode();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;

    nav_msgs::Odometry odom_msg_;

    ros::Time last_time_;

    int odom_seq_;

    bool received_first_imu_data_;
};
}

#endif // AUTO_TRAX_DEAD_RECKONING_NODE_H
