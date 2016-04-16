#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Constants
static const double kG = 9.80665;

// Default values
static const std::string kDefaultImuSubTopic = "imu/data";
static const std::string kDefaultOdometryPubTopic = "odometry";

class ImuIntegration {
  public:
    ImuIntegration();
    virtual ~ImuIntegration();

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
