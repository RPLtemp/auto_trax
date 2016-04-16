#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Default values
static const std::string kDefaultImuSubTopic = "imu/data";

class ImuIntegration {
  public:
    ImuIntegration();
    virtual ~ImuIntegration();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
};
