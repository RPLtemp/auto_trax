#include "sensor_processing/imu_integration.h"

ImuIntegration::ImuIntegration() {
  ros::NodeHandle pnh("~");

  std::string imu_sub_topic;
  pnh.param("imu_topic", imu_sub_topic, kDefaultImuSubTopic);

  imu_sub_ = nh_.subscribe(imu_sub_topic, 1, &ImuIntegration::ImuCallback, this);
}

ImuIntegration::~ImuIntegration() {

}

void ImuIntegration::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_integration");

  ImuIntegration imu_integration;

  ros::spin();

  return 0;
}
