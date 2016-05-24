#include "auto_trax_sensors/dead_reckoning_node.h"

namespace auto_trax {

DeadReckoningNode::DeadReckoningNode():
    odom_seq_(0),
    received_first_imu_data_(false) {
  ros::NodeHandle pnh("~");

  std::string imu_sub_topic;
  std::string odom_pub_topic;
  pnh.param("imu_topic", imu_sub_topic, kDefaultImuSubTopic);
  pnh.param("odom_topic", odom_pub_topic, kDefaultOdometryPubTopic);

  imu_sub_ = nh_.subscribe(imu_sub_topic, 1, &DeadReckoningNode::ImuCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic, 1);
}

DeadReckoningNode::~DeadReckoningNode() {

}

void DeadReckoningNode::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  if (!received_first_imu_data_) {
    last_time_ = ros::Time::now();
    received_first_imu_data_ = true;
    return;
  }

  ros::Time current_time = ros::Time::now();
  double dt = (double)(current_time.nsec - last_time_.nsec) * 1.0E-9;
  last_time_ = current_time;

  odom_msg_.header.seq = odom_seq_;
  odom_msg_.header.stamp.sec = current_time.sec;
  odom_msg_.header.stamp.nsec = current_time.nsec;

  // Integrate linear velocity to get position
  odom_msg_.pose.pose.position.x = odom_msg_.pose.pose.position.x + odom_msg_.twist.twist.linear.x * dt + 0.5 * imu_msg->linear_acceleration.x / kG * dt * dt;
  odom_msg_.pose.pose.position.y = odom_msg_.pose.pose.position.y + odom_msg_.twist.twist.linear.y * dt + 0.5 * imu_msg->linear_acceleration.y / kG * dt * dt;
  odom_msg_.pose.pose.position.z = odom_msg_.pose.pose.position.z + odom_msg_.twist.twist.linear.z * dt + 0.5 * (imu_msg->linear_acceleration.z + 1.0f) / kG * dt * dt;

  // Orientation comes straight from IMU
  odom_msg_.pose.pose.orientation = imu_msg->orientation;

  // Integrate linear acceleration to get linear velocity
  odom_msg_.twist.twist.linear.x = odom_msg_.twist.twist.linear.x + (imu_msg->linear_acceleration.x * dt / kG);
  odom_msg_.twist.twist.linear.y = odom_msg_.twist.twist.linear.y + (imu_msg->linear_acceleration.y * dt / kG);
  odom_msg_.twist.twist.linear.z = odom_msg_.twist.twist.linear.z + (imu_msg->linear_acceleration.z + 1.0f) * dt / kG;

  // Angular velocity comes straight from IMU
  odom_msg_.twist.twist.angular = imu_msg->angular_velocity;

  odom_pub_.publish(odom_msg_);

  odom_seq_++;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dead_reckoning_node");

  auto_trax::DeadReckoningNode dead_reckoning;

  ros::spin();

  return 0;
}
