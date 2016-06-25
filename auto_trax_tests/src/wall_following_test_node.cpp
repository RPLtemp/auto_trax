//
// Created by marius on 16.04.16.
//

#include "auto_trax_tests/wall_following_test_processor.h"

void InitializeParameters(const ros::NodeHandle& nh, WallFollowingTestBag* parameter) {
  // Retrieve all parameters or set to default
  nh.param("subscribed_rostopic_scan", parameter->subscribed_rostopic_scan, kDefaultScanSubTopic);
  nh.param("queue_subscribed_rostopic_scan", parameter->queue_size_subscriber_scan, kDefaultScanSubQueueSize);
  nh.param("pub_rostopic_dist", parameter->pub_rostopic_dist, kDefaultDistPubTopic);
  nh.param("queue_size_pub_dist", parameter->queue_size_pub_dist, kDefaultDistPubQueueSize);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "auto_trax_tests node");
  ros::NodeHandle nh;

  // Initialize parameter structure
  WallFollowingTestBag parameter;
  InitializeParameters(nh, &parameter);

  // Construct class detection_processor with ros::NodeHandle and parameter structure
  WallFollowingTest wall_following_test(nh, parameter);

  // Relative path to package
  std::string string = ros::package::getPath("auto_trax_tests");

  // Spin
  ros::spin ();
}
