//
// Created by marius on 16.04.16.
//

#include "auto_trax_pid_test/parameter/parameter_bag.h"
#include "auto_trax_pid_test/auto_trax_pid_test_processor.h"

void InitializeParameters(const ros::NodeHandle& nh, ParameterBag* parameter) {
  // Retrieve all parameters or set to default
  nh.param("subscribed_rostopic_scan", parameter->subscribed_rostopic_scan, kDefaultScanSubTopic);
  nh.param("queue_subscribed_rostopic_scan", parameter->queue_size_subscriber_scan, kDefaultScanSubQueueSize);
  nh.param("pub_rostopic_dist", parameter->pub_rostopic_dist, kDefaultDistPubTopic);
  nh.param("queue_size_pub_dist", parameter->queue_size_pub_dist, kDefaultDistPubQueueSize);
  nh.param("pub_rostopic_merged_scan", parameter->pub_rostopic_merged_scan, kDefaultMergedScanPubTopic);
  nh.param("queue_size_pub_merged_scan", parameter->queue_size_pub_merged_scan, kDefaultMergedScanPubQueueSize);
  nh.param("merged_scan_angle_increment", parameter->angle_increment, kDefaultAngleIncrement);
  nh.param("frame_id_left", parameter->frame_id_left, kDefaultFrameIdLeft);
  nh.param("frame_id_right", parameter->frame_id_right, kDefaultFrameIdRight);
  nh.param("left_camera_offset", parameter->left_camera_offset, kDefaultLeftCameraOffset);
  nh.param("left_camera_orientation", parameter->left_camera_orientation, kDefaultLeftCameraOrientation);
  nh.param("right_camera_offset", parameter->right_camera_offset, kDefaultRightCameraOffset);
  nh.param("right_camera_orientation", parameter->right_camera_orientation, kDefaultRightCameraOrientation);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "auto_trax_pid_test_node");
  ros::NodeHandle nh;

  // Initialize parameter structure
  ParameterBag parameter;
  InitializeParameters(nh, &parameter);

  // Construct class detection_processor with ros::NodeHandle and parameter structure
  AutoTraxPidTest pid_test(nh, parameter);

  // Relative path to package
  std::string string = ros::package::getPath("auto_trax_pid_test");

  // Spin
  ros::spin ();
}
