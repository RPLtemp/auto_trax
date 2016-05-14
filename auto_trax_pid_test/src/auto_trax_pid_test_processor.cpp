//
// Created by marius on 16.04.16.
//

#include "auto_trax_pid_test/auto_trax_pid_test_processor.h"

AutoTraxPidTest::AutoTraxPidTest(ros::NodeHandle nodehandle, ParameterBag params_bag):
    nh_(nodehandle),
    parameter_(params_bag) {
  ROS_DEBUG("Framework Processor started!");

  // Create Subscriber for the image
  sub_img_= nh_.subscribe(parameter_.subscribed_rostopic_img,
                                             parameter_.queue_size_subscriber_img,
                                             &AutoTraxPidTest::CallbackImg,
                                             this);
  sub_scan_= nh_.subscribe(parameter_.subscribed_rostopic_scan,
                          parameter_.queue_size_subscriber_scan,
                          &AutoTraxPidTest::CallbackScan,
                          this);

  // Create Publisher for scan
  pub_dist_ = nh_.advertise<std_msgs::Float32>(parameter_.pub_rostopic_dist,
                                              parameter_.queue_size_pub_dist);
}

AutoTraxPidTest::~AutoTraxPidTest(){
}

void AutoTraxPidTest::CallbackImg(const sensor_msgs::ImageConstPtr &im_msg) {
  ROS_DEBUG("Image received!");
  //std::cout << "Image works!!" << std::endl;
}

void AutoTraxPidTest::CallbackScan(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  ROS_DEBUG("Scan received!");

  size_t effective_scan_size = 0;
  for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
    if (scan_msg->ranges.at(i) > scan_msg->range_min && scan_msg->ranges.at(i) < scan_msg->range_max) {
      effective_scan_size++;
    }
  }
  if(effective_scan_size == 0){
    ROS_WARN("SCAN SIZE = 0");
    return;
  }

  double sum = 0;
  for (int i = 0; i < effective_scan_size; ++i) {
    if (scan_msg->ranges.at(i) > scan_msg->range_min && scan_msg->ranges.at(i) < scan_msg->range_max) {
      sum += scan_msg->ranges[i];
    }
  }
  double average = sum/effective_scan_size;
  std_msgs::Float32 result;
  result.data = average;

  pub_dist_.publish(result);
}
