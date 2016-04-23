#include "auto_trax_sensors/line_tracking.h"

LineTracking::LineTracking() {
  ros::NodeHandle pnh("~");

  std::string image_sub_topic;
  pnh.param("image_topic", image_sub_topic, kDefaultImageSubTopic);

  image_sub_ = nh_.subscribe(image_sub_topic, 1, &LineTracking::ImageCallback, this);
}

LineTracking::~LineTracking() {

}

void LineTracking::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_tracking");

  LineTracking line_tracking;

  ros::spin();

  return 0;
}
