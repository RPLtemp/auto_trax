#include "auto_trax_sensors/line_tracking.h"

LineTracking::LineTracking():
    it_(nh_) {
  ros::NodeHandle pnh("~");

  std::string image_sub_topic;
  pnh.param("image_topic", image_sub_topic, kDefaultImageSubTopic);

  image_sub_ = it_.subscribe(image_sub_topic, 1, &LineTracking::ImageCallback, this);
}

LineTracking::~LineTracking() {

}

void LineTracking::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_tracking");

  LineTracking line_tracking;

  ros::spin();

  return 0;
}
