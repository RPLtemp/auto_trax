#include "auto_trax_sensors/april_tag_detection_node.h"

namespace auto_trax {

AprilTagDetectionNode::AprilTagDetectionNode():
    it_(nh_),
    tag_codes_(AprilTags::tagCodes16h5),
    tag_detector_(tag_codes_) {
  ros::NodeHandle pnh("~");

  // Get the node parameters
  std::string image_sub_topic;
  std::string image_pub_topic;
  pnh.param("image_sub_topic", image_sub_topic, kDefaultImageSubTopic);
  pnh.param("image_pub_topic", image_pub_topic, kDefaultImagePubTopic);

  // Initialize the ROS interfaces
  image_sub_ = it_.subscribe(image_sub_topic, 1, &AprilTagDetectionNode::ImageCallback, this);
  image_pub_ = it_.advertise(image_pub_topic, 1, true);
}

AprilTagDetectionNode::~AprilTagDetectionNode() {
}

void AprilTagDetectionNode::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  cv::Mat gray_img;

  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray_img, CV_BGR2GRAY);

  // Attempt to detect any tags in the current image
  std::vector<AprilTags::TagDetection> detections = tag_detector_.extractTags(gray_img);

  // Add the detection outline to the input image
  for (int i = 0; i < detections.size(); i++) {
    detections[i].draw(cv_ptr->image);
  }

  // Publish the resultant image
  sensor_msgs::ImagePtr result_image_msg =
          cv_bridge::CvImage(std_msgs::Header(),
                             sensor_msgs::image_encodings::BGR8,
                             cv_ptr->image).toImageMsg();
  image_pub_.publish(result_image_msg);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "april_tag_detection_node");

  auto_trax::AprilTagDetectionNode april_tag_detection_node;

  ros::spin();

  return 0;
}
