#include "auto_trax_sensors/april_tag_detection_node.h"

namespace auto_trax {

AprilTagDetectionNode::AprilTagDetectionNode():
    it_(nh_),
    tag_detector_(0),
    tag_codes_(AprilTags::tagCodes36h11) {
  ros::NodeHandle pnh("~");

  // Get the node parameters
  std::string image_sub_topic;
  pnh.param("image_sub_topic", image_sub_topic, kDefaultImageSubTopic);

  // Initialize the ROS interfaces
  image_sub_ = it_.subscribe(image_sub_topic, 1, &AprilTagDetectionNode::ImageCallback, this);
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
  AprilTags::TagFamily fam = tag_detector_->thisTagFamily;
  std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(cv_ptr->image);

  // Print out each detection
  std::cout << detections.size() << " tags detected:" << std::endl;
  for (int i = 0; i < detections.size(); i++) {
    std::cout << i << std::endl;
  }

  // Show the current image including any detections
  /*if (m_draw) {
    for (int i = 0; i < detections.size(); i++) {
      // Also highlight in the image
      detections[i].draw(image);
    }
    imshow(windowName, image);
  }*/
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "april_tag_detection_node");

  auto_trax::AprilTagDetectionNode april_tag_detection_node;

  ros::spin();

  return 0;
}
