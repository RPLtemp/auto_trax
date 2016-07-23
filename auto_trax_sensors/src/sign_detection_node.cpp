#include "auto_trax_sensors/sign_detection_node.h"

namespace auto_trax {

SignDetectionNode::SignDetectionNode():
    it_(nh_) {
  ros::NodeHandle pnh("~");

  // Get the node parameters
  pnh.param("image_sub_topic", image_sub_topic_, kDefaultImageSubTopic);
  pnh.param("image_pub_topic", image_pub_topic_, kDefaultImagePubTopic);
  pnh.param("scale_factor", scale_factor_, kDefaultScaleFactor);
  pnh.param("min_neighbors", min_neighbors_, kDefaultMinNeighbors);
  pnh.param("size_min", size_min_, kDefaultSizeMin);
  pnh.param("size_max", size_max_, kDefaultSizeMax);

  if (!pnh.getParam("classifier_path", classifier_path_)) {
    ROS_ERROR("Please specify the path to the classifier");
    ros::shutdown();
  }

  image_sub_ = it_.subscribe(image_sub_topic_, 1, &SignDetectionNode::ImageCallback, this);
  image_pub_ = it_.advertise(image_pub_topic_, 1, true);
}

SignDetectionNode::~SignDetectionNode() {
}

void SignDetectionNode::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  cv::Mat gray_img, eq_img, result_img;

  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray_img, CV_RGB2GRAY);
  cv::equalizeHist(gray_img, eq_img);
  result_img = cv_ptr->image.clone();

  cv::CascadeClassifier cascade;
  if (cascade.load(classifier_path_) == false) {
    ROS_ERROR("cascade.load() failed...");
    return;
  }

  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(eq_img, faces, scale_factor_, min_neighbors_, 0, cv::Size(size_min_, size_max_));

  std::vector<cv::Rect>::const_iterator i;
  for (i = faces.begin(); i != faces.end(); ++i) {
    cv::rectangle(
                result_img,
                cv::Point(i->x, i->y),
                cv::Point(i->x + i->width, i->y + i->height),
                CV_RGB(255, 0, 0),
                2);
  }

  sensor_msgs::ImagePtr result_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_img).toImageMsg();
  image_pub_.publish(result_image_msg);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sign_detection_node");

  auto_trax::SignDetectionNode sign_detection_node;

  ros::spin();

  return 0;
}
