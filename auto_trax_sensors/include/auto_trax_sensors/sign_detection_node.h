#ifndef AUTO_TRAX_SIGN_DETECTION_NODE_H
#define AUTO_TRAX_SIGN_DETECTION_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace auto_trax {
// Default values
static const std::string kDefaultImageSubTopic = "image_raw";
static const std::string kDefaultImagePubTopic = "detected_signs";
static constexpr double kDefaultScaleFactor = 1.1;
static constexpr int kDefaultMinNeighbors = 3;
static constexpr int kDefaultSizeMin = 20;
static constexpr int kDefaultSizeMax = 20;

class SignDetectionNode {
  public:
    SignDetectionNode();
    virtual ~SignDetectionNode();

    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    std::string image_sub_topic_;
    std::string image_pub_topic_;

    std::string classifier_path_;

    double scale_factor_;
    int min_neighbors_;
    int size_min_;
    int size_max_;
};
}

#endif // AUTO_TRAX_SIGN_DETECTION_NODE_H
