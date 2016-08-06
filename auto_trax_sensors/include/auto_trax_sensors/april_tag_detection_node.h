#ifndef AUTO_TRAX_APRIL_TAG_DETECTION_NODE_H
#define AUTO_TRAX_APRIL_TAG_DETECTION_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

namespace auto_trax {
// Default values
static const std::string kDefaultImageSubTopic = "image_raw";

class AprilTagDetectionNode {
  public:
    AprilTagDetectionNode();
    virtual ~AprilTagDetectionNode();

    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    AprilTags::TagDetector* tag_detector_;
    AprilTags::TagCodes tag_codes_;
};
}

#endif // AUTO_TRAX_APRIL_TAG_DETECTION_NODE_H
