#ifndef AUTO_TRAX_LINE_TRACKING_H
#define AUTO_TRAX_LINE_TRACKING_H

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "auto_trax_sensors/occ_grid_manager.h"

// Default values
static const std::string kDefaultImageSubTopic = "image_raw";
static const std::string kDefaultImagePubTopic = "image_segmented";
static const std::string kDefaultOccGridPubTopic = "occupancy_grid";
static const int kDefaultHorizonPixels = 200;
static const int kDefaultRThresh = 255;
static const int kDefaultGThresh = 230;
static const int kDefaultBThresh = 160;
static const int kDefaultRGBRange = 10;

class LineTracking {
  public:
    LineTracking();
    virtual ~LineTracking();

    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

    void Test();

  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher occ_grid_pub_;

    int horizon_pixels_;

    cv::Scalar lower_bound_;
    cv::Scalar upper_bound_;
};


#endif // AUTO_TRAX_LINE_TRACKING_H
