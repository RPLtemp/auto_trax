#ifndef AUTO_TRAX_LINE_TRACKING_H
#define AUTO_TRAX_LINE_TRACKING_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// Default values
static const std::string kDefaultImageSubTopic = "image_raw";

class LineTracking {
  public:
    LineTracking();
    virtual ~LineTracking();

    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
};


#endif // AUTO_TRAX_LINE_TRACKING_H
