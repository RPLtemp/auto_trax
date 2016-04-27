#ifndef AUTO_TRAX_LINE_TRACKING_NODE_H
#define AUTO_TRAX_LINE_TRACKING_NODE_H

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "auto_trax_sensors/image_processing.h"
#include "auto_trax_sensors/occ_grid_manager.h"

namespace auto_trax {
// Default values
static const std::string kDefaultFrameId = "map";
static const std::string kDefaultImageSubTopic = "image_raw";
static const std::string kDefaultOccGridPubTopic = "occupancy_grid";

class LineTrackingNode {
  public:
    LineTrackingNode();
    virtual ~LineTrackingNode();

    void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

    void InitializeParameters();

    //void Test();

  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher occ_grid_pub_;

    ImageProcessing image_processing_;
    OccGridManager occ_grid_manager_;

    std::string occ_grid_frame_id_;
    std::string image_sub_topic_;
    std::string occ_grid_pub_topic_;
};
}

#endif // AUTO_TRAX_LINE_TRACKING_NODE_H
