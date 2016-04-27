#include "auto_trax_sensors/line_tracking_node.h"

namespace auto_trax {

LineTrackingNode::LineTrackingNode():
    it_(nh_) {
  InitializeParameters();

  image_sub_ = it_.subscribe(image_sub_topic_, 1, &LineTrackingNode::ImageCallback, this);
  occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occ_grid_pub_topic_, 1, true);
}

LineTrackingNode::~LineTrackingNode() {
}

void LineTrackingNode::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Segment the occupied/unoccupied space by the colored tracks
  cv::Mat segmented_image;
  image_processing_.SegmentByColoredTracks(cv_ptr->image, segmented_image);

  // Convert the image to gray-scale
  cv::cvtColor(segmented_image, segmented_image, CV_RGB2GRAY);

  // Create an occupancy grid from the segmented image
  nav_msgs::OccupancyGridPtr occ_grid(new nav_msgs::OccupancyGrid);
  occ_grid_manager_.OccGridFromBinaryImage(segmented_image, occ_grid);

  // Publish the occupancy grid
  ros::Time current_time = ros::Time::now();

  occ_grid->header.frame_id = occ_grid_frame_id_;
  occ_grid->header.stamp.sec = current_time.sec;
  occ_grid->header.stamp.nsec = current_time.nsec;

  occ_grid_pub_.publish(*occ_grid);
}

void LineTrackingNode::InitializeParameters() {
  ros::NodeHandle pnh("~");

  // Get the node parameters
  pnh.param("occ_grid_frame_id", occ_grid_frame_id_, kDefaultFrameId);
  pnh.param("image_sub_topic", image_sub_topic_, kDefaultImageSubTopic);
  pnh.param("occ_grid_pub_topic", occ_grid_pub_topic_, kDefaultOccGridPubTopic);

  // Get the image processing paremeters
  pnh.param("horizon_pixels", image_processing_.params_.horizon_pixels_, image_processing_.params_.horizon_pixels_);
  pnh.param("r_thresh", image_processing_.params_.r_thresh_, image_processing_.params_.r_thresh_);
  pnh.param("g_thresh", image_processing_.params_.g_thresh_, image_processing_.params_.g_thresh_);
  pnh.param("b_thresh", image_processing_.params_.b_thresh_, image_processing_.params_.b_thresh_);
  pnh.param("rgb_thresh", image_processing_.params_.rgb_range_, image_processing_.params_.rgb_range_);

  image_processing_.UpdateDerivedParameters(image_processing_.params_);

  // Get the occupancy grid paremeters
}

/*void LineTrackingNode::Test() {
  cv::Mat test_image;
  test_image = cv::imread("/home/pavel/tape-track.jpg", CV_LOAD_IMAGE_COLOR);

  if (!test_image.data ) {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return;
  }

  // Segment the occupied/unoccupied space by the colored tracks
  cv::Mat segmented_image;
  image_processing_.SegmentByColoredTracks(test_image, segmented_image);

  cv::imwrite("/home/pavel/test.jpg", segmented_image);

  // Convert the image to gray-scale
  cv::cvtColor(segmented_image, segmented_image, CV_RGB2GRAY);

  // Create an occupancy grid from the segmented image
  nav_msgs::OccupancyGridPtr occ_grid(new nav_msgs::OccupancyGrid);
  occ_grid_manager_.OccGridFromBinaryImage(segmented_image, occ_grid);

  // Publish the occupancy grid
  occ_grid->header.frame_id = occ_grid_frame_id_;
  occ_grid_pub_.publish(*occ_grid);
}*/
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_tracking_node");

  auto_trax::LineTrackingNode line_tracking_node;

  //
  // Testing
  //
  //line_tracking_node.Test();

  ros::spin();

  return 0;
}
