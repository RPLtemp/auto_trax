#include "auto_trax_sensors/line_tracking.h"

LineTracking::LineTracking():
    it_(nh_) {
  ros::NodeHandle pnh("~");

  std::string image_sub_topic;
  std::string image_pub_topic;
  std::string occ_grid_pub_topic;

  int r_thresh;
  int g_thresh;
  int b_thresh;
  int rgb_range;

  pnh.param("image_sub_topic", image_sub_topic, kDefaultImageSubTopic);
  pnh.param("image_pub_topic", image_pub_topic, kDefaultImagePubTopic);
  pnh.param("occ_grid_pub_topic", occ_grid_pub_topic, kDefaultOccGridPubTopic);
  pnh.param("horizon_pixels", horizon_pixels_, kDefaultHorizonPixels);
  pnh.param("r_thresh", r_thresh, kDefaultRThresh);
  pnh.param("g_thresh", g_thresh, kDefaultGThresh);
  pnh.param("b_thresh", b_thresh, kDefaultBThresh);
  pnh.param("rgb_thresh", rgb_range, kDefaultRGBRange);

  lower_bound_ = cv::Scalar(b_thresh - rgb_range, g_thresh - rgb_range, r_thresh - rgb_range);
  upper_bound_ = cv::Scalar(b_thresh + rgb_range, g_thresh + rgb_range, r_thresh + rgb_range);

  image_sub_ = it_.subscribe(image_sub_topic, 1, &LineTracking::ImageCallback, this);
  image_pub_ = it_.advertise(image_pub_topic, 1);
  occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occ_grid_pub_topic, 1, true);
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

  // Crop out only the bottom part of the image (the immediate ground area)
  cv::Rect area_to_crop(0, cv_ptr->image.rows - horizon_pixels_, cv_ptr->image.cols, horizon_pixels_);
  cv_ptr->image = cv_ptr->image(area_to_crop);

  // Filter out the RGB values that we are interested in
  cv::inRange(cv_ptr->image, lower_bound_, upper_bound_, cv_ptr->image);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());

  // Create an occupancy grid from the segmented image
  nav_msgs::OccupancyGridPtr occ_grid;
  OccGridManager::OccGridFromBinaryImage(cv_ptr->image, occ_grid);

  // Publish the occupancy grid
  occ_grid_pub_.publish(*occ_grid);
}

void LineTracking::Test() {
  cv::Mat test_image;
  test_image = cv::imread("/home/pavel/bike-path.jpg", CV_LOAD_IMAGE_COLOR);

  if (!test_image.data ) {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return;
  }

  // Crop out only the bottom part of the image (the immediate ground area)
  cv::Rect area_to_crop(0, test_image.rows - horizon_pixels_, test_image.cols, horizon_pixels_);
  cv::Mat cropped = test_image(area_to_crop);

  cv::Mat dest;
  cv::inRange(cropped, lower_bound_, upper_bound_, dest);

  cv::imwrite("/home/pavel/cropped-path.jpg", cropped);
  cv::imwrite("/home/pavel/dest-path.jpg", dest);

  // Create an occupancy grid from the segmented image
  nav_msgs::OccupancyGridPtr occ_grid(new nav_msgs::OccupancyGrid);
  OccGridManager::OccGridFromBinaryImage(dest, occ_grid);

  std::cout << occ_grid->data.size() << std::endl;

  // Publish the occupancy grid
  occ_grid->header.frame_id = "map";
  occ_grid_pub_.publish(*occ_grid);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_tracking");

  LineTracking line_tracking;

  //
  // Testing
  //
  line_tracking.Test();

  ros::spin();

  return 0;
}
