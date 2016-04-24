#include "auto_trax_sensors/line_tracking.h"

LineTracking::LineTracking():
    it_(nh_) {
  ros::NodeHandle pnh("~");

  std::string image_sub_topic;
  std::string occ_grid_pub_topic;

  int r_thresh;
  int g_thresh;
  int b_thresh;
  int rgb_range;

  pnh.param("image_sub_topic", image_sub_topic, kDefaultImageSubTopic);
  pnh.param("occ_grid_pub_topic", occ_grid_pub_topic, kDefaultOccGridPubTopic);
  pnh.param("horizon_pixels", horizon_pixels_, kDefaultHorizonPixels);
  pnh.param("r_thresh", r_thresh, kDefaultRThresh);
  pnh.param("g_thresh", g_thresh, kDefaultGThresh);
  pnh.param("b_thresh", b_thresh, kDefaultBThresh);
  pnh.param("rgb_thresh", rgb_range, kDefaultRGBRange);

  lower_bound_ = cv::Scalar(b_thresh - rgb_range, g_thresh - rgb_range, r_thresh - rgb_range);
  upper_bound_ = cv::Scalar(b_thresh + rgb_range, g_thresh + rgb_range, r_thresh + rgb_range);

  image_sub_ = it_.subscribe(image_sub_topic, 1, &LineTracking::ImageCallback, this);
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

  cv::Mat hough(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3, cv::Scalar(0,0,0));
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(cv_ptr->image, lines, 1, CV_PI/180, 50, 50, 10);

  cv::Vec4i closest_left_line;
  double max_negative_slope = -10.0;

  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    double slope = (double)(l[1] - l[3]) / (double)(l[0] - l[2]);
    if (slope > max_negative_slope && slope < 0.0) {
      max_negative_slope = slope;
      closest_left_line = l;
    }
  }

  if (cv::sum(closest_left_line).val > 0) {
    int x_up = (0 - closest_left_line[1] + max_negative_slope * closest_left_line[0]) / max_negative_slope;
    int x_down = (hough.rows - closest_left_line[1] + max_negative_slope * closest_left_line[0]) / max_negative_slope;

    cv::Point occupied_points[1][4];
    occupied_points[0][0] = cv::Point(0, 0);
    occupied_points[0][1] = cv::Point(0, hough.rows);
    occupied_points[0][2] = cv::Point(x_down, hough.rows);
    occupied_points[0][3] = cv::Point(x_up, 0);

    const cv::Point* ppt[1] = {occupied_points[0]};
    const cv::Scalar white(255, 255, 255);
    int npt[] = { 4 };
    int size_c = 1;
    cv::fillPoly(hough, ppt, npt, size_c, white);
  }

  cv::Mat im_gray;
  cv::cvtColor(hough, im_gray, CV_RGB2GRAY);

  // Create an occupancy grid from the segmented image
  nav_msgs::OccupancyGridPtr occ_grid(new nav_msgs::OccupancyGrid);
  OccGridManager::OccGridFromBinaryImage(im_gray, occ_grid);

  // Publish the occupancy grid
  occ_grid->header.frame_id = "map";
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

  cv::Mat hough(dest.rows, dest.cols, CV_8UC3, cv::Scalar(0,0,0));
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dest, lines, 1, CV_PI/180, 50, 50, 10);

  cv::Vec4i closest_left_line;
  double max_negative_slope = -10.0;

  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    double slope = (double)(l[1] - l[3]) / (double)(l[0] - l[2]);
    if (slope > max_negative_slope && slope < 0.0) {
      max_negative_slope = slope;
      closest_left_line = l;
    }
  }

  if (cv::sum(closest_left_line).val > 0) {
    int x_up = (0 - closest_left_line[1] + max_negative_slope * closest_left_line[0]) / max_negative_slope;
    int x_down = (dest.rows - closest_left_line[1] + max_negative_slope * closest_left_line[0]) / max_negative_slope;

    cv::Point occupied_points[1][4];
    occupied_points[0][0] = cv::Point(0, 0);
    occupied_points[0][1] = cv::Point(0, hough.rows);
    occupied_points[0][2] = cv::Point(x_down, hough.rows);
    occupied_points[0][3] = cv::Point(x_up, 0);

    const cv::Point* ppt[1] = {occupied_points[0]};
    const cv::Scalar white(255, 255, 255);
    int npt[] = { 4 };
    int size_c = 1;
    cv::fillPoly(hough, ppt, npt, size_c, white);
  }

  cv::imwrite("/home/pavel/cropped-path.jpg", cropped);
  cv::imwrite("/home/pavel/dest-path.jpg", dest);
  cv::imwrite("/home/pavel/hough-path.jpg", hough);

  cv::Mat im_gray;
  cv::cvtColor(hough, im_gray, CV_RGB2GRAY);

  // Create an occupancy grid from the segmented image
  nav_msgs::OccupancyGridPtr occ_grid(new nav_msgs::OccupancyGrid);
  OccGridManager::OccGridFromBinaryImage(im_gray, occ_grid);

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
  //line_tracking.Test();

  ros::spin();

  return 0;
}
