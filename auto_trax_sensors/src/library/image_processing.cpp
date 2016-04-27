#include "auto_trax_sensors/image_processing.h"

namespace auto_trax {

ImageProcessing::ImageProcessing() {

}

ImageProcessing::~ImageProcessing() {
}

void ImageProcessing::UpdateDerivedParameters(ImageProcessingParameters& params) {
  params.lower_bound_ = cv::Scalar(params.b_thresh_- params.rgb_range_,
                                   params.g_thresh_ - params.rgb_range_,
                                   params.r_thresh_ - params.rgb_range_);
  params.upper_bound_ = cv::Scalar(params.b_thresh_ + params.rgb_range_,
                                   params.g_thresh_ + params.rgb_range_,
                                   params.r_thresh_ + params.rgb_range_);
}

void ImageProcessing::SegmentByColoredTracks(const cv::Mat& img_in, cv::Mat& img_out) {
  //
  // 1. Add Canny/Hough parameters
  // 2. Check if lines were detected
  // 3. Find lines with the lowest slope
  //

  // Crop out only the bottom part of the image (the immediate ground area)
  cv::Rect area_to_crop(0, img_in.rows - params_.horizon_pixels_, img_in.cols, params_.horizon_pixels_);
  img_out = img_in(area_to_crop);

  // Detect all the pixels within the desired RGB interval
  cv::inRange(img_out, params_.lower_bound_, params_.upper_bound_, img_out);

  // Canny edge detector
  cv::Canny(img_out, img_out, 80, 250);

  // Run Hough transform to detect straight lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(img_out, lines, 1, CV_PI/180, 50, 50, 10);

  cv::Vec4i closest_left_line;
  cv::Vec4i closest_right_line;
  double max_negative_slope = std::numeric_limits<double>::min();
  double min_positive_slope = std::numeric_limits<double>::max();
  int left_wall_position = 0;
  int right_wall_position = img_out.cols;

  img_out = cv::Mat(img_out.rows, img_out.cols, CV_8UC3, kWhite);

  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    double slope = (double)(l[1] - l[3]) / (double)(l[0] - l[2]);

    //if (slope > max_negative_slope && slope < 0.0 && l[0] > left_wall_position) {
    if (slope < 0.0 && (l[0] + l[2]) > (closest_left_line[0] + closest_left_line[2])) {
      left_wall_position = l[0];
      max_negative_slope = slope;
      closest_left_line = l;
    }

    //if (slope < min_positive_slope && slope > 0.0 && l[0] < right_wall_position) {
    if (slope > 0.0 && l[0] < right_wall_position) {
      right_wall_position = l[0];
      min_positive_slope = slope;
      closest_right_line = l;
    }
  }

  int npt[] = {kPolygonPoints};

  // Process left track
  int x_up = (0 - closest_left_line[1] + max_negative_slope * closest_left_line[0]) / max_negative_slope;
  int x_down = (img_out.rows - closest_left_line[1] + max_negative_slope * closest_left_line[0]) / max_negative_slope;

  cv::Point left_wall[1][npt[0]];
  left_wall[0][0] = cv::Point(0, 0);
  left_wall[0][1] = cv::Point(0, img_out.rows);
  left_wall[0][2] = cv::Point(x_down, img_out.rows);
  left_wall[0][3] = cv::Point(x_up, 0);

  const cv::Point* left[1] = {left_wall[0]};
  cv::fillPoly(img_out, left, npt, 1, kBlack);

  // Process right track
  x_up = (0 - closest_right_line[1] + min_positive_slope * closest_right_line[0]) / min_positive_slope;
  x_down = (img_out.rows - closest_right_line[1] + min_positive_slope * closest_right_line[0]) / min_positive_slope;

  cv::Point right_wall[1][npt[0]];
  right_wall[0][0] = cv::Point(x_up, 0);
  right_wall[0][1] = cv::Point(x_down, img_out.rows);
  right_wall[0][2] = cv::Point(img_out.cols, img_out.rows);
  right_wall[0][3] = cv::Point(img_out.cols, 0);

  const cv::Point* right[1] = {right_wall[0]};
  cv::fillPoly(img_out, right, npt, 1, kBlack);
}
}
