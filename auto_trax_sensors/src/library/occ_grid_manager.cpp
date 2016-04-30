#include "auto_trax_sensors/occ_grid_manager.h"

namespace auto_trax {

OccGridManager::OccGridManager() {
}

OccGridManager::~OccGridManager() {
}

void OccGridManager::OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid) {
  // Get the dimensions of the required occupancy grid
  std::pair<double, double> dims = GetRequiredOccGridDimensions(img);

  occ_grid->info.resolution = params_.resolution_;
  occ_grid->info.width = ceil(dims.first / occ_grid->info.resolution);
  occ_grid->info.height = ceil(dims.second / occ_grid->info.resolution);

  occ_grid->data.assign(occ_grid->info.width * occ_grid->info.height, 100);

  for (int i = 0; i < occ_grid->info.width; i++) {
    for (int j = 0; j < occ_grid->info.height; j++) {
      // Convert the cell coordinates into meters
      double x = i * params_.resolution_;
      double y = j * params_.resolution_;

      // Project the point from occupancy grid plane (the ground) to the plane
      // where the camera will project its points
      double proj_x = -y + dims.second * 0.5;
      double proj_y = (dims.first - x + x_offset_) / cos(params_.angle_);

      Eigen::Vector4d pt_3d(proj_x, proj_y, z_center_, 1.0);
      Eigen::Vector3d pixels;
      img_processing_->Project3DPtToPixels(pt_3d, pixels, z_center_);

      int p_x = pixels.x();
      int p_y = pixels.y();

      int coordinate = i + j * occ_grid->info.width;

      if (img.at<uchar>(cv::Point(p_x, p_y - 280)) > 200)
        occ_grid->data.at(coordinate) = 0;
    }
  }
}

std::pair<double, double> OccGridManager::GetRequiredOccGridDimensions(cv::Mat& img) {
  std::pair<double, double> top_left_corner = ProjectPixelToGround(0, 480 - img.rows);
  std::pair<double, double> bottom_right_corner = ProjectPixelToGround(img.cols, 480);

  x_offset_ = fabs(x_ground_center_ - top_left_corner.first);

  // Verify the x-dimension range (it may not start at 0)
  return std::pair<double, double>(fabs(top_left_corner.first - bottom_right_corner.first),
                                   fabs(bottom_right_corner.second - top_left_corner.second));
}

std::pair<double, double> OccGridManager::ProjectPixelToGround(int p_x, int p_y) {
  Eigen::Vector3d pixels(p_x, p_y, 1);
  Eigen::Vector4d pt_3d;
  img_processing_->ProjectPixelsTo3D(pixels, pt_3d, z_center_);

  double x_ground = x_ground_center_ - pt_3d.y() * cos(params_.angle_);
  double y_ground = pt_3d.x();

  return std::pair<double, double>(x_ground, y_ground);
}

void OccGridManager::SetImageProcessor(ImageProcessing& img_proc) {
  img_processing_ = &img_proc;
}

void OccGridManager::UpdateDerivedParameters() {
  x_ground_center_ = params_.height_ * tan(params_.angle_);
  z_center_ = params_.height_ / cos(params_.angle_);
}
}
