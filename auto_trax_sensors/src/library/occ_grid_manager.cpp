#include "auto_trax_sensors/occ_grid_manager.h"

namespace auto_trax {

OccGridManager::OccGridManager() {
}

OccGridManager::~OccGridManager() {
}

void OccGridManager::OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid) {
  std::cout << "x_ground_center: " << x_ground_center_ << std::endl;
  std::cout << "z_center: " << z_center_ << std::endl;

  // Get the dimensions of the required occupancy grid
  std::pair<double, double> dims = GetRequiredOccGridDimensions(img);

  std::cout << dims.first << ", " << dims.second << std::endl;

  std::cout << "Offset: " << x_offset_ << std::endl;

  occ_grid->info.resolution = params_.resolution_;
  occ_grid->info.width = ceil(dims.first / occ_grid->info.resolution);
  occ_grid->info.height = ceil(dims.second / occ_grid->info.resolution);

  occ_grid->info.origin.position.x = 0.0;
  occ_grid->info.origin.position.y = (-0.5 * dims.second);
  occ_grid->info.origin.position.z = 0.0;

  occ_grid->data.assign(occ_grid->info.width * occ_grid->info.height, 100);

  for (int i = 0; i < occ_grid->info.width; i++) {
    for (int j = 0; j < occ_grid->info.height; j++) {
      // Convert the cell coordinates into meters
      double x = i * params_.resolution_;
      double y = j * params_.resolution_;

      double x_shift = x_offset_ + (dims.first - x);
      double z_shift = x_shift * cos((M_PI * 0.5) - params_.angle_);
      /*double x_ground = x_ground_center_ - x_shift;
      double theta = atan(x_ground / params_.height_);
      double d_angle = params_.angle_ - theta;
      double z_temp = sqrt(x_ground * x_ground + params_.height_ * params_.height_);
      double z = z_temp * cos(d_angle);

      // FIX THIS
      //
      double proj_x = -y + dims.second * 0.5;
      //
      //

      double proj_y = z_temp * sin(d_angle);*/

      double proj_y = sin((M_PI * 0.5) - params_.angle_) * x_shift;
      double proj_x = -y + dims.second * 0.5;
      double proj_z = z_center_ - z_shift;

      //Eigen::Vector4d pt_3d(proj_x, proj_y, z_temp, 1.0);
      //Eigen::Vector4d pt_3d(proj_x, proj_y, z, 1.0);
      Eigen::Vector4d pt_3d(proj_x, proj_y, proj_z, 0.0);
      Eigen::Vector3d pixels;
      //img_processing_->Project3DPtToPixels(pt_3d, pixels, z);
      img_processing_->Project3DPtToPixels(pt_3d, pixels, proj_z);

      int p_x = pixels.x();
      int p_y = pixels.y();

      if(p_x < 0 || p_x > img.cols || p_y < (480 - img.rows) || p_y > 480)
        continue;

      int coordinate = i + j * occ_grid->info.width;

      if (img.at<uchar>(cv::Point(p_x, p_y - 280)) > 127)
        occ_grid->data.at(coordinate) = 0;
    }
  }
}

std::pair<double, double> OccGridManager::GetRequiredOccGridDimensions(cv::Mat& img) {
  std::pair<double, double> top_left_corner = ProjectPixelToGround(0, 480 - img.rows);
  std::pair<double, double> bottom_right_corner = ProjectPixelToGround(img.cols, 480);

  x_offset_ = fabs(x_ground_center_ - top_left_corner.first);

  std::cout << "Top left: " << top_left_corner.first << ", " << top_left_corner.second << std::endl;
  std::cout << "Bottom right: " << bottom_right_corner.first << ", " << bottom_right_corner.second << std::endl;

  // Verify the x-dimension range (it may not start at 0)
  return std::pair<double, double>(fabs(top_left_corner.first - bottom_right_corner.first),
                                   fabs(top_left_corner.second) * 2.0);
}

std::pair<double, double> OccGridManager::ProjectPixelToGround(int p_x, int p_y) {
  Eigen::Vector3d pixels(p_x, p_y, 1);
  Eigen::Vector4d pt_3d;

  std::cout << " " << std::endl;

  double d_angle = atan((double)(p_y - 240) / (double)579.4302);
  double theta = params_.angle_ - d_angle;
  double z_temp = params_.height_ / cos(theta);
  double z = z_temp * cos(d_angle);

  img_processing_->ProjectPixelsTo3D(pixels, pt_3d, z);

  std::cout << pt_3d << std::endl;

  //double x_ground = x_ground_center_ - pt_3d.y() * cos(params_.angle_);
  double x_ground = pt_3d.z() * sin(params_.angle_);
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
