#include "auto_trax_sensors/occ_grid_manager.h"

namespace auto_trax {

OccGridManager::OccGridManager(ImageProcessing *img_proc) {
  img_processing_ = img_proc;
}

OccGridManager::~OccGridManager() {
}

int OccGridManager::GetFarthestOpenRow(nav_msgs::OccupancyGrid occ_grid) {
  int farthest_row = 0;
  int width = occ_grid.info.width;

  for (int i = 0; i < occ_grid.info.width; i++) {
    for (int j = 0; j < occ_grid.info.height; j++) {
      int coordinate = i + j * width;

      if (occ_grid.data.at(coordinate) == kValueUnoccupied) {
        farthest_row = i;
        break;
      }
    }
  }

  return farthest_row;
}

void OccGridManager::GetGoalPoint(nav_msgs::OccupancyGrid occ_grid, std::pair<double, double>& goal_pt) {
  int farthest_row_ind = GetFarthestOpenRow(occ_grid);

  std::vector<int> farthest_open_row = GetRow(occ_grid, farthest_row_ind);

  int unoccupied_midpoint = GetUnoccupiedRowMidpoint(farthest_open_row);

  goal_pt.first = farthest_row_ind * occ_grid.info.resolution;
  goal_pt.second = (unoccupied_midpoint - 0.5 * occ_grid.info.height) * occ_grid.info.resolution;
}

bool OccGridManager::GetRequiredOccGridDimensions(cv::Mat& img, std::pair<double, double>& dims) {
  std::pair<double, double> top_left_corner = ProjectPixelToGround(0, kFrameHeight - img.rows);
  std::pair<double, double> bottom_right_corner = ProjectPixelToGround(img.cols, kFrameHeight);

  x_offset_ = x_ground_center_ - top_left_corner.first;

  if (top_left_corner.first <= bottom_right_corner.first)
    return false;

  dims.first = top_left_corner.first - bottom_right_corner.first;
  dims.second = fabs(top_left_corner.second) * 2.0;

  return true;
}

std::vector<int> OccGridManager::GetRow(nav_msgs::OccupancyGrid occ_grid, int row_ind) {
  std::vector<int> row;
  int width = occ_grid.info.width;

  for (int i = 0; i < occ_grid.info.height; i++) {
    int coordinate = row_ind + i * width;
    row.push_back(occ_grid.data.at(coordinate));
  }

  return row;
}

int OccGridManager::GetUnoccupiedRowMidpoint(std::vector<int> row) {
  int unoccupied_first = -1;
  int unoccupied_last = row.size() - 1;

  for (int i = 0; i < row.size(); i++) {
    if (row.at(i) == kValueUnoccupied && unoccupied_first == -1)
      unoccupied_first = i;

    if (row.at(i) == kValueOccupied && unoccupied_first != -1) {
      unoccupied_last = i - 1;
      break;
    }
  }

  return 0.5 * (unoccupied_last + unoccupied_first);
}

void OccGridManager::OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid) {
  std::pair<double, double> dims;
  if (!GetRequiredOccGridDimensions(img, dims)) {
    return;
  }

  occ_grid->info.resolution = params_.resolution_;
  occ_grid->info.width = ceil(dims.first / occ_grid->info.resolution);
  occ_grid->info.height = ceil(dims.second / occ_grid->info.resolution);

  occ_grid->info.origin.position.x = 0.0;
  occ_grid->info.origin.position.y = (-0.5 * dims.second);
  occ_grid->info.origin.position.z = 0.0;

  occ_grid->data.assign(occ_grid->info.width * occ_grid->info.height, kValueOccupied);

  for (int i = 0; i < occ_grid->info.width; i++) {
    for (int j = 0; j < occ_grid->info.height; j++) {
      double x = i * params_.resolution_;
      double y = j * params_.resolution_;

      // TODO: make the calculations more general
      double x_shift = x_offset_ + (dims.first - x);
      double z_shift = x_shift * cos((M_PI * 0.5) - params_.cam_angle_);

      double proj_y = sin((M_PI * 0.5) - params_.cam_angle_) * x_shift;
      double proj_x = -y + dims.second * 0.5;
      double proj_z = z_center_ - z_shift;

      Eigen::Vector4d pt_3d(proj_x, proj_y, proj_z, 1.0);
      Eigen::Vector3d pixels;

      img_processing_->Project3DPtToPixels(pt_3d, pixels, proj_z);

      int p_x = pixels.x();
      int p_y = pixels.y();

      if(p_x < 0 || p_x > img.cols || p_y < (kFrameHeight - img.rows) || p_y > kFrameHeight)
        continue;

      int coordinate = i + j * occ_grid->info.width;

      if (img.at<uchar>(cv::Point(p_x, p_y - (kFrameHeight - img.rows))) > kThresh)
        occ_grid->data.at(coordinate) = kValueUnoccupied;
    }
  }
}

std::pair<double, double> OccGridManager::ProjectPixelToGround(int p_x, int p_y) {
  Eigen::Vector3d pixels(p_x, p_y, 1);
  Eigen::Vector4d pt_3d;

  // TODO: do calculation for 579.4302
  // TODO: make calculations more general
  double d_angle = atan((double)(p_y - 0.5 * kFrameHeight) / (double)579.4302);
  double theta = params_.cam_angle_ - d_angle;
  double z_temp = params_.cam_height_ / cos(theta);
  double z = z_temp * cos(d_angle);

  img_processing_->ProjectPixelsTo3D(pixels, pt_3d, z);

  double x_ground = pt_3d.z() * sin(params_.cam_angle_);
  double y_ground = pt_3d.x();

  return std::pair<double, double>(x_ground, y_ground);
}

void OccGridManager::UpdateParameters() {
  x_ground_center_ = params_.cam_height_ * tan(params_.cam_angle_);
  z_center_ = params_.cam_height_ / cos(params_.cam_angle_);
}
}
