#include "auto_trax_sensors/occ_grid_manager.h"

namespace auto_trax {

OccGridManager::OccGridManager() {

}

OccGridManager::~OccGridManager() {
}

void OccGridManager::OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid, int horizon_pixel) {
  // Get the dimensions of the required occupancy grid
  std::pair<double, double> dims = GetRequiredOccGridDimensions(img);

  std::cout << "x: " << dims.first << std::endl;
  std::cout << "y: " << dims.second << std::endl;

  occ_grid->info.resolution = 0.005;
  occ_grid->info.width = ceil(dims.first / occ_grid->info.resolution);
  occ_grid->info.height = ceil(dims.second / occ_grid->info.resolution);

  occ_grid->data.assign(occ_grid->info.width * occ_grid->info.height, 100);

  std::pair<double, double> projected_point(0.0, 0.0);

  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (img.at<uchar>(cv::Point(j, i)) > 200) {
        int p_x = j - 320;
        int p_y = i + (480 - horizon_pixel) - 240;

        projected_point = ProjectPixelToGround(p_x, p_y);

        int occ_grid_x = floor(projected_point.first / occ_grid->info.resolution);
        int occ_grid_y = floor(projected_point.second / occ_grid->info.resolution) + 0.5 * occ_grid->info.height;

        //int coordinate = i + j * occ_grid->info.width;
        int coordinate = occ_grid_x + occ_grid_y * occ_grid->info.width;

        occ_grid->data.at(coordinate) = 0;
      }
    }
  }
}

std::pair<double, double> OccGridManager::GetRequiredOccGridDimensions(cv::Mat& img) {
  std::pair<double, double> top_left_corner = ProjectPixelToGround(-320, 40);
  std::pair<double, double> bottom_right_corner = ProjectPixelToGround(319, 240);

  // Verify the x-dimension range (it may not start at 0)
  return std::pair<double, double>(fabs(top_left_corner.first),
                                   fabs(bottom_right_corner.second - top_left_corner.second));
}

std::pair<double, double> OccGridManager::ProjectPixelToGround(int p_x, int p_y) {
  double theta_x = atan((double)p_x / 203.8f);
  double theta_y = atan((double)p_y / 240.0f);

  double camera_theta = 45.0 * M_PI / 180.0;
  double camera_z_pos = 0.1;

  double x = camera_z_pos * tan(camera_theta - theta_y);

  double h = sqrt(camera_z_pos * camera_z_pos + x * x);

  double y = h * tan(theta_x);

  return std::pair<double, double>(x,y);
}
}
