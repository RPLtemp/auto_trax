#include "auto_trax_sensors/occ_grid_manager.h"

void OccGridManager::OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid) {
  occ_grid->info.width = img.rows;
  occ_grid->info.height = img.cols;
  occ_grid->info.resolution = 0.001;

  occ_grid->data.assign(occ_grid->info.width * occ_grid->info.height, 0);

  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (img.at<uchar>(cv::Point(j, i)) > 200) {
        int coordinate = i + j * occ_grid->info.width;

        occ_grid->data.at(coordinate) = 100;
      }
    }
  }
}
