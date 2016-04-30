#ifndef AUTO_TRAX_OCC_GRID_MANAGER_H
#define AUTO_TRAX_OCC_GRID_MANAGER_H

#include <nav_msgs/OccupancyGrid.h>

#include <auto_trax_sensors/image_processing.h>

#include <iostream>

namespace auto_trax {
// Default values
static const double kDefaultAngle = 0.25 * M_PI;
static const double kDefaultHeight = 0.1;
static const double kDefaultResolution = 0.005;

struct OccGridManagerParameters {
  OccGridManagerParameters():
      angle_(kDefaultAngle),
      height_(kDefaultHeight),
      resolution_(kDefaultResolution) {
  }

  // Camera position parameters
  double angle_;
  double height_;

  // Occupancy grid parameters
  double resolution_;
};

class OccGridManager {
  public:
    OccGridManager();
    virtual ~OccGridManager();

    void OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid);

    std::pair<double, double> GetRequiredOccGridDimensions(cv::Mat& img);

    std::pair<double, double> ProjectPixelToGround(int p_x, int p_y);

    void SetImageProcessor(ImageProcessing& img_proc);

    void UpdateDerivedParameters();

    OccGridManagerParameters params_;

  private:
    ImageProcessing* img_processing_;

    double x_ground_center_;
    double x_offset_;
    double z_center_;
};
}

#endif // AUTO_TRAX_OCC_GRID_MANAGER_H
