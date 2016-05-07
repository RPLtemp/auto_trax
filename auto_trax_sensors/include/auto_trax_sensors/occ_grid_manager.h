#ifndef AUTO_TRAX_OCC_GRID_MANAGER_H
#define AUTO_TRAX_OCC_GRID_MANAGER_H

#include <nav_msgs/OccupancyGrid.h>

#include <auto_trax_sensors/image_processing.h>

namespace auto_trax {
// Default values
static const double kDefaultAngle = 1.45;
static const double kDefaultHeight = 0.08;
static const double kDefaultResolution = 0.005;

// Constants
static const int kThresh = 0.5 * kWhite(1);
static const int kValueOccupied = 100;
static const int kValueUnoccupied = 0;

struct OccGridManagerParameters {
  OccGridManagerParameters():
      cam_angle_(kDefaultAngle),
      cam_height_(kDefaultHeight),
      resolution_(kDefaultResolution) {
  }

  // Camera position parameters
  double cam_angle_;
  double cam_height_;

  // Occupancy grid parameters
  double resolution_;
};

class OccGridManager {
  public:
    OccGridManager(ImageProcessing* img_proc);
    virtual ~OccGridManager();

    int GetFarthestOpenRow(nav_msgs::OccupancyGrid occ_grid);

    void GetGoalPoint(nav_msgs::OccupancyGrid occ_grid, std::pair<double, double>& goal_pt);

    bool GetRequiredOccGridDimensions(cv::Mat& img, std::pair<double, double> &dims);

    std::vector<int> GetRow(nav_msgs::OccupancyGrid occ_grid, int row_ind);

    int GetUnoccupiedRowMidpoint(std::vector<int> row);

    void OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid);

    std::pair<double, double> ProjectPixelToGround(int p_x, int p_y);

    void UpdateParameters();

    OccGridManagerParameters params_;

  private:
    ImageProcessing* img_processing_;

    double x_ground_center_;
    double x_offset_;
    double z_center_;
};
}

#endif // AUTO_TRAX_OCC_GRID_MANAGER_H
