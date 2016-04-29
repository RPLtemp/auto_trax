#ifndef AUTO_TRAX_OCC_GRID_MANAGER_H
#define AUTO_TRAX_OCC_GRID_MANAGER_H

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>

namespace auto_trax {

class OccGridManager {
  public:
    OccGridManager();
    virtual ~OccGridManager();

    void OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid, int horizon_pixel);

    std::pair<double, double> GetRequiredOccGridDimensions(cv::Mat& img);

    std::pair<double, double> ProjectPixelToGround(int p_x, int p_y);
};
}

#endif // AUTO_TRAX_OCC_GRID_MANAGER_H
