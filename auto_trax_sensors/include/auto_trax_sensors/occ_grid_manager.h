#ifndef AUTO_TRAX_OCC_GRID_MANAGER_H
#define AUTO_TRAX_OCC_GRID_MANAGER_H

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>

class OccGridManager {
  public:
    static void OccGridFromBinaryImage(cv::Mat& img, nav_msgs::OccupancyGridPtr& occ_grid);
};


#endif // AUTO_TRAX_OCC_GRID_MANAGER_H
