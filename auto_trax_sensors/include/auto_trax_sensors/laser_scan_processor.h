//
// Created by frank on 02.07.16.
//

#ifndef AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_H
#define AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace auto_trax {

class LaserScanProcessor {
  public:
    LaserScanProcessor(const sensor_msgs::LaserScanConstPtr& laser_scan);
    virtual ~LaserScanProcessor();

    float GetClosestRange(const std::vector<float> &ranges);

    void GetMaxValidAngle(float& angle, int& ind);

    void GetMinValidAngle(float& angle, int& ind);

    inline bool IsRangeValid(float range) {
      return (range < range_max_ && range > range_min_);
    }

  private:
    float angle_min_;
    float angle_max_;
    float angle_increment_;

    float range_min_;
    float range_max_;

    std::vector<float> ranges_;
};

} // namespace auto_trax

#endif //AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_H
