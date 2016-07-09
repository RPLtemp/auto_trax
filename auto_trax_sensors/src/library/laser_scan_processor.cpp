//
// Created by frank on 02.07.16.
//

#include "auto_trax_sensors/laser_scan_processor.h"

namespace auto_trax {

LaserScanProcessor::LaserScanProcessor(const sensor_msgs::LaserScanConstPtr &laser_scan) {
  angle_min_ = laser_scan->angle_min;
  angle_max_ = laser_scan->angle_max;
  angle_increment_ = laser_scan->angle_increment;

  range_min_ = laser_scan->range_min;
  range_max_ = laser_scan->range_max;

  ranges_ = laser_scan->ranges;
}

LaserScanProcessor::~LaserScanProcessor() {
}

float LaserScanProcessor::GetClosestRange(const std::vector<float> &ranges) {
  return *(std::min(ranges.begin(), ranges.end()));
}

void LaserScanProcessor::GetMaxValidAngle(float& angle, int& ind) {
  for (int i = ranges_.size() - 1; i >= 0; i--) {
    float range = ranges_.at(i);

    if (IsRangeValid(range)) {
      angle = angle_min_ + i * angle_increment_;
      ind = i;
      break;
    }
  }
}

void LaserScanProcessor::GetMinValidAngle(float& angle, int& ind) {
  for (int i = 0; i < ranges_.size(); i++) {
    float range = ranges_.at(i);

    if (IsRangeValid(range)) {
      angle = angle_min_ + i * angle_increment_;
      ind = i;
      break;
    }
  }
}

}
