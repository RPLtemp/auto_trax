//
// Created by frank on 02.07.16.
//

#include "auto_trax_sensors/laser_scan_processor.h"

namespace auto_trax {

    LaserScanProcessor::LaserScanProcessor(){}

    float LaserScanProcessor::GetClosestRange(const std::vector<float> &ranges)
    {
        return *(std::min(ranges.begin(), ranges.end()));
    }

}