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
        LaserScanProcessor();

        virtual ~LaserScanProcessor();

        float GetClosestRange(const std::vector<float> &ranges);

    };

}// namespace auto_trax

#endif //AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_H
