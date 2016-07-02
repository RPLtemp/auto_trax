//
// Created by frank on 02.07.16.
//

#ifndef AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_NODE_H
#define AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

namespace auto_trax {

    static const std::string kDefaultLaserScanSubTopic = "/scan";
    static const std::string kDefaultLaserScanInfoPubTopic = "/scan_info";

    class LaserScanProcessorNode {
    public:
        LaserScanProcessorNode();
        virtual ~LaserScanProcessorNode();

        void ScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber laser_scan_sub_;
        ros::Publisher laser_scan_info_pub_;
    };
}


#endif //AUTO_TRAX_SENSORS_LASER_SCAN_PROCESSOR_NODE_H
