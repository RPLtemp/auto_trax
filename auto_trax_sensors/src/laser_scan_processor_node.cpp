//
// Created by frank on 02.07.16.
//

#include "auto_trax_sensors/laser_scan_processor_node.h"

namespace auto_trax{

LaserScanProcessorNode::LaserScanProcessorNode()
{
    ros::NodeHandle pnh("~");

    std::string laser_scan_sub_topic;
    std::string laser_scan_info_pub_topic;
    pnh.param("laser_scan_topic", laser_scan_sub_topic, kDefaultLaserScanSubTopic);
    pnh.param("laser_scan_info_topic", laser_scan_info_pub_topic, kDefaultLaserScanInfoPubTopic);

    laser_scan_sub_ = nh_.subscribe(laser_scan_sub_topic, 1, &LaserScanProcessorNode::ScanCallback, this);
    laser_scan_info_pub_ = nh_.advertise<std_msgs::Float32>(laser_scan_info_pub_topic, 1);

}

    LaserScanProcessorNode::~LaserScanProcessorNode() {

}

void LaserScanProcessorNode::ScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{

}


} // namespace auto_trax

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_scan_processor");

    auto_trax::LaserScanProcessorNode Laser_scan_processor;

    ros::spin();

    return 0;
}



