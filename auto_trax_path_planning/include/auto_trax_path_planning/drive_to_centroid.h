//
// Created by frank on 09.07.16.
//

#ifndef AUTO_TRAX_PATH_PLANNING_DRIVE_TO_CENTROID_H
#define AUTO_TRAX_PATH_PLANNING_DRIVE_TO_CENTROID_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include <iostream>
#include <vector>

// Default values
static const std::string kDefaultDriveToCentroidTopic = "/angle_to_centroid";
static const int kDefaultDriveToCentroidQueueSize = 1;
static const std::string kDefaultCentroidSubTopic = "/scan_center";
static const int kDefaultCentroidSubQueueSize = 1;

class DriveToCentroid
{
public:
    DriveToCentroid(ros::NodeHandle nh);
    virtual ~DriveToCentroid();

    // Callback
    void CallbackScan (const geometry_msgs::PointStamped::ConstPtr &scan_center_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_centroid_point;
    ros::Publisher pub_angle_to_centroid_;
};

#endif //AUTO_TRAX_PATH_PLANNING_DRIVE_TO_CENTROID_H
