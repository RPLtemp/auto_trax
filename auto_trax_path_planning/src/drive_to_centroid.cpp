//
// Created by frank on 09.07.16.
//

#include "auto_trax_path_planning/drive_to_centroid.h"

DriveToCentroid::DriveToCentroid(ros::NodeHandle nh): nh_(nh){

    ROS_DEBUG("Drive to centroid Processor started!");

    sub_centroid_point = nh_.subscribe("/merged_scan",
                             1,
                             &DriveToCentroid::CallbackScan,
                             this);
    // Create publisher for scan distance
    pub_angle_to_centroid_ = nh_.advertise<std_msgs::Float64>("/angle_to_centroid", 1);
}

DriveToCentroid::~DriveToCentroid(){
}

void DriveToCentroid::CallbackScan (const geometry_msgs::PointStamped::ConstPtr &scan_center_msg){

    double angle_to_centroid = atan2(scan_center_msg->point.y,scan_center_msg->point.x);

    std_msgs::Float64 result;
    result.data = angle_to_centroid;
    pub_angle_to_centroid_.publish(result);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "auto_trax_drive_to_centroid");
    ros::NodeHandle nh;

    // Spin
    ros::spin();
}