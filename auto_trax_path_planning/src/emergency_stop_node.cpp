#include <ros/ros.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "auto_trax_emergency_stop");
    ros::NodeHandle nh;


    // Spin
    ros::spin();
}