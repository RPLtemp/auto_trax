#include <ros/ros.h>
#include <stdio.h>

int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "auto_trax_io_node");
    ros::NodeHandle nh;


std::cout << "Hello world!" << std::endl;

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}