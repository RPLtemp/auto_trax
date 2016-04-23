#ifndef AUTO_TRAX_IO_NODE_H
#define AUTO_TRAX_IO_NODE_H

#include <ros/ros.h>
#include <auto_trax_io/ApplySteeringAngle.h>
#include <auto_trax_io/JHPWMPCA9685.h>
#include <iostream>


class AutoTraxIoNode{

public:
    AutoTraxIoNode();

    PCA9685 steering_;

    bool serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                         auto_trax_io::ApplySteeringAngle::Response &res);


};

#endif // AUTO_TRAX_IO_NODE_H
