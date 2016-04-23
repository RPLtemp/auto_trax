#ifndef AUTO_TRAX_IO_NODE_H
#define AUTO_TRAX_IO_NODE_H

#include <ros/ros.h>
#include <auto_trax_io/ApplySteeringAngle.h>


class AutoTraxIoNode{

public:
    AutoTraxIoNode();

    void serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                         auto_trax_io::ApplySteeringAngle::Response &res);


};

#endif // AUTO_TRAX_IO_NODE_H
