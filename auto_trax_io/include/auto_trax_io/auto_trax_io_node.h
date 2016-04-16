
#include <ros/ros.h>
#include <auto_trax_io/ApplySteeringAngle.h>


class AutoTraxIoNode{

public:
    AutoTraxIoNode();

    void serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                         auto_trax_io::ApplySteeringAngle::Response &res);


};

