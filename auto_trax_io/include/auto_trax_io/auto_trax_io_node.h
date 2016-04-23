#ifndef AUTO_TRAX_IO_NODE_H
#define AUTO_TRAX_IO_NODE_H

#include <ros/ros.h>
#include <auto_trax_io/ApplySteeringAngle.h>
#include <auto_trax_io/JHPWMPCA9685.h>
#include <iostream>


class AutoTraxIoNode{
private:
    inline double AngleConversion(double angle_in_radians);

public:
    AutoTraxIoNode();
    ~AutoTraxIoNode();

    PCA9685 steering_;
    static const int servo_max_ = 720;
    static const int servo_min_ = 120;
    int servo_range_;
    static const double angle_range_in_rads_ = 2.26893;

    int servoZero_;
    bool serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                         auto_trax_io::ApplySteeringAngle::Response &res);


};

#endif // AUTO_TRAX_IO_NODE_H
