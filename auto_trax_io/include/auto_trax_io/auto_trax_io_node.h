#ifndef AUTO_TRAX_IO_NODE_H
#define AUTO_TRAX_IO_NODE_H

#include <ros/ros.h>
#include <auto_trax_io/ApplySteeringAngle.h>
#include <auto_trax_io/JHPWMPCA9685.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


class AutoTraxIoNode{
private:
    PCA9685 steering_;
    int servo_max_;
    int servo_min_;
    int servo_range_;
    int pwm_frequency_;
    int i2c_channel_;
    bool parameter_initialized_;
    static const double angle_range_in_rads_ = 2.26893;

    inline double AngleConversion(double angle_in_radians);

public:
    AutoTraxIoNode(int servo_max,
                   int servo_min,
                   int pwm_frequency,
                   int i2c_channel);
    ~AutoTraxIoNode();

    bool serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                         auto_trax_io::ApplySteeringAngle::Response &res);
};

#endif // AUTO_TRAX_IO_NODE_H
