#ifndef AUTO_TRAX_IO_NODE_H
#define AUTO_TRAX_IO_NODE_H

#include <ros/ros.h>
#include <auto_trax_io/ApplySteeringAngle.h>
#include <auto_trax_io/JHPWMPCA9685.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
struct AutoTraxIoParameters{
    int servo_max;
    int servo_min;
    int pwm_frequency;
    int angle_i2c_channel;
    int speed_i2c_channel;
    double minimum_linear_velocity_m_s;
    double maximum_linear_velocity_m_s;
    int zero_speed_pwm;
    int motor_max;
    int motor_min;
    int motor_lower_pwm_threshold;
    int motor_upper_pwm_threshold;
};

class AutoTraxIoNode{
private:
    PCA9685 *pwm_driver_;
    int servo_max_;
    int servo_min_;
    int motor_max_;
    int motor_min_;
    double m_upper_, m_lower_;
    double q_upper_, q_lower_;
    double minimum_linear_velocity_m_s_;
    double maximum_linear_velocity_m_s_;
    int motor_lower_pwm_threshold_;
    int motor_upper_pwm_threshold_;
    int zero_speed_pwm_;
    int servo_range_;
    int pwm_frequency_;
    int angle_i2c_channel_, speed_i2c_channel_;
    bool parameter_initialized_;
    static const double angle_range_in_rads_ = 2.26893;

    inline double AngleConversion(double angle_in_radians);
    inline double SpeedConversion(double speed_in_m_s);
public:
    AutoTraxIoNode(AutoTraxIoParameters parameters);
    ~AutoTraxIoNode();

    bool serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                         auto_trax_io::ApplySteeringAngle::Response &res);
};

#endif // AUTO_TRAX_IO_NODE_H
