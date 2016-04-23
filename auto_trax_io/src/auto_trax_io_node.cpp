#include <auto_trax_io/auto_trax_io_node.h>

AutoTraxIoNode::AutoTraxIoNode(int servo_max,
                               int servo_min,
                               int pwm_frequency,
                               int i2c_channel) :
    steering_(0x40)
{
    servo_max_ = servo_max;
    servo_min_ = servo_min;
    pwm_frequency_ = pwm_frequency;
    i2c_channel_ = i2c_channel;
    parameter_initialized_ = true;

    if (steering_.openPCA9685() < 0) {
           ROS_ERROR("COULD NOT OPEN PCA9685");
    } else {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n",steering_.kI2CAddress) ;
        servo_range_ = servo_max_ - servo_min_;
        steering_.setAllPWM(0,0) ;
        steering_.reset() ;
        steering_.setPWMFrequency(pwm_frequency_);
    }
}

AutoTraxIoNode::~AutoTraxIoNode(){
    steering_.closePCA9685();
}

inline double AutoTraxIoNode::AngleConversion(double angle_in_rads){

    double positive_angle_in_rads = angle_in_rads + angle_range_in_rads_/2;
    return (servo_range_*positive_angle_in_rads)/angle_range_in_rads_ + servo_min_;
}

bool AutoTraxIoNode::serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                                     auto_trax_io::ApplySteeringAngle::Response &res){
    double angle_in_radians;
    angle_in_radians = req.Message.steering_angle;
    double pwm_result = AngleConversion(angle_in_radians);
    std::cout <<"got a call. pwm result is: " << pwm_result << std::endl;
    if (steering_.error >= 0 && parameter_initialized_){
        steering_.setPWM(i2c_channel_,0,pwm_result);
        res.success = true;
        return true;
    }
    res.success = false;
    return false;

}

int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "auto trax io node");

    ros::NodeHandle nh;
    std::string steering_service_name;
    int pwm_frequency, i2c_channel, servo_min, servo_max;


    bool read_all_parameters = nh.getParam("steering_service_name", steering_service_name) &&
                               nh.getParam("pwm_frequency", pwm_frequency) &&
                               nh.getParam("i2c_channel", i2c_channel) &&
                               nh.getParam("servo_min", servo_min) &&
                               nh.getParam("servo_max", servo_max);

    // Check if parameters where imported
    if (! read_all_parameters){
        ROS_ERROR("Geometric Parameters not imported");
        return false;
    }

    AutoTraxIoNode node(servo_max,
                        servo_min,
                        pwm_frequency,
                        i2c_channel);

    ros::ServiceServer service = nh.advertiseService(steering_service_name, &AutoTraxIoNode::serviceCallback, &node);


    ros::spin();
    return 0;
}
