#include <auto_trax_io/auto_trax_io_node.h>

AutoTraxIoNode::AutoTraxIoNode(AutoTraxIoParameters p) :
    pwm_driver_(0x40)
{
    servo_max_ = p.servo_max;
    servo_min_ = p.servo_min;
    motor_min_ = p.motor_min;
    motor_max_ = p.motor_max;
    pwm_frequency_ = p.pwm_frequency;
    angle_i2c_channel_ = p.angle_i2c_channel;
    speed_i2c_channel_ = p.speed_i2c_channel;
    minimum_linear_velocity_m_s_ = p.minimum_linear_velocity_m_s;
    maximum_linear_velocity_m_s_ = p.maximum_linear_velocity_m_s;
    zero_speed_pwm_ = p.zero_speed_pwm;
    m_upper_ = (maximum_linear_velocity_m_s_-minimum_linear_velocity_m_s_)/(motor_max_ - motor_upper_pwm_threshold_);
    q_upper_ = minimum_linear_velocity_m_s_ - m_upper_*motor_upper_pwm_threshold_;

    m_lower_ = (-minimum_linear_velocity_m_s_+maximum_linear_velocity_m_s_)/(motor_lower_pwm_threshold_ - motor_min_ );
    q_lower_ = -minimum_linear_velocity_m_s_ - m_lower_*motor_lower_pwm_threshold_;


    parameter_initialized_ = true;

    if (pwm_driver_.openPCA9685() < 0) {
        ROS_ERROR("COULD NOT OPEN PCA9685");
    } else {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n",pwm_driver_.kI2CAddress) ;
        servo_range_ = servo_max_ - servo_min_;
        pwm_driver_.setAllPWM(0,0) ;
        pwm_driver_.reset() ;
        pwm_driver_.setPWMFrequency(pwm_frequency_);
    }
    std::cout << "parameters initialized: " << parameter_initialized_ << std::endl;
}

AutoTraxIoNode::~AutoTraxIoNode(){
    pwm_driver_.closePCA9685();
}

inline double AutoTraxIoNode::AngleConversion(double angle_in_rads){

    double positive_angle_in_rads = angle_in_rads + angle_range_in_rads_/2;
    return (servo_range_*positive_angle_in_rads)/angle_range_in_rads_ + servo_min_;
}

inline double AutoTraxIoNode::SpeedConversion(double speed_in_m_s){
    // Check if we are in the "dead zone" where the car cannot run
    if (std::abs(speed_in_m_s) < minimum_linear_velocity_m_s_) {
        return zero_speed_pwm_;
    }
    // Check if we are above saturation on both ends
    if (std::abs(speed_in_m_s) > maximum_linear_velocity_m_s_) {
        if (std::signbit(speed_in_m_s)) {
            return motor_min_;
        } else {
            return motor_max_;
        }
    }
    //if speed is negative use lower equation
    if (std::signbit(speed_in_m_s)) {
        double pwm_result = (speed_in_m_s - q_lower_) / m_lower_;
        return static_cast<int>(pwm_result);
    } else { //else use upper eq
        double pwm_result = (speed_in_m_s - q_upper_) / m_upper_;
        return static_cast<int>(pwm_result);
    }
    //just to be safe for testing purposes.
    return zero_speed_pwm_;
}


bool AutoTraxIoNode::serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                                     auto_trax_io::ApplySteeringAngle::Response &res){
    double angle_in_radians = req.Message.steering_angle;
    double speed_in_m_s = req.Message.speed;
    double pwm_angle = AngleConversion(angle_in_radians);
    double pwm_speed = SpeedConversion(speed_in_m_s);

    std::cout <<"Pwm angle: " << pwm_angle <<
               "Pwm speed: " << pwm_speed <<
                std::endl;
    if (pwm_driver_.error >= 0 && parameter_initialized_){
        pwm_driver_.setPWM(angle_i2c_channel_,0,pwm_angle);
        pwm_driver_.setPWM(speed_i2c_channel_,0,pwm_speed);
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

    AutoTraxIoParameters params;
    bool read_all_parameters = nh.getParam("steering_service_name", steering_service_name) &&
                               nh.getParam("pwm_frequency", params.pwm_frequency) &&
                               nh.getParam("angle_i2c_channel", params.angle_i2c_channel) &&
                               nh.getParam("speed_i2c_channel", params.speed_i2c_channel) &&
                               nh.getParam("servo_min", params.servo_min) &&
                               nh.getParam("servo_max", params.servo_max) &&
                               nh.getParam("motor_min", params.motor_min) &&
                               nh.getParam("motor_max", params.motor_max) &&
                               nh.getParam("minimum_linear_velocity_m_s", params.minimum_linear_velocity_m_s) &&
                               nh.getParam("maximum_linear_velocity_m_s", params.maximum_linear_velocity_m_s) &&
                               nh.getParam("zero_speed_pwm", params.zero_speed_pwm);

    // Check if parameters where imported
    if (! read_all_parameters){
        ROS_ERROR("Geometric Parameters not imported");
        return false;
    }

    AutoTraxIoNode node(params);
    ros::ServiceServer service = nh.advertiseService(steering_service_name, &AutoTraxIoNode::serviceCallback, &node);


    ros::spin();
    return 0;
}
