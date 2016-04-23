#include <auto_trax_io/auto_trax_io_node.h>

AutoTraxIoNode::AutoTraxIoNode() :
    steering_(0x40)
{
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("apply_steering_angle", &AutoTraxIoNode::serviceCallback, this);
    if (steering_.openPCA9685() < 0) {
           ROS_ERROR("COULD NOT OPEN PCA9685");
    }
    ROS_INFO("PCA9685 Device Address: 0x%02X\n",steering_.kI2CAddress) ;
    servo_range_ = servo_max_ - servo_min_;
    steering_.setAllPWM(0,0) ;
    steering_.reset() ;
    steering_.setPWMFrequency(60);
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

    if (steering_.error >= 0){
        steering_.setPWM(0,0,AngleConversion(angle_in_radians));
    }
}

int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "auto_trax_io_node");

    AutoTraxIoNode node;


    ros::Rate r(50); // 50 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
