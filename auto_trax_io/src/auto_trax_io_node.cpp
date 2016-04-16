#include <auto_trax_io/auto_trax_io_node.h>

AutoTraxIoNode::AutoTraxIoNode(){
    ros::NodeHandle nh;

//    ros::ServiceServer service = nh.advertiseService("apply_steering_angle", &AutoTraxIoNode::serviceCallback);

}

void AutoTraxIoNode::serviceCallback(auto_trax_io::ApplySteeringAngle::Request  &req,
                                     auto_trax_io::ApplySteeringAngle::Response &res){

}

int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "auto_trax_io_node");



    AutoTraxIoNode node();

    ros::Rate r(50); // 50 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
