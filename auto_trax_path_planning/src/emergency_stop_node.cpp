//

//


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "auto_trax_emergency_stop_node");
    ros::NodeHandle nh;

    // Construct class detection_processor with ros::NodeHandle and parameter structure
    auto_trax::ControllerProcessor processor(nh, parameter);

    // Spin
    ros::spin();
}