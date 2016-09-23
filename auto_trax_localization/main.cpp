#include <iostream>
#include "include/localization_node.hpp"

using namespace std;

int main(int argc, char** argv) {

  ros::init(argc, argv, "localization_node");
  ros::NodeHandle nh;

  LocalizationNode localizationNode(nh);
  ros::spin();

  return 0;
}