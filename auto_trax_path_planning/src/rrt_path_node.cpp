//
// Created by frank on 10.07.16.
//

#include "auto_trax_path_planning/rrt_path_processor.h"


int main(int argc, char** argv)
{
  ros::init (argc, argv, "auto_trax_oc_grid_node");
  ros::NodeHandle nh;

  RRTPathProcessor rrtPathProcessor(nh);

  ros::spin();
}
